#!/usr/bin/env python3
#Imports
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import tf2_ros
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_geometry_msgs import PoseStamped, do_transform_pose
import actionlib
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
import sys
import math
from actionlib_msgs.msg import GoalStatus
from util import *
import warnings
warnings.filterwarnings("ignore")

class frontierEvaluation():
    cache = pd.DataFrame({"x": [], "y": [], "cluster": [], "centroid_x": [], "centroid_y": [], "distance": []})
    occ_grid = None
    curr_grid = []
    tfBuffer = tf2_ros.Buffer()
    pos = Pose()
    locs = None
    frontiersGrid = None
    frontier_markers = None
    centroids = None
    goalPos = None
    
    def __init__(self):
        """
        Constructor setting up a node, action client, tf listener, publishers, and subscribers. 
        It also calls the main function. Note that I decided to have a main program running
        simultaneously to the subscriber callback to improve performance. 
        """        
        self.init_node()
        self.init_action_client()
        self.init_listener()
        self.init_publishers()
        self.init_subscriber()
        self.main_program()
    
    def init_node(self):
        """
        Initializes a node called explorer to support frontier exploration. 
        """        
        rospy.init_node("explorer")
    
    def init_subscriber(self):
        """
        Initializes a subscriber using rospy that takes in the map as an occupancy grid. 
        """
        sub = rospy.Subscriber("map", OccupancyGrid, self.callback)
        #rospy.spin()
    
    def init_publishers(self):
        """
        Initializes several publishers for different tasks throughout the file. 
        """        
        #Publishes markers indicating obstacle growing.
        self.pub = rospy.Publisher("/expansion", Marker, queue_size=1, latch=True)
        #Publishes an occupancy grid to show the frontiers
        self.grid_pub = rospy.Publisher("/frontiers_map", OccupancyGrid, queue_size=1, latch=True)
        #Publishes a marker array showing frontier clusters
        self.frontiers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1, latch=True)
        #Publishes markers indicating the centroids of the frontier clusters. 
        self.centroids_pub = rospy.Publisher("/centroids", Marker, queue_size=1, latch=True)
    
    def init_listener(self, source="map", target="base_footprint"):
        """
        Creates a tf listener and obtains the transformation data using the tfBuffer class variable. 

        Args:
            source (str, optional): Starting reference frame. Defaults to "map".
            target (str, optional): Target reference from for transformation. Defaults to "base_footprint".
        """        
        try:
            listener = tf2_ros.TransformListener(frontierEvaluation.tfBuffer)
            trans = frontierEvaluation.tfBuffer.lookup_transform(source, target, rospy.Time())
            frontierEvaluation.pos = trans.transform.translation
            #break
        except:
            rospy.Rate(10.0).sleep()
            #continue
    
    def init_action_client(self):
        """
        Creates an action client with the move base action in order to send goals to the robot. 
        """        
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
    
    def coordinate_callback_thread(self, x, y, z):
        """


        Args:
            x (_type_): _description_
            y (_type_): _description_
            z (_type_): _description_
        """        
        tol = 0.5
     #   dx = abs(x-frontierEvaluation.pos.position.x)
      #  dy = abs(y-frontierEvaluation.pos.position.y)
      #  d = math.sqrt(dx**2 + dy**2)
       # theta = math.atan2(dy, dx)
      #  vx = d*math.cos(theta)
       # vy = d*math.sin(theta)
        try:
            pose = Pose()

            pose.position.x = x+tol
            pose.position.y = y+tol
            pose.position.z = z
            pose.orientation.w = 1.0

    #        try:
            listener = tf2_ros.TransformListener(frontierEvaluation.tfBuffer)
            while not rospy.is_shutdown():
                rospy.sleep(1)
                try:
                    trans = frontierEvaluation.tfBuffer.lookup_transform("base_footprint", "map", rospy.Time(0))
                    print(trans)
                    break
                except:
                    raise
                    continue
            tfPose = PoseStamped()
            tfPose.pose = pose
            tfPose.header.frame_id = "base_footprint"
            tfPose.header.stamp = rospy.Time.now()
            outPose = do_transform_pose(tfPose, trans)
    #       except:
    #          rospy.Rate(10.0).sleep()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_footprint"
            goal.target_pose.pose=outPose.pose

            self.client.send_goal(goal)
            #self.client.wait_for_result()
        except:
            raise
            print("tf listener didn't work")
            rospy.Rate(10.0).sleep()


        #return goal
        
    def dbscan(self, frontiers, epsilon, min_points):
        frontiers = pd.DataFrame(frontiers)
        db = DBSCAN(eps=epsilon, min_samples=min_points, metric="l1")
        clusters = db.fit_predict(frontiers)
       # print(len(clusters))
        #print("______________")
        #print(len(frontierEvaluation.cache["x"]))
#        frontierEvaluation.cache["cluster"] = clusters
 #       frontierEvaluation.cache = frontierEvaluation.cache[frontierEvaluation.cache["cluster"] != -1]
  #      frontierEvaluation.cache = frontierEvaluation.cache.sort_values('cluster')

        frontiers["cluster"] = clusters
        frontiers = frontiers[frontiers["cluster"] != -1]
        frontiers = frontiers.sort_values('cluster')
        return frontiers
    
    def visualizeFrontiers(self, frontiersGrid, header, info):
        occGrid = OccupancyGrid(header, info, frontiersGrid.flatten(order="C"))
        self.grid_pub.publish(occGrid)
    
    def colorClusters(self, df, occupancy_grid):
        clusters = list(set(df["cluster"]))
#        clusters = list(set(frontierEvaluation.cache["cluster"]))
        markers = MarkerArray()
        centers = []
        palette = list(reversed(sns.color_palette("rainbow", len(clusters)).as_hex()))
        colors = []
        distances = []
        centroid_x = []
        centroid_y = []
        for p in palette:
            p = p.replace("#", "")
            colors.append(getRGB(p))
        
        for c in range(len(clusters)):
            #sub_df = frontierEvaluation.cache[frontierEvaluation.cache["cluster"] == clusters[c]]
            sub_df = df[df["cluster"] == clusters[c]]

            points = getPointArray([(x,y) for x,y in zip(sub_df[0], sub_df[1])], occupancy_grid)
            marker = createMarkers(points=points, indx=c, action=0, ns="frontiers", color=colors[c], scale=0.1, style=7)
            markers.markers.append(marker)

            xc = sum(sub_df[0])/len(sub_df[0])
            yc = sum(sub_df[1])/len(sub_df[1])
            centroid_x += [xc]*len(sub_df)
            centroid_y += [yc]*len(sub_df)
#            frontierEvaluation.cache["centroid_x"] += [xc]*len(sub_df)
 #           frontierEvaluation.cache["centroid_y"] += [xc]*len(sub_df)
            center = [xc, yc, 20]
          #  center = getPointArray([center], occupancy_grid)
            centers.append(center)
            centerPoint = getPointArray([center], occupancy_grid)
#            print(center)
          #  print(xc)

            distance = [calculateDistance(frontierEvaluation.pos.position, centerPoint)]*len(sub_df)
            distances += distance

        centers = getPointArray(centers, occupancy_grid)
        centerMarker = createMarkers(points=centers, indx=c, action=0, ns="centroids", color=[255,255,255], scale=0.3, style=7)
#        print(type(center))
        
        df["distance"] = distances
        df["centroid_x"] = centroid_x
        df["centroid_y"] = centroid_y
        df.sort_values("distance")

        return markers, centerMarker, df
    
    def compareGrid(self, currentGrid, newGrid):
        if sum(currentGrid) == sum(newGrid):
            return False
        return True
    
    
    def habitability(self, x, y, grid):
        centroid = (x,y)
        top = [
            (x-3, y+3), (x-2, y+3), (x-1, y+3), (x, y+3), (x+1, y+3), (x+2, y+3), (x+3, y+3),
            (x-3, y+2), (x-2, y+2), (x-1, y+2), (x, y+2), (x+1, y+2), (x+2, y+2), (x+3, y+2),
            (x-3, y+1), (x-2, y+1), (x-1, y+1), (x, y+1), (x+1, y+1), (x+2, y+1), (x+3, y+1)
            ]
        bottom = [
            (x-3, y-3), (x-2, y-3), (x-1, y-3), (x, y-3), (x+1, y-3), (x+2, y-3), (x+3, y-3),
            (x-3, y-2), (x-2, y-2), (x-1, y-2), (x, y-2), (x+1, y-2), (x+2, y-2), (x+3, y-2),
            (x-3, y-1), (x-2, y-1), (x-1, y-1), (x, y-1), (x+1, y-1), (x+2, y-1), (x+3, y-1)
        ]
        left = [
            (x-3, y+3), (x-2, y+3), (x-1, y+3), (x-3, y+2), (x-2, y+2), (x-1, y+2),
            (x-3, y+1), (x-2, y+1), (x-1, y+1), (x-3, y), (x-2, y), (x-1, y),
            (x-3, y-1), (x-2, y-1), (x-1, y-1), (x-3, y-2), (x-2, y-2), (x-1, y-2),
            (x-3, y-3), (x-2, y-3), (x-1, y-3)
        ]
        right = [
            (x+1, y+3), (x+2, y+3), (x+3, y+3), (x+1, y+2), (x+2, y+2), (x+3, y+2),
            (x+1, y+1), (x+2, y+1), (x+3, y+1), (x+1, y), (x+2, y), (x+3, y),
            (x+1, y-1), (x+2, y-1), (x+3, y-1), (x+1, y-2), (x+2, y-2), (x+3, y-2),
            (x+1, y-3), (x+2, y-3), (x+3, y-3)
        ]
        
    
    def main_program(self):
        #To keep program running
        while type(frontierEvaluation.frontiersGrid) == type(None) and not rospy.is_shutdown():
            rospy.sleep(1)
        
        goals = list(set([(x,y) for x,y in zip(frontierEvaluation.cache["centroid_x"], frontierEvaluation.cache["centroid_y"])]))

#        while len(set(frontierEvaluation.cache["cluster"])) > 0 and not rospy.is_shutdown():
        while len(goals) > 0 and not rospy.is_shutdown():
            rospy.sleep(1)


            #Publishing updated obstacles
#            if frontierEvaluation.locs:
            points = getPointArray(frontierEvaluation.locs, frontierEvaluation.occ_grid)
            markers = createMarkers(points=points, indx=50, action=0, ns="objects", scale=0.05)

            #Publishing frontiers occupancy grid
            #if type(frontierEvaluation.frontiersGrid) != type(None):
#            goalPoint = (frontierEvaluation.cache.iloc[0]["centroid_x"], frontierEvaluation.cache.iloc[0]["centroid_y"], 30)
            goalPoint = (goals[0][0], goals[0][1], 30)
            goalPoint = getPointArray([goalPoint], frontierEvaluation.occ_grid)[0]
 #           print(goalPoint)
            goalMarker = createMarkers(points=[goalPoint], indx=200, action=0, ns="goal_marker", color=[255, 0, 0], scale=0.3, style=7)
#            frontierEvaluation.centroids.markers.append(goalMarker)

               # self.visualizeFrontiers(frontierEvaluation.frontiersGrid, frontierEvaluation.occ_grid.header, frontierEvaluation.occ_grid.info)
              #  self.frontiers_pub.publish(frontierEvaluation.frontier_markers)
               # self.centroids_pub.publish(frontierEvaluation.centroids)
               # self.centroids_pub.publish(goalMarker)

            #Sending goal:
            print("sending goal...")
            self.coordinate_callback_thread(goalPoint.x, goalPoint.y, goalPoint.z)

            #Waiting for result
            result = None
            rate = rospy.Rate(1.0)
            
            while result is None and not rospy.is_shutdown():
                #Deleting:
                m = Marker(action=3)
                self.pub.publish(m)
                self.centroids_pub.publish(m)
                mrkrArr = MarkerArray()
                mrkrArr.markers.append(m)
                self.frontiers_pub.publish(mrkrArr)

                print("getting result")
                #Visualizing
                self.pub.publish(markers)
                self.visualizeFrontiers(frontierEvaluation.frontiersGrid, frontierEvaluation.occ_grid.header, frontierEvaluation.occ_grid.info)
                self.frontiers_pub.publish(frontierEvaluation.frontier_markers)
                self.centroids_pub.publish(frontierEvaluation.centroids)
                self.centroids_pub.publish(goalMarker)
                
                #Getting result
                result = self.client.get_result()

                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal found!")
                    goals.pop(0)

                
                if self.client.get_state() == GoalStatus.ABORTED:
                    rospy.loginfo("Can't find goal. Moving to the next frontier.")
                    goals.pop(0)
                    continue
                
                rate.sleep()
            

        
        if rospy.is_shutdown():
            print("Code stopping...")
            sys.exit("Rospy shut down.")
        
        elif len(set(frontierEvaluation.cache["cluster"])) == 0:
            print("Congrats! All centroids were explored! Returning to the origin....")
            self.coordinate_callback_thread(0, 0, 0)
            self.client.wait_for_result()
            sys.exit("Exiting the program. ")
    
    def callback(self, occupancy_grid):
        
        frontierEvaluation.occ_grid = occupancy_grid
        if self.compareGrid(frontierEvaluation.curr_grid, occupancy_grid.data):
            frontierEvaluation.curr_grid = occupancy_grid.data
            print("setting up")
            grid = formatGrid(occupancy_grid)

            locs = getObjects(grid)
            grid = grow(grid, locs)
            frontierEvaluation.locs = getObjects(grid)
 #           points = getPointArray(locs, occupancy_grid)
#            markers = createMarkers(points=points, indx=50, action=0, ns="objects", scale=0.03)
#            self.pub.publish(markers)

            frontierEvaluation.frontiersGrid = findFrontiers(grid)
           # header = occupancy_grid.header
           # info = occupancy_grid.info
           # self.visualizeFrontiers(frontiersGrid, header, info)

            frontiers = getObjects(frontierEvaluation.frontiersGrid)
            df = self.dbscan(frontiers, 5, 8)
            #print(df)

            frontierEvaluation.frontier_markers, frontierEvaluation.centroids, frontierEvaluation.cache = self.colorClusters(df, frontierEvaluation.occ_grid)
            print(type(frontierEvaluation.centroids))
 #           self.frontiers_pub.publish(frontier_markers)
#            self.centroids_pub.publish(centroids)

        #     print("sending goal")

        #     self.coordinate_callback_thread(goalPos.x, goalPos.y, goalPos.z)
        #    # self.client.send_goal(goalPos)

        #     result = None
        #     rate = rospy.Rate(10.0)

        #     while result is None and not rospy.is_shutdown():
        #         print("getting result")
        #         result = self.client.get_result()
        #         rate.sleep()
            
        #     if rospy.is_shutdown():
        #         print("Code stopping...")
        #         sys.exit("Rospy shut down.")
            
            print("done")
        else:
            print("There was no recorded change")

if __name__ == "__main__":
    f = frontierEvaluation()
