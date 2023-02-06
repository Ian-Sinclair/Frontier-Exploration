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
import random
import warnings
warnings.filterwarnings("ignore")

class frontierExplorer():
    #Do stuff here (:
    df = pd.DataFrame({"x": [], "y": [], "cluster": [], "centroid_x": [], "centroid_y": [], "distance": []})
    markers = {"frontiers": None, "centroids": None, "obstacles": None, "goal": None}
    grid = None
    curr_data = []
    occupancy = None
    tfBuffer = tf2_ros.Buffer()
    currentPosition = None
    centroids = []

    def __init__(self):
        self.init_node()
        self.init_action_client()
        self.init_publishers()
        self.init_subscriber()
        self.main_program()
    
    def init_node(self):
        rospy.init_node("explorer")
    
    def init_subscriber(self):
        rospy.Subscriber("map", OccupancyGrid, self.callback)
    
    def init_publishers(self):
        self.obs_pub = rospy.Publisher("/obstacles", Marker, queue_size=1, latch=True)
        self.grid_pub = rospy.Publisher("/frontiers_map", OccupancyGrid, queue_size=1, latch=True)
        self.frontiers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
        self.centroids_pub = rospy.Publisher("/centroids", Marker, queue_size=1)
        self.goal_pub = rospy.Publisher("/goal", Marker, queue_size=1)
    
    def init_listener(self):
        listener = tf2_ros.TransformListener(frontierExplorer.tfBuffer)
        try:
            trans = frontierExplorer.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time(0))
            frontierExplorer.currentPosition = trans.transform.translation
        except:
            rospy.Rate(10.0).sleep()
    
    def init_action_client(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
    
    def coordinate_callback(self, x, y, z):
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        listener = tf2_ros.TransformListener(frontierExplorer.tfBuffer)
        while not rospy.is_shutdown():
            try:
                trans = frontierExplorer.tfBuffer.lookup_transform("base_footprint", "map", rospy.Time(0))
                break
            except:
                print("tf listener didn't work")
                continue
            
        tfPose = PoseStamped()
        tfPose.pose = pose
        tfPose.header.frame_id = "base_footprint"
        tfPose.header.stamp = rospy.Time.now()
        new_pose = do_transform_pose(tfPose, trans)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_footprint"
        goal.target_pose.pose = new_pose.pose
        
        self.client.send_goal(goal)
    
    def dbscan(self, frontiers, epsilon, min_points):
        frontiers = pd.DataFrame(frontiers)
        db = DBSCAN(eps=epsilon, min_samples=min_points, metric="l2")
        clusters = db.fit_predict(frontiers)
        
        frontiers["cluster"] = clusters
        frontiers = frontiers[frontiers["cluster"] != -1]
        frontiers = frontiers.sort_values("cluster")
#        print(frontiers)
        return frontiers
    
    def visualizeFrontiers(self, frontiersGrid, header, info):
        occGrid = OccupancyGrid(header, info, frontiersGrid.flatten(order="C"))
        self.grid_pub.publish(occGrid)
    
    def calculateCentroid(self, points):
        xc = int(sum(points[0])/len(points[0]))
        yc = int(sum(points[1])/len(points[1]))
        return [xc, yc, 0]
    
    def colorClusters(self, df, occupancy_grid):
        clusters = list(set(df["cluster"]))
        markers = MarkerArray()
        centers = []
        palette = list(reversed(sns.color_palette("rainbow", len(clusters)).as_hex()))
        colors = []
        distances = []
        centroid_x = []
        centroid_y = []
        self.init_listener()
        for p in palette:
            p = p.replace("#", "")
            colors.append(getRGB(p))
        
        for c in range(len(clusters)):
            sub_df = df[df["cluster"] == clusters[c]]

            points = getPointArray([(x,y) for x,y in zip(sub_df[0], sub_df[1])], occupancy_grid)
            marker = createMarkers(points=points, indx=c, action=0, ns="frontiers", color=colors[c], scale=0.1, style=7)
            markers.markers.append(marker)

            xc = int(sum(sub_df[0])/len(sub_df[0]))
            yc = int(sum(sub_df[1])/len(sub_df[1]))
            centroid_x += [xc]*len(sub_df)
            centroid_y += [yc]*len(sub_df)

            center = [xc, yc, 20]
            centers.append(center)
            centerPoint = getPointArray([center], occupancy_grid)

            distance = [calculateDistance(frontierExplorer.currentPosition, centerPoint)]*len(sub_df)
            distances += distance

        centers = getPointArray(centers, occupancy_grid)
        centerMarker = createMarkers(points=centers, indx=c, action=0, ns="centroids", color=[255,255,255], scale=0.3, style=7)
        
        df["distance"] = distances
        df["centroid_x"] = centroid_x
        df["centroid_y"] = centroid_y
        df.sort_values("distance")
        df.reset_index()

        return markers, centerMarker, df
    
    def compareGrid(self, currentGrid, newGrid):
 #       print(currentGrid)
        print(sum(currentGrid))
        if sum(currentGrid) == sum(newGrid):
#            frontierExplorer.curr_data = newGrid
            return False
        return True
    
    def habitability(self, x, y, grid):
        for i in range(4):
            if grid[x+i][y] == 100 or grid[x-i][y] == 100 or grid[x][y+i] == 100 or grid[x][y-i] == 100:
                return False
        return True
    
    def findPoint(self, x, y, grid):
        for i in range(15):
            dx = random.randint(1, 5)
            dy = random.randint(1, 5)
            if self.habitability(x+dx, y+dy, grid) and 0<(x+dx)<384 and 0<(y+dy)<384:
                print("-------------------------------------------------------")
                print(dx)
                print(dy)
                return (x+dx, y+dy)
            elif self.habitability(x-dx, y-dy, grid) and 0<(x-dx)<384 and 0<(y-dy)<384:
                return (x-dx, y-dy)
        return None, None
    
    def main_program(self):
        while type(frontierExplorer.grid) == type(None) and not rospy.is_shutdown():
            rospy.sleep(0)
        
        while len(set(frontierExplorer.df["cluster"])) > 0  and not rospy.is_shutdown():
            rospy.sleep(0.2)

            goals = list(set([(x,y,c) for x,y,c in zip(frontierExplorer.df["centroid_x"], frontierExplorer.df["centroid_y"], frontierExplorer.df["cluster"])]))

            goalPoint = (goals[0][0], goals[0][1], 30)
            cluster = goals[0][2]
#            print(calculateDistance(frontierExplorer.currentPosition.position, goalPoint))

            print("sending goal")
            if self.habitability(goalPoint[0], goalPoint[1], frontierExplorer.grid):
                goalPoint = (goals[0][0], goals[0][1], 30)
            else:
                print("finding habitable goal")
                x,y = self.findPoint(goalPoint[0], goalPoint[1], frontierExplorer.grid)
                if x == None and y == None:
                    goalPoint = (x, y, 30)
                else:
                    print("Goal is unreachable. ")
                    self.client.cancel_all_goals()
#                    frontierExplorer.df = frontierExplorer.df[frontierExplorer.df["cluster"] != cluster]
#                    goals.pop(0)
                    continue
            goalPoint = getPointArray([goalPoint], frontierExplorer.occupancy)[0]
            self.coordinate_callback(goalPoint.x, goalPoint.y, goalPoint.z)
            frontierExplorer.markers["goal"] = createMarkers(points=[goalPoint], indx=0, action=0, ns="goal", color=[220, 90, 255], scale=0.3, style=7)
            print(calculateDistance(frontierExplorer.currentPosition, goalPoint))

            result = None
            rate = rospy.Rate(10.0)
            
            while result is None and not rospy.is_shutdown():
                #Deleting old markers
                dm = Marker(action=3)
                self.obs_pub.publish(dm)
                self.centroids_pub.publish(dm)
                self.goal_pub.publish(dm)
                dArr = MarkerArray()
                dArr.markers.append(dm)
                self.frontiers_pub.publish(dArr)

                #Publishing markers
                self.obs_pub.publish(frontierExplorer.markers["obstacles"])
                self.visualizeFrontiers(frontierExplorer.fgrid, frontierExplorer.occupancy.header, frontierExplorer.occupancy.info)
                self.frontiers_pub.publish(frontierExplorer.markers["frontiers"])
                self.centroids_pub.publish(frontierExplorer.markers["centroids"])
                self.goal_pub.publish(frontierExplorer.markers["goal"])

                #Getting result
                print("getting result...")
                result = self.client.get_result()

                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal found!")
                    frontierExplorer.df = frontierExplorer.df[frontierExplorer.df["cluster"] != cluster]
                    goals.pop(0)
                    self.client.cancel_all_goals()
                    break
                
                if self.client.get_state() == GoalStatus.ABORTED:
                    rospy.loginfo("Couldn't reach goal, moving to the next frontier.")
                    frontierExplorer.df = frontierExplorer.df[frontierExplorer.df["cluster"] != cluster]
                    goals.pop(0)
                    self.client.cancel_all_goals()
                    break
                
                #Getting position
                self.init_listener()
                print(frontierExplorer.currentPosition)
                d = calculateDistance(frontierExplorer.currentPosition, goalPoint)
                if d < 20:
                    print("###\n####\n###\n###\n###\n####\n####\n####\n")
                    print(d)
                    frontierExplorer.df = frontierExplorer.df[frontierExplorer.df["cluster"] != cluster]
   #                 goals.pop(0)
                    self.client.cancel_all_goals()
                    break

                rate.sleep()
        
        if rospy.is_shutdown():
            print("Code stopping...")
            sys.exit("rospy shut down.")
        
        elif len(set(frontierExplorer.df["cluster"])) == 0:
            print("Congrats! All centroids were explored! Returning to the origin...")
            self.coordinate_callback(0.3, 0.3, 0)
            self.client.wait_for_result()
            sys.exit("Exiting the program.")
    
    def callback(self, occupancy_grid):
        self.init_listener()
        print(self.currentPosition)
        frontierExplorer.occupancy = occupancy_grid
        if self.compareGrid(frontierExplorer.curr_data, occupancy_grid.data):
            frontierExplorer.curr_data = occupancy_grid.data
            grid = formatGrid(occupancy_grid)
            locs = getObjects(grid)
            frontierExplorer.grid = grow(grid, locs)
            locs = getObjects(grid)
            points = getPointArray(locs, occupancy_grid)
            frontierExplorer.markers["obstacles"] = createMarkers(points=points, indx=0, action=0, ns="objects", scale=0.01)

            frontierExplorer.fgrid = findFrontiers(frontierExplorer.grid)
            frontiers = getObjects(frontierExplorer.fgrid)
            df = self.dbscan(frontiers, 5, 8)

            frontierExplorer.markers["frontiers"], frontierExplorer.markers["centroids"], frontierExplorer.df = self.colorClusters(df, frontierExplorer.occupancy)
          # print(frontierExplorer.df)
           # frontier_markers, centroids, df = self.colorClusters()
           # print(df)
        
        else:
            print("No change recorded")

if __name__ == "__main__":
    f = frontierExplorer()