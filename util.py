#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
import tf2_ros
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd


def x_grow(x, y, grid, q, L=1, limit=3):
    if L < limit:
        L += 1
        x += q

        if 0 <= x < len(grid) and 0 <= y < len(grid):
            grid[x][y] = 100
        
        grid = y_grow(x, y, grid, 1, R=1)
        grid = y_grow(x, y, grid, -1, R=1)
        grid = x_grow(x, y, grid, q, L=L, limit=limit)
    return grid

def y_grow(x, y, grid, p, R=1, limit=3):
    if R < limit:
        R += 1
        y += p

        if 0 <= x < len(grid) and 0 <= y < len(grid):
            grid[x][y] = 100
        
        grid = y_grow(x, y, grid, p, R=R, limit=limit)
    
    return grid

def grow(grid, points):
    for p in points:
        grid = x_grow(p[1], p[0], grid, 1, L=1)
        grid = x_grow(p[1], p[0], grid, -1, L=1)
    
    return grid

def getObjects(grid):
    points = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 100:
                points.append([j, i])
    return points

def getPointArray(points, occupancy_grid):
    pointArray = []
#    print(points)

    for point in points:
        if type(point) == Point():
            x = (point.x*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.x
            y = (point.y*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.y                
        else:
            x = (point[0]*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.x
            y = (point[1]*occupancy_grid.info.resolution) + occupancy_grid.info.origin.position.y
        if len(point) == 3:
            z = point[2]
        else:
            z = 0

        p = Point(x, y, 0)
        pointArray.append(p)
    
    return pointArray

def formatGrid(occupancy_grid):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    grid = np.reshape(occupancy_grid.data, (width, height))
    return grid

def calculateDistance(currentPos, goal):
    if type(goal) == list: goal = goal[0]
    dx = currentPos.y - goal.x
    dy = currentPos.x - goal.y
    dist = (dx**2 + dy**2)**(0.5)
    return dist

def createMarkers(points, indx, action, ns, color=[220, 150, 225], scale=0.05, style=8):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "map"
    marker.ns = ns
    marker.id = indx
    marker.action = action
    marker.type = style
    marker.color.r = round(color[0]/255, 2)
    marker.color.g = round(color[1]/255, 2)
    marker.color.b = round(color[2]/255, 2)
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker.points = points
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    return marker

def getRGB(hexValue):
    rgbList = []
    for i in (0,2,4):
        decimal = int(hexValue[i:i+2], 16)
        rgbList.append(decimal)
    return rgbList

def findFrontiers(grid):
    w, h = grid.shape
    updatedGrid = np.zeros((w,h), int)
    for i in range(w-1):
        for j in range(h-1):
            if grid[i][j] == -1:
                if grid[i+1][j] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i-1][j] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i][j+1] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i][j-1] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i+1][j+1] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i+1][j-1] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i-1][j+1] == 0:
                    updatedGrid[i][j] = 100
                elif grid[i-1][j-1] == 0:
                    updatedGrid[i][j] = 100
    return updatedGrid