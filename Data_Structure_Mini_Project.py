
import math
import sys

Obstacles = []


class Point:
    def __init__(self, x_coord, y_coord, thetha_coord):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.thetha_coord = thetha_coord
    
        
# For simplicity, the obstacle has the thetha occupied in [0,2*pi].
# Thus, if there's a rectangular obstacle, for any car orientation inside it, assume it's occupied by obstacle 

class Obstacle:
# The rectangle obstacle spans x coord horizontally from start_x to end_x.
# It also spans y coord vertically from start_y to end_y
    def __init__(self, startx, starty, endx, endy):
        if(endx < startx):
            temp = startx
            starttime = endx
            endx = temp
        if(endy < starty):
            temp = starty
            starty = endy
            endy = temp
        self.x_start = startx 
        self.y_start = starty
        self.x_end = endx
        self.y_end = endy


def contained(point, obstacle):
    if(point.x_coord >= obstacle.x_end or point.x_coord <= obstacle.x_start):
        return False
    if(point.y_coord >= obstacle.y_end or point.y_coord <= obstacle.y_start):
        return False
    return True


def intersect(point_start, point_end, obstacle):
    if(contained(point_start,obstacle) or contained(point_end,obstacle)):
        #print("cotained")
        return True
    
    x_difference = point_end.x_coord - point_start.x_coord
    y_difference = point_end.y_coord - point_start.y_coord
    y_min = min(point_start.y_coord, point_end.y_coord)
    y_max = max(point_start.y_coord, point_end.y_coord)
    x_min = min(point_start.x_coord, point_end.x_coord)
    x_max = max(point_start.x_coord, point_end.x_coord)

    if(y_max <= obstacle.y_start or y_min >= obstacle.y_end or x_max <= obstacle.x_start or x_min >= obstacle.x_end):
        return False
        
    #y of intersection between the projection of the line to the left side of obstacle
    if(x_difference != 0):    
        ystart_intersect = point_start.y_coord + ((obstacle.x_start - point_start.x_coord)/x_difference)*y_difference
        if(obstacle.y_start < ystart_intersect and ystart_intersect < obstacle.y_end):
            if(y_min <= ystart_intersect and ystart_intersect <= y_max):
                #print("y_start_intersect")
                return True
    #y of intersection between the projection of the line to the right side of obstacle
    if(x_difference != 0):
        yend_intersect = point_start.y_coord + ((obstacle.x_end - point_start.x_coord)/x_difference)*y_difference
        if(obstacle.y_start < yend_intersect and yend_intersect < obstacle.y_end):
            if(y_min <= yend_intersect and yend_intersect <= y_max):
                #print("y_end_intersect")
                return True
    #x of intersection between the projection of the line to the bottom side of obstacle    
    if(y_difference != 0):
        xstart_intersect = point_start.x_coord + ((obstacle.y_start - point_start.y_coord)/y_difference)*x_difference
        if(obstacle.x_start < xstart_intersect and xstart_intersect < obstacle.x_end):
            if(x_min <= xstart_intersect and xstart_intersect <= x_max):
                #print("t_start_intersect")
                return True
        #x of intersection between the projection of the line to the top side of obstacle 
        xend_intersect = point_start.x_coord + ((obstacle.y_end - point_start.y_coord)/y_difference)*x_difference
        if(obstacle.t_start < xend_intersect and xend_intersect < obstacle.x_end):
            if(x_min <= xend_intersect and xend_intersect <= x_max):
                #print("t_end_intersect")
                return True
    return False

def intersect_bounded_dist_time(obstacle1, obstacle2):
    if(intersect( Point(obstacle1.x_start, obstacle1.y_start,0), Point(obstacle1.x_start, obstacle1.y_end,0), obstacle2)):
        #print("left_side_intersect")
        return True
    if(intersect(Point(obstacle1.x_start, obstacle1.y_start,0), Point(obstacle1.x_end, obstacle1.y_start,0), obstacle2)):
        #print("bottom_side_intersect")
        return True
    if(intersect(Point(obstacle1.x_end, obstacle1.y_start,0), Point(obstacle1.x_end, obstacle1.y_end,0), obstacle2)):
        #print("right_side_intersect")
        return True
    if(intersect(Point(obstacle1.x_start, obstacle1.y_end,0), Point(obstacle1.x_end, obstacle1.y_end,0), obstacle2)):
        #print("top_side_intersect")
        return True
    return False

def intersect_bounded(obstacle1, obstacle2):
    return intersect_bounded_dist_time(obstacle1, obstacle2) or intersect_bounded_dist_time(obstacle2, obstacle1)


def checkIntersectObstacles(point1, point2):
    for j in range(0,len(Obstacles)):
        if(intersect(point1, point2,Obstacles[j])):
            #print_point(point1)
            #print_point(point2)
            #print("Intersect with obstacle : ")
            #print_obstacles(Obstacles[j])
            return True
    return False









                        
                
            
            
