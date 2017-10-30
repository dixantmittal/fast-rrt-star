
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

radius = 5.0
tolerance = 0.0001
def getCircleCenterPoint(point):
    if(abs(point.thetha_coord - math.radians(90)) < tolerance or abs(point.thetha_coord - math.radians(270)) < tolerance):
        center1 = Point(point.x_coord - radius, point.y_coord, 0)
        center2 = Point(point.x_coord + radius, point.y_coord, 0)
        return [center1, center2]
    if(abs(point.thetha_coord - math.radians(0)) < tolerance or abs(point.thetha_coord - math.radians(180)) < tolerance):
        center1 = Point(point.x_coord, point.y_coord - radius, 0)
        center2 = Point(point.x_coord, point.y_coord + radius, 0)
        return [center1, center2]
    gradient_circle = math.sin(point.thetha_coord)/math.cos(point.thetha_coord)
    gradient_point = -1.0/gradient_circle
    vector = math.sqrt(gradient_point*gradient_point + 1.0)
    increase_y = (radius/vector)*(gradient_point)
    increase_x = radius/vector
    center1 = Point(point.x_coord + increase_x, point.y_coord + increase_y, 0)
    center2 = Point(point.x_coord - increase_x, point.y_coord - increase_y, 0)
    return [center1, center2]
def get_direction_counter_clockwise(circ1, strt1):
    tangent = [math.cos(strt1.thetha_coord),math.sin(strt1.thetha_coord)]
    perpendicular = [strt1.x_coord - circ1.x_coord, strt1.y_coord - circ1.y_coord]
    counter_clockwise_1 = (perpendicular[0] * tangent[1] - perpendicular[1]*tangent[0]) > 0
    return counter_clockwise_1

def ccw_test(vector_1, vector_2):
    return ((vector_1[0]*vector_2[1] - vector_1[1]*vector_2[0]) > 0)
def getDistanceCircle(circ1, circ2, strt1, strt2):
    direction_1_ccw = get_direction_counter_clockwise(circ1, strt1)
    direction_2_ccw = get_direction_counter_clockwise(circ2, strt2)
    print("Direction First CCW : ")
    print(direction_1_ccw)
    print("Direction Second CCW : ")
    print(direction_2_ccw)
    if(direction_1_ccw != direction_2_ccw):
        return -1
    differ_x = circ2.x_coord - circ1.x_coord
    differ_y = circ2.y_coord - circ1.y_coord
    distance = math.sqrt(differ_x*differ_x + differ_y*differ_y)
    print("Distance :"+str(distance))
    vector_tangent = [differ_x,differ_y]
    
    vector_to_tangent = [differ_y, -differ_x]

    direction_to_point_ccw = ccw_test(vector_to_tangent, vector_tangent)
    if(direction_to_point_ccw != direction_1_ccw):
        vector_to_tangent = [-1.0*vector_to_tangent[0], -1.0*vector_to_tangent[1] ]
        direction_to_point_ccw = direction_1_ccw
    thetha_tangent = 0.0
    if(vector_to_tangent[1] >= 0):
        thetha_tangent = math.acos(vector_to_tangent[0]/math.sqrt(vector_to_tangent[0]*vector_to_tangent[0] + vector_to_tangent[1]*vector_to_tangent[1]))
    else :
        thetha_tangent = 2*math.pi - math.acos(vector_to_tangent[0]/math.sqrt(vector_to_tangent[0]*vector_to_tangent[0] + vector_to_tangent[1]*vector_to_tangent[1]))
    thetha = 0.0
    print("Thetha to Tangent :"+str(thetha_tangent))
    if(direction_to_point_ccw):
        thetha = (thetha_tangent + math.pi/2)
        if(thetha > 2*math.pi):
            thetha -= 2*math.pi
    else:
        thetha = thetha_tangent - math.pi/2
        if(thetha < 0):
            thetha += 2*math.pi
    print("Thetha tangent used :"+str(thetha))
    if(direction_to_point_ccw):
        distance_thetha_first = thetha - strt1.thetha_coord
        if(distance_thetha_first < 0.000):
            distance_thetha_first += 2*math.pi
        distance_thetha_second = strt2.thetha_coord - thetha
        if(distance_thetha_second < 0):
            distance_thetha_second += 2*math.pi
        return ((distance_thetha_first + distance_thetha_second)*radius + distance)
    else :
        distance_thetha_first = -1*(thetha - strt1.thetha_coord)
        if(distance_thetha_first < 0):
            distance_thetha_first += 2*math.pi
        distance_thetha_second = -1*(strt2.thetha_coord - thetha)
        if(distance_thetha_second < 0):
            distance_thetha_second += 2*math.pi
        
        return ((distance_thetha_first + distance_thetha_second)*radius + distance)
        
        
    
    

def checkIntersectCircle(circle, obstacle):
    x_start_circle = circle.x_coord - radius
    x_end_circle = circle.x_coord + radius
    y_start_circle = circle.y_coord - radius
    y_end_circle = circle.y_coord + radius
    if(contained(Point(x_start_circle,y_start_circle),o) and
       contained(Point(x_start_circle,y_end_circle),o) and
       contained(Point(x_end_circle,y_start_circle),o) and
       contained(Point(x_end_circle,y_end_circle),o)):
        return True
    y_possible = [obstacle.y_start, obstacle.y_end]
    x_possible = [obstacle.x_start, obstacle.x_end]
    for y in y_possible :
        x_intersect_square = radius*radius - (circle.y_coord-y)*(circle.y_coord-y)
        if(x_intersect_square >= 0):
            x_intersect_1 = math.sqrt(x_intersect_square) + circle.x_coord
            if(obstacle.x_start <= x_intersect_1 and x_intersect_1 <= obstacle.x_end):
                return True
            x_intersect_2 =  circle.x_coord - math.sqrt(x_intersect_square)
            if(obstacle.x_start <= x_intersect_2 and x_intersect_2 <= obstacle.x_end):
                return True
    for x in x_possible :
        y_intersect_square = radius*radius - (circle.x_coord-x)*(circle.x_coord-x)
        if(y_intersect_square >= 0):
            y_intersect_1 = math.sqrt(y_intersect_square) + circle.y_coord
            if(obstacle.y_start <= y_intersect_1 and y_intersect_1 <= obstacle.y_end):
                return True
            y_intersect_2 =  circle.y_coord - math.sqrt(y_intersect_square)
            if(obstacle.y_start <= y_intersect_2 and y_intersect_2 <= obstacle.y_end):
                return True
    return False
    
def print_point(point):
    print("(X : "+str(point.x_coord) + ", Y : "+str(point.y_coord)+")")
def print_obstacles(obstacle):
    print("Start X : "+str(obstacle.x_start) + "End X : " + str(obstacle.x_end))
    print("Start Y : "+str(obstacle.y_start) + "End Y : " + str(obstacle.y_end))    
def checkConnectableRTR(circ_s, circ_e):
    differ_x = circ_e.x_coord - circ_s.x_coord
    differ_y = circ_e.y_coord - circ_s.y_coord
    distance = math.sqrt(differ_x*differ_x + differ_y*differ_y)
    
    scale_ratio = radius/distance
    vector = [scale_ratio*differ_y, scale_ratio*differ_x*(-1)]    
    for o in Obstacles:
        if(checkIntersectCircle(circ_s, o)):
            return False
        if(checkIntersectCircle(circ_e, o)):
            return False
        
        if(checkIntersect(Point(circ_s.x_coord + vector[0], circ_s.y_coord + vector[1],0), Point(circ_e.x_coord + vector[0], circ_e.y_coord + vector[1],0), o)):
            return False
        if(checkIntersect(Point(circ_s.x_coord - vector[0], circ_s.y_coord - vector[1],0), Point(circ_e.x_coord - vector[0], circ_e.y_coord - vector[1],0), o)):
            return False
    return True
def checkRTRpath(start,end):
    circle_start = getCircleCenterPoint(start)
    circle_end = getCircleCenterPoint(end)
    center_circle_best = [None, None]
    min_distance = 1000000
    for i in range(0,2):
        
        for j in range(0,2):
            
            distance = getDistanceCircle(circle_start[i],circle_end[j], start,end)
            if((distance > 0) and (distance < min_distance)):
                print_point(circle_start[i])
                print_point(circle_end[j])
                print("Distance : "+str(distance))
                center_circle_best[0] = circle_start[i]
                center_circle_best[1] = circle_end[j]
                min_distance = distance
            
    if(checkConnectableRTR(center_circle_best[0], center_circle_best[1] )):
       return min_distance
    else :
       return -1
    
point = Point(-3,-4,0)
point2 = Point(3,4,0)
centers = getCircleCenterPoint(point)
centers2 = getCircleCenterPoint(point2)

for c in centers:
    print_point(c)
for c in centers2:
    print_point(c)

#print(checkIntersectCircle(Point(0,0,0), Obstacle(-4,-2,3,2)))
#print(get_direction_counter_clockwise(centers[0], point))
#print(get_direction_counter_clockwise(centers[1], point))

print(getDistanceCircle(centers[0], centers2[0], point, point2))
print(checkRTRpath(point,point2))






                        
                
            
            
