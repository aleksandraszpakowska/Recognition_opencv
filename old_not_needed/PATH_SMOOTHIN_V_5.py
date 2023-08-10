import matplotlib as plt
import matplotlib.pyplot as pypl
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math

def read_coordinates(file_name):
    with open(file_name) as file:
        coordinates = []
        for line in file:
            row = line.split()
            x = row[0]
            y = row[1]
            coordinates.append([float(x), float(y)])
    return coordinates

def d_eucl(x_temp, y_temp, x2_temp, y2_temp):
    distance_temp = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp = distance_temp + ((y_temp - y2_temp) * (y_temp - y2_temp))
    distance_temp = math.sqrt(distance_temp)

    return distance_temp

def d_eucl2(x_temp, y_temp, x2_temp, y2_temp,goal_x, goal_y):
    distance_temp = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp = distance_temp + ((y_temp - y2_temp) * (y_temp - y2_temp))
    distance_temp = math.sqrt(distance_temp)

    distance_temp2 = (x2_temp - goal_x) * (x2_temp - goal_x)
    distance_temp2 = distance_temp2 + ((y2_temp - goal_y) * (y2_temp - goal_y))
    distance_temp2 = math.sqrt(distance_temp2)
    distance_temp3 = distance_temp + 0.5*(distance_temp2)
    return (distance_temp3)

# Check collisions with obstacles, if omit==1 than collision detected,
def check_obstacle(obstacles_coordinates, x_temp, y_temp):
    omit = 0
    for i in range(len(obstacles_coordinates)):
        # print(d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]))
        if (d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]) < 50):
            omit = 1
    return (omit)

def find_minimum_of_the_sum(list_of_coords):
    minimum = 1000
    point = []
    for item in range(len(list_of_coords)):
        sum_of_xy = list_of_coords[item][0] + list_of_coords[item][1]
        if sum_of_xy < minimum:
            minimum = sum_of_xy
            point.clear()
            point.append([list_of_coords[item][0], list_of_coords[item][1]])
            if len(list_of_coords) == 5:
                break
    return point




def search_path(closest_points,list_of_fields,iteration,goal_x,goal_y):
    path = []
    for j in range(iteration):

        if (x_draw_goal - 2) <= closest_points[0][0] <= (x_draw_goal + 2) and (y_draw_goal - 2) <= closest_points[0][1] <= (y_draw_goal + 2):
            print("You get a goal!", closest_points)
            break
        else:
            temp_min_points = []
            minimum=0
            for i in range(len(list_of_fields)):
                # euclidean_points_dist = d_eucl(closest_points[0][0], closest_points[0][1], list_of_fields[i][0], list_of_fields[i][1])
                euclidean_points_dist = d_eucl2(closest_points[0][0], closest_points[0][1], list_of_fields[i][0], list_of_fields[i][1],goal_x, goal_y)
                # print('i = ',i)
                # print('euclidean_points_dist = ',euclidean_points_dist)
                if(i==0):
                    minimum=euclidean_points_dist
                    print('minimum = ',minimum)
                    temp_min_points.clear()
                    temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                    print('temp_min_points = ',temp_min_points)
                else:
                    if euclidean_points_dist < minimum and euclidean_points_dist != 0:
                        minimum = euclidean_points_dist
                        print('minimum = ', minimum)
                        temp_min_points.clear()
                        temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                        print(temp_min_points)

                if i == (len(list_of_fields) - 1):
                    counter = 0
                    for n in range(len(list_of_fields)):
                        if (n - counter) == (len(list_of_fields) - counter - 1):
                            break
                        if temp_min_points[0] == list_of_fields[n-1][0:2]:
                            list_of_fields.pop(n-1)
                            counter = counter + 1

                    path.append((temp_min_points[0][0], temp_min_points[0][1]))
                    # closest_points.pop(0)
                    closest_points.clear()
                    closest_points = temp_min_points.copy()
                    break
    print("Path:", path)
    return (path)

def search_min_distances(path, goal_coord):
    path_distances = []
    optimal_path =[]
    for i in range(len(path)):
        euclid_dist = d_eucl(path[i][0], path[i][1], goal_coord[0][0], goal_coord[0][1])
        if i == 0:
            path_distances.append(euclid_dist)
            optimal_path.append(path[i])
        else:
            if euclid_dist < path_distances[-1]:
                optimal_path.append(path[i])
                path_distances.append((euclid_dist))
            if euclid_dist == path_distances[-1] and i+ 1 < len(path)-2:
                euclid_superset = d_eucl(path[i+1][0], path[i+1][1], goal_coord[0][0], goal_coord[0][1])
                if euclid_superset < euclid_dist:
                    optimal_path.pop(-1)
                    optimal_path.append(path[i])
                    path_distances.append(euclid_superset)
                if euclid_superset == euclid_dist and i + 2 < len(path)-2:
                    euclid_superset2 = d_eucl(path[i + 2][0], path[i + 2][1], goal_coord[0][0], goal_coord[0][1])

                    if euclid_superset2 < euclid_dist:
                        optimal_path.pop(-1)
                        optimal_path.append(path[i])
                        path_distances.append(euclid_superset2)

                else:
                    continue


    return(optimal_path)


def path_smoothing(path, iter_number):
    #split coordinates into two lists
    x_list_x = [x for x, y in path]
    y_list_y = [y for x, y in path]

    x_list = x_list_x.copy()
    y_list = y_list_y.copy()
    x0 = x_list_x.copy()
    y0 = y_list_y.copy()
    alpha = .3
    beta = .3
    new_path = []
    for i0 in range(0, iter_number):
        for i in range(1, len(path) - 1):
            # part1
            x_list[i] = x_list[i] + alpha * (x_list[i - 1] + x_list[i + 1] - 2 * x_list[i])
            y_list[i] = y_list[i] + alpha * (y_list[i - 1] + y_list[i + 1] - 2 * y_list[i])
            # part2
            x_list[i] = x_list[i] + beta * (x0[i] - x_list[i])
            y_list[i] = y_list[i] + beta * (y0[i] - y_list[i])

        if i0 == iter_number-1:
            new_path = list(zip(x_list,y_list))

    return (new_path)


def draw_path(start_coord, path, color):
    for point in range(len(path)):
        if point == 0:
            pygame.draw.line(surf, color, start_coord[0], path[point + 1], 2)
        else:
            pygame.draw.line(surf, color , path[point], path[point + 1], 2)
        if point + 1 == len(path) - 1:
            break

def getImage(path, zoom=0.07):
    return OffsetImage(pypl.imread(path), zoom=zoom)

def to_pygame(coords, height):
    #Convert coordinates into pygame coordinates (lower-left => top left).
    return (coords[0][0], height - coords[0][1])

plt.rcParams.update({
    'figure.subplot.left': 0,
    'figure.subplot.bottom': 0,
    'figure.subplot.right': 1,
    'figure.subplot.top': 1,
    "lines.marker": "o",  # available ('o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X')
    "lines.linewidth": "0.4",
    "axes.prop_cycle": plt.cycler('color', ['white']),  # line color
    "text.color": "black",  # no text in this example
    "axes.facecolor": "white",  # background of the figure
    "axes.edgecolor": "gray",
    "axes.labelcolor": "black",  # no labels in this example
    "axes.grid": "True",
    "grid.linestyle": ":",
    "grid.color": "lightgray",
    "figure.edgecolor": "white",
})

paths = [
    '../marker_images/marker_1184.png',
    '../marker_images/marker_1751.png',
    '../marker_images/marker_4076.png',
    '../marker_images/marker_1281.png'
]
paths2 = [
    '../marker_images/marker_2165.png',
    '../marker_images/marker_733.png']

paths3 = [
    '../marker_images/marker_497.png']

paths4 = [
    '../start_images/start_point.jpg']


# # determinig coordinates
# goal_coordianets = [(465, 150)]
# center_list = [(12, 14), (12, 386), (553, 386), (553, 14)]
# obstacles_coordinates = [(231, 240), (120, 143)]
# start_coordinates = [(55, 330)]

goal_coordianets = [(189, 409)]
center_list = [(53.0, 448.0), (463.0, 443.0), (461.0, 42.0), (48.0, 32.0)]
obstacles_coordinates =  [(370, 325),(261, 206 )]
start_coordinates =[(98,82)]

#
# goal_coordianets = read_coordinates('goal_detector.txt')
# center_list = read_coordinates('boundaries_detector.txt')
# obstacles_coordinates = read_coordinates('obstacles_detector.txt')
# start_coordinates = read_coordinates('start_detector.txt')
print('borders' , center_list)
# dividing lists into 2 for getting separated x and y coordinates

mins = find_minimum_of_the_sum(center_list)

x1, y1 = zip(*center_list)
x2, y2 = zip(*obstacles_coordinates)
x3, y3 = zip(*goal_coordianets)
x4, y4 = zip(*start_coordinates)
x_borders, x_obstacles, x_goal, y_borders, y_obstacles, y_goal, x_start, y_start = ([] for _ in range(8))

# finding minimum values in x,y coordinates from border
x_min = mins[0][0]
y_min = mins[0][1]

norm_coord_min = [(x_min, y_min)]

with open('../txt_files/normalizing_coord.txt', 'w') as file:
    for x, y in norm_coord_min:
        file.write('{} {}\n'.format(x, y))



# normaliziing the elements, getting new (0,0) point on map
for i in range(len(x1)):
    a = x1[i] - x_min
    x_borders.append(a)
for i in range(len(x2)):
    b = x2[i] - x_min
    x_obstacles.append(b)
for i in range(len(x3)):
    c = x3[i] - x_min
    x_goal.append(c)
for i in range(len(x4)):
    d = x4[i] - x_min
    x_start.append(d)

for i in range(len(y1)):
    a = y1[i] - y_min
    y_borders.append(a)
for i in range(len(y2)):
    b = y2[i] - y_min
    y_obstacles.append(b)
for i in range(len(y3)):
    c = y3[i] - y_min
    y_goal.append(c)
for i in range(len(y4)):
    d = y4[i] - y_min
    y_start.append(d)
import numpy as np
# insert images insted of normal points
sizex = (max(x_borders) +1) / 100
sizey = (max(y_borders) +1) /100
fig, ax = pypl.subplots()

fig.set_figheight(sizey)
fig.set_figwidth(sizex)

pypl.xticks(np.arange(0,max(x_borders),1))
pypl.yticks(np.arange(0,max(y_borders),1))

ax.scatter(x_borders, y_borders)
ax.scatter(x_obstacles, y_obstacles)
ax.scatter(x_goal, y_goal)
ax.scatter(x_start, y_start)

#get ride of the x and y axis values
pypl.tick_params(left = False, right = False , labelleft = False ,
                labelbottom = False, bottom = False)

print("borders: ", x_borders, "y: ", y_borders)
print("start: ", x_start,y_start)
print("goal: ", x_goal,y_goal )
print("obstacles: ", x_obstacles, "y : ", y_obstacles)

# display images as points in plot
for x0, y0, path in zip(x_borders, y_borders, paths):
    ab = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(ab)
for x0, y0, path in zip(x_obstacles, y_obstacles, paths2):
    bc = AnnotationBbox(getImage(path), (x0+15, y0-15), frameon=False)
    ax.add_artist(bc)
for x0, y0, path in zip(x_goal, y_goal, paths3):
    cd = AnnotationBbox(getImage(path), (x0+15, y0-15), frameon=False)
    ax.add_artist(cd)
for x0, y0, path in zip(x_start, y_start, paths4):
    de = AnnotationBbox(getImage(path), (x0+15, y0-15), frameon=False)
    ax.add_artist(de)

ax = fig.gca()
pypl.savefig('plots/my_plot.png')

import pygame
from pygame.locals import *
pygame.init()

window = pygame.display.set_mode((sizex*100, sizey*100), DOUBLEBUF)
screen = pygame.display.get_surface()

surf = pygame.image.load('../plots/my_plot.png')
#surf = pygame.transform.scale(surf_load, (650, 450))
size = surf.get_size()
print(size, "SIZEE")
################## SQUARE FILL AL ######################
goal_coordinates_pygame, start_coordinates_pygame, obstacles_coordinates_pygame1, obstacles_coordinates_pygame2 = ([] for _ in range(4))
for i in x_goal:
    for j in y_goal:
        goal_coordinates_pygame.append((i,j))

for i in range(len(x_obstacles)):
    for j in range(len(y_obstacles)):
        if i == 0 and j == 0:
            obstacles_coordinates_pygame1.append((x_obstacles[i],y_obstacles[j]))
        if i == 1 and j == 1:
            obstacles_coordinates_pygame2.append((x_obstacles[i], y_obstacles[j]))

for i in x_start:
    for j in y_start:
        start_coordinates_pygame.append((i,j))

goals_c = to_pygame(goal_coordinates_pygame,size[1])
x_draw_goal = goals_c[0]
y_draw_goal = goals_c[1]

# initial values
color = (50, 50, 50)
clockwise = True
anticlockwise = False
dist = 0  # current distance
f = []  # list of potential fields
q = []  # empty queue

# preparing obstacle coordinates for pygame window
# print("x_obst: ",x_obstacles)
# print("y_obst: ", y_obstacles)
# print(obstacles_coordinates_pygame1, " xxxx", obstacles_coordinates_pygame2)

obst_c1 = to_pygame(obstacles_coordinates_pygame1,size[1])
obst_c2 = to_pygame(obstacles_coordinates_pygame2,size[1])

obst_new_X1 = obst_c1[0]
obst_new_X2 = obst_c2[0]
obst_new_Y1 = obst_c1[1]
obst_new_Y2 = obst_c2[1]

obstacles_coordinates_new = []

obstacles_coordinates_new.append((obst_new_X1, obst_new_Y1))
obstacles_coordinates_new.append((obst_new_X2, obst_new_Y2))

previous_neighbours_list = []
q.append((x_draw_goal, y_draw_goal, dist))  # adding initial values to the queue (goal coordinates and 0 as a current distance)


while len(q) > 0:
#for i in range(600):

    x_restriction = max(x_borders)-10
    x_restriction_left = 10
    y_restriction = max(y_borders)-10
    y_restriction_bottom = 10

    previous_neighbours_list.clear()

    # looking for the same objects in lists
    intersection1 = [item1 for item1 in f
                     for item2 in q if item1 == item2]

    intersection2 = [item1 for item1 in obstacles_coordinates_new
                     for item2 in q if item1 == item2]

    if len(intersection1) == 1 or len(intersection2) == 1:
        q.pop(0)
        dist = dist + 0.5
        intersection1.clear()
        intersection2.clear()
        continue

    # if there is no intersection between lists
    else:
        if clockwise == True:
            #print(clockwise)
            # define neighbours depending of a current direction for clockwise and anticlockwise
            p0_x = q[0][0] - dist
            p0_y = q[0][1]
            p1_x = q[0][0] - dist
            p1_y = q[0][1] + dist
            p2_x = q[0][0]
            p2_y = q[0][1] + dist
            p3_x = q[0][0] + dist
            p3_y = q[0][1] + dist
            p4_x = q[0][0] + dist
            p4_y = q[0][1]
            p5_x = q[0][0] + dist
            p5_y = q[0][1] - dist
            p6_x = q[0][0]
            p6_y = q[0][1] - dist
            p8_x = q[0][0] - dist
            p8_y = q[0][1] - dist

            if len(q) == 1:
                q.append((p0_x, p0_y, dist))
                q.append((p1_x, p1_y, dist))
                q.append((p2_x, p2_y, dist))
                q.append((p3_x, p3_y, dist))
                q.append((p4_x, p4_y, dist))
                q.append((p5_x, p5_y, dist))
                q.append((p6_x, p6_y, dist))
                q.append((p8_x, p8_y, dist))
                dist = 5
            else:
                for x in q[::-1]:
                    previous_neighbours_list.append(x)
                    if len(previous_neighbours_list) >= 8:
                        break

                euclidean_0, euclidean_1, euclidean_2, euclidean_3, euclidean_4, euclidean_5, euclidean_6, euclidean_8 = ([] for _ in range(8))

                for i in previous_neighbours_list:

                    euclidean_dist1 = d_eucl(p0_x, p0_y, i[0], i[1])
                    euclidean_dist2 = d_eucl(p1_x, p1_y, i[0], i[1])
                    euclidean_dist3 = d_eucl(p2_x, p2_y, i[0], i[1])
                    euclidean_dist4 = d_eucl(p3_x, p3_y, i[0], i[1])
                    euclidean_dist5 = d_eucl(p4_x, p4_y, i[0], i[1])
                    euclidean_dist6 = d_eucl(p5_x, p5_y, i[0], i[1])
                    euclidean_dist7 = d_eucl(p6_x, p6_y, i[0], i[1])
                    euclidean_dist8 = d_eucl(p8_x, p8_y, i[0], i[1])
                #print(euclidean_dist8,euclidean_dist6,euclidean_dist7,euclidean_dist4,euclidean_dist5,euclidean_dist1,euclidean_dist2)
                if (
                        p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and euclidean_dist1 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                        if (p0_x, p0_y, dist) not in q:
                            q.append((p0_x, p0_y, dist))
                if (
                        p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and euclidean_dist2 > 15):

                    if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                        if (p1_x, p1_y, dist) not in q:
                            q.append((p1_x, p1_y, dist))
                if (
                        p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and euclidean_dist3 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p2_x-15, p2_y-15)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                        if (p2_x, p2_y, dist) not in q:
                            q.append((p2_x, p2_y, dist))
                if (
                        p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and euclidean_dist4 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                        if (p3_x, p3_y, dist) not in q:
                            q.append((p3_x, p3_y, dist))
                if (
                        p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and euclidean_dist5 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                        if (p4_x, p4_y, dist) not in q:
                            q.append((p4_x, p4_y, dist))
                if (
                        p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and euclidean_dist6 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                        if (p5_x, p5_y, dist) not in q:
                            q.append((p5_x, p5_y, dist))
                if (
                        p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and euclidean_dist7 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p6_x-15, p6_y-15, 30, 30), 1)
                        if (p6_x, p6_y, dist) not in q:
                            q.append((p6_x, p6_y, dist))
                if (
                        p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and euclidean_dist8 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                        if (p8_x, p8_y, dist) not in q:
                            q.append((p8_x, p8_y, dist))
            #print("q", q)
            clockwise = False
            anticlockwise = True  # changing the direction to opposite
            f.append(q.pop(0))  # drop current element from queue and adding it to potential fields list
            dist = dist + 1  # increase the distance

        else:
            #print("anticlcockwise")
            p0_x = q[0][0] - dist
            p0_y = q[0][1] - dist
            p1_x = q[0][0]
            p1_y = q[0][1] - dist
            p2_x = q[0][0] + dist
            p2_y = q[0][1] - dist
            p3_x = q[0][0] + dist
            p3_y = q[0][1]
            p4_x = q[0][0] + dist
            p4_y = q[0][1] + dist
            p5_x = q[0][0]
            p5_y = q[0][1] + dist
            p6_x = q[0][0] - dist
            p6_y = q[0][1] + dist
            p8_x = q[0][0] - dist
            p8_y = q[0][1]

            for x in q[::-1]:
                previous_neighbours_list.append(x)
                if len(previous_neighbours_list) >= 8:
                    break

            euclidean_0, euclidean_1, euclidean_2, euclidean_3, euclidean_4, euclidean_5, euclidean_6, euclidean_8 = ( [] for _ in range(8))

            for i in previous_neighbours_list:
                euclidean_dist1 = d_eucl(p0_x, p0_y, i[0], i[1])
                euclidean_dist2 = d_eucl(p1_x, p1_y, i[0], i[1])
                euclidean_dist3 = d_eucl(p2_x, p2_y, i[0], i[1])
                euclidean_dist4 = d_eucl(p3_x, p3_y, i[0], i[1])
                euclidean_dist5 = d_eucl(p4_x, p4_y, i[0], i[1])
                euclidean_dist6 = d_eucl(p5_x, p5_y, i[0], i[1])
                euclidean_dist7 = d_eucl(p6_x, p6_y, i[0], i[1])
                euclidean_dist8 = d_eucl(p8_x, p8_y, i[0], i[1])

            if (
                    p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and euclidean_dist1 > 15):
                if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p0_x, p0_y, 30, 30), 1)
                    if (p0_x,p0_y,dist) not in q:
                        q.append((p0_x, p0_y, dist))
            if (
                    p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and euclidean_dist2 > 15):
                if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p1_x, p1_y, 30, 30), 1)
                    if (p1_x, p1_y, dist) not in q:
                        q.append((p1_x, p1_y, dist))
            if (
                    p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and euclidean_dist3 > 15):
                if (check_obstacle(obstacles_coordinates_new, p2_x, p2_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p2_x, p2_y, 30, 30), 1)
                    if (p2_x, p2_y, dist) not in q:
                        q.append((p2_x, p2_y, dist))
            if (
                    p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and euclidean_dist4 > 15):
                if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p3_x, p3_y, 30, 30), 1)
                    if (p3_x, p3_y, dist) not in q:
                        q.append((p3_x, p3_y, dist))
            if (
                    p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and euclidean_dist5 > 15):
                if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p4_x, p4_y, 30, 30), 1)
                    if (p4_x, p4_y, dist) not in q:
                        q.append((p4_x, p4_y, dist))
            if (
                    p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and euclidean_dist6 > 15):
                if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y)) == 0:

                    pygame.draw.rect(surf, color, pygame.Rect(p5_x, p5_y, 30, 30), 1)
                    if (p5_x, p5_y, dist) not in q:
                        q.append((p5_x, p5_y, dist))
            if (
                    p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and euclidean_dist7 > 15):
                if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p6_x, p6_y, 30, 30), 1)
                    if (p6_x, p6_y, dist) not in q:
                        q.append((p6_x, p6_y, dist))
            if (
                    p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and euclidean_dist8 > 15):
                if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p8_x, p8_y, 30, 30), 1)
                    if (p8_x, p8_y, dist) not in q:
                        q.append((p8_x, p8_y, dist))

            clockwise = True
            anticlockwise = False
            f.append(q.pop(0))
            dist = dist + 1

        if len(q) == 0:
            break



closest_points = []
start_coordinates1 = []
new_f = f.copy()
#
# print("f=",f)
start_c = to_pygame(start_coordinates_pygame,size[1])
new_startX = start_c[0]
new_startY = start_c[1]
start_coordinates1.append((new_startX, new_startY))
closest_points.append((new_startX, new_startY))
# print("closest: ", closest_points)
# print("goal: ", x_draw_goal, y_draw_goal)
# print("start :", start_coordinates1)

generated_path = search_path(closest_points, new_f, 800, x_draw_goal, y_draw_goal)
optimal = search_min_distances(generated_path, [[x_draw_goal, y_draw_goal]])
path_smooth = path_smoothing(optimal,15)

with open('../txt_files/path_coords.txt', 'w') as file:
    for x, y in path_smooth:
        file.write('{} {}\n'.format(x, y))


color1=(100,255,0)
draw_path(start_coordinates1, path_smooth,color1)
print(path_smooth)
screen.blit(surf, (0,0))
pygame.display.flip()

crashed = False
while not crashed:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

pygame.quit()
