# używanie pygame do generowania wykresu
# odleglosc euklidesowa mierzona jest miedzy bieżącym punktem, a współrzędnymi celu
# minimalna ogleglosc ustawiona jest na 15



import matplotlib as plt
import matplotlib.pyplot as pypl
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math


# distance between (x_temp,y_temp) and (x2_temp,y2_temp)
def d_eucl(x_temp, y_temp, x2_temp, y2_temp):
    distance_temp = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp = distance_temp + (y_temp - y2_temp) * (y_temp - y2_temp)
    distance_temp = math.sqrt(distance_temp)
    return (distance_temp)


# Check collisions with obstacles, if omit==1 than collision detected,
def check_obstacle(obstacles_coordinates, x_temp, y_temp):
    omit = 0
    for i in range(len(obstacles_coordinates)):
        # print(d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]))
        if (d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]) < 50):
            omit = 1
    return (omit)


def getImage(path, zoom=0.06):
    return OffsetImage(pypl.imread(path), zoom=zoom)


plt.use("Agg")
import matplotlib.backends.backend_agg as agg

plt.rcParams.update({
    "lines.marker": "o",  # available ('o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X')
    "lines.linewidth": "0.4",
    "axes.prop_cycle": plt.cycler('color', ['white']),  # line color
    "text.color": "black",  # no text in this example
    "axes.facecolor": "white",  # background of the figure
    "axes.edgecolor": "gray",
    "axes.labelcolor": "black",  # no labels in this example
    "axes.grid": "True",
    "grid.linestyle": ":",  # {'-', '--', '-.', ':', '', (offset, on-off-seq), ...}
    "xtick.color": "black",
    "ytick.color": "black",
    "grid.color": "lightgray",
    "figure.facecolor": "white",  # color surrounding the plot
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


# determinig coordinates
goal_coordianets = [(465, 150)]
center_list = [(12, 14), (12, 386), (553, 386), (553, 14)]
obstacles_coordinates = [(231, 240), (120, 143)]
start_coordinates = [(55, 330)]

# dividing lists into 2 for getting separated x and y coordinates
x1, y1 = zip(*center_list)
x2, y2 = zip(*obstacles_coordinates)
x3, y3 = zip(*goal_coordianets)
x4, y4 = zip(*start_coordinates)
x_borders, x_obstacles, x_goal, y_borders, y_obstacles, y_goal, x_start, y_start = ([] for _ in range(8))

# finding minimum values in x,y coordinates from border
x_min = min(x1)
y_min = min(y1)

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

# insert images insted of normal points
fig, ax = pypl.subplots(figsize=[6.5, 4.5])
ax.scatter(x_borders, y_borders)
ax.scatter(x_obstacles, y_obstacles)
ax.scatter(x_goal, y_goal)
ax.scatter(x_start, y_start)

# display images as points in plot
for x0, y0, path in zip(x_borders, y_borders, paths):
    ab = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(ab)
for x0, y0, path in zip(x_obstacles, y_obstacles, paths2):
    bc = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(bc)
for x0, y0, path in zip(x_goal, y_goal, paths3):
    cd = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(cd)
for x0, y0, path in zip(x_start, y_start, paths4):
    de = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(de)

ax = fig.gca()
# ax.plot(x_borders, y_borders, 'g--')

canvas = agg.FigureCanvasAgg(fig)
canvas.draw()
renderer = canvas.get_renderer()
raw_data = renderer.tostring_rgb()

import pygame
from pygame.locals import *

pygame.init()

window = pygame.display.set_mode((650, 450), DOUBLEBUF)
screen = pygame.display.get_surface()

size = canvas.get_width_height()
surf = pygame.image.fromstring(raw_data, size, "RGB").convert()

color = (50, 50, 50)

################## SQUARE FILL AL ######################

x_draw_goal = x_goal[0] + 20  # for x coordiantes we have to add 20 pixels
y_draw_goal = y_goal[0] + 118  # for y coordinates we have to add 118 pixels

# initial values
clockwise = True
anticlockwise = False
dist = 0  # current distance
f = []  # list of potential fields
q = []  # empty queue

# preparing obstacle coordinates for pygame window
obst_new_X1 = x_obstacles[1] + 167
obst_new_X2 = x_obstacles[0] - 38
obst_new_Y1 = y_obstacles[1] + 49
obst_new_Y2 = y_obstacles[0] + 34

obstacles_coordinates_new = []

obstacles_coordinates_new.append((obst_new_X1, obst_new_Y1))
obstacles_coordinates_new.append((obst_new_X2, obst_new_Y2))

previous_neighbours_list = []

q.append((x_draw_goal, y_draw_goal, dist))  # adding initial values to the queue (goal coordinates and 0 as a current distance)

for i in range(500):
    x_restriction = 550
    x_restriction_left = 90
    y_restriction = 360
    y_restriction_bottom = 55

    previous_neighbours_list.clear()

    # looking for the same objects in lists
    intersection1 = [item1 for item1 in f
                     for item2 in q if item1 == item2]

    intersection2 = [item1 for item1 in obstacles_coordinates_new
                     for item2 in q if item1 == item2]

    if len(intersection1) == 1 or len(intersection2) == 1:
        q.pop(0)
        dist = dist + 1
        intersection1.clear()
        intersection2.clear()
        continue

    # if there is no intersection between lists
    else:
        if clockwise is True:
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

            # if we have first iteration, do not draw any squares and increase the distance
            if i == 0:
                q.append((p0_x, p0_y, dist))
                q.append((p1_x, p1_y, dist))
                q.append((p2_x, p2_y, dist))
                q.append((p3_x, p3_y, dist))
                q.append((p4_x, p4_y, dist))
                q.append((p5_x, p5_y, dist))
                q.append((p6_x, p6_y, dist))
                q.append((p8_x, p8_y, dist))
                dist = 3
            else:
                # counting the euclidean distance from current neigbours to the goal
                euclidean_dist1 = d_eucl(p0_x, p0_y, x_draw_goal, y_draw_goal)
                euclidean_dist2 = d_eucl(p1_x, p1_y, x_draw_goal, y_draw_goal)
                euclidean_dist3 = d_eucl(p2_x, p2_y, x_draw_goal, y_draw_goal)
                euclidean_dist4 = d_eucl(p3_x, p3_y, x_draw_goal, y_draw_goal)
                euclidean_dist5 = d_eucl(p4_x, p4_y, x_draw_goal, y_draw_goal)
                euclidean_dist6 = d_eucl(p5_x, p5_y, x_draw_goal, y_draw_goal)
                euclidean_dist7 = d_eucl(p6_x, p6_y, x_draw_goal, y_draw_goal)
                euclidean_dist8 = d_eucl(p8_x, p8_y, x_draw_goal, y_draw_goal)

                # adding neighbours to the queue
                if (
                        p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and euclidean_dist1 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p0_x, p0_y, 30, 30), 1)
                        q.append((p0_x, p0_y, dist))
                if (
                        p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and euclidean_dist2 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p1_x, p1_y, 30, 30), 1)
                        q.append((p1_x, p1_y, dist))
                if (
                        p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and euclidean_dist3 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p2_x, p2_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p2_x, p2_y, 30, 30), 1)
                        q.append((p2_x, p2_y, dist))
                if (
                        p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and euclidean_dist4 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p3_x, p3_y, 30, 30), 1)
                        q.append((p3_x, p3_y, dist))
                if (
                        p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and euclidean_dist5 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p4_x, p4_y, 30, 30), 1)
                        q.append((p4_x, p4_y, dist))
                if (
                        p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and euclidean_dist6 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p5_x, p5_y, 30, 30), 1)
                        q.append((p5_x, p5_y, dist))
                if (
                        p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and euclidean_dist7 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p6_x, p6_y, 30, 30), 1)
                        q.append((p6_x, p6_y, dist))
                if (
                        p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and euclidean_dist8 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y)) == 0:
                        pygame.draw.rect(surf, color, pygame.Rect(p8_x, p8_y, 30, 30), 1)
                        q.append((p8_x, p8_y, dist))

                clockwise = False
                anticlockwise = True  # changing the direction to opposite
                f.append(q.pop(0))  # drop current element from queue and adding it to potential fields list
                dist = dist + 1  # increase the distance

        else:
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

            # print("i = ", i, " distances = ", euclidean_dist1, euclidean_dist2, euclidean_dist3, euclidean_dist4,euclidean_dist5)

            euclidean_dist1 = d_eucl(p0_x, p0_y, x_draw_goal, y_draw_goal)
            euclidean_dist2 = d_eucl(p1_x, p1_y, x_draw_goal, y_draw_goal)
            euclidean_dist3 = d_eucl(p2_x, p2_y, x_draw_goal, y_draw_goal)
            euclidean_dist4 = d_eucl(p3_x, p3_y, x_draw_goal, y_draw_goal)
            euclidean_dist5 = d_eucl(p4_x, p4_y, x_draw_goal, y_draw_goal)
            euclidean_dist6 = d_eucl(p5_x, p5_y, x_draw_goal, y_draw_goal)
            euclidean_dist7 = d_eucl(p6_x, p6_y, x_draw_goal, y_draw_goal)
            euclidean_dist8 = d_eucl(p8_x, p8_y, x_draw_goal, y_draw_goal)
            # drawing squares
            # adding neighbours to the queue

            if (
                    p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and euclidean_dist1 > 15):
                if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p0_x, p0_y, 30, 30), 1)
                    q.append((p0_x, p0_y, dist))
            if (
                    p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and euclidean_dist2 > 15):
                if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p1_x, p1_y, 30, 30), 1)
                    q.append((p1_x, p1_y, dist))
            if (
                    p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and euclidean_dist3 > 15):
                if (check_obstacle(obstacles_coordinates_new, p2_x, p2_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p2_x, p2_y, 30, 30), 1)
                    q.append((p2_x, p2_y, dist))
            if (
                    p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and euclidean_dist4 > 15):
                if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p3_x, p3_y, 30, 30), 1)
                    q.append((p3_x, p3_y, dist))
            if (
                    p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and euclidean_dist5 > 15):
                if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p4_x, p4_y, 30, 30), 1)
                    q.append((p4_x, p4_y, dist))
            if (
                    p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and euclidean_dist6 > 15):
                if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p5_x, p5_y, 30, 30), 1)
                    q.append((p5_x, p5_y, dist))
            if (
                    p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and euclidean_dist7 > 15):
                if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p6_x, p6_y, 30, 30), 1)
                    q.append((p6_x, p6_y, dist))
            if (
                    p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and euclidean_dist8 > 15):
                if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y)) == 0:
                    pygame.draw.rect(surf, color, pygame.Rect(p8_x, p8_y, 30, 30), 1)
                    q.append((p8_x, p8_y, dist))

            clockwise = True
            anticlockwise = False
            f.append(q.pop(0))
            dist = dist + 1

screen.blit(surf, (0, 0))
pygame.display.flip()

crashed = False
while not crashed:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

pygame.quit()

# plots coordinates on pygame screen
# (82,54)   (585, 54)


# (82,402)  (585, 402)
