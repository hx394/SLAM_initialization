# this program will generate a .g2o file containing 2D poses that represent
# - a robot travering in a circle
# - a noisy dataset to test the limits of MASAT

import math
import random

radius = 10

# generate 10000 points to match size of other datasets
num_points = 10000 

# error
# simulates real life error from sensor measurements
# different error types provides more realistic result
err_stdev_pos = 0.5
err_stdev_ortn = 0.2

# information matrix - variance between x, y, theta
i_xx = 1/(err_stdev_pos ** 2) # I_xx variance of x displacement
i_xy = 0 # I_xy covariance between x and y
i_xt = 0 # I_xtheta covariance between x and theta
i_yy = 1/(err_stdev_pos ** 2) # I_yy variance of y displacement
i_yt = 0 # I_ytheta covariance between x and theta
i_tt = 1/(err_stdev_ortn ** 2) # I_thetatheta variance of theta displacement

# points generation
# we do not add error here because we assume that the robot
# can programatially move in a near-perfect circle
circle_points = []
for i in range(num_points):
    # creates the ith% of the circle angle
    angle = 2 * math.pi * i / num_points

    # center the circle at 0, 0 -> no translation
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)

    circle_points.append((x, y, angle))

# create the edges and add in error
# edges are created between consecutive points (min spanning tree esque)
edges = []
for i in range(num_points):
    # loop closure means that the ith+1 point can be 0
    i2 = (i + 1) % num_points
    x1, y1, theta1 = circle_points[i]
    x2, y2, theta2 = circle_points[i2]

    # create the edge as the distance between points plus noise
    edgex = x2 - x1 + random.gauss(0, err_stdev_pos)
    edgey = y2 - y1 + random.gauss(0, err_stdev_pos)
    edgetheta = theta2 - theta1 + random.gauss(0, err_stdev_ortn)

    edges.append((i, i2, edgex, edgey, edgetheta))

# write the dataset to a .g2o file
with open("noisy2Dcircle.g2o", "w") as f:
    for i in range(len(circle_points)):
        x, y, theta = circle_points[i]
        f.write(f"VERTEX_SE2 {i} {x:.6f} {y:.6f} {theta:.6f}\n")

    for i in range(len(edges)):
        i1, i2, dx, dy, dtheta = edges[i]
        f.write(f"EDGE_SE2 {i1} {i2} {dx:.6f} {dy:.6f} {dtheta:.6f} {i_xx} {i_xy} {i_xt} {i_yy} {i_yt} {i_tt}\n")

    print("written successfully")
