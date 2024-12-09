# this program will generate a .g2o file containing 2D poses that represent
# - a robot travering in a circle
# - a sparse dataset without many sensor readings to test the limits of MASAT

import math
import random

radius = 10

# generate 10000 points to match size of other datasets
num_points = 10000 

# error
# simulates real life error from sensor measurements
# different error types provides more realistic result
err_stdev_pos = 0.05
err_stdev_ortn = 0.01

# points generation
# we do not add error here because we assume that the robot
# can programatially move in a near-perfect circle
circle_points = []
for i in range(num_points):
    # creates the ith% of the circle angle
    angle = 2 * math.pi * i / num_points

    # center the circle at 0, 0 -> no translation
    x = math.cos(angle)
    y = math.sin(angle)

    circle_points.append((x, y, angle))

# create the sparse dataset
# - sample points from the circle_points randomly at a size 10% of original
# - add in edges for sampled points

num_sparse_points = 1000
random_indices = sorted(random.sample(range(num_points), num_sparse_points))
sparse_circle_points = [circle_points[i] for i in random_indices]

# create the edges and add in error
# edges are created between consecutive points (min spanning tree esque)
sparse_edges = []
for i in range(num_sparse_points):
    # loop closure means that the ith+1 point can be 0
    i2 = (i + 1) % num_sparse_points
    x1, y1, theta1 = sparse_circle_points[i]
    x2, y2, theta2 = sparse_circle_points[i2]

    # create the edge as the distance between points plus noise
    edgex = x2 - x1 + random.gauss(0, err_stdev_pos)
    edgey = y2 - y1 + random.gauss(0, err_stdev_pos)
    edgetheta = theta2 - theta1 + random.gauss(0, err_stdev_ortn)

    sparse_edges.append((i, i2, edgex, edgey, edgetheta))

# print(sparse_edges)
# write the dataset to a .g2o file
with open("sparse2Dcircle.g2o", "w") as f:
    for i in range(len(sparse_circle_points)):
        x, y, theta = sparse_circle_points[i]
        f.write(f"VERTEX_SE2 {i} {x:.6f} {y:.6f} {theta:.6f}\n")

    for i in range(len(sparse_edges)):
        i1, i2, dx, dy, dtheta = sparse_edges[i]
        f.write(f"EDGE_SE2 {i1} {i2} {dx:.6f} {dy:.6f} {dtheta:.6f}\n")

    print("written successfully")