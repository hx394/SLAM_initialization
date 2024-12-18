import cv2
import apriltag
import time
import gtsam
import gtsam.noiseModel
import numpy as np
import os
from gtsam.utils.plot import plot_trajectory
import matplotlib.pyplot as plt

is3D=True
graph, initial_estimate=gtsam.readG2o("./output3.g2o",is3D)


def plot_2d_trajectory(values, title="2D Trajectory", axis_labels=("X", "Y")):
    """Custom 2D trajectory plotter."""
    x_vals = []
    y_vals = []

    # Extract x, y from Pose2 values
    for key in values.keys():
        pose = values.atPose2(key)
        x_vals.append(pose.x())
        y_vals.append(pose.y())

    # Plot trajectory
    plt.figure()
    plt.plot(x_vals, y_vals, color='green',linestyle='dashed',linewidth=1,markersize=2, label="Trajectory")  # Line with markers
    plt.title(title)
    plt.xlabel(axis_labels[0])
    plt.ylabel(axis_labels[1])
    plt.legend()
    plt.grid(True)
    plt.axis("equal")  # Ensure equal aspect ratio
    plt.show()


# def X(i):
#     return gtsam.symbol('x',i)

# def L(j):
#     return int(gtsam.symbol('o',j))
# def P(k):
#     return int(gtsam.symbol('p',k))

#print("Current working directory:", os.getcwd())



if is3D:
    plot_trajectory(0,initial_estimate)
    plt.show()
else:
    plot_2d_trajectory(initial_estimate)

# Define noise model for the projection factors
#measurement_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)  # Example: standard deviation of 1 pixel


#pose_prior=gtsam.Pose3(gtsam.Rot3(*R),gtsam.Point3(*t))
#initial_estimate.insert(X(0), pose_prior)
#pose_noise=gtsam.noiseModel.Diagonal.Sigmas(np.array(
#    [1,1,1,1,1,1]))
#graph.push_back(gtsam.PriorFactorPose3(X(0),pose_prior,pose_noise))

#point_noise=gtsam.noiseModel.Isotropic.Sigma(3,0.000001)
# Insert 3D points (AprilTag corners) into initial estimate
# for i, point_3D in enumerate(points_3D):



    
#     factor=gtsam.PriorFactorPoint3(P(i),point_3D,point_noise)
#     initial_estimate.insert(P(i),point_3D)
#     graph.push_back(factor)
    
# Add projection factors for each corner point

# Define noise models


# for i, (point_3D, point_2D) in enumerate(zip(points_3D, observed_2D)):
    
 
    
#      factor=gtsam.GenericProjectionFactorCal3_S2(point_2D, measurement_noise, X(0), P(i), K)
    
#      graph.push_back(factor)
    


# Provide an initial estimate for the camera pose
#initial_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.5))  # Guess the camera is 0.5 meters in front of the tag
#initial_pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.1, -0.1, 0.1), gtsam.Point3(0.1, 0.1, 1.0))  # Vary initial estimate
#initial_estimate.insert(gtsam.symbol('x', 0), initial_pose)
#print("graph:")
#print(graph)
#print("initial estimate:")
#print(initial_estimate)
initial_error = graph.error(initial_estimate)
print("Initial error before optimization:", initial_error)
# Optimize the factor graph using Levenberg-Marquardt optimizer
params=gtsam.LevenbergMarquardtParams()
params.setVerbosityLM("SUMMARY")
params.setVerbosity("TERMINATION") 
#params.setVerbosityLM("TRYDELTA") 

start_time = time.time()  # Record start time
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,params)
result = optimizer.optimize()

end_time = time.time()  # Record end time

# Calculate runtime
runtime = end_time - start_time
print(f"Optimization Runtime: {runtime:.4f} seconds")
#optimizer=gtsam.DoglegOptimizer(graph,initial_estimate)




#print("result:")
#print(result)
# Extract and print the optimized camera pose relative to AprilTag
#camera_pose = result.atPose3(X(0))
#print("Estimated camera pose relative to AprilTag:")
#print(camera_pose)
print("Error after optimizing:")
print(optimizer.error())
if is3D:
    plot_trajectory(1,result)
    plt.show()
else:
    plot_2d_trajectory(result)

