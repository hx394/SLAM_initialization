# Northeastern University students try to improve MASAT intial guess for SLAM
============================================
## Contributors: Hongzhen Xu, Ananya Tadigadapa, Shuyue Gao
============================================
This repository contains the modified code for the *MASAT algorithm*: a robust and efficent approach for creating inital estimates prior to pose-graph optimization in off-line SLAM. 

There are two new approaches introduced and implemented by Northeastern University students in the code.

### The paper describing the original approach:
[Károly Harsányi, Attila Kiss, Tamás Szirányi, András Majdik: MASAT: A fast and robust algorithm for pose-graph initialization](https://www.sciencedirect.com/science/article/pii/S0167865519303241)

### The modifications made by NEU students:
Please see the report in the git repo.


### About experiment codes
To run the files in "experiment codes" folder, make sure you have the correct path and set "is3D" parameter to a correct boolean.


### Requirements
- Eigen (http://eigen.tuxfamily.org)

### Guide
Compile:
```sh
./make
```

There will be 3 runnable files:
MASAT the former algorithm,
MASAT_sa the simple average on result of MASAT,
MASAT_rbfs the rolling BFS approach to MASAT.


Run:
```sh
./MASAT <PATH_TO_INPUT_FILE> <PATH_TO_OUTPUT_FILE>
./MASAT_sa <PATH_TO_INPUT_FILE> <PATH_TO_OUTPUT_FILE>
./MASAT_rbfs <PATH_TO_INPUT_FILE> <PATH_TO_OUTPUT_FILE>
```



The input files must be in .g2o format. There are some examples in the ```./iinput_output_and_graph``` folder.

Once the inital guess is finished, the authors of former paper suggest using the [g2o](https://github.com/RainerKuemmerle/g2o) framework for optimization and visualization.

Actually, we Northeastern University students use gtsam for the optimization and visualization and evaluation.
