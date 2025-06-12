# Unscented Kalman Filter for Highway

## 1. Overview
This project implements an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy *lidar* and *radar* measurements. We'll then evaluate the quality of our filter using RMSE values.

<img src="assets/ukf_highway_tracked.gif" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has its own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so we're only tracking along the X/Y axis in this project.


## 2. Table of Contents
- [Project Instructions](#build)
- [UKF Implementation](#implementation)
- [Acknowledgements](#acknowledgements)


## 3. Project Instructions <a name="build"></a>
The main program can be built and ran by doing the following from the project top directory.

1. Clone this repo with LFS, which can be done in two ways:
  1. `git lfs clone https://github.com/moorissa/lidar-obstacle-detector.git` OR
  2. Alternatively:
  ```bash
    git clone https://github.com/moorissa/lidar-obstacle-detector.git
    cd lidar-obstacle-detector  # ensure no duplicated names in the same directory
    git lfs pull
  ```
  If LFS continues causing (submission) issues:
   - Upload PCD files to a cloud service (Google Drive, Dropbox) and include download links
   - Use smaller sample PCD files that don't require LFS
   - Compress the PCD files if possible
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make -j`
4. Run it: `./ukf_highway`

In short, you can rerun it with: `rm -rf ./* && cmake .. && make && ./ukf_highway`

<img src="assets/ukf_highway.png" width="700" height="400" />

#### Dependencies
* cmake >= 3.10
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 3.8
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* PCL >= 1.10
  * [Documentation](https://pointclouds.org/downloads/)

#### Currently used (2025 version):
* MacOS: Sequoia 15.5
* cmake: 3.31.7
* GNU make: 3.81
* gcc: 
  ```
  Target: arm64-apple-darwin24.5.0
  Thread model: posix
  InstalledDir: /Library/Developer/CommandLineTools/usr/bin
  ```
* pcl: stable 1.15.0

#### Generating Additional Data
If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment and performing clustering.


## 4. UKF Implementation <a name="implementation"></a>

## 4.1. Parameters
File(s): `highway.h` - there are a number of parameters we can modify for debugging purpose.
- `trackCars` list can toggle on/off cars for UKF object to track
- `projectedTime` and `projectedSteps` controls the visualization of predicted position in the future
- `visualize_pcd` sets the visualization of Lidar point cloud data

```c++
// Set which cars to track with UKF
std::vector<bool> trackCars = {true,true,true};
// Visualize sensor measurements
bool visualize_lidar = true;
bool visualize_radar = true;
bool visualize_pcd = false;
// Predict path in the future using UKF
double projectedTime = 0;
int projectedSteps = 0;
```

## 4.2. Code Walkthrough
Instead of using mathematical approximations (like linearization), the UKF approach:
1. Samples the uncertainty space with carefully chosen "sigma points"
2. Runs each sample through the exact nonlinear motion model
3. Statistically combines the results to get the predicted mean and covariance

This gives much more accurate predictions for nonlinear systems compared to traditional Kalman filters. In the following steps, we will learn that `ProcessMeasurement` function = Update cycle that:
- Takes in new sensor data (LIDAR or RADAR measurement)
- Calls `Prediction()` first, then calls the appropriate update function (`UpdateLidar` or `UpdateRadar`)
- This is the complete measurement processing cycle

In this case, `Prediction` = Predict step only of the predict-update cycle. It uses motion model to estimate where object should be at current time, and no sensor data involved - purely physics/motion-based prediction.

#### The Full Cycle
Overall, we're implementing `ProcessMeasurement()` function that does:
1. `Prediction(dt)`      -> "Where should object be now?" (physics)
2. `UpdateLidar/Radar()` -> "Correct prediction with sensor data"

Process Measurement basically manages the timing and calls both steps in sequence whenever new sensor data arrives -- which aligns with the standard Kalman filter pattern:
1. *Predict:* Use motion model to forecast state
2. *Update:* Use sensor measurement to correct the forecast


### 1. Initialize UKF attributes
File(s): `ukf.cpp`
- dimension of the state vector `n_x_`
- state vector `x_`
- covariance matrix `P_`
- dimension of the augmented state vector `n_aug_`
- predicted sigma points matrix `Xsig_pred_`
- sigma points weights vector `weights_`
- standard deviation of longitudinal acceleration noise `std_a_`
- standard deviation of yaw acceleration noise `std_yawdd_`
- sigma points spreading parameter `lambda_`


### 2. Implement process measurement
File(s): `ukf.cpp` -> `UKF::ProcessMeasurement`

For the very first incoming measurement, state vector `x_`, covariance matrix `P_`, and timestamp `time_us_` are initialized according to the raw data `meas_package.raw_measurements_` and `meas_package.timestamp_`.

For the following measurements, timestamp `time_us_` is recorded, a sequence of functions are called to `Prediction()` and `UpdateLidar()`/`UpdateRadar()`.

Main functionality of `UKF::ProcessMeasurement`:
- Initialization: On first measurement, initializes the state vector with position data from either LIDAR (direct x,y) or RADAR (converted from polar coordinates)
- Prediction: Uses the motion model to predict where the object should be at the current timestamp
- Update: Corrects the prediction using the actual sensor measurement
Key points:

The state vector has 5 elements: `[px, py, v, yaw, yawd]` representing position, velocity, yaw angle, and yaw rate
- LIDAR gives direct cartesian coordinates, while RADAR provides polar coordinates that need conversion
- The filter alternates between prediction (based on motion model) and update (based on sensor measurements)
- Time intervals are calculated and converted from microseconds to seconds for the prediction step

This is a typical sensor fusion implementation where both LIDAR and RADAR measurements are used to track an object's state over time.


### 3. Implement prediction
File(s): `ukf.cpp` -> `UKF::Prediction()`

The main functionality of this UKF Prediction function is to predict where an object will be at the next time step, while properly accounting for uncertainty and nonlinear motion. Meaning, given the current state estimate and elapsed time (`delta_t`), predict the object's future position, velocity, heading, and turn rate.

The prediction process is the same for both Lidar and Radar measurements.

- creates an augmented mean vector `x_aug` and augmented state covariance matrix `P_aug`
- generate sigma points matrix `Xsig_aug` for previously estimated state vector
- predict sigma points matrix `Xsig_pred_` for the current state vector 
- predict the state mean `x_` and covariance `P_` using weights and predicted sigma points

#### Key Capabilities
- Handles nonlinear motion: Unlike linear Kalman filters, this can handle objects that turn (curved trajectories) as well as straight-line motion.
- Uncertainty propagation: It doesn't just predict a single "best guess" - it predicts how uncertain we should be about that prediction by tracking how uncertainty grows over time.
- Process noise modeling: Accounts for the fact that our motion model isn't perfect - real objects experience random accelerations and disturbances we can't predict exactly.


### 4. Implement updates on Lidar and Radar data
File(s): `ukf.cpp` -> `UKF::UpdateLidar()` and `UKF::UpdateRadar()` 

The steps to update Lidar and Radar measurements are similar, except Lidar points are in the **Cartesian** coordinates but Radar points are in the **Polar** coordinates. Therefore, they differ in the measurement dimension `n_z`, dimension of matrices, and the transformation equations.

Generally, they follow the same steps to update the measurement.

- transform the predicted sigma points `Xsig_pred_` into measurement space `Zsig` based on the sensor types
- calculate the mean state `z_` and covariance matrix `S` with noise considered
- calculate cross-correlation matrix `Tc` between state space and measurement space
- calculate the Kalman gain `K`
- update the state vector `x_` and covariance `P_`


### 5. Test run

The screenshot shown below is one of the simulation moments. The ego car is green while the other traffic cars are blue. The red spheres above cars represent the `(x,y)` lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The green spheres above cars represent the predicted path that cars would move in the near future.

On the left-hand side, the root mean squared errors (RMSE) for position `(x,y)` and velocity `(Vx, Vy)` are calculated in realtime, which represent the prediction accuracy.

<img src="assets/ukf_highway_result.png" width="700" height="400" />



## Ackowledgements <a name="acknowledgements"></a>
* [Udacity Sensor Fusion Program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

For any questions or feedback, feel free to email [moorissa.tjokro@columbia.edu](mailto:moorissa.tjokro@columbia.edu).


