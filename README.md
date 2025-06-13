# Camera-based 2D Feature Tracking
*Building Towards Collision Detection*

## 1. Overview
The main objective of this project is to develop a complete collision detection system. This project builds the core feature tracking capabilities we'll need for collision detection. We'll explore different detector/descriptor combinations to find the optimal performance balance.

<img src="images/keypoints.png" width="700" height="400" />

#### Phase 1: Data Foundation
We'll first set up the groundwork for efficient image processing:
- Load and manage image sequences
- Design robust data structures
- Implement ring buffer optimization for memory efficiency

#### Phase 2: Keypoint Detection Showdown
Compare multiple detection algorithms in action:
- HARRIS - Classic corner detection
- FAST - Speed-optimized features
- BRISK - Binary robust features
- SIFT - Scale-invariant detection

We'll then evaluate each on keypoint quantity and processing speed

#### Phase 3: Feature Matching Arsenal
We then use two powerful matching approaches:
- Brute Force - Exhaustive but thorough matching
- FLANN - Fast approximate matching for real-time performance

#### Phase 4: Performance Benchmarking
With the complete framework ready, we perform the following evals:
- Test algorithm combinations systematically
- Measure and compare performance metrics
- Identify the winning detector/descriptor pairs

This is the first phase of our next project that will integrate lidar to add depth perception and leverage deep Learning / neural networks for object detection. Together this will complete the final project of creating a collision detection system with robust real-world performance.


## 2. Table of Contents
- [Project Instructions](#build)
- [Implementation](#implementation)
- [Acknowledgements](#acknowledgements)


## 3. Project Instructions <a name="build"></a>
The main program can be built and ran by doing the following from the project top directory.

1. Clone this repo with LFS, which can be done in two ways:
  1. `git lfs clone <this repo>` OR
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
4. Run it: `./2D_feature_tracking`

In short, we can rerun it with: `rm -rf ./* && cmake .. && make && ./ukf_highway`

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
* OpenCV >= 4.1
  * [Documentation](https://opencv.org/)

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


## 4. Implementation <a name="implementation"></a>

### MP.1 Data Buffer Optimization
The "ring buffer" is naively implemented by pushing new frames onto the dataBuffer until the size is reached. Then the buffer size is reduced by erasing the oldest frame.

### MP.2 Keypoint Detection
The HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT keypoint detectors were implemented. The methods were made selectable by changing the string's definition.

### MP.3 Keypoint Removal
The `.contains` method of the VehicleRect object is used to detect and remove keypoints outside of the rectangle.

### MP.4 Keypoint Descriptors
The BRIEF, ORB, FREAK, AKAZE and SIFT descriptors were implemented. The methods were made selectable by changing the string's definition.

### MP.5 Descriptor Matching
The FLANN matcher and kNN matchers are implemented. The methods were made selectable by changing the string's definition

### MP.6 Descriptor Distance Ratio
Lowe's distance ratio test was implemented by comparing the best and second best matches to decide if a pair of matched keypoints should be stored.

### MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

| Detector | image0 | image1 | image2 | image3 | image4 | image5 | image6 | image7 | image8 | image9 | Neighborhood size |
| :---:    | :---:  | :---:  | :---:  |  :---: | :---:  | :---:  | :---:  | :---:  | :---:  | :---:  | :---: |
| SHI-TOMASI | 125 | 118 | 123 | 120 | 120 | 113 | 114 | 123 | 111 | 112 | 4
| HARRIS | 17 | 14 | 18 | 21 | 26 | 43 | 18 | 30 | 26 | 34 | 6
| FAST | 121 | 115 | 127 | 122 | 111 | 113 | 107 | 103 | 112 | 117 | 7
| BRISK | 264 | 282 | 282 | 277 | 297 | 279 | 289 | 272 | 266 | 254 | 21
| ORB | 92 | 102 | 106 | 113 | 109 | 125 | 130 | 129 | 127 | 128 | 57
| AKAZE | 166 | 157 | 161 | 155 | 163 | 164 | 173 | 175 | 177 | 179 | 7.8
| SIFT | 138 | 132 | 124 | 137 | 134 | 140 | 137 | 148 | 159 | 137 | 5.6

### MP.7 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

Some combinations of detector and descriptor doesn't make sense, those results are N/A.

| Detector,Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| SHITOMASI | 686 |922|866|688|N/A|900|
| HARRIS | 138|169 |164|134|N/A|167|
| FAST | 638 |805|831|645|N/A|763|
| BRISK | **1426** | **1512** |1379|1386|N/A| **1529** |
| ORB | 678 |486|691|395|N/A|742|
| AKAZE | 1020 |1169|1096|1002|1199|1176|
| SIFT | 491 |695|N/A|492|N/A|759|

### MP.7 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Some combinations of detector and descriptor doesn't make sense, those results are N/A.

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| SHITOMASI| 17.98 |21.38|18.8|52.4079|N/A| 31.82|
| HARRIS | 13.28|14.50 |14.24|32.07| N/A| 22.31|
| FAST| **3.98** | **3.72** | **3.12** |22.91|N/A|14.72|
| BRISK| 34.84 |33.42|40.72|54.59|N/A|54.73|
| ORB| 5.82 |6.91|11.12|22.52|N/A|23.83|
| AKAZE| 51.55|53.93 |57.92|75.42|93.73|67.89|
| SIFT| 68.54 |88.18|N/A|113.33|N/A|137.67|

The top 3 detector/descriptor combinations are found by evaluating the tables above.

In terms of number of matched keypoints (More is better)
  1. BRISK/SIFT
  2. BRISK/BRIEF
  3. BRISK/BRISK

In terms of execution time (Small is better)
  1. FAST/ORB
  2. FAST/BRIEF
  3. FAST/BRISK


## 5. Ackowledgements <a name="acknowledgements"></a>
* [Udacity Sensor Fusion Program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

For any questions or feedback, feel free to email [moorissa.tjokro@columbia.edu](mailto:moorissa.tjokro@columbia.edu).


