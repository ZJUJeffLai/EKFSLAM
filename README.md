#  EKF-SLAM
In lab for this week, you will implement SLAM both in
simulation as well as on the car. The first phase requires
that you implement a standard Extended Kalman Filter in
order to EKF-SLAM in simulation. The second phase of the
lab will require you to run a pre-built SLAM package using
ROS and the RP-LIDAR to map a hallway in Kemper. In Phase
2, you will work in your original car construction groups.


## Phase 1: EKF-SLAM with known Data Associations
For Phase 1, you will program a SLAM algorithm in a
simulated environment using an Extended Kalman Filter.
Your implementation will be made in C++.

### Setup
You can choose to do this lab locally or on the server,
however it is recommended you run it locally as you
will need to visualize a lot of things, and it can be
a hassle to setup X11 forwarding on the server.
The only dependency for this lab that needs to be installed
is Python2.7 with matplotlib and numpy. There is also a
dockerfile included with the project that installs these
dependencies for you. In the `docker` directory run
```
docker build -t eec193_lab5 .
```
After building the image you will need to setup X11
forwarding for your local machine. This will be dependent
on you computer's operating system. On OSX,
I had to do the following in terminal after installing
Xquartz.
```
xhost + 127.0.0.1
docker run -it -v `pwd`:/workspace -e DISPLAY==host.docker.internal:0 eec193_lab5 /bin/bash
```

These instructions should change depending on your OS.

### Project Structure
Your project consists of three folders, a data folder,
an include folder, and a src folder. The data folder
contains all the sensor data information for the
simulation.The include folder includes all public headers
necessary for the project. In this case, a public header
is any header file that is used in multiple .cpp files.
The src folder is where the majority of your programming
will happen. This is the directory that contains the actual
implementations for the interfaces you defined in the
headers. You also store private headers, ie headers that
are only used in one implementation file.

### Step 1: Reading in Sensor and Map Information
#### Reading in Sensor Information
The first step for this lab is to read in all the sensor
readings from the file sensor.dat. The file consists of
two types of sensor readings, `ODOMETRY` and `SENSOR`.
There is a struct in the file `sensor.h` called `record` that
consists of an `OdoReading` and a vector of
`LaserReading`. This represents all the sensor
observations at 1 time step. In this file there is also a
class called `MeasurementPackage`. This class consists of
a public `vector` of `Record`. This represents all the
measurements for each time step. The second is a public
function `initialize` that is used to read in the
information from the file `sensor.dat` and create the
`vector` of `Record`. This will allow you to have access
to all the sensor information throughout the rest of the
project.

Note: Typically if you have a lightweight implementation
for a class you can throw it directly into the header
rather than separating interface and implementation. This
is why we implement `MeasurementPackage` directly in
`sensor_info.h`.

#### Sensor.dat Structure
There are two possible types of data in `sensor.dat`. The
first is an odometry reading, and the other a sensor
reading. At every time step there will be one piece of
odometry data, and potentially multiple sensor readings.
As an example,
```
ODOMETRY 0.100692392654 0.100072845247 0.000171392857486
SENSOR 1 1.89645381418 0.374031885671
SENSOR 2 3.85367751107 1.51951017943
```
For the odometry reading, the first entry represents the
first rotation, the second entry represents the
first translation and the last entry represents the second
translation. For each sensor reading the first coordinate
represents the range of the observed landmark, and the
second entry represents the angle relative to the car.
`SENSOR 1` means the robot observed landmark 1. This is
how you can keep track of each landmark without
associating them at every step.  

#### World.dat Structure
The world.dat file contains the ground truth information
for all the simulated landmarks.
```
1 2 1
2 0 4
```
These are the first two entries in the world.dat file.
The first entry in each row is the landmark ID. The
last two entries represent the groundtruth (x,y) coordinate
of the landmark.

#### Reading in Map information
The map information in `world.dat` consists of the locations
of all simulated landmarks. The `mapper.h` file is where
we will read implement a class to read in data about the
world. This functions nearly exactly the same as our
`MeasurementPackage` class that we used to read in sensor
information. In this class, there is a `struct` called
`MapPoint` which holds all the information needed to read
in map data. The `Mapper` class is nearly the same as
the  `MeasurementPackage` class. You will need to
implement a file reader same as the `sensor_info.h` file.

### Step 2: Visualizing the Simulation
The next step of the lab requires plotting the landmarks,
the robot's position, and then the estimated states of
the robot's position as well as the estimations of the
position of each landmark. There are two utility functions
already provided in order to plot the Gaussians around
each landmark as well as the robot called DrawEllipse. In
the file `plotter.h` in the directory `include/helper`,
you will find the utility functions as well as a function
called `Plot_State`. You must fill this function out in
order to plot the state at each time step. In order to do
the visualizations we use the same matplotlib wrapper from
the first lab.  Using the p

### Step 3: Implementing the EKF
You have been provided with an `ekfslam.h` file that exists
within the `src` directory. This header file defines all
the functions needed in order to implement EKFSLAM, as well
as all the necessary matrices that need to be tracked. At
this point, we will start using the Eigen Library in order
to do all the matrix operations necessary to implement
the EKF-SLAM algorithm. Refer to [this](https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html)
tutorial from the official Eigen website to become more
familiar with the library. There are three main functions
that need to be implemented for the `EKFSLAM` class. The
constructor to initialize the mean and covariance
matrices/vectors, the prediction step, and the correction
step. We use a class here, as all the variables for the
class will be self-contained within it. A key thing to note,
is that we have the known data associations between time
steps for each landmarks. These are tracked by the landmark
IDs. So you will not need to do any data association for
the lab.

#### Constructor
In order to add noise to the model, we define a noise
motion constant in the constructor. It us up to you
as the programmer to both make the proper size for the
noise matrix Q, as well as add the noise due to motion.

#### Prediction
There are two separate interfaces defined for the
prediction step, one that uses odometry and one that uses
velocity. You MUST implement both, however, you will only
be provided with data for the odometry model. The velocity
implementation will be used later on, so it is necessary
that you implement it here. For the odometry information,
you will use the `MeasurementPackage` class from before.

Bonus: Implement a Prediction function that takes in both
sensor measurements to get a better prediction of the
motion.

#### Correction
In the correction step, you will use the `Mapper` class
combined with the `MeasurementPackage` in order to
update the mean and covariances for your EKF. One thing to
note is that observed landmarks is a vector of booleans.
This represents the IDs of the observed landmarks. This is
how we can guarantee that this lab has known
data associations for observed landmarks.

#### Calibrating the Noise Matrices
One very important thing to remember about the Kalman
Filter is that it is a tunable algorithm. One of the main
parameters that needs to be tuned is the modeling of
the noise matrices Q and R. This is something that you will
have to play with in order to find a good answer. As part
of this lab, you should try and find the optimal values
to model the sensor noise and motion noise. It is very
typical for these two values to be something you can't
just find. You have to go through the work of calibrating
your filter to find the optimal noise for each of your
noise and observation models. 


### Step 4: Create the Main file
This is the last step when coding the SLAM algorithm,
but you need to combine everything and run it in one
singular main file. You must loop through all available
sensor data you must plot the estimates of each state
at every time step. Your main file must take in two and
only two arguments, the filepath to the sensor data and
world data. Additionally, if the user does not input any
files or an incorrect number of arguments the program
should print usage instructions and terminate. If you
make your main file properly, you should see a
plot that evolves for each update step. Your final image
should look like the following.
![Final Output](images/2019/02/Screen Shot 2019-02-15 at 4.46.38 PM.png)

### Step 5: Makefile
You need to create a Makefile in order to compile your
code. It doesn't need to be complex but it does need to
be functional. It should consist of two rules. The first
should built the project and spit out an executable called
`EKFSLAM`. The second should be a standard clean command
that removes an intermediate executables. Your Makefile
should output all executables into a directory called
`bin`. You will be provided with a skeleton Makefile that
you must properly fill out according to the specifications
outlined above.

### Phase 1 Submission Details
Your Phase 1 submission should be a zip file that includes
the following.
- The include directory holding all necessary headers
- A src directory with the files `ekfslam.cpp`, `ekfslam.h` and `main.cpp`
- A Makefile that compiles your program using g++

Your code needs to be commented and styled
according to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
I recommend downloading a linter in order to ensure your
code follows style. In terms of commenting, please make
sure that each function you have has well-defined inputs
and outputs. Each function should have a comment with
the following.
- A description of what the function is doing
- Its Inputs
- Its Outputs (if necessary)

I also recommend commenting on code you feel is too
verbose or confusing. If there is a piece of code that
you have written and the TA's can not understand what you
are doing then it is safe to say it needs a comment. You
will be graded on this.

### Phase 1 Grading Breakdown
- EKF-SLAM Implementation (Odometry): 50 points
- EKF-SLAM Implementation (Velocity): 20 points
- Makefile: 10 points
- Style and Comments: 10 points
- Interactive Grading: 10 points
