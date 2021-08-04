# log-to-las

This program takes the log files from [this project](https://github.com/amook-maxtc/pi-controller) and outputs one las file.

## Program Flow

[Pogram Flow](docs/pp_workflow.PNG)

The three log files taken in are logs from the IMU, GPS, and Lidar. Each point is first adjusted for its rotation, transforming it from the vehicle frame to the local frame. 
Taken from the [u-blox integration maunal](https://cdn.sparkfun.com/assets/learn_tutorials/1/1/7/2/ZED-F9R_Integrationmanual__UBX-20039643_.pdf):

## Rotation

[Frames](docs/frames.PNG)

[Rotation matrix](docs/zedf9rRotation.PNG)

That rotation matrix is used, along with the pitch, roll, and heading from the .imulog file. 
In the future, [pyquaternion](http://kieranwynn.github.io/pyquaternion/) will be used to apply [slerp](https://en.wikipedia.org/wiki/Slerp), Spherical Linear Interpolation, allowing to find the approximate rotation inbetween imu data points (points are recorded much faster than imu is recorded, so there are many lidar points between two imu data points). 

## GPS Translation

Now that every point is using a common frame, each point is translated using the gps data. The first point in the input las is the base point. Each point's x,y,z coordinates are determined by finding the distance from the base point. The ZED-F9R module uses the WSG-84 model of the earth, which is an ellipsoidal model--not a perfect sphere. So, finding the displacement between two latitude and longitude values is non-trivial, and depends on where in the world the data is. Here is the mathematics behind finding the displacement:

[Reference 1](docs/web_GPS_globe.gif)

[Reference 2](docs/web_GPS_ellipse.gif)

[Labeled ref](docs/ellipse_math_labeled.png)

[Math Derivation](docs/latlongderivation.PNG)

## Important links

Here are some useful links used during development:

1.) [pylas](https://pylas.readthedocs.io/en/latest/index.html)
2.) [pyquaternion](http://kieranwynn.github.io/pyquaternion/)
3.) [wsg-84 math](https://adamchukpa.mcgill.ca/web_ssm/web_GPS_eq.html)
4.) [wsg-84 math 2](https://www.quora.com/How-do-I-find-the-radius-of-an-ellipse-at-a-given-angle-to-its-axis)
5.) [Test to see if GPS translation functioned](https://www.meridianoutpost.com/resources/etools/calculators/calculator-latitude-longitude-distance.php?)
