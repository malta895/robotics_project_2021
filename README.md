# Robotics Project 2020/2021 | First delivery

| Name             | Polimi ID | Email                           |
| ---              | ----      | ---                             |
| Luca Maltagliati | 10492601  | luca.maltagliati@mail.polimi.it |

## Description of the files and nodes

Most important files include:  

* `launch/project1.launch`: A launch file which launches every necessary node to comply with project specifications.
<!-- TODO continue -->
## Name of the parameters

The following parameters are used to set the start point of the object and to set the base name of the topic:  

* `init_pose_x`
* `init_pose_y`
* `init_pose_theta`


All the parameters mentioned above are set in the `project1.launch` file.

## Description of how to start/use the nodes

From a terminal in the root folder of the project:  

    catkin_make
    source devel/setup.bash
    roslaunch launch/project1.launch
    
## Structure of the *tf* tree

See the file `frames.pdf`

## Structure of the custom message

As per the project specification, the custom message is structured as follows:

```
nav_msgs/Odometry odom
std_msgs/String method
```

The field `method` can either have the value "Euler" or "Runge-Kutta", according to the method used to integrate the odometry.

The message is published under the topic `/scout_integrated_odom_custom`

## Other infos

I estimated the `apparent baseline` and `gear ratio` values from both the odometry and ground truth pose. For this purpose I created a dedicated node, `parameters_calibrator`. The node, by using message filters, calculates for each syncronization step the value of such parameters, and then it incrementally calculates the mean in order to get a value as close as possible to the real one. In order to get a cleaner estimation I imposed bounds on the values that could contribute to the mean, with the help of plotting tools like `rviz`.

Then, by observing the plotted odometries, I verified that the values calculated starting from the `/scout_odom`are accurate: the resulting odometry is practically overlapped to the reference one; on the other hand, the odometry resulting from the parameters estimated from the ground truth pose diverges a lot from the reference one, and its track follows more or less the one of the `/gt_pose`. This is expected because of the lack of precision of the odometry measured by the robot itself, as suggested during the project presentation.


