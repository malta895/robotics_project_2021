# Robotics Project 2019/2020 | First delivery

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

The custom message that displays the distance is located at:  
`src/distance_calculator/msg/Distance.msg`  
and it's defined as follows:  

    string flag 
    float64 distance

As required by project specifications, a `nan` distance is published when GPS signal is absent in at least one of the received sources.
In this case the flag reports the message "NotAvailable".

## Other infos
