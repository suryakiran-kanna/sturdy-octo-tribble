# sturdy-octo-tribble
Project 6 Team 1 - EE5900 Intro to Robotics

## Development steps

Connect to jackal-3 through ssh and launch usb cam node
```
$ ssh rsestudent@jackal4  (password:rsestudent)
$ cd sturdy-octo-tribble/lab_six_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch obj_track usb-cam_test.launch
``` 

After cloning this repository into local workspace, follow these steps
```
$ cd lab_six_ws
$ catkin_make
$ source devel/setup.bash
$ source remote-jackal.sh
$ rosrun obj_track track.launch
```

