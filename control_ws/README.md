This workspace contains the controllers for the robot.  

1. Differential Drive Controller  

  The 'diff_drive' package implements a differential drive controller using 'ros2_controllers'. The controller is based on [Example 2](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2) from 'ros2_control_demos'.
  Additionally, modifications to the example were guided by this [YouTube tutorial](https://www.youtube.com/watch?v=J02jEKawE5U&t=420s), which references the following repository: [diffdrive_arduino (humble)](https://github.com/joshnewans/diffdrive_arduino/tree/humble).  
  
  Features  
  - Controls the motors using input from a keyboard or a PS4 controller.  
  - Receives feedback from motor encoders for precise movement.  
  - Runs on a microcontroller using Micro-ROS.  
  - PlatformIO is used for Micro-ROS development.  
  
  Micro-ROS Setup  
    The Micro-ROS workspace ('micro_ros_ws') was set up following the instructions from this [YouTube video](https://www.youtube.com/watch?v=Nf7HP9y6Ovo).

  Joystick Control
    The joystick control implementation was based on this [YouTube tutorial](https://www.youtube.com/watch?v=F5XlNiCKbrY&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=16).  
