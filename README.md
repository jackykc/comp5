# Jacky Chung and Yi Zhang
## Purpose
In this competition we demonstrate how to do box pushing using the turtlebot, we also used techniques from previous competitions, including ARTag recognition, mapping, navigation, and an additional webcam for line following. A majority of this is the same from [competition 3](https://github.com/jackykc/comp3/blob/master/README.md). The new task introduced is to push a box with ARTag into a parking spot marked by a different ARTag.

## Prerequisites
* Kobuki Turtlebot with an Asus Xtion Pro and a usb camera
* Ubuntu 16.04
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Desktop or Desktop-Full)
* [Turtlebot](http://wiki.ros.org/action/show/Robots/TurtleBot), [Kobuki Packages](http://wiki.ros.org/kobuki), [camera](http://wiki.ros.org/openni_camera), [cv_camera](http://wiki.ros.org/cv_camera), and [navigation](https://github.com/ros-planning/navigation) 
  ```
  sudo apt-get install ros-kinetic-turtlebot
  sudo apt-get install ros-kinetic-kobuki
  sudo apt-get install ros-kinetic-kobuki-core
  sudo apt-get install ros-kinetic-openni2-camera
  sudo apt-get install ros-kinetic-openni2-launch
  sudo apt-get install ros-kinetic-cv-camera
  sudo apt-get install ros-kinetic-navigation
  ```
## Resources used
HSV Thresholding (From Competition 2/3)
https://docs.opencv.org/3.1.0/df/d9d/tutorial_py_colorspaces.html

Contour Detection (From Competition 2/3)
https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html

AR Tracking 
http://wiki.ros.org/ar_track_alvar

## Execution:
[Physical setup of the robot is located at the bottom of the README](https://github.com/jackykc/comp5/blob/master/README.md#setting-up-the-robot)
1. Build and source setup.bash
   ```
   (In catkin_ws directory after cloning repo into catkin_ws/src)
   catkin_make
   source ./devel/setup.bash
   ```
1. Connect to the Kobuki, Asus Xtion Pro and usb camera
1. Place Kobuki on race track with usb camera looking at the track, facing downwards
1. Startup all required nodes
   `roslaunch comp5 all.launch (will launch the below nodes)`
   * kobuki base
   * amcl, map server, move base
   * ar_track_alvar
   
   `roslaunch comp5 3dsensor.launch (launch the 3d sensors separately due to connection issues)`
   
   `roslaunch comp5 cam_bottom.launch device_id:=${} (use the device id to set the usb camera)`
1. Start the competition five node `roslaunch comp5 comp5.launch`

## Concepts and code

* State Machine
![alt text](https://raw.githubusercontent.com/jackykc/comp3/master/comp3sm.png)

The state machine for competition four is as shown in the above. For competition five, although the states for 'TASK1', 'TASK2', and 'TASK3' exists in the state machine, we never get into those states. Competition five only enter the states 'GO', 'STOP', and 'TASK4'

## Competition Four
```
1. In the GO state, the turtlebot does line tracking.
2. Upon seeing a red line, we move to a stop state, incrementing 
   the number of stops it has seen
3. After stopping, it will then move onto one of the four tasks
   or the finish state after the last red line has been seen
4. Task four occurs before task three.
```
(With the exception of Task 4, the rest are kept the [same](https://github.com/jackykc/comp3/blob/master/README.md))
* GO (Line Tracking)
  1. Convert each frame into HSV format.
  1. Use opencv’s inRange function to filter out the desired color.
  1. Crop out the frame’s most top part.
  1. Use opencv’s moments function to to find the centroid of color in each frame. 
  1. Calculate the distance between color regin and the center of the frame which is the error that the turtlebot is off from     the course.
  1. Use P control to convert the error to how much the turtlebot needs to turn.
* Task 1 (Count objects with a red bottom at location 1)
  1. Threshold to keep only the red pixels of the image
  1. Blur threshold image
  1. Find and count contours larger than a specified minimum size from blurred image
* Task 2 (Count objects at location 2 and determine green shape)
  1. Create two thresholded images, one for red and one for green
  1. Find contours bigger than a specified minimum size for both images
  1. Count the returned contours
  1. Get the shape of the green contour using the number of sides of the contour
* Task 3 (Find shape from location 3 that matches the green one from location 2)
  * For each of the stop lines in task three:
    1. Threshold to keep only the red pixels of the image
    1. Blur thresholded image
    1. Get the shape of the largest red contour using the number of sides
    1. Match the shape with the one obtained from task two
* Task 4 (Visit each parking spot’s front, signaling if the parking spot or the box on it has an ARTag, or the same shape as that of location two)
  1. Set current pose of the robot for the amcl node (This corner of the room has already been mapped)
  1. Visit each of the parking spots’ front that has been waypointed before
  1. Check each spot and signal for an ARTag or shape
      * If /ar_pose_marker is publishing an AR tag at current parking spot, we have found the ARTag
      * Use same method from task 3 to find the parking spot with the correct shape
      * For competition four, box pushing was not run

### Code explanations (Only task four differs)
[Competition Two code explanations included here for the tasks one, two, and three](https://github.com/jackykc/comp2/blob/master/README.md#code-explanations)

[Competition Three code explanations included here for the color detection of task four]
(https://github.com/jackykc/comp3#code-explanations-only-task-four-differs)

Once we waypoint into the [middle of location four using waypoint](https://github.com/jackykc/comp5/blob/master/src/comp5.py#L677), we then go through each box that can have a colored object
``` python
# start colored object detection
callback_state = 4
# go through each of the parking spots with shapes
for index, pose in enumerate(waypoints_color):
    goal = goal_pose(pose)
    client.send_goal(goal)
    client.wait_for_result()

    # reset the shape of the current parking spot
    is_shape = False
    shape_id_counts["task4"][0] = 0
    shape_id_counts["task4"][1] = 0
    shape_id_counts["task4"][2] = 0
    wait_time = rospy.Time.now() + rospy.Duration(2)
    while rospy.Time.now()<wait_time:
        display_led(0)
        if (numpy.sum(shape_id_counts["task4"]) != 0):
            current_shape = get_shape(numpy.argmax(shape_id_counts["task4"]))
            rospy.loginfo(current_shape)
            if chosen_shape == current_shape:
                is_shape = True
    # signal if shape is found
    if is_shape:
        wait_time = rospy.Time.now() + rospy.Duration(0.5)
        while rospy.Time.now() < wait_time:
            sound_pub.publish(Sound(0))
            display_led(4)
```


### Competition Five
* Competition strategy
With each task being worth different points, the goal was to maximize the total points with multiple runs within ten minutes. Since Box pushing was worth more points than the other tasks, we decided to only do the box pushing task and ignore everything else to save time.

```
1. In the GO state, the turtlebot does line tracking.
2. Upon seeing a full red line, one that extends on both sides of the white line, we move to a stop state, incrementing 
   the number of stops it has seen
3. At the correct stop count, it will either move onto task four or signal a lap has been completed
```
* GO (Line Tracking, same as competition four)
  1. Convert each frame into HSV format.
  1. Use opencv’s inRange function to filter out the desired color.
  1. Crop out the frame’s most top part.
  1. Use opencv’s moments function to to find the centroid of color in each frame. 
  1. Calculate the distance between color regin and the center of the frame which is the error that the turtlebot is off from     the course.
  1. Use P control to convert the error to how much the turtlebot needs to turn.
* Task 4 (Visit each parking spot’s front, signaling if the parking spot or the box on it has an ARTag)
  1. Set current pose of the robot for the amcl node (This corner of the room has already been mapped)
  1. Visit each of the parking spots’ front that has been waypointed before
  1. Check each spot and signal for an ARTag on box and behind the parking spot
      * If /ar_pose_marker is publishing an AR tag at current parking spot, we have found the ARTag
      * If the id's of the markers for both the box and the stand has been found, (2 or for the stand, 1 for the box), stop searching
  1. Go the left or right of the parking spot infront of the box, based on the push direction 
  1. Go foward to the parking spot beside the box and orient robot for a foward push toward ar tag parking spot
  1. Push the box to ar tag parking spot
  1. Exit location four and head to finish line
* Repeat until 10 minutes run out
  
### Code explanations
Once we waypoint into the [middle of location four using waypoint](https://github.com/jackykc/comp5/blob/master/src/comp5.py#L677), we start the [box pushing task]((https://github.com/jackykc/comp5/blob/master/src/comp5.py#L709))
``` python
# go through each waypoint right infront of parking spot, facing ar tags
for index, pose in enumerate(waypoints_ar_tag):
    if (box_pos is not None) and (stand_pos is not None):
        break

    goal = goal_pose(pose)
    ... # go to goal
    ...
    
    if current_marker_pose is not None: # ar tag found
        # set the index of the waypoints that contain the ar tags
        if current_marker_id == 1:
            box_pos = index
        elif (current_marker_id == 2) or (current_marker_id == 3):
            stand_pos = index

        if current_marker_pose:
            ar_detected = True

        ... # signal ar tag found
        ...

# now both ar tags have been found
if (box_pos is not None) and (stand_pos is not None):
    difference = box_pos - stand_pos
    # negative difference means that the box is to the left
    # set the correct list of waypoints that orients the robot facing the push direction
    if difference < 0:
        push_from_pos = box_pos - 1
        temp_waypoints = waypoints_left
    else:
        push_from_pos = box_pos + 1
        temp_waypoints = waypoints_right        
    # from the waypoints used to search for the box,
    # go to one that is to the left or right of the box depending on which way we push
    goal = goal_pose(waypoints_ar_tag[push_from_pos])
    client.send_goal(goal)
    client.wait_for_result()

    # push spot beside the box, facing box for a straight push to parking spot 
    goal = goal_pose(temp_waypoints[push_from_pos])
    client.send_goal(goal)
    client.wait_for_result()

    # push the box
    twist = Twist()
    twist.linear.x = 0.29
    twist.angular.z = 0
    ...
    ...
    while rospy.Time.now() < wait_time:
        cmd_vel_pub.publish(twist)

```
Video:
https://youtu.be/mtUz5HRYZk0

## Setting up the robot
1. Starting from the bottom, attach four 50mm poles to the middle plate and the base
1. Attach four 203mm poles onto the middle plate.
1. Attach four 50mm, one onto each of the 203mm poles to extend their length
1. Attach the 3D sensor pole as well as the Asus Xtion Pro plate and XtionPro near the back of the robot facing fowards
1. Attach the top plate ontop of the 203+50mm poles, with the large openning for wires to go through towards the front of the robot
1. Put the box pushing foam between the middle plate and the base infront of the robot. Use tape if the foam position changes when moving the robot.
1. Mount the usb camera on the top plate as shown in photos, use tape to keep it secure. The camera view should be facing down, just above the point where it cannot see the foam. Use `roslaunch comp5 cam_bottom.launch device_id=${} (use the device id to set the usb camera)` and `rqt` to check the image
1. Place you laptop on the top plate, securing with a seatbelt and plugging in the usb's.

For placement of the poles, foam, and XtionPro:
* [Left side view](https://raw.githubusercontent.com/jackykc/comp5/master/resource/a.jpg)
* [Right side view](https://raw.githubusercontent.com/jackykc/comp5/master/resource/d.jpg)
* [Front view](https://raw.githubusercontent.com/jackykc/comp5/master/resource/b.jpg)
* [Back view](https://raw.githubusercontent.com/jackykc/comp5/master/resource/c.jpg)

For placement of the usb camera:
* [Side](https://raw.githubusercontent.com/jackykc/comp5/master/resource/e.jpg)
* [Under](https://raw.githubusercontent.com/jackykc/comp5/master/resource/f.jpg)

