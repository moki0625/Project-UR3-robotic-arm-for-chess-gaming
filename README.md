# Team04 Final Project


## Final work package
 
**The project is in package chess_robot, it consists with the following source codes:**
- sensing_node.cpp
- planning_node.cpp
- action_node.cpp
- chess_manager.cpp

**service files:**
- moveArm.srv
- pickChess.srv
- placeChess.srv
- chessCommand.srv
- getpiececellcode.srv
- getpiecepose.srv
- getOccupiedFields.srv

**launch file:**
  - chess_launch_all.launch
  
## Launch Instructions

### For executing a demo simulation do the following:

`[T1] export GAZEBO_MODEL_PATH=~/git-projects/final-work/chesslab_setup/models/:${GAZEBO_MODEL_PATH}`
 
`[T1] roslaunch team04_final_project chess_launch_all.launch`
 
1. Click on the Play Button in Gazebo  
1. In the rqt_gui choose for controller manager 'team_A_arm/controller_manager' and for the controller choose the 'joint_trajectory_controller'
1. Cloose the rqt_gui window
1. Press key to start a single movement, until it is required to send a command:

**To send a command,open a second terminal and type one of the following options:**

`[T2] rosservice call /chess_command "chesspiece: '206' field: 'B4'"`

This command will move chess piece 206 on cell B4

`[T2] rosservice call /chess_command "chesspiece: '206' field: 'C2'"`

This command will move chess piece 206 on cell C2 and kill the chess piece that is standing on it

### Removing the killed chess piece (only for kill move)

1. When asked, copy the following line to attach the chess piece to the gripper

`[T2] rosservice call /link_attacher_node/attach '{model_name_1: 'pawnW6', link_name_1: 'link', model_name_2: 'team_A_arm', link_name_2: 'team_A_gripper_left_follower'}'`

2. When the robot arrives in the safe zone, copy the following line to detach the chess piece from the gripper  

`[T2] rosservice call /link_attacher_node/detach '{model_name_1: 'pawnW6', link_name_1: 'link', model_name_2: 'team_A_arm', link_name_2: 'team_A_gripper_left_follower'}'`

### Placing the new chess piece

1. When asked, copy the following line to attach the chess piece to the gripper**

`[T2] rosservice call /link_attacher_node/attach '{model_name_1: 'pawnB6', link_name_1: 'link', model_name_2: 'team_A_arm', link_name_2: 'team_A_gripper_left_follower'}'`

2. When the robot arrives at the desired location, copy the following line to detach the chess piece from the gripper

 `[T2] rosservice call /link_attacher_node/detach '{model_name_1: 'pawnB6', link_name_1: 'link', model_name_2: 'team_A_arm', link_name_2: 'team_A_gripper_left_follower'}'`
