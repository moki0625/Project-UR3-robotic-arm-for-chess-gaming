#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <team04_final_project/moveArm.h>
#include <team04_final_project/pickChess.h>
#include <team04_final_project/placeChess.h>

// #include <gazebo_ros_link_attacher/Attach.h>



int trajpoints;
int arm_joint_number = 6;
int grip_joint_number = 1;

double time_tolerance = 1;
double open_grip = 0.5;   // gripper position
double close_grip = 0.6;  // gripper position
double max_effort = 5;    // gripper effort

bool constrained;
bool out_of_frame;
double robot_x;
double robot_y;
double robot_z;


control_msgs::FollowJointTrajectoryGoal goalTraj; // store goal trajectory
control_msgs::GripperCommandGoal goalGrip;        // store gripper goal
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *robotClientArm;

std::vector<double> currentjoints;
std::vector<double> currentjoints_grip;

std::vector<float> set_limits;
std::vector<double> joint_state; // store joint state

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *robotClient;

// header of move arm trajectory function
//typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ArmClientType;
//bool moveArmTrajectory(control_msgs::FollowJointTrajectoryGoal goal, double trajduration, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *rClient);

// header of move gripper trajectory function
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripClientType;
bool moveGripTrajectory(control_msgs::GripperCommandGoal goal, actionlib::SimpleActionClient<control_msgs::GripperCommandAction> &rClient);



/////////////////////////////////////////////////////////////////////////////////
///////////////////////    Arm control action   /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: error_code is %d", result->error_code);

}

//Callback function: Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

//Callback function: Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  //ROS_INFO("************: desired angles are (%f,%f)", feedback->desired.positions[0], feedback->desired.positions[1]);
  ROS_INFO("Got Feedback: current positions are (%f,%f,%f,%f,%f,%f)", feedback->actual.positions[0], feedback->actual.positions[1],
                                                                      feedback->actual.positions[2], feedback->actual.positions[3],
                                                                      feedback->actual.positions[4], feedback->actual.positions[5]);
}



//Sends the goal to the FollowJointTrajectory action server and waits for the result for trajduration seconds
//If not able to reach the goal within timeout, it is cancelled
bool moveArmTrajectory(control_msgs::FollowJointTrajectoryGoal goal, double trajduration, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *rClient)
{

  //ROS_DEBUG("Before sendGoal");
  //Set timestamp and send goal
  goal.trajectory.header.stamp = ros::Time::now();
  //rClient->sendGoal(goal);
  rClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  //Wait for the action to return. Timeout set to the trajduration plus the goal time tolerance)
  bool finished_before_timeout = rClient->waitForResult(ros::Duration(trajduration)+ goal.goal_time_tolerance);

  //Get final state
  actionlib::SimpleClientGoalState state = rClient->getState();
  if (finished_before_timeout) {
      //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
      //Reports SUCCEEDED if finished and goal reached
      ROS_INFO(" ***************** Robot action finished: %s  *****************",state.toString().c_str());
  } else {
      //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
      ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
      //Preempting task
      ROS_ERROR("I am going to preempt the task...");
      rClient->cancelGoal();
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////////
////////////////////////  Gripper control action  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//Callback function: Called once when the goal completes
void doneCb_grip(const actionlib::SimpleClientGoalState& state,
            const control_msgs::GripperCommandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  std::stringstream result_message;
  result_message << "Gripper Motion Result: Position = " << result->position << ", Effort = "<< result->effort<<std::endl;
  if(result->stalled) result_message << " - STALLED ";
  if(result->reached_goal) result_message << " - REACHED GOAL ";
  ROS_INFO_STREAM(result_message.str());

}

//Callback function: Called once when the goal becomes active
void activeCb_grip()
{
  ROS_INFO("Goal just went active");
}

//Callback function: Called every time feedback is received for the goal
void feedbackCb_grip(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
{
  std::stringstream feedback_message;
  feedback_message << "Got Feedback of gripper: Position = " << feedback->position << ", Effort = "<< feedback->effort<<std::endl;
  if(feedback->stalled) feedback_message << " - STALLED ";
  if(feedback->reached_goal) feedback_message << " - REACHED GOAL ";
  ROS_INFO_STREAM(feedback_message.str());
}

//Sends the goal to the GripperCommandAction action server
//If not able to reach the goal within timeout, it is cancelled
bool moveGripTrajectory(control_msgs::GripperCommandGoal goal, actionlib::SimpleActionClient<control_msgs::GripperCommandAction> &rClient)
{
    ROS_INFO("Moving gripper ");

    //Print the goal to be achieved
    // ROS_INFO("currentjoint: %f", currentjoints_grip);
    ROS_INFO("gripper command position: %f", goalGrip.command.position);
    ROS_INFO("gripper command max_effort: %f", goalGrip.command.max_effort);

    //Send goal
    rClient.sendGoal(goalGrip, &doneCb_grip, &activeCb_grip, &feedbackCb_grip);

    //Wait for the action to return. Timeout set to the trajduration
    bool finished_before_timeout = rClient.waitForResult(); //no finite timeout

    //Get final state
    actionlib::SimpleClientGoalState state = rClient.getState();
    if (finished_before_timeout) {
        //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
        //Reports SUCCEEDED if finished and goal reached
        ROS_INFO(" ***************** Gripper action finished: %s  *****************",state.toString().c_str());
    } else {
        //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
        ROS_ERROR("Gripper action did not finish before the timeout: %s",
                state.toString().c_str());
        //Preempting task
        ROS_ERROR("I am going to preempt the task...");
        rClient.cancelGoal();
    }
    return finished_before_timeout;
}




/////////////////////////////////////////////////////////////////////////////////
///////////////////////////  main control action  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// Callback function to move arm
bool moveArm(
  team04_final_project::moveArm::Request &req,
  team04_final_project::moveArm::Response &resp){

    ROS_INFO_STREAM("-----------------MOVE ARM SERVICE----------------------");
    //currentjoints.resize(6);

    //Set trajectory names
    goalTraj.trajectory.joint_names.resize(6);
    goalTraj.trajectory.joint_names[0] = "team_A_shoulder_pan_joint";
    //goalTraj.trajectory.joint_names[0] = "team_A_elbow_joint";
    goalTraj.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
    goalTraj.trajectory.joint_names[2] = "team_A_elbow_joint";
    //goalTraj.trajectory.joint_names[2] = "team_A_shoulder_pan_joint";
    goalTraj.trajectory.joint_names[3] = "team_A_wrist_1_joint";
    goalTraj.trajectory.joint_names[4] = "team_A_wrist_2_joint";
    goalTraj.trajectory.joint_names[5] = "team_A_wrist_3_joint";


      ROS_INFO_STREAM("Set trajectory to "<<req.goalPoint.positions[0]);
      ROS_INFO_STREAM("Set trajectory to "<<req.goalPoint.positions[1]);
      ROS_INFO_STREAM("Set trajectory to "<<req.goalPoint.positions[2]);
      ROS_INFO_STREAM("Set trajectory to "<<req.goalPoint.positions[3]);
      ROS_INFO_STREAM("Set trajectory to "<<req.goalPoint.positions[4]);
      ROS_INFO_STREAM("Set trajectory to "<<req.goalPoint.positions[5]);

      //Set goal trajectory
      goalTraj.trajectory.points.resize(1);
      goalTraj.trajectory.points[0].positions.resize(6);

      // Set the start position of the joints
      goalTraj.trajectory.points[0].positions[0] = req.goalPoint.positions[0];
      goalTraj.trajectory.points[0].positions[1] = req.goalPoint.positions[1];
      goalTraj.trajectory.points[0].positions[2] = req.goalPoint.positions[2];
      goalTraj.trajectory.points[0].positions[3] = req.goalPoint.positions[3];
      goalTraj.trajectory.points[0].positions[4] = req.goalPoint.positions[4];
      goalTraj.trajectory.points[0].positions[5] = req.goalPoint.positions[5];

      goalTraj.trajectory.points[0].time_from_start = ros::Duration(10);


    ROS_INFO_STREAM("----------------------------------------Set trajectory done" << goalTraj);


    moveArmTrajectory(goalTraj, req.trajduration, robotClientArm);
    ROS_INFO_STREAM("-------------------------------------------Move arm done");


    return true;
  }


// Callback function to pick a chess
bool pickChess(
  team04_final_project::pickChess::Request &req,
  team04_final_project::pickChess::Response &resp){

    ros::Rate pause(5);
/*
    ArmClientType robotClientArm("team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
    if(!robotClientArm.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR(" *** action server not available ***ros::spin(); ");
    };
*/
    GripClientType robotClientGripper("team_A_arm/gripper/gripper_controller/gripper_cmd");
    if(!robotClientGripper.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR(" *** action server not available *** ");
    };

    currentjoints.resize(6);




//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   open gripper     /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
    // set goal command
    goalGrip.command.position = open_grip;
    goalGrip.command.max_effort = max_effort;
    //Call function to send goal
    moveGripTrajectory(goalGrip, robotClientGripper);
    ROS_INFO("Open gripper down");
    pause.sleep();
    pause.sleep();

/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////       move arm down       ////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

      //Set trajectory names
      goalTraj.trajectory.joint_names.resize(6);
      goalTraj.trajectory.joint_names[0] = "team_A_elbow_joint";
      goalTraj.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
      goalTraj.trajectory.joint_names[2] = "team_A_shoulder_pan_joint";
      goalTraj.trajectory.joint_names[3] = "team_A_wrist_1_joint";
      goalTraj.trajectory.joint_names[4] = "team_A_wrist_2_joint";
      goalTraj.trajectory.joint_names[5] = "team_A_wrist_3_joint";


      //Set goal trajectory
      goalTraj.trajectory.points.resize(1);
      goalTraj.trajectory.points[0].positions.resize(6);



      // Set the start position of the joints
      goalTraj.trajectory.points[0].positions[0] = req.graspConfig.positions[0];
      goalTraj.trajectory.points[0].positions[1] = req.graspConfig.positions[1];
      goalTraj.trajectory.points[0].positions[2] = req.graspConfig.positions[2];
      goalTraj.trajectory.points[0].positions[3] = req.graspConfig.positions[3];
      goalTraj.trajectory.points[0].positions[4] = req.graspConfig.positions[4];
      goalTraj.trajectory.points[0].positions[5] = req.graspConfig.positions[5];

      goalTraj.trajectory.points[0].time_from_start = ros::Duration(10);
        // for (int i = 0; i < req.joint_names.size(); i++){
        //     goalTraj.trajectory.joint_names[i] = req.joint_names[i];
        //   }

      //Call function to send goal
      moveArmTrajectory(goalTraj, req.trajduration, robotClientArm);
      ROS_INFO("Arm down done");
      pause.sleep();

////////////////////////////////////////////////////////////////////////////////////
/////////////////////////       pick chess       ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
      // set goal command
      goalGrip.command.position = close_grip;
      goalGrip.command.max_effort = max_effort;
      //Call function to send goal
      moveGripTrajectory(goalGrip, robotClientGripper);

      std::cout<<"\nUSE SERVICE TO LINK THE CHESS, PRESS A KEY TO CONTINUE..."<<std::endl;
      std::cin.get();
      std::cin.get();

      ROS_INFO("Pick chess done");

      pause.sleep();


/////////////////////////////////////////////////////////////////////////////////////
////////////////////////         move arm up            /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

      // Set the start position of the joints
      goalTraj.trajectory.points[0].positions[0] = req.pregraspConfig.positions[0];
      goalTraj.trajectory.points[0].positions[1] = req.pregraspConfig.positions[1];
      goalTraj.trajectory.points[0].positions[2] = req.pregraspConfig.positions[2];
      goalTraj.trajectory.points[0].positions[3] = req.pregraspConfig.positions[3];
      goalTraj.trajectory.points[0].positions[4] = req.pregraspConfig.positions[4];
      goalTraj.trajectory.points[0].positions[5] = req.pregraspConfig.positions[5];

      goalTraj.trajectory.points[0].time_from_start = ros::Duration(10);
        // for (int i = 0; i < req.joint_names.size(); i++){
        //     goalTraj.trajectory.joint_names[i] = req.joint_names[i];
        //   }


    //Call function to send goal
    moveArmTrajectory(goalTraj, req.trajduration, robotClientArm);
    ROS_INFO("Arm up done");

    return true;
}


// Callback function to place a chess
bool placeChess(
  team04_final_project::placeChess::Request &req,
  team04_final_project::placeChess::Response &resp){


    ros::Rate pause(2);
/*
    ArmClientType robotClientArm("team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
    if(!robotClientArm.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR(" *** action server not available *** ");
    };
*/
    GripClientType robotClientGripper("team_A_arm/gripper/gripper_controller/gripper_cmd");
    if(!robotClientGripper.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR(" *** action server not available *** ");
    };

    // currentjoints.resize(6);


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////         arm down          /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
    //Set trajectory names
    goalTraj.trajectory.joint_names.resize(6);
    goalTraj.trajectory.joint_names[0] = "team_A_shoulder_pan_joint";
    goalTraj.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
    goalTraj.trajectory.joint_names[2] = "team_A_elbow_joint";
    goalTraj.trajectory.joint_names[3] = "team_A_wrist_1_joint";
    goalTraj.trajectory.joint_names[4] = "team_A_wrist_2_joint";
    goalTraj.trajectory.joint_names[5] = "team_A_wrist_3_joint";


    // Set the start position of the joints
    goalTraj.trajectory.points.resize(1);
    goalTraj.trajectory.points[0].positions.resize(6);
    goalTraj.trajectory.points[0].positions[0] = req.placeConfig.positions[0];
    goalTraj.trajectory.points[0].positions[1] = req.placeConfig.positions[1];
    goalTraj.trajectory.points[0].positions[2] = req.placeConfig.positions[2];
    goalTraj.trajectory.points[0].positions[3] = req.placeConfig.positions[3];
    goalTraj.trajectory.points[0].positions[4] = req.placeConfig.positions[4];
    goalTraj.trajectory.points[0].positions[5] = req.placeConfig.positions[5];

    goalTraj.trajectory.points[0].time_from_start = ros::Duration(10);

    //Call function to send goal
    moveArmTrajectory(goalTraj, req.trajduration, robotClientArm);
    ROS_INFO("Arm down done");
    pause.sleep();


////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        place chess      //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

    std::cout<<"\nUSE SERVICE TO UNLINK THE CHESS, PRESS A KEY TO CONTINUE..."<<std::endl;
    std::cin.get();
    std::cin.get();

    // set goal command
    goalGrip.command.position = open_grip;
    goalGrip.command.max_effort = max_effort;
    //Call function to send goal
    moveGripTrajectory(goalGrip, robotClientGripper);

    ROS_INFO("Place chess done");

    pause.sleep();
    pause.sleep();


/////////////////////////////////////////////////////////////////////////////////
///////////////////////         arm up            ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
    // Set the start position of the joints
    goalTraj.trajectory.points[0].positions[0] = req.placeupConfig.positions[0];
    goalTraj.trajectory.points[0].positions[1] = req.placeupConfig.positions[1];
    goalTraj.trajectory.points[0].positions[2] = req.placeupConfig.positions[2];
    goalTraj.trajectory.points[0].positions[3] = req.placeupConfig.positions[3];
    goalTraj.trajectory.points[0].positions[4] = req.placeupConfig.positions[4];
    goalTraj.trajectory.points[0].positions[5] = req.placeupConfig.positions[5];

    goalTraj.trajectory.points[0].time_from_start = ros::Duration(10);
      // for (int i = 0; i < req.joint_names.size(); i++){
      //     goalTraj.trajectory.joint_names[i] = req.joint_names[i];
      //   }

    //Call function to send goal
    moveArmTrajectory(goalTraj, req.trajduration, robotClientArm);
    ROS_INFO("Arm up down");

    return true;
}



int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "action_module");
  ros::NodeHandle nh;


  robotClientArm = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
  //ArmClientType robotClientArm("team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
  if(!robotClientArm->waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR(" *** action server not available(action module error) *** ");
  };
  //


  // Define the services
  ROS_INFO("**** start robot_controller node (action module) ****");

  ros::ServiceServer server1 = nh.advertiseService("team04_final_project/move_arm", &moveArm);
  ros::ServiceServer server2 = nh.advertiseService("team04_final_project/pick_chess",&pickChess);
  ros::ServiceServer server3 = nh.advertiseService("team04_final_project/place_chess",&placeChess);

  // Subscribe to the current joint state.
  // ros::Subscriber sub = nh.subscribe("/team_A_arm/joint_states", 1000, &currentJointStates);
  ROS_INFO("**** END robot_controller node (action module) ****");

  ros::spin();

  return 0;
  }
