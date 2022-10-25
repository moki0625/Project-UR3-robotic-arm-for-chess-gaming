#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
//#include <cmath>
// add header
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <team04_final_project/getpiecepose.h>
#include <team04_final_project/transfercoordinate.h>
#include <team04_final_project/pregraspToBase.h>
#include <team04_final_project/optik.h>
#include <sensor_msgs/JointState.h>





using namespace std;

std::vector<double> currentjoints;

//geometry_msgs::TransformStamped ts_aruco_201;
//geometry_msgs::TransformStamped ts_aruco_582;


// CallBack function
bool getPiecepose(
  team04_final_project::getpiecepose::Request &req,
  team04_final_project::getpiecepose::Response &resp){
    ros::NodeHandle n;
    ros::Rate rate(10.0);

    ROS_INFO_STREAM("Get the piece pose of "<<req.id);

    string pose_id = "aruco_frame_" + req.id; //"aruco_frame_" + req.id

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //ros::Rate rate(10.0);

    geometry_msgs::TransformStamped transformStamped;
    for(int i=0;i<15;i++){
    // get the piece pose
      try{
        transformStamped = tfBuffer.lookupTransform("world", pose_id, ros::Time(0));
      }
      catch(tf2::TransformException &ex){
        ROS_WARN("%d - %s", i, ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    resp.stampedpose = transformStamped;

    rate.sleep();

    return true;
  }

bool transCoordinate(
  team04_final_project::transfercoordinate::Request &req,
  team04_final_project::transfercoordinate::Response &resp){

    ros::NodeHandle n;
    ros::Rate rate(0.2);

    //ROS_INFO_STREAM("Start transform the coordinate from: " << req.x << req.y << req.z);

    geometry_msgs::TransformStamped original_pose = req.original_pose;

      double offset = 0.15;
      geometry_msgs::TransformStamped trans_pose;

      trans_pose.transform.translation.x = 0; //original_pose.transform.translation.x;
      trans_pose.transform.translation.y = 0; //original_pose.transform.translation.y;
      trans_pose.transform.translation.z = offset; //original_pose.transform.translation.z + offset;
      tf2::Quaternion q;
      q.setRPY(0, 3.141592654, 0);
      trans_pose.transform.rotation.x = q.x();
      trans_pose.transform.rotation.y = q.y();
      trans_pose.transform.rotation.z = q.z();
      trans_pose.transform.rotation.w = q.w();

      //trans_pose.transform.rotation.x = original_pose.transform.rotation.x;
      //trans_pose.transform.rotation.y = -original_pose.transform.rotation.y;
      //trans_pose.transform.rotation.z = -original_pose.transform.rotation.z;
      //trans_pose.transform.rotation.w = original_pose.transform.rotation.w;



      //pub_transCoor.publish(trans_pose);

      //rate.sleep();
      // create a broadcaster
      static tf2_ros::TransformBroadcaster br;

      ROS_INFO("**** Start to broadcast the transfered coordinate ****");

      trans_pose.header.stamp = ros::Time::now();
      trans_pose.header.frame_id = original_pose.child_frame_id;//   "world";
      trans_pose.child_frame_id = "pregrasp_frame";

      br.sendTransform(trans_pose);

      resp.coordinate = trans_pose;


      ROS_INFO_STREAM("The transformed result is: " << trans_pose);
      rate.sleep();
      rate.sleep();
      rate.sleep();


      ROS_INFO_STREAM("Start to get transform between Base frame and pregrasp_frame");

      //string base_linke = "team_A_base_link";
      //string goal_frame = req.frame;
      //string goal_frame = "pregrasp_frame";

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);

      geometry_msgs::TransformStamped transformStamped;
      for(int i=0;i<100;i++){
      // get the piece pose
        try{
          transformStamped = tfBuffer.lookupTransform("pregrasp_frame", "team_A_base_link", ros::Time(0));
        }
        catch(tf2::TransformException &ex){
          ROS_WARN("%d - %s", i, ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
      }

      resp.transcoord = transformStamped;
      ROS_INFO_STREAM("The pregrasp to base transform result is: " << transformStamped);
    return true;
  }
/*
bool pregraspToBase(
  team04_final_project::pregraspToBase::Request &req,
  team04_final_project::pregraspToBase::Response &resp){

    ros::NodeHandle n;
    ros::Rate rate(10.0);

    ROS_INFO_STREAM("Start to get transform between Base frame and pregrasp_frame");

    //string base_linke = "team_A_base_link";
    //string goal_frame = req.frame;
    //string goal_frame = "pregrasp_frame";

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;
    for(int i=0;i<15;i++){
    // get the piece pose
      try{
        transformStamped = tfBuffer.lookupTransform(req.frame, "team_A_base_link", ros::Time(0));
      }
      catch(tf2::TransformException &ex){
        ROS_WARN("%d - %s", i, ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    resp.transPose = transformStamped;

    return true;

  }

bool optik(
  team04_final_project::optik::Request &req,
  team04_final_project::optik::Response &resp){

    std::vector<double> distance;

    for(long unsigned int i = 0; i < req.iksolution.size(); i++){
      //calculate the distance between each ik solution to the original pose
      distance[i] = 0;
      for(int j = 0; j < 6; j++){
        double sum = req.iksolution[i].ik[j] - currentjoints[j];
        distance[i] = distance[i] + sum*sum;
        ROS_INFO_STREAM("Get the optimal solution of ik: " << distance[i] << "the index is" << i);
      }

    }
    //resp.opik = std::min_element(distance.begin(), distance.end());
    //ROS_INFO_STREAM("Get the optimal solution of ik: ");
    int opt_idx;
    for(unsigned int i = 0; i < (distance.size()-1); i++){
      if(distance[i] <= distance[i+1]){
        opt_idx = i;
      }else{
        opt_idx = i+1;
      }
    }

    std::vector<double> opt_ik;
    opt_ik.resize(req.iksolution[opt_idx].ik.size());
    for(unsigned int i = 0; i < req.iksolution[opt_idx].ik.size(); i++){
      resp.opik.push_back(req.iksolution[opt_idx].ik[i]);
    }
    ROS_INFO_STREAM("The opt index is : " << opt_idx);
    return true;

  }
*/
// subscriber callback function
void currentJointStates(const sensor_msgs::JointState& joint_states_arm) {
  currentjoints.resize(6);
  for(int i=0; i<6;i++)
  {
    if(joint_states_arm.name[i] == "team_A_shoulder_pan_joint") currentjoints[0]= joint_states_arm.position[i];
    else if(joint_states_arm.name[i] == "team_A_shoulder_lift_joint") currentjoints[1]= joint_states_arm.position[i];
    else if(joint_states_arm.name[i] == "team_A_elbow_joint") currentjoints[2]= joint_states_arm.position[i];
    else if(joint_states_arm.name[i] == "team_A_wrist_1_joint") currentjoints[3]= joint_states_arm.position[i];
    else if(joint_states_arm.name[i] == "team_A_wrist_2_joint") currentjoints[4]= joint_states_arm.position[i];
    else if(joint_states_arm.name[i] == "team_A_wrist_3_joint") currentjoints[5]= joint_states_arm.position[i];
  }
}


int main(int argc, char** argv){

  ros::init(argc, argv, "planning_module");

  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe("/team_A_arm/joint_states", 1000, &currentJointStates);
  //ros::Duration(1.0).sleep();

  //call the subscriber to get the current joints states


  // Set the service
  ros::ServiceServer service1 = nh.advertiseService("getpiecepose", &getPiecepose);
  ros::ServiceServer service2 = nh.advertiseService("transfercoordinate", &transCoordinate);
  //ros::ServiceServer service3 = nh.advertiseService("pregrasptobase", &pregraspToBase);
  //ros::ServiceServer service4 = nh.advertiseService("optik", &optik);



  ros::spin();


  return 0;
};
