#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/UInt32MultiArray.h>
#include <tf/tf.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
// add header
#include <team04_final_project/getpiecepose.h>
#include <team04_final_project/getpiececellcode.h>
#include <team04_final_project/getFieldpose.h>
#include <team04_final_project/getOccupiedFields.h>
#include <team04_final_project/occupiedFields.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>

using namespace std;

string letter;
string number;
string cellcode;
bool mapping_done = false;
std::vector<int> marker_arr;
std::vector<string> occupied_fields;

team04_final_project::occupiedFields occ_msg;

bool getFieldpose(
  team04_final_project::getFieldpose::Request &req,
  team04_final_project::getFieldpose::Response &resp){

  std::vector<char> v(req.chess_code.begin(),req.chess_code.end());

  if (v[1] == '1'){
    resp.stampedpose.transform.translation.x = -0.025;
  }
  if (v[1] == '2'){
    resp.stampedpose.transform.translation.x = -0.075;
  }
  if (v[1] == '3'){
    resp.stampedpose.transform.translation.x = -0.125;
  }
  if (v[1] == '4'){
    resp.stampedpose.transform.translation.x = -0.175;
  }
  if (v[1] == '5'){
    resp.stampedpose.transform.translation.x = 0.025;
  }
  if (v[1] == '6'){
    resp.stampedpose.transform.translation.x = 0.075;
  }
  if (v[1] == '7'){
    resp.stampedpose.transform.translation.x = 0.125;
  }
  if (v[1] == '8'){
    resp.stampedpose.transform.translation.x = 0.175;
  }

  if (v[0] == 'A'){
    resp.stampedpose.transform.translation.y = 0.175;
  }
  if (v[0] == 'B'){
    resp.stampedpose.transform.translation.y = 0.125;
  }
  if (v[0] == 'C'){
    resp.stampedpose.transform.translation.y = 0.075;
  }
  if (v[0] == 'D'){
    resp.stampedpose.transform.translation.y = 0.025;
  }
  if (v[0] == 'E'){
    resp.stampedpose.transform.translation.y = -0.025;
  }
  if (v[0] == 'F'){
    resp.stampedpose.transform.translation.y = -0.075;
  }
  if (v[0] == 'G'){
    resp.stampedpose.transform.translation.y = -0.125;
  }
  if (v[0] == 'H'){
    resp.stampedpose.transform.translation.y = -0.175;
  }

  resp.stampedpose.transform.translation.z = 0.02;
  resp.stampedpose.transform.rotation.w = 1;

  return true;

}


bool getPieceCellCode(
    team04_final_project::getpiececellcode::Request &req,
    team04_final_project::getpiececellcode::Response &resp){

    string pose_id = "aruco_frame_" + req.id;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);

    geometry_msgs::TransformStamped transformStamped;
    for(int i=0;i<10;i++){
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

    if (transformStamped.transform.translation.x > -0.20 && transformStamped.transform.translation.x < -0.15){
      number = "1";
    }
    if (transformStamped.transform.translation.x > -0.15 && transformStamped.transform.translation.x < -0.10){
      number = "2";
    }
    if (transformStamped.transform.translation.x > -0.10 && transformStamped.transform.translation.x < -0.05){
      number = "3";
    }
    if (transformStamped.transform.translation.x > -0.05 && transformStamped.transform.translation.x < 0){
      number = "4";
    }
    if (transformStamped.transform.translation.x > 0 && transformStamped.transform.translation.x < 0.05){
      number = "5";
    }
    if (transformStamped.transform.translation.x > 0.05 && transformStamped.transform.translation.x < 0.10){
      number = "6";
    }
    if (transformStamped.transform.translation.x > 0.10 && transformStamped.transform.translation.x < 0.15){
      number = "7";
    }
    if (transformStamped.transform.translation.x > 0.15 && transformStamped.transform.translation.x < 0.20){
      number = "8";
    }
    if (transformStamped.transform.translation.y > -0.20 && transformStamped.transform.translation.y < -0.15){
      letter = "A";
    }
    if (transformStamped.transform.translation.y > -0.15 && transformStamped.transform.translation.y < -0.10){
      letter = "B";
    }
    if (transformStamped.transform.translation.y > -0.10 && transformStamped.transform.translation.y < -0.05){
      letter = "C";
    }
    if (transformStamped.transform.translation.y > -0.05 && transformStamped.transform.translation.y < 0){
      letter = "D";
    }
    if (transformStamped.transform.translation.y > 0 && transformStamped.transform.translation.y < 0.05){
      letter = "E";
    }
    if (transformStamped.transform.translation.y > 0.05 && transformStamped.transform.translation.y < 0.10){
      letter = "F";
    }
    if (transformStamped.transform.translation.y > 0.10 && transformStamped.transform.translation.y < 0.15){
      letter = "G";
    }
    if (transformStamped.transform.translation.y > 0.15 && transformStamped.transform.translation.y < 0.20){
      letter = "H";
    }
    if (transformStamped.transform.translation.x > 0.2 || transformStamped.transform.translation.x < -0.2 ||
        transformStamped.transform.translation.y > 0.2 || transformStamped.transform.translation.y < -0.2){
          letter="Cal";
          number="Marker";
    }

    cellcode = letter+number;

    resp.cellcode = cellcode;
    //rate.sleep();

    ROS_INFO_STREAM("The piece cellcode is: " << cellcode);

    return true;
}

// Mapping Function
void mapping(){
    //team04_final_project::getpiecepose::Request &req,
    //team04_final_project::getpiecepose::Response &resp){

    //ROS_INFO_STREAM("Get the piece pose of "<<req.id);
    std::cout<<"------------------ mapping started -----------------------"<<std::endl;
    occupied_fields.clear();

    for (unsigned int j=0; j<marker_arr.size(); j++){
        string pose_id = "aruco_frame_" + std::to_string(marker_arr[j]);


        //string pose_id = req.id;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate rate(100.0);

        geometry_msgs::TransformStamped transformStamped;
        for(int i=0;i<10;i++){
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


        ROS_INFO_STREAM("The piece pose is: " << transformStamped);


        if (transformStamped.transform.translation.x > -0.05 && transformStamped.transform.translation.x < 0){
          number = "1";
        }
        if (transformStamped.transform.translation.x > -0.10 && transformStamped.transform.translation.x < -0.05){
          number = "2";
        }
        if (transformStamped.transform.translation.x > -0.15 && transformStamped.transform.translation.x < -0.10){
          number = "3";
        }
        if (transformStamped.transform.translation.x > -0.20 && transformStamped.transform.translation.x < -0.15){
          number = "4";
        }
        if (transformStamped.transform.translation.x > 0 && transformStamped.transform.translation.x < 0.05){
          number = "5";
        }
        if (transformStamped.transform.translation.x > 0.05 && transformStamped.transform.translation.x < 0.10){
          number = "6";
        }
        if (transformStamped.transform.translation.x > 0.10 && transformStamped.transform.translation.x < 0.15){
          number = "7";
        }
        if (transformStamped.transform.translation.x > 0.15 && transformStamped.transform.translation.x < 0.20){
          number = "8";
        }
        if (transformStamped.transform.translation.y > -0.20 && transformStamped.transform.translation.y < -0.15){
          letter = "H";
        }
        if (transformStamped.transform.translation.y > -0.15 && transformStamped.transform.translation.y < -0.10){
          letter = "G";
        }
        if (transformStamped.transform.translation.y > -0.10 && transformStamped.transform.translation.y < -0.05){
          letter = "F";
        }
        if (transformStamped.transform.translation.y > -0.05 && transformStamped.transform.translation.y < 0){
          letter = "E";
        }
        if (transformStamped.transform.translation.y > 0 && transformStamped.transform.translation.y < 0.05){
          letter = "D";
        }
        if (transformStamped.transform.translation.y > 0.05 && transformStamped.transform.translation.y < 0.10){
          letter = "C";
        }
        if (transformStamped.transform.translation.y > 0.10 && transformStamped.transform.translation.y < 0.15){
          letter = "B";
        }
        if (transformStamped.transform.translation.y > 0.15 && transformStamped.transform.translation.y < 0.20){
          letter = "A";
        }
        if (transformStamped.transform.translation.x > 0.2 || transformStamped.transform.translation.x < -0.2 ||
            transformStamped.transform.translation.y > 0.2 || transformStamped.transform.translation.y < -0.2){
              letter="Cal";
              number="Marker";
        }

        if(letter!="" && number!= "" && transformStamped.transform.translation.z != 0){
          occupied_fields.push_back(letter+number);
        }
        else{
          occupied_fields.push_back("NoTransform");
        }

        std::cout << "Chess Piece Location " << occupied_fields[j] << '\n';
        //rate.sleep();
    }
    std::cout << "All occupied fields: " << '\n';

    occ_msg.vector_occFields.resize(occupied_fields.size());

    for(unsigned int k=0; k<occupied_fields.size(); k++){
      //std::cout << "index" << k << '\n';
      std::cout << occupied_fields[k] << '\n';
      occ_msg.vector_occFields[k]=occupied_fields[k];
    }
    std::cout << "------------------ mapping completed -----------------------" << '\n';

    mapping_done = true;

    return;
}

bool getOccupiedFields(
    team04_final_project::getOccupiedFields::Request &req,
    team04_final_project::getOccupiedFields::Response &resp){

    //ROS_INFO_STREAM("Get the piece pose of "<<req.id);
    std::cout<<"------------------ mapping started -----------------------"<<std::endl;
    occupied_fields.clear();

    for (unsigned int j=0; j<marker_arr.size(); j++){
        string pose_id = "aruco_frame_" + std::to_string(marker_arr[j]);


        //string pose_id = req.id;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate rate(100.0);

        geometry_msgs::TransformStamped transformStamped;
        for(int i=0;i<10;i++){
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


        ROS_INFO_STREAM("The piece pose is: " << transformStamped);


        if (transformStamped.transform.translation.x > -0.05 && transformStamped.transform.translation.x < 0){
          number = "1";
        }
        if (transformStamped.transform.translation.x > -0.10 && transformStamped.transform.translation.x < -0.05){
          number = "2";
        }
        if (transformStamped.transform.translation.x > -0.15 && transformStamped.transform.translation.x < -0.10){
          number = "3";
        }
        if (transformStamped.transform.translation.x > -0.20 && transformStamped.transform.translation.x < -0.15){
          number = "4";
        }
        if (transformStamped.transform.translation.x > 0 && transformStamped.transform.translation.x < 0.05){
          number = "5";
        }
        if (transformStamped.transform.translation.x > 0.05 && transformStamped.transform.translation.x < 0.10){
          number = "6";
        }
        if (transformStamped.transform.translation.x > 0.10 && transformStamped.transform.translation.x < 0.15){
          number = "7";
        }
        if (transformStamped.transform.translation.x > 0.15 && transformStamped.transform.translation.x < 0.20){
          number = "8";
        }
        if (transformStamped.transform.translation.y > -0.20 && transformStamped.transform.translation.y < -0.15){
          letter = "H";
        }
        if (transformStamped.transform.translation.y > -0.15 && transformStamped.transform.translation.y < -0.10){
          letter = "G";
        }
        if (transformStamped.transform.translation.y > -0.10 && transformStamped.transform.translation.y < -0.05){
          letter = "F";
        }
        if (transformStamped.transform.translation.y > -0.05 && transformStamped.transform.translation.y < 0){
          letter = "E";
        }
        if (transformStamped.transform.translation.y > 0 && transformStamped.transform.translation.y < 0.05){
          letter = "D";
        }
        if (transformStamped.transform.translation.y > 0.05 && transformStamped.transform.translation.y < 0.10){
          letter = "C";
        }
        if (transformStamped.transform.translation.y > 0.10 && transformStamped.transform.translation.y < 0.15){
          letter = "B";
        }
        if (transformStamped.transform.translation.y > 0.15 && transformStamped.transform.translation.y < 0.20){
          letter = "A";
        }
        if (transformStamped.transform.translation.x > 0.2 || transformStamped.transform.translation.x < -0.2 ||
            transformStamped.transform.translation.y > 0.2 || transformStamped.transform.translation.y < -0.2){
              letter="Cal";
              number="Marker";
        }

        if(letter!="" && number!= "" && transformStamped.transform.translation.z != 0){
          occupied_fields.push_back(letter+number);
        }
        else{
          occupied_fields.push_back("NoTransform");
        }

        std::cout << "Chess Piece Location " << occupied_fields[j] << '\n';
        //rate.sleep();
    }
    std::cout << "All occupied fields: " << '\n';

    occ_msg.vector_occFields.resize(occupied_fields.size());
    resp.occFields.resize(occupied_fields.size());

    for(unsigned int k=0; k<occupied_fields.size(); k++){
      //std::cout << "index" << k << '\n';
      std::cout << occupied_fields[k] << '\n';
      occ_msg.vector_occFields[k]=occupied_fields[k];
      resp.occFields[k]=occupied_fields[k];
    }
    std::cout << "------------------ mapping completed -----------------------" << '\n';

    mapping_done = true;

    return true;
}

void markers_list_MessageReceived(const std_msgs::UInt32MultiArray& msg) {
       marker_arr.clear();
       for (std::vector<unsigned int>::const_iterator it = msg.data.begin(); it != msg.data.end(); ++it){
         //std::size_t pos = marker_arr.size();
         marker_arr.push_back(*it);
         //std::cout << arr[pos] << '\n';
         //std::cout << "size" << pos << '\n';
       }

       //std::cout << "------------------ received camera marker list -----------------------" << '\n';

       //mapping();
       ros::Rate rate(1.0);
       return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sensing_module");
  ros::NodeHandle nh;

  //ros::Duration(1.0).sleep();

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("/aruco_marker_publisher/markers_list", 1000, &markers_list_MessageReceived);

  // Set the service
  ros::ServiceServer server = nh.advertiseService("piece_cellcode", &getPieceCellCode);
  ros::ServiceServer server2 = nh.advertiseService("occupied_fields", &getOccupiedFields);
  ros::ServiceServer server3 = nh.advertiseService("field_pose", &getFieldpose);


  // Create a publisher object
  /*ros::Publisher pub = nh.advertise<team04_final_project::occupiedFields>("chess_setup/occupied_fields", 1000);
  while(ros::ok()) {
    if (mapping_done){
      pub.publish(occ_msg);
      mapping_done=false;
    }
    // Let ROS take over.
    ros::spinOnce();
  }*/
  ros::spin();
  //return 0;
};
