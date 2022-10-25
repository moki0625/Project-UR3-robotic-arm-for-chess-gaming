#include <ros/ros.h>
#include <ros/package.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <team04_final_project/getpiecepose.h>
#include <team04_final_project/getpiececellcode.h>
#include <team04_final_project/getFieldpose.h>
#include <team04_final_project/transfercoordinate.h>
#include <team04_final_project/getOccupiedFields.h>
#include <team04_final_project/pregraspToBase.h>
#include <team04_final_project/moveArm.h>
#include <team04_final_project/pickChess.h>
#include <team04_final_project/placeChess.h>
#include <team04_final_project/chessCommand.h>
#include <chesslab_setup/ik.h>
#include <ur3ik/UR3IK.h>

#include <kautham/OpenProblem.h>
#include <kautham/CheckCollision.h>
#include <list>
#include <iostream>
#include <sensor_msgs/JointState.h>

using namespace std;

bool command_received = false;
string moving_piece;
string desired_location;

// Joint Values in initial position
std::vector<double> currentjoints = {-1.890013, -1.439965, -2.139970, -1.150044, 1.599841, -1.949876};


// Map with all poses from the chessfields
map<string,geometry_msgs::Pose> chessfield_poses;

// Poses
geometry_msgs::Pose pre_grasp_pose;
geometry_msgs::Pose pick_pose;
geometry_msgs::Pose pre_place_pose;
geometry_msgs::Pose place_pose;

// IK vector
std::vector<double> ik_sol;

// function to get the optimal ik
int optik(std::vector<std::vector<double>> iksolution){

  int opt_idx = 0;
  std::vector<double> distance;
  distance.resize(8);
  ROS_INFO_STREAM("----------------START OPT IK ---------------------");
  for(long unsigned int i = 0; i < iksolution.size(); i++){
    //calculate the distance between each ik solution to the original pose

    distance[i] = 0;
    double sum = 0;
    for(int j = 0; j < 6; j++){
      std::cout << "IK is of [" << i << ',' << j << "]"<< iksolution[i][j] << endl <<"\n";
      //std::cout << currentjoints[j] << endl << "\n";
      sum = iksolution[i][j] - currentjoints[j];
      distance[i] = distance[i] + abs(sum);
    }

    std::cout << "distance " << i << " " << distance[i] << '\n';

  }
  double temp_distance = 600;
  for(long unsigned int i = 0; i < distance.size()-1; i++){
    if(distance[i] < temp_distance && distance[i] > 0){
      opt_idx = i;
      std::cout << "Temporary Distance: " << temp_distance << '\n';
      temp_distance = distance[i];
      std::cout << "Distance: " << distance[i] << '\n';
      std::cout << "New optimal index is: " << opt_idx << '\n';
    }
  }
  currentjoints = iksolution[opt_idx];
  ROS_INFO_STREAM("The opt index is : " << opt_idx);
  return opt_idx;
}

// callback function to send command
bool chessCommand(
  team04_final_project::chessCommand::Request &req,
  team04_final_project::chessCommand::Response &resp){

    moving_piece = req.chesspiece;
    desired_location = req.field;

    ROS_INFO("Command received");
    resp.success = true;

    command_received = true;

    return true;
}

// Collision check
//! Function that wraps the call to the kautham service that opens a problem
bool kauthamOpenProblem( string modelFolder, string problemFile )
{
    ros::NodeHandle nh;
    ros::service::waitForService("/kautham_node/OpenProblem");

    kautham::OpenProblem kthopenproblem_srv;
    std::string model = modelFolder;
    ros::ServiceClient kthopenproblem_client = nh.serviceClient<kautham::OpenProblem>("/kautham_node/OpenProblem");
    kthopenproblem_srv.request.problem = problemFile;
    kthopenproblem_srv.request.dir.resize(1);
    kthopenproblem_srv.request.dir[0] = model;
    kthopenproblem_client.call(kthopenproblem_srv);
    if (kthopenproblem_srv.response.response == true) {
        ROS_INFO( "Kautham Problem opened correctly" );
    } else {
        ROS_ERROR( "ERROR Opening Kautham Problem" );
        ROS_ERROR( "models folder: %s", kthopenproblem_srv.request.dir[0].c_str() );
        ROS_ERROR( "problem file: %s", kthopenproblem_srv.request.problem.c_str() );
        return false;
    }
    return true;
}

//! Function that wraps the call to the kautham service that checks for collisions
bool kauthamCheckCollision(std::vector<double> conf, std::string *collObjName, std::string *msg)
{
    ros::NodeHandle nh;
    ros::service::waitForService("/kautham_node/CheckCollision");
    ros::ServiceClient check_collision_obstacles_client = nh.serviceClient<kautham::CheckCollision>("/kautham_node/CheckCollision");

    conf.push_back(0.5);  // gripper joint value

    //convert to normalized controls
    std::vector<float> controls(7);
    for(int i=0; i<6; i++){
        //<limit effort="54.0" lower="-3.141592654" upper="3.141592654" velocity="3.2"/>
        controls[i] = (conf[i]+3.141592654)/6.283185308;
    }
    //<limit effort="60" lower="0.0" upper="0.872664444444" velocity="1.91986177778"/>
    controls[6] = (conf[6]-0.0)/0.872664444444;


    kautham::CheckCollision check_collision_obstacles_srv;
    check_collision_obstacles_srv.request.config = controls;

    check_collision_obstacles_client.call(check_collision_obstacles_srv);

    *collObjName = check_collision_obstacles_srv.response.collidedObs;
    *msg = check_collision_obstacles_srv.response.msg;

    if(!check_collision_obstacles_srv.response.collisionFree)
    {
        // ROS_WARN("The current configuration is not collision free!");
        return false;
    }
    else
    {
        // ROS_INFO("The current configuration is collision free!");
        return true;
    }
}

bool computeIK(geometry_msgs::Pose goalPose){

  ros::NodeHandle nh;

  // Transform pose from world frame to robot base link frame
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::PoseStamped pose_from_world;
  geometry_msgs::PoseStamped pose_from_baselink;

  pose_from_world.pose = goalPose;
  pose_from_world.header.frame_id = "world";

  int transform_done = 0;
  while(!transform_done){
    try{
      tfBuffer.transform(pose_from_world, pose_from_baselink, "team_A_base_link");
      transform_done = 1;
    }
    catch (tf2::TransformException &ex) {
      //ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  ros::service::waitForService("/chesslab_setup/inversekin", ros::Duration(5));
  ros::ServiceClient client_urik = nh.serviceClient<chesslab_setup::ik>("/chesslab_setup/inversekin");
  chesslab_setup::ik::Request req_urik;
  chesslab_setup::ik::Response resp_urik;

  req_urik.pose = pose_from_baselink.pose;
  client_urik.call(req_urik, resp_urik);

  std::vector<std::vector<double>> iksolution;
  iksolution.resize(resp_urik.ik_solution.size());
  for(unsigned int i=0; i<resp_urik.ik_solution.size(); i++)
  {
    for(int j=0; j<6; j++){
      iksolution[i].push_back(resp_urik.ik_solution[i].ik[j]);
    }
  }


  int idx = optik(iksolution);
  std::cout << "the optimal ik index is: " << idx << endl << "\n";


  std::string collision_obj_name;
  std::string check_msg;
  int coll_free_idx = 0;

  // Collision check
  for(unsigned int i = 0; i< iksolution.size(); i++){
    if(kauthamCheckCollision(iksolution[i], &collision_obj_name, &check_msg)){
      coll_free_idx = i;
      std::cout << "coll_free" << coll_free_idx << '\n';
      break;
    }
  }

  ik_sol = iksolution[idx];

  return true;

}

bool performMovement(geometry_msgs::Pose goalPose, string action){
  ros::NodeHandle nh;

  //arguments for moveArm service
  ros::service::waitForService("team04_final_project/move_arm", ros::Duration(5));
  ros::ServiceClient client_movearm = nh.serviceClient<team04_final_project::moveArm>("team04_final_project/move_arm");
  team04_final_project::moveArm::Request req_movearm;
  team04_final_project::moveArm::Response resp_movearm;

  // arguments for pickChess service
  ros::service::waitForService("team04_final_project/pick_chess", ros::Duration(5));
  ros::ServiceClient client_pickchess = nh.serviceClient<team04_final_project::pickChess>("team04_final_project/pick_chess");
  team04_final_project::pickChess::Request req_pickchess;
  team04_final_project::pickChess::Response resp_pickchess;

  // arguments for placeChess service
  ros::service::waitForService("team04_final_project/place_chess", ros::Duration(5));
  ros::ServiceClient client_placechess = nh.serviceClient<team04_final_project::placeChess>("team04_final_project/place_chess");
  team04_final_project::placeChess::Request req_placechess;
  team04_final_project::placeChess::Response resp_placechess;



  if (action == "home" || action == "move"){

    computeIK(goalPose);
    req_movearm.goalPoint.positions = ik_sol;
    req_movearm.trajduration = 15.0;

    client_movearm.call(req_movearm,resp_movearm);
  }
  else if (action == "pick"){

    std::vector<double> ik_pregrasp;
    std::vector<double> ik_grasp;

    geometry_msgs::Pose Pregrasp_Pose;
    geometry_msgs::Pose Grasp_Pose;

    Grasp_Pose = goalPose;
    Pregrasp_Pose.position.x = goalPose.position.x;
    Pregrasp_Pose.position.y = goalPose.position.y;
    Pregrasp_Pose.position.z = goalPose.position.z + 0.05;
    Pregrasp_Pose.orientation= goalPose.orientation;

    //call ik compute function;
    if(computeIK(Pregrasp_Pose)){
      ik_pregrasp = ik_sol;
    }

    if(computeIK(Grasp_Pose)){
      ik_grasp = ik_sol;
    }
    //req_pickchess.pregraspConfig.positions = iksolution[coll_free_idx];
    req_pickchess.pregraspConfig.positions = ik_pregrasp;
    req_pickchess.graspConfig.positions = ik_grasp;
    req_movearm.trajduration = 15.0;
    client_pickchess.call(req_pickchess,resp_pickchess);
  }
  else if (action == "place"){


    std::vector<double> ik_placeup;
    std::vector<double> ik_place;

    geometry_msgs::Pose Placeup_pose;
    geometry_msgs::Pose Place_pose;

    Place_pose = goalPose;
    Placeup_pose.position.x = goalPose.position.x;
    Placeup_pose.position.y = goalPose.position.y;
    Placeup_pose.position.z = goalPose.position.z + 0.05;
    Placeup_pose.orientation = goalPose.orientation;

    if(computeIK(Placeup_pose)){
      ik_placeup = ik_sol;
    }

    if(computeIK(Place_pose)){
      ik_place = ik_sol;
    }


    req_placechess.placeConfig.positions = ik_place;
    req_placechess.placeupConfig.positions = ik_placeup;
    req_placechess.trajduration = 15.0;
    client_placechess.call(req_placechess,resp_placechess);

  }
  return true;
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "planning_module_client");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);


  ros::ServiceServer service1 = nh.advertiseService("chess_command",&chessCommand);


  ///////////////////////////   Collision check    ///////////////////////////////

      ROS_INFO("**** starting kautham setup ****");
      string kautham_path = ros::package::getPath("kautham");
      string modelFolder = kautham_path + "/demos/models/";
      string problemFile = kautham_path + "/demos/OMPL_geo_demos/chess/OMPL_RRTconnect_chess_ur3_gripper_1_flat.xml";
      ROS_INFO("**** Loading %s ****",problemFile.c_str());


      kauthamOpenProblem(modelFolder, problemFile);

  ////////////////////////////////////////////////////////////////////////////////
  //////////////////////////   Define pre-set poses   ////////////////////////////

      geometry_msgs::Pose home_pose;
      home_pose.position.x = 0.33;
      home_pose.position.y = -0.11;
      home_pose.position.z = 0.48;
      home_pose.orientation.x = 0.707;
      home_pose.orientation.y = 0.707;
      home_pose.orientation.z = 0.0;
      home_pose.orientation.w = 0.0;

      geometry_msgs::Pose safe_zone;
      safe_zone.position.x = 0.05;
      safe_zone.position.y = -0.30;
      safe_zone.position.z = 0.30;
      safe_zone.orientation.x = 0.707;
      safe_zone.orientation.y = 0.707;
      safe_zone.orientation.z = 0.0;
      safe_zone.orientation.w = 0.0;

  ////////////////////////////////////////////////////////////////////////////////


  //sleep(3.0);
  //Create client objects for the services
  ros::ServiceClient client_getpiecepose = nh.serviceClient<team04_final_project::getpiecepose>("getpiecepose");
  ros::ServiceClient client_getfieldpose = nh.serviceClient<team04_final_project::getFieldpose>("field_pose");
  ros::ServiceClient client_getpiececellcode = nh.serviceClient<team04_final_project::getpiececellcode>("piece_cellcode");
  ros::ServiceClient client_transfercoordinate = nh.serviceClient<team04_final_project::transfercoordinate>("transfercoordinate");
  ros::ServiceClient client_getoccupiedfields = nh.serviceClient<team04_final_project::getOccupiedFields>("occupied_fields");
  //ros::ServiceClient client_pregrasptobase = nh.serviceClient<team04_final_project::pregraspToBase>("pregrasptobase");

  ros::ServiceClient client_movearm = nh.serviceClient<team04_final_project::moveArm>("team04_final_project/move_arm");
  ros::ServiceClient client_pickchess = nh.serviceClient<team04_final_project::pickChess>("team04_final_project/pick_chess");
  ros::ServiceClient client_placechess = nh.serviceClient<team04_final_project::placeChess>("team04_final_project/place_chess");


  ROS_INFO("**** CLIENT PROGRAM IS OPEN ****");

  std::vector<string> occupied_fields = { "A1", "B1", "C1", "D1", "E1", "F1", "G1", "H1",
                                          "A2", "B2", "C2", "D2", "E2", "F2", "G2", "H2",
                                          "A7", "B7", "C7", "D7", "E7", "F7", "G7", "H7",
                                          "A8", "B8", "C8", "D8", "E8", "F8", "G8", "H8"};

  ////////////////////////////////////////////////
  // Initiating Chessfield Map with all poses
  ////////////////////////////////////////////////
  string chessfield_code;
  for (unsigned int i=17; i<25; i++){ //ASCII CODE FOR A-H
    for (unsigned int j=1; j<9; j++){ //Number from 1 to 8
      chessfield_code = i+'0';
      chessfield_code = chessfield_code + to_string(j);

      chessfield_poses[chessfield_code].position.x=-0.175+((j-1)*0.05);
      chessfield_poses[chessfield_code].position.y=0.175-((i-17)*0.05);
      chessfield_poses[chessfield_code].position.z=0.0;
      chessfield_poses[chessfield_code].orientation.x=0.707;
      chessfield_poses[chessfield_code].orientation.y=0.707;
      chessfield_poses[chessfield_code].orientation.z=0.0;
      chessfield_poses[chessfield_code].orientation.w=0.0;
    }
  }

  //arguments for getOccupiedFields service
  ros::service::waitForService("occupied_fields", ros::Duration(5));

  team04_final_project::getOccupiedFields::Request req_getoccField;
  team04_final_project::getOccupiedFields::Response resp_getoccField;

  req_getoccField.mapping_request = true;

  ROS_INFO("**** call get occupiedFields service ****");

  ///////////////////////////////////////////////////////////////////
  // If the whole chess field should be mapped and all occupied fields should be retrieved,
  // uncomment the following lines:

  /*
  client_getoccupiedfields.call(req_getoccField, resp_getoccField);

  occupied_fields.resize(resp_getoccField.occFields.size());
  occupied_fields = resp_getoccField.occFields;

  std::cout << "*************OCCUPIED FIELDS*************" << '\n';

  for (string i: occupied_fields){
    std::cout << i << ' ';
  }
  std::cout << "********************" << '\n';
  */
  ///////////////////////////////////////////////////////////////////


  //arguments for getOccupiedFields service
  ros::service::waitForService("field_pose", ros::Duration(5));
  team04_final_project::getFieldpose::Request req_getFieldpose;
  team04_final_project::getFieldpose::Response resp_getFieldpose;

  //arguments for getpiecepose service
  ros::service::waitForService("getpiecepose", ros::Duration(5));
  team04_final_project::getpiecepose::Request req_getpiecepose;
  team04_final_project::getpiecepose::Response resp_getpiecepose;

  //arguments for getpiececellcode service
  ros::service::waitForService("piece_cellcode", ros::Duration(5));
  team04_final_project::getpiececellcode::Request req_getpiececellcode;
  team04_final_project::getpiececellcode::Response resp_getpiececellcode;

  //arguments for transform service
  ros::service::waitForService("transfercoordinate", ros::Duration(5));
  team04_final_project::transfercoordinate::Request req_trans;
  team04_final_project::transfercoordinate::Response resp_trans;

  std::cout<<"\nPRESS A KEY TO START THE CHESS GAME..."<<std::endl;
  std::cin.get();


  while (ros::ok()) {
    if (command_received){

      int moving_piece_int = stoi(moving_piece);
      double chess_size;
      if(((200<moving_piece_int)&&(moving_piece_int<209))||((300<moving_piece_int)&&(moving_piece_int<309))){
        chess_size = 0.04;
      }
      else if(((208<moving_piece_int)&&(moving_piece_int<215))||((308<moving_piece_int)&&(moving_piece_int<315))){
        chess_size = 0.06;
      }
      else if(((214<moving_piece_int)&&(moving_piece_int<217))||((314<moving_piece_int)&&(moving_piece_int<317))){
        chess_size = 0.08;
      }
      ROS_INFO("Chess size %f", chess_size);


      //performMovement(home_pose, "home");

      bool cell_occupied = false;
      for (unsigned int i=0; i<occupied_fields.size();i++){ //replace with resp_getoccField.occFields[i]
        if (desired_location == occupied_fields[i]){ //replace with resp_getoccField.occFields[i]
          std::cout << "i: " << i << '\n';

          cell_occupied = true;
        }
      }

      if (cell_occupied == true){
        std::cout << "KILL MOVE" << '\n';

        ////////////////////////////////////////////////
        // MOVE KILLED PIECE OUT OF THE Field
        ////////////////////////////////////////////////
        double gripper_size = 0.16;

        std::cout << "Desired location: " << desired_location << '\n';
        pre_grasp_pose = chessfield_poses[desired_location];
        pre_grasp_pose.position.z = chess_size + 0.2;
        performMovement(pre_grasp_pose, "move");

        pick_pose = chessfield_poses[desired_location];
        pre_grasp_pose.position.z = chess_size + gripper_size;
        performMovement(pick_pose, "pick");

        pre_place_pose = safe_zone;
        pre_place_pose.position.z = chess_size + 0.2;
        performMovement(pre_place_pose, "move");

        place_pose = pre_place_pose;
        place_pose.position.z = chess_size + gripper_size;
        performMovement(place_pose, "place");

        std::cout << "KILLED CHESS PIECE REMOVED" << '\n';

        ////////////////////////////////////////////////
        // MOVE THE PIECE THAT KILLS
        ////////////////////////////////////////////////

        req_getpiececellcode.id = moving_piece;
        client_getpiececellcode.call(req_getpiececellcode,resp_getpiececellcode);
        std::cout << "Piece CELLCODE is: " << resp_getpiececellcode << '\n';

        pre_grasp_pose = chessfield_poses[resp_getpiececellcode.cellcode];
        pre_grasp_pose.position.z = chess_size + 0.2;
        performMovement(pre_grasp_pose, "move");

        pick_pose = chessfield_poses[resp_getpiececellcode.cellcode];
        pick_pose.position.z = chess_size + gripper_size;
        performMovement(pick_pose, "pick");

        pre_place_pose = chessfield_poses[desired_location];
        pre_place_pose.position.z = chess_size + 0.2;
        performMovement(pre_place_pose, "move");

        place_pose = pre_place_pose;
        place_pose.position.z = chess_size + gripper_size;
        performMovement(place_pose, "place");

        std::cout << "CHESS PIECE MOVED" << '\n';


        std::cout << "*************OCCUPIED FIELDS*************" << '\n';
        auto itr = std::find(occupied_fields.begin(), occupied_fields.end(), resp_getpiececellcode.cellcode);
        if (itr != occupied_fields.end()) occupied_fields.erase(itr);

        for (string i: occupied_fields){
          std::cout << i << ' ';
        }
        std::cout << "********************" << '\n';

        command_received = false;
      }

      else{
        std::cout << "NORMAL MOVE" << '\n';

        ////////////////////////////////////////////////
        // MOVE THE PIECE
        ////////////////////////////////////////////////
        double gripper_size = 0.16;
        req_getpiececellcode.id = moving_piece;
        client_getpiececellcode.call(req_getpiececellcode,resp_getpiececellcode);
        std::cout << "Piece CELLCODE is: " << resp_getpiececellcode << '\n';

        pre_grasp_pose = chessfield_poses[resp_getpiececellcode.cellcode];
        pre_grasp_pose.position.z = chess_size + 0.2;
        performMovement(pre_grasp_pose, "move");

        pick_pose = chessfield_poses[resp_getpiececellcode.cellcode];
        pick_pose.position.z = chess_size + gripper_size;
        performMovement(pick_pose, "pick");

        pre_place_pose = chessfield_poses[desired_location];
        pre_place_pose.position.z = chess_size + 0.2;
        performMovement(pre_place_pose, "move");

        place_pose = pre_place_pose;
        place_pose.position.z = chess_size + gripper_size;
        performMovement(place_pose, "place");

        std::cout << "CHESS PIECE MOVED" << '\n';


        std::cout << "*************OCCUPIED FIELDS*************" << '\n';
        auto itr = std::find(occupied_fields.begin(), occupied_fields.end(), resp_getpiececellcode.cellcode);
        if (itr != occupied_fields.end()) occupied_fields.erase(itr);
        occupied_fields.push_back(desired_location);

        for (string i: occupied_fields){
          std::cout << i << ' ';
        }
        std::cout << "********************" << '\n';

        command_received = false;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
