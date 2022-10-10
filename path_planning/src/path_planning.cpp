/************************************************
 Path planner using bezier library.
 This file which operate path planning represents
 one node of the entire demonstrator
 ************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include "bezier/grinding_surfacing.hpp"
#include <fanuc_grinding_path_planning/PathPlanningService.h> // Description of the Service we will use
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
using namespace std;
using namespace KDL;
boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;
boost::shared_ptr<ros::NodeHandle> node;
EigenSTL::vector_Isometry3d way_points_vector;

/** Status publisher */
boost::shared_ptr<ros::Publisher> status_pub;

/** Name of the move_group used to move the robot during calibration */
const std::string move_group_name("grinding_disk");
/** Name of the TCP that should be used to compute the trajectories */
const std::string tcp_name("/tool0");

/** Bezier path planner object*/
boost::shared_ptr<BezierGrindingSurfacing> bezier;

/** File path of the mesh used by the path planner*/
std::string input_mesh_filename = "";

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Always true
 */
bool pathPlanning(fanuc_grinding_path_planning::PathPlanningService::Request &req,
                  fanuc_grinding_path_planning::PathPlanningService::Response &res)
{
  if (!req.SurfacingMode)
  {
    res.ReturnStatus = false;
    res.ReturnMessage = "Grinding mode is not implemented yet!";
    return true;
  }


  std_msgs::String status;
  std::vector<bool> is_grinding_pose;

  if (req.Compute)
  {
    std::string package = "fanuc_grinding_path_planning";
    //Get package path
    std::string mesh_ressource = "package://" + package + "/meshes/";
    std::string mesh_ressource_file = "file://";

    // Determine lean angle axis
    std::string lean_angle_axis;
    BezierGrindingSurfacing::AXIS_OF_ROTATION lean_axis;
    if (req.AngleX == true)
      lean_axis = BezierGrindingSurfacing::X;
    else if (req.AngleY == true)
      lean_axis = BezierGrindingSurfacing::Y;
    else if (req.AngleZ == true)
      lean_axis = BezierGrindingSurfacing::Z;
    else
    {
      res.ReturnStatus = false;
      res.ReturnMessage = "Please select a lean angle axis for the effector";
      return true;
    }
      input_mesh_filename = req.ScanFileName;
      cout<<"file name "<<input_mesh_filename<<endl;
    if (!bezier || input_mesh_filename.compare(req.CADFileName) != 0)
    {
      bezier.reset(
          new BezierGrindingSurfacing(input_mesh_filename, req.GrinderWidth, req.CoveringPercentage, req.ExtricationRadius,
                                      req.LeanAngle, lean_axis));
    }

    // Save the PLY file name from command line


      status.data = "Generate Bezier trajectory";
    status_pub->publish(status);
    std::string error_string;

      error_string = bezier->generateTrajectory(way_points_vector, is_grinding_pose, req.GrinderWidth,
                                              req.CoveringPercentage, req.ExtricationRadius, req.LeanAngle, lean_axis);

    if (!error_string.empty())
    {
      res.ReturnStatus = false;
      res.ReturnMessage = error_string;
      return true;
    }
  }

  // Copy the vector of Eigen poses into a vector of ROS poses
  std::vector<geometry_msgs::Pose> way_points_msg;

    KDL::Tree tree;
//    std::string ename = ros::package::getPath("ur_e_description");
//    kdl_parser::treeFromFile(filename+"/urdf/ur5e_robot.urdf", tree);


    std::string robot_desc_string;
    node->param("robot_description", robot_desc_string, string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }


    KDL::Chain chain;
    tree.getChain("base_link", "tool0", chain);


    unsigned int n = chain.getNrOfJoints();
    KDL::JntArray in(n);
    in(0) = KDL::deg2rad * 30;
    in(1) = KDL::deg2rad * -10;
    in(2) = KDL::deg2rad * 10;
    in(3) = KDL::deg2rad * 15;
    in(4) = KDL::deg2rad * 10;
    in(5) = KDL::deg2rad * 10;
    ChainFkSolverPos_recursive fwdkin(chain);
    Eigen::Matrix<double, 6, 1> L;
    L(0) = 1; L(1) = 1; L(2) = 1;
    L(3) = 0.01; L(4) = 0.01; L(5) = 0.01;
    ChainIkSolverPos_LMA solver(chain, L,1E-3);
    JntArray q_init(n);
    JntArray q_sol(n);
    JntArray q_out(n);
    q_init.data.setRandom();
    q_init.data *= 0;



  int error_point=0;
  int su=0;

  for (Eigen::Affine3d pose : way_points_vector)
  {
      su++;
          geometry_msgs::Pose tmp,txt;
          pose.translation() += Eigen::Vector3d(0, 0, req.TrajectoryZOffset);
          tf::poseEigenToMsg(pose, tmp);

      txt.position.x = tmp.position.x;
      txt.position.y = tmp.position.y;
      txt.position.z = tmp.position.z;

      txt.orientation.x = tmp.orientation.x;
      txt.orientation.y = tmp.orientation.y;
      txt.orientation.z = tmp.orientation.z;
      txt.orientation.w = tmp.orientation.w;

      Rotation r1=Rotation::Quaternion(tmp.orientation.x,tmp.orientation.y,tmp.orientation.z,tmp.orientation.w);
      Vector v1(tmp.position.x,tmp.position.y,tmp.position.z);
      Frame f1(r1,v1);
      int state = solver.CartToJnt(q_init, f1, q_sol);

      if (state==0) {
          q_init=q_sol;
          way_points_msg.push_back(tmp);
      }
      else {
          error_point++;
          }
  }
  std::cout<<"error_point "<<error_point<<std::endl;
    res.RobotPosesOutput = way_points_msg;
  for (std::vector<bool>::const_iterator iter(is_grinding_pose.begin()); iter != is_grinding_pose.end(); ++iter)
    res.IsGrindingPose.push_back(*iter);


  if (!req.Simulate)
  {
    res.ReturnStatus = true;
    res.ReturnMessage = boost::lexical_cast<std::string>(way_points_msg.size()) + " poses generated";
    return true;
  }

  tf::TransformListener listener;
  listener.waitForTransform("base_link", tcp_name, ros::Time::now(), ros::Duration(1.0));

  // Execute this trajectory
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.wait_for_execution = true;
  ros::ServiceClient executeKnownTrajectoryServiceClient = node->serviceClient < moveit_msgs::ExecuteKnownTrajectory
      > ("/execute_kinematic_path");

  double percentage = group->computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory);
//    double percentage=1;
  status.data = boost::lexical_cast<std::string>(percentage*100) + "% of the trajectory will be executed";
  status_pub->publish(status);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = srv.request.trajectory;
	    // 执行运动
	    group->execute(plan);
  //executeKnownTrajectoryServiceClient.call(srv);
//    learning_communication::Person msg;
//    msg.finish =true;
//    chatter_pub.pulse
    res.ReturnStatus = true;

  return true;
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "path_planning");
  node.reset(new ros::NodeHandle);

  status_pub.reset(new ros::Publisher);
  *status_pub = node->advertise <std_msgs::String> ("scanning_status", 1);

  // Initialize move group
  group.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));
  group->setPoseReferenceFrame("base_link");
  group->allowReplanning(true);
    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
//    group->setGoalPositionTolerance(0.01);
//    group->setGoalOrientationTolerance(0.01);
  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("path_planning_service", pathPlanning);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
    sleep(1);
  }
  spinner.stop();
  return 0;
}
