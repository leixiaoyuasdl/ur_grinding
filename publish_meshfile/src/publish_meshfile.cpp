/************************************************
Publish_meshfile file code
This file which displays the CAD mesh in rviz
represents one node of the entire
demonstrator
************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>

#include <fanuc_grinding_publish_meshfile/PublishMeshfileService.h> // Description of the Service we will use

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

boost::shared_ptr<ros::NodeHandle> node;

boost::shared_ptr<ros::Publisher> pub;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Alway true
 */
bool publishMeshFile(fanuc_grinding_publish_meshfile::PublishMeshfileService::Request &req,
                     fanuc_grinding_publish_meshfile::PublishMeshfileService::Response &res)
{

  if (req.ColorA <= 0.1)
    ROS_WARN_STREAM("publishMeshFile: Alpha is set to 0, mesh will be invisible!");
  pcl::PolygonMesh mesh;

  if (boost::filesystem::path (req.MeshName).extension () == ".ply")
  {
    if (pcl::io::loadPLYFile(req.MeshName, mesh))
    {
      ROS_ERROR_STREAM("publishMeshFile: Could not read PLY file");
      return true;
    }
  }
  else
  {
      std::cout<<req.MeshName<<std::endl;

    pcl::io::loadPolygonFile(req.MeshName, mesh);
  }
  visualization_msgs::Marker marker;
  if(mesh.polygons.size() == 0)
  {
    ROS_WARN_STREAM("publishMeshFile: There is no polygon in file, publishing a point cloud");
    *pub = node->advertise<sensor_msgs::PointCloud2>(req.MarkerName, 1, true);
    mesh.cloud.header.frame_id = "base_link";
    pub->publish(mesh.cloud);
  }
  else
  {
    *pub = node->advertise<visualization_msgs::Marker>(req.MarkerName, 1, true); // Latched

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = req.PosX;
    marker.pose.position.y = req.PosY;
    marker.pose.position.z = req.PosZ;
    marker.pose.orientation.x = req.RotX;
    marker.pose.orientation.y = req.RotY;
    marker.pose.orientation.z = req.RotZ;
    marker.pose.orientation.w = req.RotW;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = req.ColorA; // Don't forget to set the alpha!
    marker.color.r = req.ColorR;
    marker.color.g = req.ColorG;
    marker.color.b = req.ColorB;
    marker.lifetime = ros::Duration();
    // Only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "file://" + req.MeshName;
    pub->publish(marker);
  }
  while (pub->getNumSubscribers() < 1)
  {
    ROS_WARN_STREAM("publishMeshFile: No subscriber to the marker: " + req.MarkerName);
    if (!req.WaitForSubscriber)
      break;
    sleep(1);
  }

  res.ReturnStatus = true;
  res.ReturnMessage = req.MarkerName + " published";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_meshfile");

  node.reset(new ros::NodeHandle);
  pub.reset(new ros::Publisher);
  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("publish_meshfile_service", publishMeshFile);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (node->ok())
  {
    sleep(1);
  }

  spinner.stop();
  return 0;
}
