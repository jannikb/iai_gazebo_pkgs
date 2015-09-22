#include <iai_gazebo_visibility_mover/visibility_mover.hpp>
#include <gazebo_msgs/SpawnModel.h>

using namespace iai_gazebo;

VisibilityMover::VisibilityMover(const ros::NodeHandle& nh) :
    nh_( nh )
{
}

VisibilityMover::~VisibilityMover()
{
}

bool VisibilityMover::start()
{
  if(!nh_.getParam("robot_description", robot_description_))
  {
    ROS_ERROR("[%s] Could not find param 'robot_description' in namespace '%s'",
        nh_.getNamespace().c_str(), nh_.getNamespace().c_str());
    return false;
  }

  spawn_urdf_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  if(!spawn_urdf_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/spawn_urdf_model'",
        nh_.getNamespace().c_str());
    return false;
  }

  // TODO: remove this from here and put it into the service callback
  if(!spawnUrdf())
    return false;

  return true;
}

bool VisibilityMover::spawnUrdf()
{
  gazebo_msgs::SpawnModel srv;
  srv.request.model_name = "Boxy";
  srv.request.model_xml = robot_description_;
  srv.request.robot_namespace = "boxy";

  if(spawn_urdf_client_.call(srv))
  {
    if(!srv.response.success)
    {
      ROS_ERROR("[%s] Spawn Urdf unsuccessful: %s", nh_.getNamespace().c_str(),
        srv.response.status_message.c_str()); 
      return false;
    }
    else 
      return true;
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/spawn_urdf_model'.",
        nh_.getNamespace().c_str());
    return false;
  }
}
