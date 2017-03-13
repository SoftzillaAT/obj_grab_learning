#include "obj_grab_learning/obj_grab_learning_node.h"

using namespace std;

namespace obj_grab_learning {

  /* constructor */
  ObjGrabLearning::ObjGrabLearning(ros::NodeHandle &nh, string topic) : 
    node_handle(&nh), 
    client("metahand_grasp_server", true), 
    drop("metahand_drop_server", true), 
    place("metahand_place_server", true),
    metahand("hand_controller/actuate_hand", true)
  {

    node_handle = &nh;
    ROS_INFO("Waiting for servers....");

    client.waitForServer();
    drop.waitForServer();
    place.waitForServer();
    metahand.waitForServer();
    ROS_INFO("Servers ready...");
  }

  void ObjGrabLearning::runDemo()
  {

    ROS_INFO("Run demo!");
    squirrel_manipulation_msgs::BlindGraspGoal grasp_goal;
    grasp_goal.object_id = "test";
    grasp_goal.heap_center_pose.header.stamp = ros::Time::now();
    grasp_goal.heap_center_pose.header.frame_id = "origin";
    grasp_goal.heap_center_pose.pose.position.x = 0.489;
    grasp_goal.heap_center_pose.pose.position.y = -0.416;
    grasp_goal.heap_center_pose.pose.orientation.x = 0.0;
    grasp_goal.heap_center_pose.pose.orientation.y = 0.0;
    grasp_goal.heap_center_pose.pose.orientation.z = 0.0;
    grasp_goal.heap_center_pose.pose.orientation.w = 1.0;
    grasp_goal.heap_bounding_cylinder.diameter = 0.2;
    grasp_goal.heap_bounding_cylinder.height = 0.08;


    kclhand_control::ActuateHandGoal open_hand;
    open_hand.command = 0;
    open_hand.force_limit = 1;
    client.sendGoal(grasp_goal);
    client.waitForResult();

    ROS_INFO("Enter 'd' for drop and 'p' for placement");
    char c = getchar();
    
    switch(c)
    { 
      case 'd': {
        ROS_INFO("Dropping:");
        squirrel_manipulation_msgs::DropGoal drop_goal;
        drop_goal.destination_id = "somewhere";
        drop.sendGoal(drop_goal);
        ROS_INFO("Waiting for completion...");
        drop.waitForResult();
        ROS_INFO("Dropping complete.");
        }
        break;

      case 'p': {
          ROS_INFO("Placing");
          squirrel_manipulation_msgs::PutDownGoal put_down_goal;
          put_down_goal.destination_id = "somewhere";
          put_down_goal.destPoseSE2.header.frame_id = "origin";
          put_down_goal.destPoseSE2.pose.position.x = 0.30;
          put_down_goal.destPoseSE2.pose.position.y = -0.515;
          put_down_goal.destPoseSE2.pose.position.z = 0.2;
          put_down_goal.destPoseSE2.pose.orientation.w = -0.408;
          put_down_goal.destPoseSE2.pose.orientation.x = 0.896;
          put_down_goal.destPoseSE2.pose.orientation.y = 0.154;
          put_down_goal.destPoseSE2.pose.orientation.z = 0.088;

          place.sendGoal(put_down_goal);
          ROS_INFO("Waiting for completion...");
          place.waitForResult();
          ROS_INFO("Placing complete.");
          }
          break;

      default:
         ROS_INFO("Invalid input. End now!");
    }
  }

} // close namespace

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "obj_grab_learning");
  if(!ros::isInitialized())
    return 1;
  ros::NodeHandle n;
  ROS_INFO("OBJECT GRAB LEARNING STARTET v2!");

  obj_grab_learning::ObjGrabLearning my_node(n, "metahand_grasp_server");
  ROS_INFO("Class crated");
  my_node.runDemo();

  ros::spin();
  return 0;
}
