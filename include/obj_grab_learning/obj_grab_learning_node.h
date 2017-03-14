#ifndef OBJ_GRAB_LEARNING_NODE_H
#define OBJ_GRAB_LEARNING_NODE_H

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>

#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/AddWaypoint.h"
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/CreatePRM.h>
#include <rosplan_knowledge_msgs/Filter.h>

#include "mongodb_store/message_store.h"

// action lib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "squirrel_manipulation_msgs/DropAction.h"
#include "squirrel_manipulation_msgs/BlindGraspAction.h"
#include "squirrel_manipulation_msgs/PutDownAction.h"
#include "kclhand_control/ActuateHandAction.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"

namespace obj_grab_learning {

	class ObjGrabLearning {

		private:
			// Node Handle
			ros::NodeHandle* node_handle;
      actionlib::SimpleActionClient<squirrel_manipulation_msgs::BlindGraspAction> client; 
      actionlib::SimpleActionClient<squirrel_manipulation_msgs::DropAction> drop;
      actionlib::SimpleActionClient<squirrel_manipulation_msgs::PutDownAction> place;
      actionlib::SimpleActionClient<kclhand_control::ActuateHandAction> metahand;
      actionlib::SimpleActionClient<squirrel_object_perception_msgs::LookForObjectsAction> look;
    public:

			/* constructor */
			ObjGrabLearning(ros::NodeHandle &nh, std::string topic);
			void graspObject();
      void startRecognition();

	};


}

#endif
