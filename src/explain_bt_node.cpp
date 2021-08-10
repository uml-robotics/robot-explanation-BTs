#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "explain_bt/Explain.h"
#include "ExplainableBT.h"
#include "dummy_nodes.h"

using namespace BT;

int main (int argc, char **argv)
{
   BehaviorTreeFactory factory;

   using namespace DummyNodes;
   factory.registerNodeType<ApproachObject>("ApproachObject");
   factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
   GripperInterface gripper;
   factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
   factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));
   
   ROS_INFO("Explainble BT node started.");
      
   auto tree = factory.createTreeFromFile("./my_tree.xml");
   ROS_INFO("BT created from file.");

   return 0;
}