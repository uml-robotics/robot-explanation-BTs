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
   
   auto tree = factory.createTreeFromFile("/root/catkin_ws/src/robot-explanation-BTs/src/my_tree.xml");
   ROS_INFO("BT created from file.");

   ExplainableBT explainable_tree(tree);

   ros::init(argc, argv, "explainable_bt_server");
   ros::NodeHandle n;
   ros::ServiceServer service = n.advertiseService("explainable_bt", &ExplainableBT::explain_callback, &explainable_tree);
   ros::AsyncSpinner spinner(1); // Use 1 thread
   spinner.start();
   
   ROS_INFO("Explainable BT node started.");

   // This executeTick method runs the BT without explainability
   // tree.root_node->executeTick();
   
   // This option runs the BT with explainability
   explainable_tree.execute();

   ros::waitForShutdown();

   return 0;
}