#include <iostream>
#include <fstream>
#include <string>
#include "ros/ros.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "explainBT/Explain.h"
#include "ExplainableBT.h"
using namespace std;


int main (int argc, char **argv)
{
   ROS_INFO("Explainble BT node started.");
   
   
   BehaviorTreeFactory factory;
   auto tree = factory.createTreeFromFile("./my_tree.xml");
   ROS_INFO("BT created from file.");

   return 0;
}