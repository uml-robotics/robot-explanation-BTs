#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
// #include "countdown.h"

using namespace BT;

namespace DummyNodes
{
    // Example of custom SyncActionNode (synchronous action)
    // without ports.
    class ApproachObject : public BT::SyncActionNode
    {
      public:
        ApproachObject(const std::string& name) :
            BT::SyncActionNode(name, {})
        {
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {
            std::cout << "ApproachObject: " << this->name() << std::endl;
            //countdown(5);
            return BT::NodeStatus::SUCCESS;
        }
    };


    // Simple function that return a NodeStatus
    BT::NodeStatus CheckBattery()
    {
        std::cout << "[ Battery: OK ]" << std::endl;
        //countdown(5);
        return BT::NodeStatus::SUCCESS;
    }

    // We want to wrap into an ActionNode the methods open() and close()
    class GripperInterface
    {
      public:
        GripperInterface(): _open(true) {}

        NodeStatus open() {
            _open = true;
            std::cout << "GripperInterface::open" << std::endl;
            //countdown(10);
            return NodeStatus::SUCCESS;
        }

        NodeStatus close() {
            std::cout << "GripperInterface::close" << std::endl;
            _open = false;
            //countdown(5);
            return NodeStatus::SUCCESS;
        }

      private:
        bool _open; // shared information
    };
}
