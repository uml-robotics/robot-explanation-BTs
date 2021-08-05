#pragma once

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

class BehaviorTracker : public BT::StdCoutLogger {
public:
    BehaviorTracker(const BT::Tree &tree) : tree(tree), StdCoutLogger(tree) {

    }

    void callback(BT::Duration timestamp, const BT::TreeNode &node, BT::NodeStatus prev_status, BT::NodeStatus status) override {
        ticking_node_uid_ = node.UID();

        StdCoutLogger::callback(timestamp, node, prev_status, status);
    }

    BT::TreeNode* get_running_node() {
        BT::TreeNode *running_node = nullptr;
        BT::applyRecursiveVisitor(tree.root_node, [this, &running_node](BT::TreeNode* node_visiting) {
            if (running_node != nullptr) {
                return;
            }

            if (node_visiting->UID() == ticking_node_uid_) {
                running_node = node_visiting;
            }
        });
        return running_node;
    }

    BT::TreeNode* get_running_node_different_control_parent() {
        BT::TreeNode *running_node = this->get_running_node();

        BT::TreeNode *p = running_node->getParent();
        while (p->name().empty() || p->short_description() == running_node->short_description()) {
            p = p->getParent();
        }

        return p;
    }

    BT::TreeNode* get_overall_goal_node() {
        return tree.root_node;
    }

    BT::TreeNode* get_tree_parent() {
        BT::TreeNode *running_node = this->get_running_node();

        BT::TreeNode *p = running_node->getParent();
        while (p != nullptr && p->type() != BT::NodeType::SUBTREE) {
            p = p->getParent();
        }

        return p;
    }

private:
    const BT::Tree &tree;
    uint16_t ticking_node_uid_;
};
