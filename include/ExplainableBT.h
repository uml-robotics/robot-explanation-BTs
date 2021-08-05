// This file contains Ports and ExplainableBT classes.
// ExplainableBT class contains all the proposed algorithms in the paper.
// The only input needed is a behavior tree. Please refer to 
//   https://www.behaviortree.dev/ to get familiar with it.
// Please use our fork https://github.com/uml-robotics/BehaviorTree.CPP asked
//   it contains modifications that make the algorithms easy to reason about.
// You should also be familiar with ROS.

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "explainBT/Explain.h"
#include "BehaviorTracker.h"
#include <boost/algorithm/string/predicate.hpp> // starts_with

class Ports {
public:
    Ports(const std::unordered_map<std::string, std::string>& ports) : ports(ports) {}

    std::set<std::string> get_keyed_value_set() {
        std::set<std::string> set;
        for (const auto &kv : ports) {
            std::string value = kv.second;
            if (is_keyed(value)) {
                set.emplace(value);
            }
        }
        return set;
    }

    bool has_keyed_value(const std::string &port) {
        std::set<std::string> set;
        for (const auto &kv : ports) {
            std::string value = kv.second;
            if (is_keyed(value)) {
                if (value == port) {
                    return true;
                }
            }
        }
        return false;
    }
private:
    static bool is_keyed(const std::string &value) {
        return boost::starts_with(value, "{");
    }

    const std::unordered_map<std::string, std::string>& ports;
};

class ExplainableBT {
public:
    explicit ExplainableBT(const BT::Tree & tree) : tree(tree), behavior_tracker(tree) {
        printTreeRecursively(tree.root_node);
    }

    BT::NodeStatus execute() {
        return tree.root_node->executeTick();
    }

    bool explain_callback(explainBT::Explain::Request &req, explainBT::Explain::Response &res) {
        const std::string q =  req.what; // question
        std::string a; // answer

        ROS_INFO_STREAM("Q: " << q);
        BT::TreeNode* n = behavior_tracker.get_running_node();
        if (q == "What are you doing?") {
            a = "I " + n->short_description() + ".";
        }
        else if (q == "Why are you doing this?") {
            std::string goal = behavior_tracker.get_running_node_different_control_parent()->name();

            a = "I " + (n->short_description()) + " in order to " + goal + ".";
        }
        else if (q == "What is your subgoal?") {
            BT::TreeNode* tree_parent = behavior_tracker.get_tree_parent();
            if (tree_parent == nullptr)
                a = "Sorry. I don't have a subgoal.";
            else {
                std::string subgoal = tree_parent->name();
                a = "My subgoal is to " + subgoal + ".";
            }
        }
        else if (q == "How do you achieve your subgoal?" || q== "What are the steps for your subgoal?") {
            BT::TreeNode* tree_parent = behavior_tracker.get_tree_parent();
            if (tree_parent == nullptr)
                a = "Sorry. I don't have a subgoal.";
            else {
                std::string goal = tree_parent->name();
                std::vector<BT::TreeNode *> steps = find_steps(tree_parent);
                a = "To achieve the subgoal \"" + goal + "\", I need to do " + std::to_string(steps.size()) +
                    " steps. ";
                for (int i = 0; i < steps.size(); ++i) {
                    a += std::to_string(i + 1) + ". " + steps.at(i)->name() + ". ";
                }
            }
        }
        else if (q == "What is your goal?") {
            std::string goal = behavior_tracker.get_overall_goal_node()->name();

            a = "My goal is to " + goal + ".";
        }
        else if (q == "How do you achieve your goal?" || q == "What are the steps for your goal") {
            std::string goal = behavior_tracker.get_overall_goal_node()->name();
            std::vector<BT::TreeNode*> steps = find_steps(tree.root_node);
            a = "To achieve the goal \"" + goal + "\", I need to do " + std::to_string(steps.size()) + " steps. ";
            for (int i = 0; i < steps.size(); ++i) {
                a += std::to_string(i+1) + ". " + steps.at(i)->name() + ". ";
            }
        }
        else if (q == "What went wrong?") {
            BT::TreeNode *running_node = behavior_tracker.get_running_node();

            bool is_wrong = false;
            bool is_fell_back = false;

            BT::FallbackNode* fallback_node = nullptr;

            BT::TreeNode *p = running_node->getParent();
            while (p != nullptr && p->type() != BT::NodeType::SUBTREE) {
                ROS_INFO_STREAM(p->short_description());

                bool is_fallback_node = (dynamic_cast<BT::FallbackNode*>(p) != nullptr);
                if (is_fallback_node) {
                    //
                    // Fallback node
                    //
                    fallback_node = dynamic_cast<BT::FallbackNode*>(p);
                    ROS_INFO_STREAM("Fallback node found: " << fallback_node->short_description());
                    if (fallback_node->child(0)->status() == BT::NodeStatus::FAILURE) {
                        is_wrong = true;
                        is_fell_back = true;

                        a = "I could not " + fallback_node->short_description() + " because ";

                        // find the failed child
                        const BT::TreeNode* failed_child;
                        BT::applyRecursiveVisitorSelectively(fallback_node, [&failed_child](const BT::TreeNode* node) -> bool {
                            if (node->has_failed() && (node->type() == BT::NodeType::CONDITION || node->type() == BT::NodeType::ACTION)) {
                                failed_child = node;
                                return true;
                            }
                            return false;
                        });

                        if (failed_child->getParent() != nullptr) {
                            if (failed_child->getParent()->short_description() != fallback_node->short_description()) {
                                a += "I was unable to " + failed_child->getParent()->short_description() + " as ";
                            }
                        }

                        a += failed_child->short_description() + " failed.";

                        break;
                    }
                }

                bool is_retry_node = (dynamic_cast<BT::RetryNode*>(p) != nullptr);
                if (is_retry_node) {
                    auto retry_node = dynamic_cast<BT::RetryNode *>(p);
                    ROS_INFO_STREAM("Retry node found: " << retry_node->short_description());

                    if (retry_node->is_retrying()) {
                        is_wrong = true;

                        // check if have non-null parent
                        BT::TreeNode *rp = retry_node->getParent();
                        while (rp == nullptr) {
                            rp = rp->getParent();
                        }

                        if (rp != nullptr) {
                            a = "I am retrying for attempt " + std::to_string(retry_node->n_th_retry()) + " to " + rp->short_description() + ". ";

                            // find the failed child
                            const BT::TreeNode* failed_child;
                            BT::applyRecursiveVisitorSelectively(retry_node, [&failed_child](const BT::TreeNode* node) -> bool {
                                if (node->has_failed() && (node->type() == BT::NodeType::CONDITION || node->type() == BT::NodeType::ACTION)) {
                                    failed_child = node;
                                    return true;
                                }
                                return false;
                            });

                            auto fp = failed_child->getParent();
                            while (fp->name().empty()) {
                                fp = fp->getParent();
                            }

                            a += "I could not " + fp->short_description() + " because " + failed_child->short_description() + " failed.";;

                            break;
                        }
                    }
                }
                p = p->getParent();
            }

            if (is_fell_back) {
                // find if there is a parent Retry node retrying. If so, say that

                p = fallback_node->getParent();
                while (p != nullptr && p->type() != BT::NodeType::SUBTREE) {

                    bool is_retry_node = (dynamic_cast<BT::RetryNode*>(p) != nullptr);
                    if (is_retry_node) {
                        auto retry_node = dynamic_cast<BT::RetryNode *>(p);
                        ROS_INFO_STREAM("Retry node found: " << retry_node->short_description());

                        if (retry_node->is_retrying()) {

                            // check if have non-null parent
                            BT::TreeNode *rp = retry_node->getParent();
                            while (rp == nullptr) {
                                rp = rp->getParent();
                            }

                            if (rp != nullptr) {
                                a += " I am retrying for attempt " + std::to_string(retry_node->n_th_retry()) + " to " + rp->short_description() + ".";
                            }
                        }
                    }
                    p = p->getParent();
                }
            }

            if ( ! is_wrong)
                a = "Nothing went wrong.";
        }
        else if (boost::starts_with(q, "Can you")) {
            std::string asked = q.substr(8, q.size() - 8 - 1);
            ROS_INFO_STREAM(asked);
            a = asked;

            // get tree parent
            BT::TreeNode* tree_parent = behavior_tracker.get_tree_parent();
            if (tree_parent == nullptr)
                tree_parent = tree.root_node;
            ROS_INFO_STREAM("tree parent: " + tree_parent->short_description());

            // build supported nodes
            std::vector<BT::TreeNode*> supported_nodes;
            auto visitor = [&supported_nodes](BT::TreeNode* node) {
                bool is_sequence_node = (dynamic_cast<BT::SequenceNode*>(node) != nullptr);
                if ((is_sequence_node || node->type() == BT::NodeType::SUBTREE) && (! node->name().empty())) {

                    // if a node with the same name is added, don't add this node
                    bool has_same_name_node_added = false;
                    for (auto sn : supported_nodes) {
                        if (sn->name() == node->name()) {
                            has_same_name_node_added = true;
                            break;
                        }
                    }

                    if ( ! has_same_name_node_added) {
                        ROS_INFO_STREAM("V " << node->short_description());
                        supported_nodes.emplace_back(node);
                    }
                }
            };
            applyRecursiveVisitor(tree_parent, visitor);

            // find the node, reply if not supported
            BT::TreeNode* supported_node;
            bool is_supported = false;
            for (auto sn : supported_nodes) {
                if (sn->name() == asked) {
                    is_supported = true;
                    supported_node = sn;
                    break;
                }
            }
            if ( ! is_supported) {
                a = "Sorry. I cannot redo \"" + asked + "\".";
            }
            else {

                ROS_INFO_STREAM("\"" + asked + "\" is supported");

                auto ns = find_self_contained_behavior_node(supported_node);
                auto goal = behavior_tracker.get_overall_goal_node();
                auto subgoal = behavior_tracker.get_tree_parent();
                ROS_INFO_STREAM("Checking " << goal->UID() << " vs. " << subgoal->UID());
                if (goal->UID() == subgoal->UID()) {
                    ROS_INFO_STREAM("must have a subtree...");
                } else {
                    auto goal_sequence = dynamic_cast<BT::SequenceNode *>(goal);
                    std::cout << "Before: " << goal_sequence->childrenCount();
                    goal_sequence->insertChildAfter(ns, subgoal);
                    std::cout << "After: " << goal_sequence->childrenCount();
                    BT::printTreeRecursively(goal);

                    a = "Yes. I will " + asked + " after " + subgoal->name() + ".";
                }
            }
        }
        else {
            a = "Sorry. I don't understand that \"" + req.what + "\"";
        }

        ROS_INFO_STREAM("A: " << a);
        res.reply = a;
        return true;
    }

private:

    BT::TreeNode* find_self_contained_behavior_node(BT::TreeNode* supported_node) {

        // build unique, keyed input ports
        std::set<std::string> unique_keyed_input_ports;
        applyRecursiveVisitor(supported_node, [&unique_keyed_input_ports](BT::TreeNode *node) {
            if (node->type() == BT::NodeType::ACTION) { // action node only
                auto set = Ports(node->config().input_ports).get_keyed_value_set();
                unique_keyed_input_ports.insert(set.begin(), set.end());
            }
        });

        // find an ancestor node providing all dynamic input ports
        auto ns = supported_node;
        for (auto &dynamic_input : unique_keyed_input_ports) {
            ROS_INFO_STREAM("looking for " << dynamic_input);
            bool found = false;

            while ( ! found) {
                applyRecursiveVisitor(ns, [&found, &dynamic_input](BT::TreeNode *node) {
                    if (found) {
                        return;
                    }

                    if (node->type() == BT::NodeType::ACTION) { // action node only
                        if (Ports(node->config().output_ports).has_keyed_value(dynamic_input)) {
                            ROS_INFO_STREAM("found " << dynamic_input << "from node: " << node->short_description());
                            found = true;
                        }
                    }
                });

                if (!found) { // go up a level
                    ns = ns->getParent();
                    ROS_INFO_STREAM("went up to: " << ns->short_description());
                }
            }
        }

        return ns;
    }

    std::vector<BT::TreeNode*> find_steps(BT::TreeNode* parent_node) {
        std::vector<BT::TreeNode*> steps;

        auto visitor = [this, &parent_node, &steps](BT::TreeNode* node) -> bool {
            if ( node->name().empty() || node->name() == parent_node->name() || node->type() == BT::NodeType::DECORATOR) {
                ROS_INFO_STREAM("X " << node->short_description());
                return false;
            }
            else {
                ROS_INFO_STREAM("V " << node->short_description());
                steps.emplace_back(node);
                return true;
            }
        };

        applyRecursiveVisitorSelectively(parent_node, visitor);

        return steps;
    }

    const BT::Tree &tree;
    BehaviorTracker behavior_tracker;
};
