# Robot Explanation Generation Using Behavior Trees (BTs)

This reposotory contains code and the behavior trees in XML for the following manuscript submitted to ACM Transactions on Human-Robot Interaction (THRI).

 - Zhao Han, Daniel Giger, Jordan Allspaw, Michael S. Lee, Henny Admoni, and Holly A. Yanco. __Building The Foundation of Robot Explanation Generation Using Behavior Trees__.

## Code

`ExplainableBT.h` contains all the proposed algorithms, implemented in C++ using the [BehaviorTree.CPP](https://www.behaviortree.dev/) library. Pease read the header comment for its usage.

 - It also depends on `BehaviorTracker.h`, which tracks the node currently ticking in order to generate explanations related to it.

The `main_service` folder contains a ROS service package to answer the 5 questions mentioned in the paper as well as dynamic behavior insertion as subgoal.

```bash
rosservice call /explain "what: 'What are you doing?'"
rosservice call /explain "what: 'Why are you doing this?'"
rosservice call /explain "what: 'What is your goal?'"
rosservice call /explain "what: 'How do you achieve your goal?'"
rosservice call /explain "what: 'What is your subgoal?'"
rosservice call /explain "what: 'How do you achieve your subgoal?'"

rosservice call /explain "what: 'Can you place screw into caddy?'"
rosservice call /explain "what: 'Can you pick screw?'"
```

## Behavior Trees in XML

`kitting-tree.xml` contains the tree of the meta kitting task, including the 3 tasks mentioned in the paper:

1. go pick screw
2. go place screw
3. go insert large gear

`taxi-domain-tree.xml` contains the tree for the optimal policy of the Taxi domain.
