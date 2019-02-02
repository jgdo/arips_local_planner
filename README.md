# arips_local_planner

Local planner plugin implementing the ROS [base_local_planner](http://wiki.ros.org/base_local_planner) interface for 2D robot navigation.

This package should be seen as an alpha version being still under construction.

## Basic idea: 

Create a potential field starting from the goal position and move the robot into the direction of the negative gradient of the potential field. 

Note that the potential field is different from the scoring approach used by the standard ROS [dwa planner](http://wiki.ros.org/dwa_local_planner) / [Trajectory Rollout](http://wiki.ros.org/base_local_planner), since there the obstacle/path/goal costs are added together element-wise, making it had to find parameters which make the robot avoid going too close too obstacles but still allow passing narrow passages. 

The potential field is created by the Dijkstra algorithm, using the inflated obstacle costmap to score the movement from one cell to an other. This way the potential field always pulls the robot even through very narrow passages, and at the same time tries to keep the most possible distance from obstacles. 

The planner is best suited for robots which are either holonomic or can rotate in place (e.g. differential drive).
