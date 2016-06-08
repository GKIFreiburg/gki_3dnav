Comparison evaluation
=====================

Preparation
-----------
- Code Feature switchable via param
- Maps, etc. prepared

Steps
-----
1. Sample poses in environment
2a. Form pairs of poses as queries
2b. Evaluate each query on both features
3. Analyze results

Pose Sampling
-------------
rostopic echo /valid_poses > poses_map_50.yaml
rosrun gki_3dnav_planner sample_valid_poses N max_tries, e.g.
rosrun gki_3dnav_planner sample_valid_poses 50 5000 for getting 50 samples

Evaluation
----------
Fix evaluate_pose_queries.cpp, so that it switches the param for the query (TODO, generalize)
Start move_base, planner, etc. so it can run
Start eval runner:
rosrun gki_3dnav_planner evaluate_pose_queries
Send poses:
cat poses_map_50.yaml | rostopic pub /valid_poses geometry_msgs/PoseArray
Runner should start and give: evaluate_pose_queries.yaml

Analyze
-------
rosrun gki_3dnav_planner analyze_queries.py evaluate_pose_queries.yaml

Autoscaling behavior needs some fixing, currently as min/max per stat hardcoded.

