#!/bin/bash

if [ $# -lt 2 ]; then
  echo "Usage: "
  echo "./record.sh <FUNCTION_ID> <FILE_PATH> "
  exit 1
fi

echo "./record.sh $1 $2"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

FUNCTION_ID=$1
FILE_PATH=$2

if [ -d "$FILE_PATH" ]; then
  rm -r "$FILE_PATH"
fi

common_topics=(
  /image_raw/compressed
  /joint_states/mc
  /monitor
  /odom/localization_odom
  /odom/mc_odom
  /rgb_image
)

navigation_topics=(
  /cmd_pos
  /cmd_vel
  /electronic_fence
  /electronic_map
  /laser_scan
  /localization_state
  /navigation_cmd
  /navigation_state
  /navigo/bn/cmn/vis/candidate_local_paths
  /navigo/bn/cmn/vis/evaluated_goal
  /navigo/bn/cmn/vis/global_path
  /navigo/bn/cmn/vis/goal
  /navigo/bn/cmn/vis/goals
  /navigo/bn/cmn/vis/local_path
  /navigo/bn/cmn/vis/predicted_goal
  /navigo/bn/cmn/vis/predicted_goal_path
  /navigo/bn/cmn/vis/relocalization_goal
  /navigo/cm/cmn/vis/polygon_slowdown
  /navigo/cm/cmn/vis/polygon_stop
  /navigo/cm/cmn/vis/selective_stop
  /navigo/cs/cmn/intf/cmd_vel_raw
  /navigo/cs/cmn/intf/cmd_vel_valid
  /navigo/cs/lpc/vis/base_candidate_plan
  /navigo/cs/lpc/vis/best_local_plan
  /navigo/cs/lpc/vis/footprint_collision
  /navigo/cs/lpc/vis/footprint_samples
  /navigo/cs/lpc/vis/free_paths
  /navigo/cs/lpc/vis/laser_point_cloud
  /navigo/cs/lpc/vis/local_planner_goal
  /navigo/cs/lpc/vis/lookahead_point
  /navigo/cs/lpc/vis/received_global_plan
  /navigo/cs/ppc/vis/lookahead_point
  /navigo/cs/ppc/vis/received_global_plan
  /navigo/ea/cmn/intf/nav_error_report
  /navigo/gc/cmn/vis/costmap
  /navigo/gc/cmn/vis/costmap_updates
  /navigo/lc/cmn/vis/costmap
  /navigo/lc/cmn/vis/costmap_updates
  /navigo/lcc/cmn/vis/costmap
  /navigo/lcc/cmn/vis/costmap_updates
  /navigo/ms/cmn/intf/map
  /navigo/ps/cmn/vis/global_esdf_pos_distance
  /navigo/ps/cmn/vis/local_esdf_pos_distance
  /navigo/ps/cmn/vis/planned_path
  /navigo/ps/cmn/vis/quadtree_topo_graph
  /navigo/ps/slp/vis/start_end_visualization
  /navigo/ps/stp/dbg/planner_debug_info
  /navigo/ps/stp/vis/front_end_exploration_traj_visualization
  /navigo/ps/stp/vis/front_end_goal_connection_traj_visualization
  /navigo/ps/stp/vis/front_end_middle_traj_visualization
  /navigo/ps/stp/vis/front_end_shot_traj_visualization
  /navigo/ps/stp/vis/front_end_start_connection_traj_visualization
  /navigo/ps/stp/vis/front_end_traj_visualization
  /navigo/ps/stp/vis/quadtree_dijkstra_progress
  /navigo/ps/stp/vis/quadtree_multi_inflation_results
  /navigo/ps/stp/vis/quadtree_start_goal_grids
  /navigo/ps/stp/vis/safe_drive_corridor
  /navigo/ps/stp/vis/start_end_visualization
  /navigo/ps/stp/vis/tracking_searcher_each_expansion_step_planner_goal
  /navigo/ps/tpg/vis/three_dim_search_path
  /navigo/ps/tpg/vis/three_dim_topo_edges
  /navigo/ps/tpg/vis/three_dim_topo_nodes
  /navigo/wf/cmn/intf/update_tracking_goal
  /navigo/wf/cmn/vis/original_reference_path
  /odom/current_pose
  /perception/detection3d
  /pers/state
  /security_cam/target_in_map
  /set_desired_angular_vel_z
  /set_desired_linear_vel_x
  /set_desired_linear_vel_y
  /set_lookahead_distance
  /start_navigation
  /uwb_point
)

arc_topics=(
  /arc/arc_change_flag
  /arc/arc_module_states_debug_info
  /arc/arc_state
  /arc/arc_state_debug_info
  /arc/calibration_state
  /arc/dock_pose
  /arc/dock_state
  /arc/mc_mode_cmd
  /arc/mc_state
  /arc/nav_coarse_alignment_mode_cmd
  /arc/nav_coarse_alignment_state
  /arc/nav_contact_alignment_mode_cmd
  /arc/nav_contact_alignment_state
  /arc/nav_fine_alignment_mode_cmd
  /arc/nav_fine_alignment_state
  /arc/perception_dock_pose
  /arc/perception_mode_cmd
  /arc/perception_state
  /arc/slam_dock_pose
  /arc/slam_mode_cmd
  /arc/slam_state
  /arc/start_arc
)

uwb_topics=(
  /uwb_point
)

detection_topics=(
  /cluster_points
  /perception/detection2d
  /perception/detection3d
  /perception/fusion/target3d
  /perception/fusion3d
  /perception/image_vis/compressed
  /perception/target3d
  /preprocess_points
  /security_cam/target_in_map
)

segmentation_topics=(
  /seg_vis/compressed
)

lidar_topics=(
  /livox/imu
  /livox/lidar
)

gps_topics=(
  /uni_best_nav
  /uni_heading
  /gnss/data
)

mapping_topics=(
  /path
  /world_points
)

topics=("${common_topics[@]}")

case "$FUNCTION_ID" in
Tracking | TrackingWithObstacles)
  topics+=("${navigation_topics[@]}" "${detection_topics[@]}" "${lidar_topics[@]}" "${gps_topics[@]}")
  ;;
TrackingUWB)
  topics+=("${navigation_topics[@]}" "${uwb_topics[@]}")
  ;;
Patrol | Exploration)
  topics+=("${navigation_topics[@]}" "${lidar_topics[@]}" "${gps_topics[@]}" "${segmentation_topics[@]}" "${arc_topics[@]}")
  ;;
Mapping)
  topics+=("${mapping_topics[@]}" "${lidar_topics[@]}" "${gps_topics[@]}")
  ;;
*)
  echo "Unsupported FUNCTION_ID: $FUNCTION_ID. Use one of {Tracking|TrackingUWB|TrackingWithObstacles|Patrol|Exploration|Mapping}."
  exit 1
  ;;
esac

echo "Recording to bag: $FILE_PATH"
echo "$FUNCTION_ID Topics:"
for t in "${topics[@]}"; do
  echo "  $t"
done

ros2 bag record -s mcap --compression-mode file --compression-format zstd -o "$FILE_PATH" "${topics[@]}"

#ros2 bag record -o "$FILE_PATH" "${topics[@]}"

exit 0
