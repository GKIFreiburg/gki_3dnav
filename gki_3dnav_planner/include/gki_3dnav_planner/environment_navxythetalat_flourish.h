#ifndef ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H
#define ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H

#include <cstdio>
#include <vector>
#include "gki_3dnav_planner/environment_navxythetalat_generic.h"
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <navigation/mapping/metaMap/base/traversableMap.h>
#include <sbpl/utils/key.h>
#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include "timing/timing.h"

#include <nav_msgs/Path.h>
#include <interactive_markers/interactive_marker_server.h>

/// Planning environment for x, y, theta planning with 2.5d collision checking for the BoniRob.
/**
 * There are multiple continuous and discretized coordinate systems.
 *
 * External interface with continuous (double) coordinates:
 * These are assumed to be in the scene's planning frame. If the costmap is in
 * another frame, no conversion is performed and planning will likely fail.
 * Thus, for now, the costmap's frame must be identical to the scene's planning
 * frame.
 *
 * Environment continuous coordinates:
 * The environment assumes a coordinate system with origin at 0, 0 that can be
 * directly discretized to the internal grid. This should be used internally
 * only.
 *
 * Discretized coordinates:
 * These are always discretized with the same linear and angular resolution for
 * all discretized coordinate systems. The angular resolution is given by
 * NumThetaDirs, the linear resolution is cellsize_m and should be identical to
 * the costmap resolution and the motion primitive's resolution. There are
 * possibly two discretizatons:
 *
 * Internal environment coordinates:
 * The actual underlying x, y, theta indices in the environment used for
 * planning, i.e., the ones that are converted to/from SBPL state IDs.  This is
 * usually identical with the internal grid in the environment that is used for
 * the cost lookup.
 * All functions taking discretized coordinates use these coordinates!
 * For now, the grid is 0-indexed, initialized with the costmap size and the
 * 0,0 grid index lies at the costmap 0, 0.
 *
 * Costmap: The x, y indices of the ROS costmap. For now the internal grid is
 * initialized from the same size and offset as the costmap. If that is ever to
 * be changed, i.e., to adapt to a costmap too small to fit the octomap size,
 * care has to be taken that all functions taking grid coordinates are used
 * correctly.
 * All functions in the environment use environment indices.
 * External use, e.g., updating the grid, usually from the costmap cannot assume
 * that the grid coordinates are valid for costmap coordinates. These functions
 * would need to use the ...Costmap style function, which convert.
 */
class EnvironmentNavXYThetaLatFlourish : public EnvironmentNavXYThetaLatGeneric
{
 public:
  EnvironmentNavXYThetaLatFlourish(ros::NodeHandle& nhPriv);
  virtual ~EnvironmentNavXYThetaLatFlourish();

  // check if the cell given by indices x, y is inside the map and the height is safe for the robot
  bool IsValidCell(int X, int Y);
  // check if the cell given by indices x, y is inside the map 
  bool IsWithinMapCell(int X, int Y);
  // check if all cells in the footprint given the robot is at pose x, y, theta are valid. Uses isValidCell.
  bool IsValidConfiguration(int X, int Y, int Theta);
  // check if all cells in the footprint given the robot is at pose are valid. Uses IsValidCell.
  bool IsValidConfiguration(sbpl_xy_theta_pt_t pose);
  // check if the traversable map says cell x, y is traversable or not
  bool IsCloseToObstacle(int x, int y);
  bool IsCloseToObstacle(Eigen::Vector2i index);
  double getMapOffsetX() { return mapOffsetX; }
  double getMapOffsetY() { return mapOffsetY; }
  const Ais3dTools::TraversableMap& traversableMap() const { return tMap; }

  // if x, y, theta is a valid configuration:
  // sets the start state for the planning task to x_m, y_m, theta_rad
  // creates a hash entry with the corresponding state id, returns the state id
  // else: returns -1
  virtual int SetStart(double x_m, double y_m, double theta_rad);
  // if x, y, theta is a valid configuration:
  // sets the goal state for the planning task to x_m, y_m, theta_rad
  // creates a hash entry with the corresponding state id, returns the state id
  // else: returns -1
  virtual int SetGoal(double x_m, double y_m, double theta_rad);
  // sets the parameter on whether we want to use the freespace heuristic
  // bool useFreespaceHeuristic(bool on) { useFreespaceHeuristic_ = on; }
  // create a new hash entry for the given state
  virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);
  virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);

  /// Update internal representation of the planner for a plan request.
  /**
   * Called, whenever makePlan is called. Start and Goal state have already been set and
   * env_->updateForPlanRequest() has also been called.
   *
   * This object will be deleted after the query.
   *
   * If possible should return a XYThetaStateChangeQuery that can be passed to the search algorithm.
   * If NULL is returned, the planner will plan from scratch.
   */
  //virtual sbpl_xytheta_planner::XYThetaStateChangeQuery* updateForPlanRequest();
  virtual void updateForPlanRequest();

  /// Returns the scene's planning frame.
  virtual std::string getPlanningFrame() const;

  /// Transform a pose in any frame by a suitable method to the planning frame.
  virtual bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped & pose);

  /// Get the x/y dimensions of the OctoMap.
  virtual bool getExtents(double& minX, double& maxX, double& minY, double& maxY);

  // delete all entries in the full_body_traversability_cost_infos
  virtual void clear_full_body_traversability_cost_infos();

  // get a geometry_msgs::Pose from a given state id
  geometry_msgs::Pose poseFromStateID(int stateID) const;
  
  // compute and publish the wheel cells relative to the current state 
  // TODO check
  void computeWheelPositions();

  virtual moveit_msgs::DisplayTrajectory pathToDisplayTrajectory(const std::vector<geometry_msgs::PoseStamped>& path) const;
  void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath, std::vector<sbpl_xy_theta_pt_t>* xythetaPath);
  void resetTimingStats();
  void printTimingStats();


  // get euclidean distance from state FromStateID to state ToStateID in m
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
  // get euclidean distance from start in m
  virtual int GetStartHeuristic(int stateID);
  // get euclidean distance to goal in m
  virtual int GetGoalHeuristic(int stateID);
  //int getFreespaceCost(int deltaX, int deltaY, int theta_start, int theta_end);

  // not doing anything right now
  void EnsureHeuristicsUpdated(bool bGoalHeuristics);
  //bool UpdateCost(int x, int y, unsigned char newcost);
  //unsigned char GetMapCost(int x, int y);

  /// Update the planning scene directly from the running MoveGroup instance.
  virtual void update_planning_scene();
  /// Update the planning scene to a custom one.
  virtual void update_planning_scene(planning_scene::PlanningSceneConstPtr scene);
  /// Use this to access the PlanningScene. Never use internal data structures for that.
  virtual planning_scene::PlanningSceneConstPtr getPlanningScene() const;

  //int ComputeCosts(int SourceX, int SourceY, int SourceTheta);

  // DEBUGGING
  /// Publish the currently used planning scene instance.
  virtual void publish_planning_scene();
  //virtual void publish_expanded_states();
  void publish_wheel_cells(std::vector<Eigen::Vector2i> wheelCells);
  void publish_traversable_map();
  void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void checkPlanValidity(const nav_msgs::Path& plan);
  
  int count;
  int past;

 protected:
  struct FullBodyTraversabilityCost
  {
    int cost;
    bool initialized;

    FullBodyTraversabilityCost()
    {
      initialized = false;
      cost = INFINITECOST;
    }
  };


  bool wheelCellsValid(Eigen::Isometry3f worldToBaseLink);
  
  // compute the cost of state x, y, theta 
  // if a wheel position is outside the map: infty
  // if the cell x, y is outside the map or too high: infty
  // else: cost = distance to goal
  int GetCellCost(int X, int Y, int Theta);
  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);
  virtual void SetConfiguration(int width, int height,
			const unsigned char* mapdata,
			int startx, int starty, int starttheta,
			int goalx, int goaly, int goaltheta,
			double cellsize_m, double nominalvel_mpersecs,
			double timetoturn45degsinplace_secs,
			const std::vector<sbpl_2Dpt_t> & robot_perimeterV);

  // functions to convert from grid indices to world coordinates (cell center)
  sbpl_xy_theta_pt_t gridToWorld(int x, int y, int theta) const;
  void gridToWorld(int x_d, int y_d, int theta_d, double& x_c, double& y_c, double& theta_c) const;
  void grid2dToWorld(int x_d, int y_d, double& x_c, double& y_c) const;
  void worldToGrid(sbpl_xy_theta_pt_t pose, int& x, int& y, int& theta) const;
  void worldToGrid(double x_c, double y_c, double theta_c, int& x, int& y, int& theta) const;
  void world2dToGrid(double x_c, double y_c, int& x, int& y) const;
  Eigen::Vector2i world2dToGrid(Eigen::Vector2f xy_c) const;

  bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
  const FullBodyTraversabilityCost& get_full_body_traversability_cost_info(EnvNAVXYTHETALATHashEntry_t* state);

  std::vector<FullBodyTraversabilityCost> full_body_traversability_cost_infos;
  //freespace_mechanism_heuristic::HeuristicCostMap* freespace_heuristic_costmap;
  //bool useFreespaceHeuristic_;

  planning_scene::PlanningScenePtr scene;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
  std::string scene_update_name;  ///< Scene updates are queried by this service.
  ros::Publisher planning_scene_publisher;
  ros::Publisher wheel_cells_publisher;
  ros::Publisher traversable_map_publisher;
  std::vector<std::string> allowed_collision_links;

  // offsets to convert costmap coordinates to world coordinates for 3d collision checks
  double mapOffsetX;
  double mapOffsetY;
  Ais3dTools::TraversableMap tMap;
  Eigen::Vector3f frWheelInRobotCoordinates, flWheelInRobotCoordinates, rrWheelInRobotCoordinates, rlWheelInRobotCoordinates;

  Timing* timeActionCost;
  //Timing* timeActionCostParent;
  Timing* timeFullBodyCollision;
  Timing* timeConfigCollisionCheck;
  Timing* timeTrafoComputation;

  float robotHeight;
  float robotBodyHeight;
  float robotBodyWidth; 
  float robotBodyLength;
  float robotArmLength;
  float robotMinSafeDistance;
  float robotSafeHeight;

  tf::TransformListener* tfListener;

  std::string planningFrameID;
  
  //DEBUG
  ros::Publisher nontravaction_array_publisher;
  ros::Publisher action_array_publisher;
  ros::Publisher endtheta_array_publisher;
  ros::Publisher nontrav_endtheta_array_publisher;
  //ros::Subscriber plan_subscriber;
  interactive_markers::InteractiveMarkerServer* interserver;
};

#endif // ENVIRONMENT_NAVXYTHETALAT_FLOURISH_H
