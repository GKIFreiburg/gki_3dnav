#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "gki_3dnav_planner/environment_navxythetalat_moveit.h"
#include <sbpl/headers.h>
#include <geometry_msgs/Pose2D.h>
#include <gki_3dnav_planner/SampleValidPoses.h>
#include <nav_core/base_global_planner.h>
#include "gki_3dnav_planner/sbpl_xytheta_planner.h"

namespace gki_3dnav_planner
{

class GKI3dNavPlanner : public sbpl_xytheta_planner::SBPLXYThetaPlanner
{
public:
    GKI3dNavPlanner();
    GKI3dNavPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    virtual ~GKI3dNavPlanner() { }

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    virtual EnvironmentNavXYThetaLatGeneric* createEnvironment(ros::NodeHandle & nhPriv);
    virtual bool initializeEnvironment(const std::vector<sbpl_2Dpt_t> & footprint,
            double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename);

//    /**
//   * @brief Given a goal pose in the world, compute a plan
//   * @param start The start pose
//   * @param goal The goal pose
//   * @param plan The plan... filled by the planner
//   * @return True if a valid plan was found, false otherwise
//   */
//    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
//                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    /**
   * @brief Given a goal pose in the world, compute a plan
   * @param scene The initial scene specified as a moveit planning scene
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
    bool makePlanFromScene(planning_scene::PlanningSceneConstPtr scene,
                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

protected:
    unsigned char determinePossiblyCircumscribedCostmapCost(costmap_2d::Costmap2DROS* costmap_ros);
    unsigned char costMapCostToSBPLCost(unsigned char newcost);

    virtual sbpl_xytheta_planner::XYThetaStateChangeQuery* updateForPlanRequest();

protected:
    unsigned char lethal_obstacle_;             ///< lethal_obstacle scaled threshold for env
    unsigned char inscribed_inflated_obstacle_; ///< inscribed thresh for env
    unsigned char sbpl_cost_multiplier_;        ///< scaling multiplier env_cost * this = costmap_cost

    EnvironmentNavXYThetaLatMoveit* my_env_; // to avoid dynamic_cast from the generic env_
};

}

#endif

