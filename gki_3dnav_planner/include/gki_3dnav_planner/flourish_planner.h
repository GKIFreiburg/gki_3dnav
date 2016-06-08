#ifndef FLOURISH_PLANNER_H
#define FLOURISH_PLANNER_H

#include <iostream>
#include <vector>

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

#include "gki_3dnav_planner/environment_navxythetalat_flourish.h"
#include <geometry_msgs/Pose2D.h>

#include <gki_3dnav_planner/SampleValidPoses.h>

//global representation
#include "gki_3dnav_planner/sbpl_xytheta_planner.h"
#include <nav_core/base_global_planner.h>


namespace flourish_planner
{

  class FlourishPlanner: public sbpl_xytheta_planner::SBPLXYThetaPlanner
  {
  public:

    FlourishPlanner();
    FlourishPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    virtual ~FlourishPlanner() { }

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
 
    virtual EnvironmentNavXYThetaLatGeneric* createEnvironment(ros::NodeHandle & nhPriv);
    virtual bool initializeEnvironment(const std::vector<sbpl_2Dpt_t> & footprint,
				       double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename);
    
  protected:
    //unsigned char costMapCostToSBPLCost(unsigned char newcost);
    unsigned char determinePossiblyCircumscribedCostmapCost(costmap_2d::Costmap2DROS* costmap_ros);

    //unsigned char lethal_obstacle_;             ///< lethal_obstacle scaled threshold for env
    //unsigned char inscribed_inflated_obstacle_; ///< inscribed thresh for env
    //unsigned char sbpl_cost_multiplier_;        ///< scaling multiplier env_cost * this = costmap_cost
  };
}
;

#endif

