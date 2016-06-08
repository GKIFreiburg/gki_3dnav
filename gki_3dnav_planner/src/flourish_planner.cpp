/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mike Phillips
 *********************************************************************/

#include <gki_3dnav_planner/flourish_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
//#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>
#include <gki_3dnav_planner/PlannerStats.h>
#include <sbpl/planners/planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iomanip>

#include "color_tools/color_tools.h"

#include <costmap_2d/inflation_layer.h>
#include <eigen_conversions/eigen_msg.h>

PLUGINLIB_EXPORT_CLASS(flourish_planner::FlourishPlanner, nav_core::BaseGlobalPlanner);

namespace flourish_planner{

  /*class LatticeSCQ: public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNavXYThetaLatFlourish* env, std::vector<nav2dcell_t> const & changedcellsV) :
      env_(env), changedcellsV_(changedcellsV)
    {
    
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const
    {
      if (predsOfChangedCells_.empty() && !changedcellsV_.empty())
	env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const
    {
      if (succsOfChangedCells_.empty() && !changedcellsV_.empty())
	env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNavXYThetaLatFlourish * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
  };

  FlourishPlanner::FlourishPlanner() :
    initialized_(false),
    costmap_ros_(NULL),
    forward_search_(false),
    initial_epsilon_(0),
    env_(NULL),
    sbpl_cost_multiplier_(0),
    force_scratch_limit_(0),
    lethal_obstacle_(0),
    inscribed_inflated_obstacle_(0),
    planner_(NULL),
    allocated_time_(0)
  {
  }*/

  FlourishPlanner::FlourishPlanner() : sbpl_xytheta_planner::SBPLXYThetaPlanner()//,
				       //lethal_obstacle_(0),
				       //inscribed_inflated_obstacle_(0),
				       //sbpl_cost_multiplier_(0) //TODO check
  {
    
  }

  FlourishPlanner::FlourishPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    sbpl_xytheta_planner::SBPLXYThetaPlanner(name, costmap_ros)
  {
    //initialize(name, costmap_ros);
  }

  EnvironmentNavXYThetaLatGeneric* FlourishPlanner::createEnvironment(ros::NodeHandle & nhPriv){
    EnvironmentNavXYThetaLatFlourish* env = new EnvironmentNavXYThetaLatFlourish(nhPriv);

    /*if(!env->SetEnvParameter("cost_inscribed_thresh",
			     costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
      }*/
    unsigned char cost_possibly_circumscribed_thresh = determinePossiblyCircumscribedCostmapCost(costmap_ros_);
    if(!env->SetEnvParameter("cost_possibly_circumscribed_thresh", cost_possibly_circumscribed_thresh)){
      //costMapCostToSBPLCost(cost_possibly_circumscribed_thresh))) {
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    return env;
  }
  

  bool FlourishPlanner::initializeEnvironment(const std::vector<sbpl_2Dpt_t> & footprint,
					      double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename)
  {
    bool ret;
    EnvironmentNavXYThetaLatFlourish* envFlourish = dynamic_cast<EnvironmentNavXYThetaLatFlourish*>(env_);
    if(envFlourish == NULL) {
      ROS_FATAL("%s: env_ is not a EnvironmentNavXYThetaLatFlourish", __PRETTY_FUNCTION__);
      exit(1);
    }

    try{
      ret = envFlourish->InitializeEnv(envFlourish->traversableMap().size()(0), // width
				       envFlourish->traversableMap().size()(1), // height
				       NULL, // mapdata
				       // start/goal (x, y, theta)
				       // we really don't care at this point as this depends on the query,
				       // use traversable map origin, so that the coords are in the map.
				       0.0, 0.0, 0.0,
				       0.0, 0.0, 0.0,
				       0, 0, 0, //goal tolerance
				       footprint, envFlourish->traversableMap().getResolution(), trans_vel, timeToTurn45Degs, 
				       0, motion_primitive_filename.c_str());
				       //lethal_obstacle_, motion_primitive_filename.c_str());
    } catch (SBPL_Exception* e){
      ROS_ERROR("SBPL encountered a fatal exception! %s", e->what());
      ret = false;
    }
    if (!ret){
      ROS_ERROR("SBPL initialization failed in FlourishPlanner::initializeEnvironment!");
      exit(1);
    }
    return ret;
  }

  unsigned char FlourishPlanner::determinePossiblyCircumscribedCostmapCost(costmap_2d::Costmap2DROS* costmap_ros){
    unsigned char cost_possibly_circumscribed_thresh = 2;
    return cost_possibly_circumscribed_thresh;
  }

  void FlourishPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    ros::NodeHandle nhPriv("~/" + name);
    ROS_INFO("Name is %s", name.c_str());

    // TODO?
    /*int lethal_obstacle;
    nhPriv.param("lethal_obstacle", lethal_obstacle, 20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_ - 1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE / inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz", lethal_obstacle, inscribed_inflated_obstacle_, sbpl_cost_multiplier_);*/

    SBPLXYThetaPlanner::initialize(name, costmap_ros);

    // set the map from costmap
    EnvironmentNavXYThetaLatFlourish* envFlourish = dynamic_cast<EnvironmentNavXYThetaLatFlourish*>(env_);
    if(envFlourish == NULL) {
      ROS_FATAL("%s: env_ is not a EnvironmentNavXYThetaLatFlourish", __PRETTY_FUNCTION__);
      exit(1);
    }
    envFlourish->publish_traversable_map();
  }

  //Taken from Sachin's sbpl_cart_planner
  //This rescales the costmap according to a rosparam which sets the obstacle cost
  /*unsigned char FlourishPlanner::costMapCostToSBPLCost(unsigned char newcost)
  {
    if(newcost == costmap_2d::LETHAL_OBSTACLE)
      return lethal_obstacle_;
    else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return inscribed_inflated_obstacle_;
    else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
      return 0;
    else
      return (unsigned char) (newcost / sbpl_cost_multiplier_ + 0.5);
      }*/

};
