/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2014, University of Freiburg
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
 * Author: Christian Dornhege, Andreas Hertle, Mike Phillips
 *********************************************************************/

#include <gki_3dnav_planner/3dnav_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <gki_3dnav_planner/PlannerStats.h>
#include <sbpl/planners/planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iomanip>

#include "color_tools/color_tools.h"
#include <costmap_2d/inflation_layer.h>
#include <eigen_conversions/eigen_msg.h>

PLUGINLIB_EXPORT_CLASS(gki_3dnav_planner::GKI3dNavPlanner, nav_core::BaseGlobalPlanner);

namespace gki_3dnav_planner
{

GKI3dNavPlanner::GKI3dNavPlanner() : sbpl_xytheta_planner::SBPLXYThetaPlanner(),
    lethal_obstacle_(0),
    inscribed_inflated_obstacle_(0),
    sbpl_cost_multiplier_(0)
{
}

GKI3dNavPlanner::GKI3dNavPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    sbpl_xytheta_planner::SBPLXYThetaPlanner(name, costmap_ros)
{
}

EnvironmentNavXYThetaLatGeneric* GKI3dNavPlanner::createEnvironment(ros::NodeHandle & nhPriv)
{
    EnvironmentNavXYThetaLatMoveit* env = new EnvironmentNavXYThetaLatMoveit(nhPriv,
            costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getOriginY());

    if(!env->SetEnvParameter("cost_inscribed_thresh",
                costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
        exit(1);
    }
    unsigned char cost_possibly_circumscribed_thresh = determinePossiblyCircumscribedCostmapCost(costmap_ros_);
    if(!env->SetEnvParameter("cost_possibly_circumscribed_thresh",
                costMapCostToSBPLCost(cost_possibly_circumscribed_thresh))) {
        ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
        exit(1);
    }

    return env;
}

bool GKI3dNavPlanner::initializeEnvironment(const std::vector<sbpl_2Dpt_t> & footprint,
        double trans_vel, double timeToTurn45Degs, const std::string & motion_primitive_filename)
{
    return env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
            costmap_ros_->getCostmap()->getSizeInCellsY(), // height
            NULL, // mapdata
            // start/goal (x, y, theta)
            // we really don't care at this point as this depends on the query,
            // use costmap origin, so that the coords are in the map.
            costmap_ros_->getCostmap()->getOriginX() + costmap_ros_->getCostmap()->getResolution(),
            costmap_ros_->getCostmap()->getOriginY() + costmap_ros_->getCostmap()->getResolution(), 0.0,
            costmap_ros_->getCostmap()->getOriginX() + costmap_ros_->getCostmap()->getResolution(),
            costmap_ros_->getCostmap()->getOriginY() + costmap_ros_->getCostmap()->getResolution(), 0.0,
            0, 0, 0, //goal tolerance
            footprint, costmap_ros_->getCostmap()->getResolution(),
            trans_vel, timeToTurn45Degs,
            lethal_obstacle_, motion_primitive_filename.c_str());
}

unsigned char GKI3dNavPlanner::determinePossiblyCircumscribedCostmapCost(costmap_2d::Costmap2DROS* costmap_ros)
{
    // TODO: footprint updates after initialization are not supported
    unsigned char cost_possibly_circumscribed_thresh = 0;
    // check if the costmap has an inflation layer
    for(std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer =
            costmap_ros->getLayeredCostmap()->getPlugins()->begin();
            layer != costmap_ros->getLayeredCostmap()->getPlugins()->end(); ++layer) {
        boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer =
            boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
        if(!inflation_layer)
            continue;

        cost_possibly_circumscribed_thresh = inflation_layer->computeCost(
                costmap_ros->getLayeredCostmap()->getCircumscribedRadius() / 
                costmap_ros->getCostmap()->getResolution());

        // TODO footprint should come dynamically from moveit state (+ need extra checks that it's updatable)
        // for AD* will make stuff immovable (force_scratch)
        // 0.95 is PR2 + extended arms hack instead of the getCircumscribedRadius from the
        // hardcoded footprint in costmap config.
        // Once this comes from scene, it needs to be updatable and the costmap circumscribed
        // needs to be set somehow, so that the "normal" functions work correctly.
        // Might be sufficient to only set the correct value to env.
        cost_possibly_circumscribed_thresh = inflation_layer->computeCost(
                0.95 /
                costmap_ros->getCostmap()->getResolution());
        ROS_INFO("With radius circumscribed %f - cost %d (SBPL %d)", 0.95, cost_possibly_circumscribed_thresh,
                costMapCostToSBPLCost(cost_possibly_circumscribed_thresh));
        ROS_INFO("Radii: inscribed: %f circumscribed: %f",
                costmap_ros->getLayeredCostmap()->getInscribedRadius(),
                costmap_ros->getLayeredCostmap()->getCircumscribedRadius());
    }
    return cost_possibly_circumscribed_thresh;
}

void GKI3dNavPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle nhPriv("~/" + name);

    // determine cost thresholds and scaling for SBPL costs
    int lethal_obstacle;
    nhPriv.param("lethal_obstacle", lethal_obstacle, 20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_ - 1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE / inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",
            lethal_obstacle, inscribed_inflated_obstacle_, sbpl_cost_multiplier_);
    // now costMapCostToSBPLCost is usable

    SBPLXYThetaPlanner::initialize(name, costmap_ros);

    // set the map from costmap
    EnvironmentNavXYThetaLatMoveit* envMoveit = dynamic_cast<EnvironmentNavXYThetaLatMoveit*>(env_);
    if(envMoveit == NULL) {
        ROS_FATAL("%s: env_ is not a EnvironmentNavXYThetaLatMoveit", __PRETTY_FUNCTION__);
        exit(1);
    }
    for(ssize_t x = 0; x < costmap_ros_->getCostmap()->getSizeInCellsX(); ++x) {
        for(ssize_t y = 0; y < costmap_ros_->getCostmap()->getSizeInCellsY(); ++y) {
            envMoveit->UpdateCostFromCostmap(
                    x, y, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(x, y)));
        }
    }

    // costmap frame must be environment's planning frame
    std::string costmapFrameId = costmap_ros_->getGlobalFrameID();
    if(!costmapFrameId.empty() && !(costmapFrameId[0] == '/')) {
        costmapFrameId = "/" + costmapFrameId;
    }
    if(costmapFrameId != getPlanningFrame()) {
        ROS_FATAL("Costmap is not in the planning frame (%s), but in frame %s and will not be converted! This is unlikely to produce correct results. Make sure the costmap frame and MoveIt planning frame are the same.",
                getPlanningFrame().c_str(), costmap_ros_->getGlobalFrameID().c_str());
        exit(1);
    }

    ROS_INFO("GKI3dNavPlanner initialized successfully");
}

//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char GKI3dNavPlanner::costMapCostToSBPLCost(unsigned char newcost)
{
    if(newcost >= costmap_2d::LETHAL_OBSTACLE)     // lethal and no_information are lethal
        return lethal_obstacle_;
    else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        return inscribed_inflated_obstacle_;
    else if(newcost == 0)
        return 0;
    else
        return (unsigned char) (newcost / sbpl_cost_multiplier_ + 0.5);
}

sbpl_xytheta_planner::XYThetaStateChangeQuery* GKI3dNavPlanner::updateForPlanRequest()
{
    EnvironmentNavXYThetaLatMoveit* envMoveit = dynamic_cast<EnvironmentNavXYThetaLatMoveit*>(env_);
    if(envMoveit == NULL) {
        ROS_ERROR("%s: env is not a EnvironmentNavXYThetaLatMoveit", __PRETTY_FUNCTION__);
        return NULL;
    }

    std::vector<nav2dcell_t> changedcells;
    for(unsigned int x = 0; x < costmap_ros_->getCostmap()->getSizeInCellsX(); x++) {
        for(unsigned int y = 0; y < costmap_ros_->getCostmap()->getSizeInCellsY(); y++) {
            unsigned char oldCost = envMoveit->GetMapCostForCostmap(x, y);
            unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(x, y));
            if(oldCost == newCost)
                continue;

            envMoveit->UpdateCostFromCostmap(x, y, newCost);

            nav2dcell_t nav2dcell;
            if(envMoveit->posCostmapToGrid(x, y, nav2dcell.x, nav2dcell.y)) {
                changedcells.push_back(nav2dcell);
            } else {
                ROS_WARN_THROTTLE(1.0, "%s: Costmap cell %d %d not in grid.", __PRETTY_FUNCTION__, x, y);
            }
        }
    }

    return new sbpl_xytheta_planner::XYThetaStateChangeQuery(env_, changedcells);
}

}

