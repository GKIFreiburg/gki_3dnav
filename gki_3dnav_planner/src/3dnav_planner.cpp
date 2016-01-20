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

using namespace std;
using namespace ros;

PLUGINLIB_EXPORT_CLASS(gki_3dnav_planner::GKI3dNavPlanner, nav_core::BaseGlobalPlanner);

namespace gki_3dnav_planner
{

class LatticeSCQ: public StateChangeQuery
{
public:
    LatticeSCQ(EnvironmentNavXYThetaLatMoveit* env, std::vector<nav2dcell_t> const & changedcellsV) :
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

    EnvironmentNavXYThetaLatMoveit * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

GKI3dNavPlanner::GKI3dNavPlanner() :
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
}

GKI3dNavPlanner::GKI3dNavPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
        initialized_(false), costmap_ros_(NULL)
{
    initialize(name, costmap_ros);
}

void GKI3dNavPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        private_nh_ = new ros::NodeHandle("~/" + name);
        ros::NodeHandle nh(name);

        ROS_INFO("Name is %s", name.c_str());

        private_nh_->param("planner_type", planner_type_, string("ARAPlanner"));
        private_nh_->param("allocated_time", allocated_time_, 10.0);
        private_nh_->param("initial_epsilon", initial_epsilon_, 3.0);
        private_nh_->param("environment_type", environment_type_, string("XYThetaLattice"));
        private_nh_->param("forward_search", forward_search_, bool(false));
        private_nh_->param("primitive_filename", primitive_filename_, string(""));
        private_nh_->param("force_scratch_limit", force_scratch_limit_, 500);
        private_nh_->param("use_freespace_heuristic", use_freespace_heuristic_, true);

        double nominalvel_mpersecs, timetoturn45degsinplace_secs;
        private_nh_->param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
        private_nh_->param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
        bool track_expansions;
        private_nh_->param("track_expansions", track_expansions, false);

        int lethal_obstacle;
        private_nh_->param("lethal_obstacle", lethal_obstacle, 20);
        lethal_obstacle_ = (unsigned char) lethal_obstacle;
        inscribed_inflated_obstacle_ = lethal_obstacle_ - 1;
        sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE / inscribed_inflated_obstacle_ + 1);
        ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz", lethal_obstacle, inscribed_inflated_obstacle_, sbpl_cost_multiplier_);

        costmap_ros_ = costmap_ros;

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

        env_ = new EnvironmentNavXYThetaLatMoveit(*private_nh_, costmap_ros_->getCostmap()->getOriginX(), costmap_ros_->getCostmap()->getOriginY());

        // check if the costmap has an inflation layer
        // Warning: footprint updates after initialization are not supported here
        unsigned char cost_possibly_circumscribed_tresh = 0;
        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin(); layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
        {
            boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
            if (!inflation_layer)
                continue;

            cost_possibly_circumscribed_tresh = inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius()/costmap_ros_->getCostmap()->getResolution());
            ROS_INFO("Radii: inscribed: %f circumscribed: %f", costmap_ros_->getLayeredCostmap()->getInscribedRadius(), costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
        }

        if (!env_->SetEnvParameter("cost_inscribed_thresh", costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
        {
            ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
            exit(1);
        }
        if (!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_possibly_circumscribed_tresh)))
        {
            ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
            exit(1);
        }
        int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
        vector<sbpl_2Dpt_t> perimeterptsV;
        perimeterptsV.reserve(footprint.size());
        for (size_t ii(0); ii < footprint.size(); ++ii)
        {
            sbpl_2Dpt_t pt;
            pt.x = footprint[ii].x;
            pt.y = footprint[ii].y;
            perimeterptsV.push_back(pt);
        }

        bool ret;
        try
        {
            ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
                    costmap_ros_->getCostmap()->getSizeInCellsY(), // height
                    0, // mapdata
                    0, 0, 0, // start (x, y, theta, t)
                    0, 0, 0, // goal (x, y, theta)
                    0, 0, 0, //goal tolerance
                    perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs, timetoturn45degsinplace_secs, obst_cost_thresh, primitive_filename_.c_str());
        } catch (SBPL_Exception& e)
        {
            ROS_ERROR("SBPL encountered a fatal exception!");
            ret = false;
        }
        if (!ret)
        {
            ROS_ERROR("SBPL initialization failed!");
            exit(1);
        }
        for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
            for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
                env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

        if ("ARAPlanner" == planner_type_)
        {
            ROS_INFO("Planning with ARA*");
            planner_ = new ARAPlanner(env_, forward_search_);
            dynamic_cast<ARAPlanner*>(planner_)->set_track_expansions(track_expansions);
        }
        else if ("ADPlanner" == planner_type_)
        {
            ROS_INFO("Planning with AD*");
            planner_ = new ADPlanner(env_, forward_search_);
        }
        else
        {
            ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
            exit(1);
        }

        ROS_INFO("[gki_3dnav_planner] Initialized successfully");
        plan_pub_ = private_nh_->advertise<nav_msgs::Path>("plan", 1);
        stats_publisher_ = private_nh_->advertise<gki_3dnav_planner::PlannerStats>("planner_stats", 10);
        traj_pub_ = private_nh_->advertise<moveit_msgs::DisplayTrajectory>("trajectory", 5);
        expansions_publisher_ = private_nh_->advertise<visualization_msgs::MarkerArray>("expansions", 3, true);

        srv_sample_poses_ = private_nh_->advertiseService("sample_valid_poses",
                &GKI3dNavPlanner::sampleValidPoses, this);

        srand48(time(NULL));

        initialized_ = true;
    }
}

//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char GKI3dNavPlanner::costMapCostToSBPLCost(unsigned char newcost)
{
    if (newcost == costmap_2d::LETHAL_OBSTACLE)
        return lethal_obstacle_;
    else if (newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        return inscribed_inflated_obstacle_;
    else if (newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
        return 0;
    else
        return (unsigned char) (newcost / sbpl_cost_multiplier_ + 0.5);
}

void GKI3dNavPlanner::publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    gki_3dnav_planner::PlannerStats stats;
    std::vector< ::PlannerStats > planner_stats;
    planner_->get_search_stats(&planner_stats);
    for(int i = 0; i < planner_stats.size(); ++i) {
        gki_3dnav_planner::PlannerStat stat;
        stat.eps = planner_stats[i].eps;
        stat.suboptimality = planner_stats[i].suboptimality;
        stat.time = planner_stats[i].time;
        stat.g = planner_stats[i].g;
        stat.cost = planner_stats[i].cost;
        stat.expands = planner_stats[i].expands;
        stats.stats.push_back(stat);
    }
    stats_publisher_.publish(stats);
}

bool GKI3dNavPlanner::transformPoseToPlanningFrame(geometry_msgs::PoseStamped& stamped)
{
    planning_scene::PlanningSceneConstPtr scene = env_->getPlanningScene();
    if (! scene->getTransforms().sameFrame(scene->getPlanningFrame(), stamped.header.frame_id))
    {
        if (! scene->getTransforms().canTransform(stamped.header.frame_id))
        {
            return false;
        }
        Eigen::Affine3d original_goal;
        tf::poseMsgToEigen(stamped.pose, original_goal);
        Eigen::Affine3d map_goal;
        scene->getTransforms().transformPose(stamped.header.frame_id, original_goal, map_goal);
        tf::poseEigenToMsg(map_goal, stamped.pose);
        stamped.header.frame_id = scene->getPlanningFrame();
    }
    return true;
}

bool GKI3dNavPlanner::sampleValidPoses(gki_3dnav_planner::SampleValidPoses::Request & req, gki_3dnav_planner::SampleValidPoses::Response & resp)
{
    env_->clear_full_body_collision_infos();
    env_->update_planning_scene();
    planning_scene::PlanningSceneConstPtr scene = env_->getPlanningScene();
    collision_detection::CollisionWorld::ObjectConstPtr octomapObj = scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if(!octomapObj)
        return false;
    if(octomapObj->shapes_.size() != 1)
        return false;

    const shapes::OcTree* o = static_cast<const shapes::OcTree*>(octomapObj->shapes_[0].get());
    assert(o->octree);
    const octomap::OcTree & octree = *(o->octree);
    geometry_msgs::Point min, max;
    octree.getMetricMin(min.x, min.y, min.z);
    octree.getMetricMax(max.x, max.y, max.z);

    geometry_msgs::Pose pose;
    resp.poses.header.frame_id = scene->getPlanningFrame();
    int numTries = 0;
    while(numTries < req.max_tries) {
        // sample pose and add to resp
        pose.position.x = min.x + (max.x - min.x) * drand48();
        pose.position.y = min.y + (max.y - min.y) * drand48();
        double theta = -M_PI + 2 * M_PI * drand48();
        pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        numTries++;
        try {
            int ret = env_->SetStart(pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
                    pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta);
            if(ret < 0)
                continue;
        } catch (SBPL_Exception& e) {
            continue;
        }

        resp.poses.poses.push_back(pose);
        if(resp.poses.poses.size() >= req.n)
            break;
    }

    return resp.poses.poses.size() >= req.n;
}

bool GKI3dNavPlanner::makePlan(planning_scene::PlanningSceneConstPtr scene, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    const robot_state::RobotState& state = scene->getCurrentState();
    geometry_msgs::PoseStamped start;
    start.pose.position.x = state.getVariablePosition("world_joint/x");
    start.pose.position.y = state.getVariablePosition("world_joint/y");
    start.pose.position.z = 0;
    start.pose.orientation = tf::createQuaternionMsgFromYaw(state.getVariablePosition("world_joint/theta"));
    start.header.frame_id = scene->getPlanningFrame();
    env_->clear_full_body_collision_infos();
    env_->update_planning_scene(scene);
    env_->publish_planning_scene();
    return makePlan_(start, goal, plan);
}

bool GKI3dNavPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    env_->clear_full_body_collision_infos();
    env_->update_planning_scene();
    env_->publish_planning_scene();
    private_nh_->getParam("use_freespace_heuristic", use_freespace_heuristic_);
    env_->useFreespaceHeuristic(use_freespace_heuristic_);
    env_->count = 0;
    env_->past = 0;
    bool planOK = makePlan_(start, goal, plan);
    if(planOK) {
        moveit_msgs::DisplayTrajectory traj = env_->pathToDisplayTrajectory(plan);
        traj_pub_.publish(traj);
        ROS_INFO("Published traj");
        //ros::Duration(10.0).sleep();
    }

    //env_->useFreespaceHeuristic(false);
    //std::vector<geometry_msgs::PoseStamped> plan2;
    //env_->count = 0;
    //env_->past = 0;
    //bool plan2OK = makePlan_(start, goal, plan2);
    //if(plan2OK) {
    //    moveit_msgs::DisplayTrajectory traj = env_->pathToDisplayTrajectory(plan2);
    //    traj_pub_.publish(traj);
    //    ROS_INFO("Published traj2 1s");
    //    ros::Duration(1.0).sleep();
    //}

    //if(!planOK && plan2OK) {
    //    plan = plan2;
    //    return plan2OK;
    //}
    return planOK;
}

bool GKI3dNavPlanner::makePlan_(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)
    {
        ROS_ERROR("Global planner is not initialized");
        return false;
    }
    ROS_INFO("Planning frame is %s", env_->getPlanningScene()->getPlanningFrame().c_str());
    if (! transformPoseToPlanningFrame(start))
    {
        ROS_ERROR("Unable to transform start pose into planning frame");
        return false;
    }
    if (! transformPoseToPlanningFrame(goal))
    {
        ROS_ERROR("Unable to transform goal pose into planning frame");
        return false;
    }

    ROS_INFO("[gki_3dnav_planner] getting start point (%g,%g) goal point (%g,%g)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
    double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

    int startId = 0;
    planner_->force_planning_from_scratch();
    try
    {
        int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
        startId = ret;
        if (ret < 0 || planner_->set_start(ret) == 0)
        {
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    } catch (SBPL_Exception& e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }

    try
    {
        int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
        if (ret < 0 || planner_->set_goal(ret) == 0)
        {
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
    } catch (SBPL_Exception& e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }

    ROS_INFO("Start state Heur: %d", env_->GetGoalHeuristic(startId));

    int offOnCount = 0;
    int onOffCount = 0;
    int allCount = 0;
    vector<nav2dcell_t> changedcellsV;

    for (unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++)
    {
        for (unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++)
        {

            unsigned char oldCost = env_->GetMapCost(ix, iy);
            unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy));

            if (oldCost == newCost)
                continue;

            allCount++;

            //first case - off cell goes on

            if ((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
                    && (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
            {
                offOnCount++;
            }

            if ((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
                    && (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
            {
                onOffCount++;
            }
            env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

            nav2dcell_t nav2dcell;
            nav2dcell.x = ix;
            nav2dcell.y = iy;
            changedcellsV.push_back(nav2dcell);
        }
    }

    try
    {
        if (!changedcellsV.empty())
        {
            StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
            planner_->costs_changed(*scq);
            delete scq;
        }

        if (allCount > force_scratch_limit_)
            planner_->force_planning_from_scratch();
    } catch (SBPL_Exception& e)
    {
        ROS_ERROR("SBPL failed to update the costmap");
        return false;
    }

    //setting planner parameters
    ROS_DEBUG("allocated:%f, init eps:%f\n", allocated_time_, initial_epsilon_);
    planner_->set_initialsolution_eps(initial_epsilon_);
    planner_->set_search_mode(false);

    ROS_DEBUG("[gki_3dnav_planner] run planner");
    vector<int> solution_stateIDs;
    int solution_cost;
    try
    {
        env_->resetTimingStats();
        int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
        env_->printTimingStats();
        if (ret)
            ROS_DEBUG("Solution is found\n");
        else
        {
            ROS_INFO("Solution not found\n");
            publishStats(solution_cost, 0, start, goal);
            return false;
        }
    } catch (SBPL_Exception& e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while planning");
        return false;
    }

    ROS_DEBUG("size of solution=%d", (int )solution_stateIDs.size());

    vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
    try
    {
        env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
    } catch (SBPL_Exception& e)
    {
        ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
        return false;
    }
    // if the plan has zero points, add a single point to make move_base happy
    if (sbpl_path.size() == 0)
    {
        EnvNAVXYTHETALAT3Dpt_t s(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
        sbpl_path.push_back(s);
    }

    ROS_DEBUG("Plan has %d points.\n", (int )sbpl_path.size());
    ros::Time plan_time = ros::Time::now();

    plan.clear();
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(sbpl_path.size());
    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = plan_time;
    for (unsigned int i = 0; i < sbpl_path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();

        pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
        pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
        pose.pose.position.z = start.pose.position.z;

        tf::Quaternion temp;
        temp.setRPY(0, 0, sbpl_path[i].theta);
        pose.pose.orientation.x = temp.getX();
        pose.pose.orientation.y = temp.getY();
        pose.pose.orientation.z = temp.getZ();
        pose.pose.orientation.w = temp.getW();

        plan.push_back(pose);

        gui_path.poses[i] = plan[i];
    }
    plan_pub_.publish(gui_path);
    publishStats(solution_cost, sbpl_path.size(), start, goal);

    // DeBUG
    env_->publish_expanded_states();
    publish_expansions();
    return true;
}

void GKI3dNavPlanner::publish_expansions()
{
    ARAPlanner* pl = dynamic_cast<ARAPlanner*>(planner_);
    if(!pl)
        return;

    const std::vector< std::vector<int> > & gen_states = pl->get_generated_states();
    const std::vector< std::vector<int> > & exp_states = pl->get_expanded_states();

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker mark;
    mark.type = visualization_msgs::Marker::ARROW;
    mark.scale.x = 0.1;
    mark.scale.y = 0.01;
    mark.scale.z = 0.01;
    mark.color.a = 0.1;
    mark.header.frame_id = env_->getPlanningScene()->getPlanningFrame();
    color_tools::HSV hsv;
    hsv.s = 1.0;
    hsv.v = 1.0;
    for(int iteration = 0; iteration < exp_states.size(); iteration++) {
        std::stringstream ss;
        ss << "expansions_" << std::setfill('0') << std::setw(2) << iteration;
        mark.ns = ss.str();
        hsv.h = 300.0 * (1.0 - 1.0*iteration/exp_states.size());
        color_tools::convert(hsv, mark.color);
        int state = 0;
        mark.action = visualization_msgs::Marker::ADD;
        for(; state < exp_states[iteration].size(); state++) {
            mark.id = state;
            mark.pose = env_->poseFromStateID(exp_states[iteration][state]);
            mark.pose.position.z += 0.3 * (1.0 - 1.0*iteration/exp_states.size());
            ma.markers.push_back(mark);
        }
        mark.action = visualization_msgs::Marker::DELETE;
        for(; state < 1000; state++) {
            mark.id = state;
            ma.markers.push_back(mark);
        }
    }
    for(int iteration = 0; iteration < gen_states.size(); iteration++) {
        std::stringstream ss;
        ss << "generated_" << std::setfill('0') << std::setw(2) << iteration;
        mark.ns = ss.str();
        hsv.h = 300.0 * (1.0 - 1.0*iteration/gen_states.size());
        color_tools::convert(hsv, mark.color);
        int state = 0;
        mark.action = visualization_msgs::Marker::ADD;
        for(; state < gen_states[iteration].size(); state++) {
            mark.id = state;
            mark.pose = env_->poseFromStateID(gen_states[iteration][state]);
            mark.pose.position.z += 0.3 * (1.0 - 1.0*iteration/gen_states.size());
            ma.markers.push_back(mark);
        }
        mark.action = visualization_msgs::Marker::DELETE;
        for(; state < 1000; state++) {
            mark.id = state;
            ma.markers.push_back(mark);
        }
    }

    expansions_publisher_.publish(ma);
}

}

