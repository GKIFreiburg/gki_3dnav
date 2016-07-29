#include <gki_3dnav_planner/sbpl_xytheta_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gki_3dnav_msgs/PlannerStats.h>
#include <sbpl/planners/planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iomanip>
#include <angles/angles.h>
#include "color_tools/color_tools.h"

namespace sbpl_xytheta_planner
{

XYThetaStateChangeQuery::XYThetaStateChangeQuery(EnvironmentNavXYThetaLatGeneric* env,
        const std::vector<nav2dcell_t> & changedcells) : env_(env)
{
    for(std::vector<nav2dcell_t>::const_iterator it = changedcells.begin(); it != changedcells.end(); ++it) {
        nav2dcell_t cell;
        cell.x = it->x;
        cell.y = it->y;
        changedcells_.push_back(cell);
    }
}

const std::vector<int> * XYThetaStateChangeQuery::getPredecessors() const
{
    if(predsOfChangedCells_.empty() && !changedcells_.empty())
        env_->GetPredsofChangedEdges(&changedcells_, &predsOfChangedCells_);
    return &predsOfChangedCells_;
}

const std::vector<int> * XYThetaStateChangeQuery::getSuccessors() const
{
    if(succsOfChangedCells_.empty() && !changedcells_.empty())
        env_->GetSuccsofChangedEdges(&changedcells_, &succsOfChangedCells_);
    return &succsOfChangedCells_;
}

SBPLXYThetaPlanner::SBPLXYThetaPlanner() :
    initialized_(false), costmap_ros_(NULL), initial_epsilon_(0),
    env_(NULL), force_scratch_limit_(0), planner_(NULL), allocated_time_(0), private_nh_(NULL)
{
}

SBPLXYThetaPlanner::SBPLXYThetaPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    initialized_(false), planner_(NULL), env_(NULL), costmap_ros_(NULL)
{
    initialize(name, costmap_ros);
}

void SBPLXYThetaPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(initialized_)
        return;

    ROS_INFO("Planner Name is %s", name.c_str());
    private_nh_ = new ros::NodeHandle("~/" + name);
    costmap_ros_ = costmap_ros;

    readDynamicParameters();

    if(!createAndInitializeEnvironment()) {
        ROS_FATAL("Environment creation or initialization failed!");
        exit(1);
    }
    if(!createPlanner()) {
        ROS_FATAL("Failed to create search planner!");
        exit(1);
    }

    ROS_INFO("sbpl_xytheta_planner: Initialized successfully");
    plan_pub_ = private_nh_->advertise<nav_msgs::Path>("plan", 1);
    stats_publisher_ = private_nh_->advertise<gki_3dnav_msgs::PlannerStats>("planner_stats", 10);
    traj_pub_ = private_nh_->advertise<moveit_msgs::DisplayTrajectory>("trajectory", 5);

    expansions_publisher_ = private_nh_->advertise<visualization_msgs::MarkerArray>("expansions", 3, true);

    private_nh_->param("expansion_color_scheme", expansion_color_scheme_, std::string("costmap"));
    pub_expansion_map_ = private_nh_->advertise<nav_msgs::OccupancyGrid>("expansion_map", 3, true);
    pub_generation_map_ = private_nh_->advertise<nav_msgs::OccupancyGrid>("generation_map", 3, true);
    pub_expansion_first_map_ = private_nh_->advertise<nav_msgs::OccupancyGrid>("expansion_first_map", 3, true);
    pub_generation_first_map_ = private_nh_->advertise<nav_msgs::OccupancyGrid>("generation_first_map", 3, true);

    srv_sample_poses_ = private_nh_->advertiseService("sample_valid_poses",
            &SBPLXYThetaPlanner::sampleValidPoses, this);

    srand48(time(NULL));
    initialized_ = true;
}

bool SBPLXYThetaPlanner::createAndInitializeEnvironment()
{
    // Environment creation and initialization
    ROS_ASSERT(env_ == NULL);
    env_ = createEnvironment(*private_nh_);
    if(env_ == NULL) {
        ROS_ERROR("Failed to create environment.");
        return false;
    }

    std::vector<sbpl_2Dpt_t> footprintPoints;
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    footprintPoints.reserve(footprint.size());
    for(size_t i = 0; i < footprint.size(); ++i) {
        sbpl_2Dpt_t pt;
        pt.x = footprint[i].x;
        pt.y = footprint[i].y;
        footprintPoints.push_back(pt);
    }

    double trans_vel, rot_vel;
    private_nh_->getParam("trans_vel", trans_vel);
    private_nh_->getParam("rot_vel", rot_vel);
    std::string motion_primitive_filename;
    private_nh_->param("motion_primitive_filename", motion_primitive_filename, std::string(""));

    bool ret = true;
    try {
        double timeToTurn45Degs = M_PI_4/rot_vel;
        ret = initializeEnvironment(footprintPoints, trans_vel, timeToTurn45Degs, motion_primitive_filename);
    } catch(SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception initializing the environment!");
        ret = false;
    } catch(SBPL_Exception* e) {
        ROS_ERROR("SBPL encountered a fatal exception initializing the environment!");
        ret = false;
    }
    return ret;
}

bool SBPLXYThetaPlanner::createPlanner()
{
    ROS_ASSERT(env_ != NULL);
    ROS_ASSERT(planner_ == NULL);

    std::string planner_type;
    private_nh_->param("planner_type", planner_type, std::string("ARAPlanner"));
    bool forward_search;    // TODO check
    private_nh_->param("forward_search", forward_search, false);
    bool track_expansions;
    private_nh_->param("track_expansions", track_expansions, false);

    if(planner_type == "ARAPlanner") {
        ROS_INFO("Planning with ARA*");
        planner_ = new ARAPlanner(env_, forward_search);
        dynamic_cast<ARAPlanner*>(planner_)->set_track_expansions(track_expansions);
    } else if(planner_type == "ADPlanner") {
        ROS_INFO("Planning with AD*");
        planner_ = new ADPlanner(env_, forward_search);
    } else {
        ROS_ERROR("Unknown planner type: %s (supported: ARAPlanner or ADPlanner)", planner_type.c_str());
        return false;
    }
    return true;
}

std::string SBPLXYThetaPlanner::getPlanningFrame() const
{
    return env_->getPlanningFrame();
}

void SBPLXYThetaPlanner::readDynamicParameters()
{
    private_nh_->param("allocated_time", allocated_time_, 10.0);
    private_nh_->param("initial_epsilon", initial_epsilon_, 3.0);
    private_nh_->param("force_scratch_limit", force_scratch_limit_, 500);
}

void SBPLXYThetaPlanner::publishStats(int solution_cost, int solution_size, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
	gki_3dnav_msgs::PlannerStats stats;
    std::vector< ::PlannerStats > planner_stats;
    planner_->get_search_stats(&planner_stats);
    for(int i = 0; i < planner_stats.size(); ++i) {
    	gki_3dnav_msgs::PlannerStat stat;
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

bool SBPLXYThetaPlanner::sampleValidPoses(gki_3dnav_msgs::SampleValidPoses::Request & req,
		gki_3dnav_msgs::SampleValidPoses::Response & resp)
{
    geometry_msgs::Point min, max;
    if(!env_->getExtents(min.x, max.x, min.y, max.y))
        return false;

    geometry_msgs::Pose pose;
    resp.poses.header.frame_id = getPlanningFrame();
    int numTries = 0;
    while(numTries < req.max_tries) {
        // sample pose and add to resp
        pose.position.x = min.x + (max.x - min.x) * drand48();
        pose.position.y = min.y + (max.y - min.y) * drand48();
        double theta = -M_PI + 2 * M_PI * drand48();
        pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        numTries++;
        try {
            int ret = env_->SetStart(pose.position.x, pose.position.y, theta);
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

bool SBPLXYThetaPlanner::makePlan(const geometry_msgs::PoseStamped& startPose,
        const geometry_msgs::PoseStamped& goalPose,
        std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_) {
        ROS_ERROR("Global planner is not initialized");
        return false;
    }

    readDynamicParameters();
    if(force_scratch_limit_ == -1) {
        ROS_INFO("force_scratch_limit_ set to -1: Hard re-creating environment and planner.");
        delete planner_;
        delete env_;
        planner_ = NULL;
        env_ = NULL;

        if(!createAndInitializeEnvironment()) {
            ROS_ERROR("Environment creation or initialization failed!");
            return false;
        }
        if(!createPlanner()) {
            ROS_ERROR("Failed to create search planner!");
            return false;
        }
    }

    env_->updateForPlanRequest();

    ROS_INFO("Planning frame is %s", getPlanningFrame().c_str());

    geometry_msgs::PoseStamped start = startPose;
    if(!env_->transformPoseToPlanningFrame(start)) {
        ROS_ERROR("Unable to transform start pose into planning frame");
        return false;
    }
    geometry_msgs::PoseStamped goal = goalPose;
    if(!env_->transformPoseToPlanningFrame(goal)) {
        ROS_ERROR("Unable to transform goal pose into planning frame");
        return false;
    }

    double theta_start = tf::getYaw(start.pose.orientation);
    double theta_goal = tf::getYaw(goal.pose.orientation);
    ROS_INFO("sbpl_xytheta_planner: setting start (%.2f, %.2f, %.2f deg), goal (%.2f, %.2f, %.2f deg)",
            start.pose.position.x, start.pose.position.y, angles::to_degrees(theta_start),
            goal.pose.position.x, goal.pose.position.y, angles::to_degrees(theta_goal));

    int startId = 0;
    try {
        int ret = env_->SetStart(start.pose.position.x, start.pose.position.y, theta_start);
        startId = ret;
        if(ret < 0 || planner_->set_start(ret) == 0) {
            ROS_ERROR("ERROR: failed to set start state\n");
            return false;
        }
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
    }

    try {
        int ret = env_->SetGoal(goal.pose.position.x, goal.pose.position.y, theta_goal);
        if(ret < 0 || planner_->set_goal(ret) == 0) {
            ROS_ERROR("ERROR: failed to set goal state\n");
            return false;
        }
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
    }

    // Test for dynamic updates
    XYThetaStateChangeQuery* scq = updateForPlanRequest();  // FIXME must this happen before SetStart to
    // determine that start is valid? Or does updateForPlanRequest need a current start?
    if(scq == NULL) {
        planner_->force_planning_from_scratch();
    } else {
        try {
            if(!scq->changedcells_.empty()) {
                planner_->costs_changed(*scq);
            }
            if(scq->changedcells_.size() >= force_scratch_limit_)
                planner_->force_planning_from_scratch();
        } catch(SBPL_Exception& e) {
            ROS_ERROR("SBPL failed to handle StateChangeQuery");
            return false;
        }
        delete scq;
    }

    ROS_INFO("Start state Heur: %d", env_->GetGoalHeuristic(startId));
    ROS_DEBUG("allocated time: %.1f, initial eps: %.2f\n", allocated_time_, initial_epsilon_);
    planner_->set_initialsolution_eps(initial_epsilon_);
    planner_->set_search_mode(false);   // TODO

    ROS_DEBUG("Running planner");
    std::vector<int> solution_stateIDs;
    int solution_cost;
    try {
        env_->resetTimingStats();
        int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
        env_->printTimingStats();
        if(ret) {
            ROS_DEBUG("Solution is found\n");
        } else {
            ROS_INFO("Solution not found\n");
            publish_expansions();
            publish_expansion_map();
            publishStats(solution_cost, 0, start, goal);
            return false;
        }
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while planning");
        return false;
    }

    ROS_DEBUG("solution length %zu", solution_stateIDs.size());

    std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
    try {
        env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
    } catch (SBPL_Exception& e) {
        ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
        return false;
    }

    // TODO check if the plan has zero points, add a single point to make move_base happy
    if(sbpl_path.size() == 0) {
        EnvNAVXYTHETALAT3Dpt_t s(start.pose.position.x, start.pose.position.y, theta_start);
        sbpl_path.push_back(s);
    }

    ROS_DEBUG("Plan has %zu steps.", sbpl_path.size());

    ros::Time plan_time = ros::Time::now();
    plan.clear();
    nav_msgs::Path gui_path;
    gui_path.poses.resize(sbpl_path.size());
    gui_path.header.frame_id = getPlanningFrame();
    gui_path.header.stamp = plan_time;
    for(unsigned int i = 0; i < sbpl_path.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = getPlanningFrame();

        pose.pose.position.x = sbpl_path[i].x;
        pose.pose.position.y = sbpl_path[i].y;
        pose.pose.position.z = start.pose.position.z;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(sbpl_path[i].theta);
        plan.push_back(pose);
        gui_path.poses[i] = plan[i];
    }

    plan_pub_.publish(gui_path);
    publishStats(solution_cost, sbpl_path.size(), start, goal);
    moveit_msgs::DisplayTrajectory traj = env_->pathToDisplayTrajectory(plan);
    traj_pub_.publish(traj);
    publish_expansions();
    publish_expansion_map();
    return true;
}

void SBPLXYThetaPlanner::publish_expansions()
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
    mark.header.frame_id = getPlanningFrame();
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

void SBPLXYThetaPlanner::fillGrid(nav_msgs::OccupancyGrid & grid, const std::vector< std::set<int> > & gridDirections, int maxDirections)
{
    assert(grid.data.size() == gridDirections.size());
    for(int i = 0; i < gridDirections.size(); ++i) {
        if(expansion_color_scheme_ == "costmap") {
            // 100 for lethal
            // 99 for inscribed
            // 0 for no obstacle
            if(gridDirections[i].empty())
                grid.data[i] = 0;
            else if(gridDirections[i].size() == maxDirections)
                grid.data[i] = 100;
            else if(gridDirections[i].size() == maxDirections - 1)
                grid.data[i] = 99;
            else {
                double percTheta = (double)(gridDirections[i].size())/(double)(maxDirections - 2);
                grid.data[i] = 98.0 * percTheta;
            }
        } else {
            double percTheta = (double)gridDirections[i].size()/(double)maxDirections;
            if(gridDirections[i].empty())
                grid.data[i] = -1;
            else
                grid.data[i] = 100.0 * percTheta;
        }
    }
}


void SBPLXYThetaPlanner::publish_expansion_map()
{
    ARAPlanner* pl = dynamic_cast<ARAPlanner*>(planner_);
    if(!pl)
        return;

    // setup an empty grid map for the environment
    std::vector<SBPL_xytheta_mprimitive> prim_vec;
    double dummy;
    unsigned char dummyC;
    int sizeX, sizeY, numThetas;
    double resolution;
    // TODO check impl in flourish cfg
    env_->GetEnvParms(&sizeX, &sizeY, &numThetas,
            &dummy, &dummy, &dummy,     // start
            &dummy, &dummy, &dummy,     // goal
            &resolution, &dummy, &dummy,
            &dummyC, &prim_vec);
    double minX, maxX, minY, maxY;
    env_->getExtents(minX, maxX, minY, maxY);
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = getPlanningFrame();
    grid.info.resolution = resolution;
    grid.info.width = sizeX;
    grid.info.height = sizeY;
    grid.info.origin.position.x = minX;
    grid.info.origin.position.y = minY;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height, -1);
    std::vector< std::set<int> > gridExpansionsDirections(grid.data.size());
    std::vector< std::set<int> > gridExpansionsFirstDirections(grid.data.size());
    std::vector< std::set<int> > gridGenerationsDirections(grid.data.size());
    std::vector< std::set<int> > gridGenerationsFirstDirections(grid.data.size());

    const std::vector< std::vector<int> > & gen_states = pl->get_generated_states();
    const std::vector< std::vector<int> > & exp_states = pl->get_expanded_states();

    for(int iteration = 0; iteration < exp_states.size(); iteration++) {
        int state = 0;
        for(; state < exp_states[iteration].size(); state++) {
            int x, y, theta;
            env_->GetCoordFromState(exp_states[iteration][state], x, y, theta);
            gridExpansionsDirections[x + y * grid.info.width].insert(theta);
            if(iteration == 0)
                gridExpansionsFirstDirections[x + y * grid.info.width].insert(theta);
        }
    }
    for(int iteration = 0; iteration < gen_states.size(); iteration++) {
        int state = 0;
        for(; state < gen_states[iteration].size(); state++) {
            int x, y, theta;
            env_->GetCoordFromState(gen_states[iteration][state], x, y, theta);
            gridGenerationsDirections[x + y * grid.info.width].insert(theta);
            if(iteration == 0)
                gridGenerationsFirstDirections[x + y * grid.info.width].insert(theta);
        }
    }

    fillGrid(grid, gridExpansionsDirections, numThetas);
    pub_expansion_map_.publish(grid);
    fillGrid(grid, gridExpansionsFirstDirections, numThetas);
    pub_expansion_first_map_.publish(grid);
    fillGrid(grid, gridGenerationsDirections, numThetas);
    pub_generation_map_.publish(grid);
    fillGrid(grid, gridGenerationsFirstDirections, numThetas);
    pub_generation_first_map_.publish(grid);
}

}

