#include <gki_3dnav_planner/environment_navxythetalat_moveit.h>
#include <geometry_msgs/PoseArray.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/key.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include "timing/timing.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

EnvironmentNavXYThetaLatMoveit::EnvironmentNavXYThetaLatMoveit(ros::NodeHandle & nhPriv,
        double costmapOffsetX, double costmapOffsetY) :
    EnvironmentNavXYThetaLatGeneric(nhPriv),
    worldOriginX(costmapOffsetX), worldOriginY(costmapOffsetY)
{
    nhPriv.param("scene_update_name", scene_update_name, move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    nhPriv.getParam("allowed_collision_links", allowed_collision_links);

    update_planning_scene();

    planning_scene_publisher = nhPriv.advertise<moveit_msgs::PlanningScene>("planning_scene_3dnav", 1, true);

    timeActionCost = new Timing("action_cost", true, Timing::SP_STATS, false);
    timeActionCostParent = new Timing("action_cost_parent", true, Timing::SP_STATS, false);
    timeFullBodyCollision = new Timing("full_body_collision", true, Timing::SP_STATS, false);
    time3dCheck = new Timing("3d_check", true, Timing::SP_STATS, false);
}

EnvironmentNavXYThetaLatMoveit::~EnvironmentNavXYThetaLatMoveit()
{
    delete timeActionCost;
    delete timeActionCostParent;
    delete timeFullBodyCollision;
    delete time3dCheck;
    scene.reset();
    scene_monitor.reset();
}

bool EnvironmentNavXYThetaLatMoveit::InitializeEnv(int width, int height, const unsigned char* mapdata,
        double startx, double starty, double starttheta,
        double goalx, double goaly, double goaltheta,
        double goaltol_x, double goaltol_y, double goaltol_theta,
        const std::vector<sbpl_2Dpt_t>& perimeterptsV, double cellsize_m,
        double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
        unsigned char obsthresh, const char* sMotPrimFile)
{
    poseWorldToEnv(startx, starty, starttheta, startx, starty, starttheta); 
    poseWorldToEnv(goalx, goaly, goaltheta, goalx, goaly, goaltheta); 
    return EnvironmentNavXYThetaLatGeneric::InitializeEnv(width, height, mapdata,
            startx, starty, starttheta, goalx, goaly, goaltheta,
            goaltol_x, goaltol_y, goaltol_theta,
            perimeterptsV, cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs,
            obsthresh, sMotPrimFile);
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatMoveit::SetGoal(double x_m, double y_m, double theta_rad)
{
    // first convert world coordinates to internal grid coordinates
    int x, y, theta;
    if(!poseWorldToGrid(x_m, y_m, theta_rad, x, y, theta)) {
        SBPL_ERROR("ERROR: goal state %d %d %d not in grid\n", x, y, theta);
        return -1;
    }
    // next convert the world coordinates to internal env coordinates to be used by parent
    poseWorldToEnv(x_m, y_m, theta_rad, x_m, y_m, theta_rad);

    if(EnvironmentNAVXYTHETALAT::SetGoal(x_m, y_m, theta_rad) == -1)
        return -1;

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
    ROS_ASSERT(OutHashEntry != NULL);   // should have been created by parent
    if(in_full_body_collision(OutHashEntry)) {
        SBPL_ERROR("ERROR: goal state %d, %d in collision\n", x, y);
        EnvNAVXYTHETALAT.goalstateid = -1;
        return -1;
    }

    return EnvNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNavXYThetaLatMoveit::SetStart(double x_m, double y_m, double theta_rad)
{
    // first convert world coordinates to internal grid coordinates
    int x, y, theta;
    if(!poseWorldToGrid(x_m, y_m, theta_rad, x, y, theta)) {
        SBPL_ERROR("ERROR: start state %d %d %d not in grid\n", x, y, theta);
        return -1;
    }
    // next convert the world coordinates to internal env coordinates to be used by parent
    poseWorldToEnv(x_m, y_m, theta_rad, x_m, y_m, theta_rad);

    if(EnvironmentNAVXYTHETALAT::SetStart(x_m, y_m, theta_rad) == -1)
        return -1;

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
    ROS_ASSERT(OutHashEntry != NULL);
    if (in_full_body_collision(OutHashEntry)) {
        SBPL_ERROR("ERROR: start state in collision\n", x, y);
        EnvNAVXYTHETALAT.startstateid = -1;
        return -1;
    }

    return EnvNAVXYTHETALAT.startstateid;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatMoveit::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
    // the CreateNewHashEntry... functions always create a new entry, so we can assume that happens and
    // add the full_body_collision_infos entry now.
    // Do this before, instead of after the call as exceptions will appear only after a StateID2CoordTable
    // entry was created, so that at least StateID2CoordTable and full_body_collision_infos are 
    // consistent.
    full_body_collision_infos.push_back(FullBodyCollisionInfo());
    EnvNAVXYTHETALATHashEntry_t* he = EnvironmentNAVXYTHETALAT::CreateNewHashEntry_lookup(X, Y, Theta);

    return he;
}

EnvNAVXYTHETALATHashEntry_t* EnvironmentNavXYThetaLatMoveit::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
    full_body_collision_infos.push_back(FullBodyCollisionInfo());
    EnvNAVXYTHETALATHashEntry_t* he = EnvironmentNAVXYTHETALAT::CreateNewHashEntry_hash(X, Y, Theta);

    return he;
}

int EnvironmentNavXYThetaLatMoveit::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action)
{
    timeActionCost->start();
    ScopeExit se(boost::bind(&Timing::end, timeActionCost));

    timeActionCostParent->start();
    int cost = EnvironmentNAVXYTHETALAT::GetActionCost(SourceX, SourceY, SourceTheta, action);
    timeActionCostParent->end();
    if(cost >= INFINITECOST)
        return cost;

    // full body check to maybe disregard state
    int endX = SourceX + action->dX;
    int endY = SourceY + action->dY;
    int endTheta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(endX, endY, endTheta)) == NULL) {
        // might have to create a end entry
        OutHashEntry = (this->*CreateNewHashEntry)(endX, endY, endTheta);
    }
    if(in_full_body_collision(OutHashEntry))
        return INFINITECOST;

    return cost;
}

bool EnvironmentNavXYThetaLatMoveit::getExtents(double & minX, double & maxX, double & minY, double & maxY)
{
    update_planning_scene();    // get current octomap
    planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
    collision_detection::CollisionWorld::ObjectConstPtr octomapObj = scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if(!octomapObj)
        return false;
    if(octomapObj->shapes_.size() != 1)
        return false;

    const shapes::OcTree* o = static_cast<const shapes::OcTree*>(octomapObj->shapes_[0].get());
    assert(o->octree);
    const octomap::OcTree & octree = *(o->octree);
    double minz, maxz;
    octree.getMetricMin(minX, minY, minz);
    octree.getMetricMax(maxX, maxY, maxz);
    // TODO octomap vs. costmap extends -> This should be extents of underlying grid
    if(worldOriginX < minX)
        minX = worldOriginX;
    if(worldOriginY < minY)
        minY = worldOriginY;
    return true;
}

void EnvironmentNavXYThetaLatMoveit::updateForPlanRequest()
{
    EnvironmentNavXYThetaLatGeneric::updateForPlanRequest();
    update_planning_scene();
    publish_planning_scene();
}

// PlanningScene handling

// TODO should provide proper costs_changed trigger
void EnvironmentNavXYThetaLatMoveit::update_planning_scene()
{
    scene_monitor->requestPlanningSceneState(scene_update_name);
    planning_scene_monitor::LockedPlanningSceneRO ps(scene_monitor);
    scene = planning_scene::PlanningScene::clone(ps);
    // FIXME for now clone the whole scene instead of keeping the LockedPlanningSceneRO
    // and returning that in getPlanningScene() dependent on a manual one being set
    // in update_planning_scene overload or not.

    collision_detection::AllowedCollisionMatrix & allowed_collisions = scene->getAllowedCollisionMatrixNonConst();
    forEach(const std::string & cl, allowed_collision_links)
        allowed_collisions.setDefaultEntry(cl, true);

    // cached entries aren't valid any more.
    clear_full_body_collision_infos();
}

void EnvironmentNavXYThetaLatMoveit::update_planning_scene(planning_scene::PlanningSceneConstPtr scn)
{
    this->scene = planning_scene::PlanningScene::clone(scn);

    collision_detection::AllowedCollisionMatrix & allowed_collisions = scene->getAllowedCollisionMatrixNonConst();
    forEach(const std::string & cl, allowed_collision_links)
        allowed_collisions.setDefaultEntry(cl, true);

    clear_full_body_collision_infos();
}

planning_scene::PlanningSceneConstPtr EnvironmentNavXYThetaLatMoveit::getPlanningScene() const
{
    return scene;
}

std::string EnvironmentNavXYThetaLatMoveit::getPlanningFrame() const
{
    return getPlanningScene()->getPlanningFrame();
}

void EnvironmentNavXYThetaLatMoveit::publish_planning_scene()
{
    moveit_msgs::PlanningScene msg;
    getPlanningScene()->getPlanningSceneMsg(msg);
    planning_scene_publisher.publish(msg);
}

void EnvironmentNavXYThetaLatMoveit::clear_full_body_collision_infos()
{
    for(size_t i = 0; i < full_body_collision_infos.size(); i++) {
        full_body_collision_infos[i].initialized = false;
    }
}

bool EnvironmentNavXYThetaLatMoveit::transformPoseToPlanningFrame(geometry_msgs::PoseStamped & pose)
{
    planning_scene::PlanningSceneConstPtr scene = getPlanningScene();
    if(!scene->getTransforms().sameFrame(getPlanningFrame(), pose.header.frame_id)) {
        if(!scene->getTransforms().canTransform(pose.header.frame_id)) {
            return false;
        }
        Eigen::Affine3d poseEigen;
        tf::poseMsgToEigen(pose.pose, poseEigen);
        Eigen::Affine3d poseTransformed;
        scene->getTransforms().transformPose(pose.header.frame_id, poseEigen, poseTransformed);
        tf::poseEigenToMsg(poseTransformed, pose.pose);
        pose.header.frame_id = getPlanningFrame();
    }
    return true;
}

bool EnvironmentNavXYThetaLatMoveit::poseWorldToEnv(double wx, double wy, double wth,
        double & ex, double & ey, double & eth) const
{
    ex = wx - worldOriginX;
    ey = wy - worldOriginY;
    eth = wth;
    return true;
}

bool EnvironmentNavXYThetaLatMoveit::poseWorldToGrid(double wx, double wy, double wth, int & gx, int & gy, int & gth) const
{
    double ex, ey, eth;
    bool ret = true;
    ret &= poseWorldToEnv(wx, wy, wth, ex, ey, eth);
    ret &= PoseContToDisc(ex, ey, eth, gx, gy, gth);
    return ret;
}

bool EnvironmentNavXYThetaLatMoveit::posCostmapToGrid(int cx, int cy, int & gx, int & gy) const
{
    gx = cx;
    gy = cy;

    // check the grid matches, we don't care about the costmap
    return (gx >= 0) && (gx < EnvNAVXYTHETALATCfg.EnvWidth_c) 
        && (gy >= 0) && (gy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

bool EnvironmentNavXYThetaLatMoveit::poseEnvToWorld(double ex, double ey, double eth,
        double & wx, double & wy, double & wth) const
{
    wx = ex + worldOriginX;
    wy = ey + worldOriginY;
    wth = eth;
    return true;
}

bool EnvironmentNavXYThetaLatMoveit::poseGridToWorld(int gx, int gy, int gth,
        double & wx, double & wy, double & wth) const
{
    double ex, ey, eth;
    bool ret = true;
    ret &= PoseDiscToCont(gx, gy, gth, ex, ey, eth);
    ret &= poseEnvToWorld(ex, ey, eth, wx, wy, wth);
    return ret;
}

bool EnvironmentNavXYThetaLatMoveit::posGridToCostmap(int gx, int gy, int & cx, int & cy) const
{
    cx = gx;
    cy = gy;

    // check the grid matches, we don't care about the costmap
    return (gx >= 0) && (gx < EnvNAVXYTHETALATCfg.EnvWidth_c) 
        && (gy >= 0) && (gy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

geometry_msgs::Pose EnvironmentNavXYThetaLatMoveit::poseFromStateID(int stateID) const
{
    int ix, iy, ith;
    GetCoordFromState(stateID, ix, iy, ith);

    geometry_msgs::Pose pose;
    pose.position.z = 0;
    double theta;
    poseGridToWorld(ix, iy, ith, pose.position.x, pose.position.y, theta);

    pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    return pose;
}

bool EnvironmentNavXYThetaLatMoveit::UpdateCostFromCostmap(int cx, int cy, unsigned char newcost)
{
    int gx, gy;
    bool ret = posCostmapToGrid(cx, cy, gx, gy);
    if(!ret)
        return false;
    return UpdateCost(gx, gy, newcost);
}

unsigned char EnvironmentNavXYThetaLatMoveit::GetMapCostForCostmap(int cx, int cy)
{
    int gx, gy;
    bool ret = posCostmapToGrid(cx, cy, gx, gy);
    if(!ret)
        return EnvNAVXYTHETALATCfg.obsthresh;
    return GetMapCost(gx, gy);
}

void EnvironmentNavXYThetaLatMoveit::ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,
        std::vector<sbpl_xy_theta_pt_t>* xythetaPath)
{
    // parent env doesn't know anything about world vs. env, convert the result here
    EnvironmentNAVXYTHETALAT::ConvertStateIDPathintoXYThetaPath(stateIDPath, xythetaPath);
    forEach(sbpl_xy_theta_pt_t & pt, *xythetaPath) {
        poseEnvToWorld(pt.x, pt.y, pt.theta, pt.x, pt.y, pt.theta);
    }
}


bool EnvironmentNavXYThetaLatMoveit::in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state)
{
    timeFullBodyCollision->start();
    ScopeExit se(boost::bind(&Timing::end, timeFullBodyCollision));
    // TODO flag that we are allowed to do that by cost_possibly_circumscribed_cost
    // must be correctly set
    // TODO can do that in costmap as this is applicability and not cost -> better cost resolution
    if(EnvNAVXYTHETALATCfg.Grid2D[state->X][state->Y] < EnvNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh)
        return false;
    return get_full_body_collision_info(state).collision;
}

const EnvironmentNavXYThetaLatMoveit::FullBodyCollisionInfo& EnvironmentNavXYThetaLatMoveit::get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state)
{
    ROS_ASSERT_MSG(full_body_collision_infos.size() > state->stateID, "full_body_collision: state_id mismatch!");
    if (! full_body_collision_infos[state->stateID].initialized)
    {
        time3dCheck->start();
        // Static uses this to get an initialized state from the scene
        // that we can subsequently change
        static robot_state::RobotState robot_state = scene->getCurrentState();
        double x, y, theta;
        poseGridToWorld(state->X, state->Y, state->Theta, x, y, theta);
        robot_state.setVariablePosition("world_joint/x", x);
        robot_state.setVariablePosition("world_joint/y", y);
        robot_state.setVariablePosition("world_joint/theta", theta);
        robot_state.update();
        full_body_collision_infos[state->stateID].initialized = true;
        full_body_collision_infos[state->stateID].collision = getPlanningScene()->isStateColliding(robot_state);
        //ROS_INFO("get_full_body_collision_info for (%.2f, %.2f, %.2f) = %d",
        //        x, y, theta,
        //        full_body_collision_infos[state->stateID].collision);
        time3dCheck->end();
    }
    return full_body_collision_infos[state->stateID];
}

void EnvironmentNavXYThetaLatMoveit::resetTimingStats()
{
    EnvironmentNavXYThetaLatGeneric::resetTimingStats();
    timeActionCost->getStats().reset();
    timeActionCostParent->getStats().reset();
    timeFullBodyCollision->getStats().reset();
    time3dCheck->getStats().reset();
}

void EnvironmentNavXYThetaLatMoveit::printTimingStats()
{
    EnvironmentNavXYThetaLatGeneric::printTimingStats();
    timeActionCost->printStats(true);
    timeActionCostParent->printStats(true);
    timeFullBodyCollision->printStats(true);
    time3dCheck->printStats(true);
}

moveit_msgs::DisplayTrajectory EnvironmentNavXYThetaLatMoveit::pathToDisplayTrajectory(const std::vector<geometry_msgs::PoseStamped> & path) const
{
    moveit_msgs::DisplayTrajectory dtraj = EnvironmentNavXYThetaLatGeneric::pathToDisplayTrajectory(path);
    dtraj.model_id = "pr2";
    return dtraj;
}


bool EnvironmentNavXYThetaLatMoveit::InitializeEnv(const char* sEnvFile,
        const std::vector<sbpl_2Dpt_t>& perimeterptsV,
        const char* sMotPrimFile)
{
    return EnvironmentNAVXYTHETALAT::InitializeEnv(sEnvFile, perimeterptsV, sMotPrimFile);
}

bool EnvironmentNavXYThetaLatMoveit::InitializeEnv(const char* sEnvFile)
{
    return EnvironmentNAVXYTHETALAT::InitializeEnv(sEnvFile);
}

