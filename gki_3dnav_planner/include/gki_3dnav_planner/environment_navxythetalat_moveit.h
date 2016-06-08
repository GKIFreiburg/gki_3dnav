#ifndef ENVIRONMENT_NAVXYTHETALAT_MOVEIT_H
#define ENVIRONMENT_NAVXYTHETALAT_MOVEIT_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "gki_3dnav_planner/environment_navxythetalat_generic.h"
#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include "timing/timing.h"

/// Planning environment for x, y, theta planning with full 3d collision checking by MoveIt.
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
 * the costmap resolution and the motion primitive's resolution.  The Octomap's
 * resolution is irrelevant as this is used only for 3D checks. There are
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
class EnvironmentNavXYThetaLatMoveit : public EnvironmentNavXYThetaLatGeneric
{
    public:
        EnvironmentNavXYThetaLatMoveit(ros::NodeHandle & nhPriv, double costmapOffsetX, double costmapOffsetY);
        virtual ~EnvironmentNavXYThetaLatMoveit();

        virtual bool InitializeEnv(int width, int height, const unsigned char* mapdata,
                double startx, double starty, double starttheta,
                double goalx, double goaly, double goaltheta,
                double goaltol_x, double goaltol_y, double goaltol_theta,
                const std::vector<sbpl_2Dpt_t>& perimeterptsV, double cellsize_m,
                double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                unsigned char obsthresh, const char* sMotPrimFile);

        virtual int SetGoal(double x_m, double y_m, double theta_rad);
        virtual int SetStart(double x_m, double y_m, double theta_rad);
        virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);
        virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);

        /// Returns the scene's planning frame.
        virtual std::string getPlanningFrame() const;

        /// Transform a pose in any frame by a suitable method to the planning frame.
        virtual bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped & pose);

        /// Get the x/y dimensions of the OctoMap.
        virtual bool getExtents(double & minX, double & maxX, double & minY, double & maxY);

        /// Convert a pose from the world coordinate system in the planning frame
        /// into the environment's 0 origin system.
        virtual bool poseWorldToEnv(double wx, double wy, double wth, double & ex, double & ey, double & eth) const;

        /// Convert a pose from the world coordinate system in the planning frame
        /// into the environment's discretized coordinates.
        virtual bool poseWorldToGrid(double wx, double wy, double wth, int & gx, int & gy, int & gth) const;

        /// Convert a grid index from the ROS costmap coords to
        /// the environment's discretized coordinates.
        virtual bool posCostmapToGrid(int cx, int cy, int & gx, int & gy) const;

        /// Convert a pose from the environment's 0 origin system to world coordinates.
        virtual bool poseEnvToWorld(double ex, double ey, double eth, double & wx, double & wy, double & wth) const;

        /// Convert a pose from the environment's discretized coordinates into world coordinates.
        virtual bool poseGridToWorld(int gx, int gy, int gth, double & wx, double & wy, double & wth) const;

        /// Convert a grid index from the environment's discretized coordinates to the ROS costmap.
        virtual bool posGridToCostmap(int gx, int gy, int & cx, int & cy) const;

        geometry_msgs::Pose poseFromStateID(int stateID) const;

        /// Update the internal grid cost value for indices cx, cy from the costmap.
        /// The indices will be converted from costmap indices to the internal grid indices.
        /**
         * \Returns true if the index was found in the internal grid.
         */
        virtual bool UpdateCostFromCostmap(int cx, int cy, unsigned char newcost);

        /// Get the internal grid cost value for the costmap indices cx, cy.
        /// The indices will be converted from costmap indices to the internal grid indices.
        /**
         * \Returns obstthresh if the coordinates are not in the internal grid.
         */
        virtual unsigned char GetMapCostForCostmap(int cx, int cy);

        void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,
                std::vector<sbpl_xy_theta_pt_t>* xythetaPath);

        virtual void updateForPlanRequest();

        virtual void clear_full_body_collision_infos();

        /// Update the planning scene directly from the running MoveGroup instance.
        virtual void update_planning_scene();
        /// Update the planning scene to a custom one.
        virtual void update_planning_scene(planning_scene::PlanningSceneConstPtr scene);
        /// Publish the currently used planning scene instance.
        virtual void publish_planning_scene();
        /// Use this to access the PlanningScene. Never use internal data structures for that.
        virtual planning_scene::PlanningSceneConstPtr getPlanningScene() const;

        virtual moveit_msgs::DisplayTrajectory pathToDisplayTrajectory(
                const std::vector<geometry_msgs::PoseStamped> & path) const;

        void resetTimingStats();
        void printTimingStats();

    protected:
        struct FullBodyCollisionInfo {
            bool initialized;
            bool collision;

            FullBodyCollisionInfo() {
                initialized = false;
                collision = true;
            }
        };

        virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

        bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
        const FullBodyCollisionInfo& get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state);

        // disallow these to be called, won't work correctly any more
        virtual bool InitializeEnv(const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV,
                const char* sMotPrimFile);
        virtual bool InitializeEnv(const char* sEnvFile);

    protected:
        std::vector<FullBodyCollisionInfo> full_body_collision_infos;

        planning_scene::PlanningScenePtr scene;
        planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
        std::string scene_update_name;              ///< Scene updates are queried by this service.
        ros::Publisher planning_scene_publisher;
        std::vector<std::string> allowed_collision_links;

        /// Position of the 0, 0 grid index in the world
        double worldOriginX;
        double worldOriginY;

        Timing* timeActionCost;
        Timing* timeActionCostParent;
        Timing* timeFullBodyCollision;
        Timing* time3dCheck;
};

#endif

