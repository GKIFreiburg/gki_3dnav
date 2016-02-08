#ifndef ENVIRONMENT_NAVXYTHETALAT_GENERIC_H
#define ENVIRONMENT_NAVXYTHETALAT_GENERIC_H

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include "freespace_mechanism_heuristic/freespace_mechanism_heuristic.h"
#include "timing/timing.h"

struct ScopeExit
{
    ScopeExit(boost::function<void ()> fn) : function_(fn) { }
    ~ScopeExit() { function_(); }

    boost::function<void()> function_;
};

/// Generic x, y, theta environment that does not assume a specific grid.
/**
 * Child classes should make this behave like a discretized x, y, theta lattice.
 *
 * All poses are assumed to be in one frame given by getPlanningFrame().
 */
class EnvironmentNavXYThetaLatGeneric : public EnvironmentNAVXYTHETALAT
{
    public:
        EnvironmentNavXYThetaLatGeneric(ros::NodeHandle & nhPriv);
        virtual ~EnvironmentNavXYThetaLatGeneric();

        /// Returns the tf frame that this env assumes all poses to be in.
        virtual std::string getPlanningFrame() const = 0;

        /// Transform a pose in any frame by a suitable method to the planning frame.
        virtual bool transformPoseToPlanningFrame(geometry_msgs::PoseStamped & pose) = 0;

        /// Return a pose in the planning frame for stateID.
        virtual geometry_msgs::Pose poseFromStateID(int stateID) const = 0;

        /// Get the x/y dimensions of the underlying environment.
        virtual bool getExtents(double & minX, double & maxX, double & minY, double & maxY) = 0;


        /// Update the internal representation to be current for a plan request.
        /// Called, whenever makePlan is called.
        virtual void updateForPlanRequest();

        // heuristic handling
        virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
        virtual int GetStartHeuristic(int stateID);
        virtual int GetGoalHeuristic(int stateID);

        bool useFreespaceHeuristic(bool on);
        virtual int getFreespaceCost(int deltaX, int deltaY, int theta_start, int theta_end);

        /// Provides a MoveIt planning scene of the current state (for pathToDisplayTrajectory).
        /**
         * If this is not available, it is handled gracefully, but pathToDisplayTrajectory won't work.
         */
        virtual planning_scene::PlanningSceneConstPtr getPlanningScene() const {
            return planning_scene::PlanningSceneConstPtr();
        }

        /// Convert a path into a DisplayTrajectory.
        virtual moveit_msgs::DisplayTrajectory pathToDisplayTrajectory(
                const std::vector<geometry_msgs::PoseStamped> & path) const;

        virtual void resetTimingStats();
        virtual void printTimingStats();

    protected:
        ros::NodeHandle nhPriv_;

        freespace_mechanism_heuristic::HeuristicCostMap* freespace_heuristic_costmap;
        bool useFreespaceHeuristic_;

        Timing* timeFreespace;
        Timing* timeHeuristic;
};

#endif

