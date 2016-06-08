/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ENVIRONMENT_XYT_3D_COLLISIONS_H_
#define __ENVIRONMENT_XYT_3D_COLLISIONS_H_

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>
#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

class Environment_xyt_3d_collisions: public EnvironmentNAVXYTHETALATTICE
{
public:
    Environment_xyt_3d_collisions(double costmapOffsetX, double costmapOffsetY);

    ~Environment_xyt_3d_collisions();

    /**
     * \brief sets start in meters/radians
     */
    virtual int SetStart(double x, double y, double theta);

    /**
     * \brief sets goal in meters/radians
     */
    virtual int SetGoal(double x, double y, double theta);

    /**
     * \brief sets goal tolerance. (Note goal tolerance is ignored currently)
     */
    virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_theta)
    { /**< not used yet */
    }

    /**
     * \brief returns state coordinates of state with ID=stateID
     */
    virtual void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;

    /**
     * \brief returns stateID for a state with coords x,y,theta
     */
    virtual int GetStateFromCoord(int x, int y, int theta);

    /**
     * \brief returns the actions / motion primitives of the passed path.
     */
    virtual void GetActionsFromStateIDPath(std::vector<int>* stateIDPath, std::vector<EnvNAVXYTHETALATAction_t>* action_list);

    /** \brief converts a path given by stateIDs into a sequence of
     *         coordinates. Note that since motion primitives are short actions
     *         represented as a sequence of points,
     *         the path returned by this function contains much more points than the
     *         number of points in the input path. The returned coordinates are in
     *         meters,meters,radians
     */
    virtual void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath, std::vector<sbpl_xy_theta_pt_t>* xythetaPath);

    /**
     * \brief prints state info (coordinates) into file
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief returns all predecessors states and corresponding costs of actions
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief returns all successors states, costs of corresponding actions
     *        and pointers to corresponding actions, each of which is a motion
     *        primitive
     *        if actionindV is NULL, then pointers to actions are not returned
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);

    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual int GetTrueCost(int parentID, int childID);
    virtual bool isGoal(int id);
    //virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);
    //virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    //virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);

    /** \brief this function fill in Predecessor/Successor states of edges
     *         whose costs changed
     *         It takes in an array of cells whose traversability changed, and returns
     *         (in vector preds_of_changededgesIDV) the IDs of all states that have
     *         outgoing edges that go through the changed cells
     */
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV, std::vector<int> *preds_of_changededgesIDV);

    /**
     * \brief same as GetPredsofChangedEdges, but returns successor states.
     *        Both functions need to be present for incremental search
     */
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV, std::vector<int> *succs_of_changededgesIDV);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetStartHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintVars()
    {
    }

    void update_planning_scene();
    void update_planning_scene(planning_scene::PlanningSceneConstPtr scene);
    void publish_planning_scene();
    void clear_full_body_collision_infos();

    void publish_expanded_states();
    planning_scene::PlanningSceneConstPtr getPlanningScene()
    {
        return scene;
    }

protected:
    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvNAVXYTHETALATHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    std::vector<EnvNAVXYTHETALATHashEntry_t*> StateID2CoordTable;

    struct FullBodyCollisionInfo
    {
        bool initialized;
        bool collision;

        FullBodyCollisionInfo()
        {
            initialized = false;
            collision = true;
        }
    };
    std::vector<FullBodyCollisionInfo> full_body_collision_infos;

    EnvNAVXYTHETALATHashEntry_t** Coord2StateIDHashTable_lookup;

    planning_scene::PlanningScenePtr scene;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;
    ros::Publisher planning_scene_publisher;
    ros::Publisher pose_array_publisher;

    virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);

    virtual EnvNAVXYTHETALATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta);

    //pointers to functions
    EnvNAVXYTHETALATHashEntry_t* (Environment_xyt_3d_collisions::*GetHashEntry)(int X, int Y, int Theta);
    EnvNAVXYTHETALATHashEntry_t* (Environment_xyt_3d_collisions::*CreateNewHashEntry)(int X, int Y, int Theta);

    virtual void InitializeEnvironment();

    sbpl_xy_theta_pt_t discreteToContinuous(int x, int y, int theta);
    bool in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state);
    const FullBodyCollisionInfo& get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state);
    virtual void PrintHashTableHist(FILE* fOut);

    // offsets to convert costmap coordinates to world coordinates for 3d collision checks
    double costmapOffsetX;
    double costmapOffsetY;
};

#endif

