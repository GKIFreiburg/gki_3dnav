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

#include <cmath>
#include <cstring>
#include <ctime>
#include <geometry_msgs/PoseArray.h>
#include "gki_3dnav_planner/environment_xyt_3d_collisions.h"
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;
//#define TIME_DEBUG 1
#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*EnvNAVXYTHETALATCfg.NumThetaDirs + \
                                  Y*EnvNAVXYTHETALATCfg.EnvWidth_c*EnvNAVXYTHETALATCfg.NumThetaDirs)

static long int checks = 0;

static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

Environment_xyt_3d_collisions::Environment_xyt_3d_collisions(double costmapOffsetX, double costmapOffsetY):costmapOffsetX(costmapOffsetX), costmapOffsetY(costmapOffsetY)
{
    GetHashEntry = NULL;
    CreateNewHashEntry = NULL;
    HashTableSize = 0;
    Coord2StateIDHashTable = NULL;
    Coord2StateIDHashTable_lookup = NULL;
    scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    // TODO: read topic from param, so it can be changed to tfd_planning scene
    scene_monitor->requestPlanningSceneState("/get_planning_scene");
    scene = scene_monitor->getPlanningScene();
    collision_detection::AllowedCollisionMatrix& allowed_collisions = scene->getAllowedCollisionMatrixNonConst();
    allowed_collisions.setDefaultEntry("br_caster_r_wheel_link", true);
    allowed_collisions.setDefaultEntry("br_caster_l_wheel_link", true);
    allowed_collisions.setDefaultEntry("bl_caster_r_wheel_link", true);
    allowed_collisions.setDefaultEntry("bl_caster_l_wheel_link", true);
    allowed_collisions.setDefaultEntry("fr_caster_r_wheel_link", true);
    allowed_collisions.setDefaultEntry("fr_caster_l_wheel_link", true);
    allowed_collisions.setDefaultEntry("fl_caster_r_wheel_link", true);
    allowed_collisions.setDefaultEntry("fl_caster_l_wheel_link", true);
    ros::NodeHandle nh("~");
    planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
    pose_array_publisher = nh.advertise<geometry_msgs::PoseArray>("expanded_states", 1, true);
}

Environment_xyt_3d_collisions::~Environment_xyt_3d_collisions()
{
    SBPL_PRINTF("destroying XYTHETALAT\n");

    //delete the states themselves first
    for (int i = 0; i < (int) StateID2CoordTable.size(); i++)
    {
        delete StateID2CoordTable.at(i);
        StateID2CoordTable.at(i) = NULL;
    }
    StateID2CoordTable.clear();

    //delete hashtable
    if (Coord2StateIDHashTable != NULL)
    {
        delete[] Coord2StateIDHashTable;
        Coord2StateIDHashTable = NULL;
    }
    if (Coord2StateIDHashTable_lookup != NULL)
    {
        delete[] Coord2StateIDHashTable_lookup;
        Coord2StateIDHashTable_lookup = NULL;
    }
}

void Environment_xyt_3d_collisions::GetCoordFromState(int stateID, int& x, int& y, int& theta) const
{
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    theta = HashEntry->Theta;
}

int Environment_xyt_3d_collisions::GetStateFromCoord(int x, int y, int theta)
{
    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL)
    {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    return OutHashEntry->stateID;
}

void Environment_xyt_3d_collisions::GetActionsFromStateIDPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETALATAction_t>* action_list)
{
    vector<EnvNAVXYTHETALATAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c;
    int sourcex_c, sourcey_c, sourcetheta_c;

    SBPL_PRINTF("checks=%ld\n", checks);

    action_list->clear();

    for (int pind = 0; pind < (int) (stateIDPath->size()) - 1; pind++)
    {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

        for (int sind = 0; sind < (int) SuccIDV.size(); sind++)
        {
            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
            {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1)
        {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
            SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c, targettheta_c);
            throw new SBPL_Exception();
        }

#if DEBUG
        SBPL_FPRINTF(fDeb, "Start: %.3f %.3f %.3f Target: %.3f %.3f %.3f Prim ID, Start Theta: %d %d\n",
                sourcex_c, sourcey_c, sourcetheta_c,
                targetx_c, targety_c, targettheta_c,
                actionV[bestsind]->aind, actionV[bestsind]->starttheta);
#endif

        action_list->push_back(*(actionV[bestsind]));
    }
}

void Environment_xyt_3d_collisions::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<sbpl_xy_theta_pt_t>* xythetaPath)
{
    vector<EnvNAVXYTHETALATAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c;
    int sourcex_c, sourcey_c, sourcetheta_c;

    SBPL_PRINTF("checks=%ld\n", checks);

    xythetaPath->clear();

#if DEBUG
    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

    for (int pind = 0; pind < (int) (stateIDPath->size()) - 1; pind++)
    {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
        SBPL_FPRINTF(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
                targetx_c, targety_c, targettheta_c, (int)SuccIDV.size());
#endif

        for (int sind = 0; sind < (int) SuccIDV.size(); sind++)
        {
#if DEBUG
            int x_c, y_c, theta_c;
            GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
            SBPL_FPRINTF(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c);
#endif

            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
            {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1)
        {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
            SBPL_PRINTF("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c, targettheta_c);
            throw new SBPL_Exception();
        }

        //now push in the actual path
        int sourcex_c, sourcey_c, sourcetheta_c;
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        double sourcex, sourcey;
        sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
        sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
        //TODO - when there are no motion primitives we should still print source state
        for (int ipind = 0; ipind < ((int) actionV[bestsind]->intermptV.size()) - 1; ipind++)
        {
            //translate appropriately
            sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
            intermpt.x += sourcex;
            intermpt.y += sourcey;

#if DEBUG
            int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETALATCfg.cellsize_m);
            int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETALATCfg.cellsize_m);
            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ",
                    intermpt.x, intermpt.y, intermpt.theta,
                    nx, ny,
                    ContTheta2Disc(intermpt.theta, EnvNAVXYTHETALATCfg.NumThetaDirs), EnvNAVXYTHETALATCfg.Grid2D[nx][ny]);
            if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
            else SBPL_FPRINTF(fDeb, "\n");
#endif

            //store
            xythetaPath->push_back(intermpt);
        }
    }
}

//returns the stateid if success, and -1 otherwise
int Environment_xyt_3d_collisions::SetGoal(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);

    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsWithinMapCell(x, y))
    {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    if (!IsValidConfiguration(x, y, theta))
    {
        SBPL_PRINTF("WARNING: goal configuration is invalid\n");
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL)
    {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    if (in_full_body_collision(OutHashEntry))
    {
        SBPL_ERROR("ERROR: goal state in collision\n", x, y);
    }

    //need to recompute start heuristics?
    if (EnvNAVXYTHETALAT.goalstateid != OutHashEntry->stateID)
    {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

    EnvNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

    EnvNAVXYTHETALATCfg.EndX_c = x;
    EnvNAVXYTHETALATCfg.EndY_c = y;
    EnvNAVXYTHETALATCfg.EndTheta = theta;

    return EnvNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int Environment_xyt_3d_collisions::SetStart(double x_m, double y_m, double theta_rad)
{
    int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
    int theta = ContTheta2Disc(theta_rad, EnvNAVXYTHETALATCfg.NumThetaDirs);

    if (!IsWithinMapCell(x, y))
    {
        SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x, y);
        return -1;
    }

    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

    if (!IsValidConfiguration(x, y, theta))
    {
        SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x, y, theta);
    }

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(x, y, theta)) == NULL)
    {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
    }
    if (in_full_body_collision(OutHashEntry))
    {
        SBPL_ERROR("ERROR: start state in collision\n", x, y);
    }

    //need to recompute start heuristics?
    if (EnvNAVXYTHETALAT.startstateid != OutHashEntry->stateID)
    {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true;
    }

    //set start
    EnvNAVXYTHETALAT.startstateid = OutHashEntry->stateID;
    EnvNAVXYTHETALATCfg.StartX_c = x;
    EnvNAVXYTHETALATCfg.StartY_c = y;
    EnvNAVXYTHETALATCfg.StartTheta = theta;

    return EnvNAVXYTHETALAT.startstateid;
}

void Environment_xyt_3d_collisions::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL)
        fOut = stdout;

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvNAVXYTHETALAT.goalstateid && bVerbose)
    {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
        SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
    else
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETALATCfg.cellsize_m), DISCXY2CONT(HashEntry->Y, EnvNAVXYTHETALATCfg.cellsize_m), DiscTheta2Cont(HashEntry->Theta, EnvNAVXYTHETALATCfg.NumThetaDirs));
}

EnvNAVXYTHETALATHashEntry_t* Environment_xyt_3d_collisions::GetHashEntry_lookup(int X, int Y, int Theta)
{
    if (X < 0 || X >= EnvNAVXYTHETALATCfg.EnvWidth_c || Y < 0 || Y >= EnvNAVXYTHETALATCfg.EnvHeight_c || Theta < 0 || Theta >= EnvNAVXYTHETALATCfg.NumThetaDirs)
        return NULL;
    int index = XYTHETA2INDEX(X, Y, Theta);
    return Coord2StateIDHashTable_lookup[index];
}

EnvNAVXYTHETALATHashEntry_t* Environment_xyt_3d_collisions::GetHashEntry_hash(int X, int Y, int Theta)
{
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(X, Y, Theta);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
        SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n",
                binid, X, Y, (int)Coord2StateIDHashTable[binid].size());

        PrintHashTableHist(fDeb);
    }
#endif

    //iterate over the states in the bin and select the perfect match
    vector<EnvNAVXYTHETALATHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
    for (int ind = 0; ind < (int) binV->size(); ind++)
    {
        EnvNAVXYTHETALATHashEntry_t* hashentry = binV->at(ind);
        if (hashentry->X == X && hashentry->Y == Y && hashentry->Theta == Theta)
        {
#if TIME_DEBUG
            time_gethash += clock()-currenttime;
#endif
            return hashentry;
        }
    }

#if TIME_DEBUG
    time_gethash += clock()-currenttime;
#endif

    return NULL;
}

EnvNAVXYTHETALATHashEntry_t* Environment_xyt_3d_collisions::CreateNewHashEntry_lookup(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);
    full_body_collision_infos.push_back(FullBodyCollisionInfo());

    int index = XYTHETA2INDEX(X, Y, Theta);

#if DEBUG
    if(Coord2StateIDHashTable_lookup[index] != NULL)
    {
        SBPL_ERROR("ERROR: creating hash entry for non-NULL hashentry\n");
        throw new SBPL_Exception();
    }
#endif

    Coord2StateIDHashTable_lookup[index] = HashEntry;

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int) StateID2IndexMapping.size() - 1)
    {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

EnvNAVXYTHETALATHashEntry_t* Environment_xyt_3d_collisions::CreateNewHashEntry_hash(int X, int Y, int Theta)
{
    int i;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

    HashEntry->X = X;
    HashEntry->Y = Y;
    HashEntry->Theta = Theta;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();

    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);
    full_body_collision_infos.push_back(FullBodyCollisionInfo());

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta);

    //insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int) StateID2IndexMapping.size() - 1)
    {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;
}

void Environment_xyt_3d_collisions::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    if (actionV != NULL)
    {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvNAVXYTHETALAT.goalstateid)
        return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
    {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int) HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY))
            continue;

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        // 3d collision check
        if (in_full_body_collision(OutHashEntry))
            continue;
//        cost += get_full_body_cost_penalty(OutHashEntry);
//        if (cost >= INFINITECOST)
//            continue;

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL)
            actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void Environment_xyt_3d_collisions::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    //TODO- to support tolerance, need:
    // a) generate preds for goal state based on all possible goal state variable settings,
    // b) change goal check condition in gethashentry c) change
    //    getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta].size());
    CostV->reserve(EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta].size());

    //iterate through actions
    vector<EnvNAVXYTHETALATAction_t*>* actionsV = &EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta];
    for (aind = 0; aind < (int) EnvNAVXYTHETALATCfg.PredActionsV[(unsigned int) HashEntry->Theta].size(); aind++)
    {

        EnvNAVXYTHETALATAction_t* nav3daction = actionsV->at(aind);

        int predX = HashEntry->X - nav3daction->dX;
        int predY = HashEntry->Y - nav3daction->dY;
        int predTheta = nav3daction->starttheta;

        //skip the invalid cells
        if (!IsValidCell(predX, predY))
            continue;

        //get cost
        int cost = GetActionCost(predX, predY, predTheta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta);
        }
        // 3d collision check
        if (in_full_body_collision(OutHashEntry))
            continue;
//        cost += get_full_body_cost_penalty(OutHashEntry);
//        if (cost >= INFINITECOST)
//            continue;

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

void Environment_xyt_3d_collisions::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    int cost;

#if DEBUG
    if(state->StateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if((int)state->Actions.size() != 0)
    {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //goal state should be absorbing
    if (state->StateID == EnvNAVXYTHETALAT.goalstateid)
        return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];

    //iterate through actions
    for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
    {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int) HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY))
            continue;

        //get cost
        cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

#if TIME_DEBUG
        clock_t currenttime = clock();
#endif

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }
        // 3d collision check
        if (in_full_body_collision(OutHashEntry))
            continue;
//        cost += get_full_body_cost_penalty(OutHashEntry);
//        if (cost >= INFINITECOST)
//            continue;

        //add the action
        CMDPACTION* action = state->AddAction(aind);
        action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
        time3_addallout += clock()-currenttime;
#endif
    }
}

void Environment_xyt_3d_collisions::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;

    //increment iteration for processing savings
    iteration++;

    for (int i = 0; i < (int) changedcellsV->size(); i++)
    {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int) affectedpredstatesV.size(); sind++)
        {
            affectedcell = affectedpredstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
            {
                preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }
        }
    }
}

void Environment_xyt_3d_collisions::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
{
    nav2dcell_t cell;
    sbpl_xy_theta_cell_t affectedcell;
    EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;

    SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
    throw new SBPL_Exception();

    //increment iteration for processing savings
    iteration++;

    //TODO - check
    for (int i = 0; i < (int) changedcellsV->size(); i++)
    {
        cell = changedcellsV->at(i);

        //now iterate over all states that could potentially be affected
        for (int sind = 0; sind < (int) affectedsuccstatesV.size(); sind++)
        {
            affectedcell = affectedsuccstatesV.at(sind);

            //translate to correct for the offset
            affectedcell.x = affectedcell.x + cell.x;
            affectedcell.y = affectedcell.y + cell.y;

            //insert only if it was actually generated
            affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta);
            if (affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
            {
                succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
                affectedHashEntry->iteration = iteration; //mark as already inserted
            }
        }
    }
}

void Environment_xyt_3d_collisions::InitializeEnvironment()
{
    EnvNAVXYTHETALATHashEntry_t* HashEntry;

    int maxsize = EnvNAVXYTHETALATCfg.EnvWidth_c * EnvNAVXYTHETALATCfg.EnvHeight_c * EnvNAVXYTHETALATCfg.NumThetaDirs;

    if (maxsize <= SBPL_XYTHETALAT_MAXSTATESFORLOOKUP)
    {
        SBPL_PRINTF("environment stores states in lookup table\n");

        Coord2StateIDHashTable_lookup = new EnvNAVXYTHETALATHashEntry_t*[maxsize];
        for (int i = 0; i < maxsize; i++)
            Coord2StateIDHashTable_lookup[i] = NULL;
        GetHashEntry = &Environment_xyt_3d_collisions::GetHashEntry_lookup;
        CreateNewHashEntry = &Environment_xyt_3d_collisions::CreateNewHashEntry_lookup;

        //not using hash table
        HashTableSize = 0;
        Coord2StateIDHashTable = NULL;
    }
    else
    {
        SBPL_PRINTF("environment stores states in hashtable\n");

        //initialize the map from Coord to StateID
        HashTableSize = 4 * 1024 * 1024; //should be power of two
        Coord2StateIDHashTable = new vector<EnvNAVXYTHETALATHashEntry_t*> [HashTableSize];
        GetHashEntry = &Environment_xyt_3d_collisions::GetHashEntry_hash;
        CreateNewHashEntry = &Environment_xyt_3d_collisions::CreateNewHashEntry_hash;

        //not using hash
        Coord2StateIDHashTable_lookup = NULL;
    }

    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    //create start state
    if ((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.StartTheta)) == NULL)
    {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.StartTheta);
    }
    EnvNAVXYTHETALAT.startstateid = HashEntry->stateID;

    //create goal state
    if ((HashEntry = (this->*GetHashEntry)(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.EndTheta)) == NULL)
    {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.EndTheta);
    }
    EnvNAVXYTHETALAT.goalstateid = HashEntry->stateID;

    //initialized
    EnvNAVXYTHETALAT.bInitialized = true;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int Environment_xyt_3d_collisions::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{
    return inthash(inthash(X1) + (inthash(X2) << 1) + (inthash(Theta) << 2)) & (HashTableSize - 1);
}

void Environment_xyt_3d_collisions::PrintHashTableHist(FILE* fOut)
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < HashTableSize; j++)
    {
        if ((int) Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int) Coord2StateIDHashTable[j].size() < 5)
            s1++;
        else if ((int) Coord2StateIDHashTable[j].size() < 25)
            s50++;
        else if ((int) Coord2StateIDHashTable[j].size() < 50)
            s100++;
        else if ((int) Coord2StateIDHashTable[j].size() < 100)
            s200++;
        else if ((int) Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d\n", s0, s1, s50, s100, s200, s300, slarge);
}

int Environment_xyt_3d_collisions::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)StateID2CoordTable.size()
            || ToStateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvNAVXYTHETALATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    //TODO - check if one of the gridsearches already computed and then use it.

    return (int) (NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);

}

int Environment_xyt_3d_collisions::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int) (NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(HashEntry->X, HashEntry->Y, EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int) (((double) __max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
}

int Environment_xyt_3d_collisions::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int) (NAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, HashEntry->X, HashEntry->Y));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int) (((double) __max(h2D, hEuclid)) / EnvNAVXYTHETALATCfg.nominalvel_mpersecs);
}

int Environment_xyt_3d_collisions::SizeofCreatedEnv()
{
    return (int) StateID2CoordTable.size();
}

//------------------------------------------------------------------------------

void Environment_xyt_3d_collisions::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    isTrueCost->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    if (actionV != NULL)
    {
        actionV->clear();
        actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    }

    //goal state should be absorbing
    if (SourceStateID == EnvNAVXYTHETALAT.goalstateid)
        return;

    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
    {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int) HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY))
            continue;

        if (!actionV)
        { //if we are supposed to return the action, then don't do lazy
            EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
            if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
                OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
            SuccIDV->push_back(OutHashEntry->stateID);
            CostV->push_back(nav3daction->cost);
            isTrueCost->push_back(false);
            continue;
        }

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST)
            continue;

        EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
        {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        isTrueCost->push_back(true);
        if (actionV != NULL)
            actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
}

int Environment_xyt_3d_collisions::GetTrueCost(int parentID, int childID)
{
    EnvNAVXYTHETALATHashEntry_t* fromHash = StateID2CoordTable[parentID];
    EnvNAVXYTHETALATHashEntry_t* toHash = StateID2CoordTable[childID];

    for (int i = 0; i < EnvNAVXYTHETALATCfg.actionwidth; i++)
    {
        EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[(unsigned int) fromHash->Theta][i];
        int newX = fromHash->X + nav3daction->dX;
        int newY = fromHash->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY))
            continue;

        EnvNAVXYTHETALATHashEntry_t* hash;
        if ((hash = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL)
            continue;
        if (hash->stateID != toHash->stateID)
            continue;

        //get cost
        int cost = GetActionCost(fromHash->X, fromHash->Y, fromHash->Theta, nav3daction);

        if (cost >= INFINITECOST)
            return -1;
        return cost;
    }
    printf("this should never happen! we didn't find the state we need to get the true cost for!\n");
    throw new SBPL_Exception();
    return -1;
}

void Environment_xyt_3d_collisions::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    GetSuccs(SourceStateID, SuccIDV, CostV, actionV);
}

void Environment_xyt_3d_collisions::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    GetLazySuccs(SourceStateID, SuccIDV, CostV, isTrueCost, actionV);
}

bool Environment_xyt_3d_collisions::isGoal(int id)
{
    return EnvNAVXYTHETALAT.goalstateid == id;
}

sbpl_xy_theta_pt_t Environment_xyt_3d_collisions::discreteToContinuous(int x, int y, int theta)
{
    sbpl_xy_theta_pt_t pose;
    pose.x = DISCXY2CONT(x, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.y = DISCXY2CONT(y, EnvNAVXYTHETALATCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(theta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    return pose;
}


bool Environment_xyt_3d_collisions::in_full_body_collision(EnvNAVXYTHETALATHashEntry_t* state)
{
    return get_full_body_collision_info(state).collision;
}

const Environment_xyt_3d_collisions::FullBodyCollisionInfo& Environment_xyt_3d_collisions::get_full_body_collision_info(EnvNAVXYTHETALATHashEntry_t* state)
{
    ROS_ASSERT_MSG(full_body_collision_infos.size() > state->stateID, "full_body_collision: state_id mismatch!");
    if (! full_body_collision_infos[state->stateID].initialized)
    {
        collision_detection::CollisionRequest request;
        collision_detection::CollisionResult result;
        sbpl_xy_theta_pt_t pose = discreteToContinuous(state->X, state->Y, state->Theta);
        robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
        robot_state.setVariablePosition("world_joint/x", pose.x + costmapOffsetX);
        robot_state.setVariablePosition("world_joint/y", pose.y + costmapOffsetY);
        robot_state.setVariablePosition("world_joint/theta", pose.theta);
        scene->checkCollision(request, result);
        full_body_collision_infos[state->stateID].initialized = true;
        full_body_collision_infos[state->stateID].collision = result.collision;
    }
    return full_body_collision_infos[state->stateID];
}

void Environment_xyt_3d_collisions::update_planning_scene()
{
    scene_monitor->requestPlanningSceneState("/get_planning_scene");
    scene = scene_monitor->getPlanningScene();
}

void Environment_xyt_3d_collisions::update_planning_scene(planning_scene::PlanningSceneConstPtr scene)
{
    this->scene = planning_scene::PlanningScene::clone(scene);
}

void Environment_xyt_3d_collisions::publish_planning_scene()
{
    moveit_msgs::PlanningScene msg;
    scene->getPlanningSceneMsg(msg);
    planning_scene_publisher.publish(msg);
}

void Environment_xyt_3d_collisions::publish_expanded_states()
{
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = scene->getPlanningFrame();
    for (size_t id = 0; id < full_body_collision_infos.size(); id++)
    {
        if (full_body_collision_infos[id].initialized && ! full_body_collision_infos[id].collision)
        {
            EnvNAVXYTHETALATHashEntry_t* state = StateID2CoordTable[id];
            sbpl_xy_theta_pt_t coords = discreteToContinuous(state->X, state->Y, state->Theta);
            msg.poses.push_back(geometry_msgs::Pose());
            geometry_msgs::Pose& pose = msg.poses.back();
            pose.position.x = coords.x + costmapOffsetX;
            pose.position.y = coords.y + costmapOffsetY;
            pose.position.z = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(coords.theta);
        }
    }
    pose_array_publisher.publish(msg);
}

void Environment_xyt_3d_collisions::clear_full_body_collision_infos()
{
    for (size_t i = 0; i < full_body_collision_infos.size(); i++)
    {
        full_body_collision_infos[i].initialized = false;
    }
}

