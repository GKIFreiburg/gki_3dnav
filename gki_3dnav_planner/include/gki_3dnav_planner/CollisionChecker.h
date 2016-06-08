/*
 * CollisionChecker.h
 *
 *  Created on: May 27, 2015
 *      Author: andreas
 */

#ifndef COLLISIONCHECKER_H_
#define COLLISIONCHECKER_H_

#include <tr1/unordered_map.h>
#include "gki_3dnav_planner/Params.h"

namespace gki_3dnav_planner
{

struct CellData
{
    size_t id;
    tf::Point position;
};

//typedef std::tr1::unordered_map<Octokey, CellData, Octokey::KeyHash> KeyCellDataMap;

struct Cost
{
    int action;
    double roll;
    double pitch;
    double deltaRoll;
    double deltaPitch;
    double stepPitch;
    int total;
    Cost() :
            action(0), roll(0), pitch(0), deltaRoll(0), deltaPitch(0), stepPitch(0), total(0)
    {
    }
    Cost(int actionCost) :
            action(actionCost), roll(0), pitch(0), deltaRoll(0), deltaPitch(0), stepPitch(0), total(0)
    {
    }

    Cost& operator+=(const Cost& other)
    {
        Cost& result = *this;
        result.action += other.action;
        result.roll += other.roll;
        result.pitch += other.pitch;
        result.deltaRoll += other.deltaRoll;
        result.deltaPitch += other.deltaPitch;
        result.stepPitch += other.stepPitch;
        result.total += other.total;
        result.total = fmin(result.total, params::infiniteCosts);
        return result;
    }

    Cost operator+(const Cost& other) const
    {
        Cost result = *this;
        result.action += other.action;
        result.roll += other.roll;
        result.pitch += other.pitch;
        result.deltaRoll += other.deltaRoll;
        result.deltaPitch += other.deltaPitch;
        result.stepPitch += other.stepPitch;
        result.total += other.total;
        result.total = fmin(result.total, params::infiniteCosts);
        return result;
    }

    void recalculateTotal()
    {
        total = fmax(total, fmax(action, action * (1 + roll + pitch + deltaRoll + deltaPitch + stepPitch)));
    }
};

class CollisionChecker
{
public:
    CollisionChecker();
    virtual ~CollisionChecker();

    bool isPoseInCollision(const tf::Pose& pose) const;

private:

};

} /* namespace gki_3dnav_planner */

#endif /* COLLISIONCHECKER_H_ */
