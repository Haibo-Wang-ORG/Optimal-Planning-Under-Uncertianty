/*
 * robotnavigation.h
 *
 *  Created on: 18/07/2014
 *      Author: haibowang
 */

#ifndef ROBOTNAVIGATION_H_
#define ROBOTNAVIGATION_H_

#include "simulator.h"

/**
 * The state corresponds to the position of the robot.
 */
class ROBOT_STATE: public STATE
{
public:
	ROBOT_STATE()
	{
		X = 1;
		Y = 1;
	}

	ROBOT_STATE(int x, int y): X(x), Y(y) {}

	void Copy(ROBOT_STATE robotstate)
	{
		X = robotstate.X;
		Y = robotstate.Y;
	}

	int X, Y;
};

class ROBOT_NAVIGATION: public SIMULATOR
{
public:
	ROBOT_NAVIGATION();
	virtual ~ROBOT_NAVIGATION();

	virtual STATE* CreateStartState() const;
	virtual void FreeState(STATE* state) const;
	virtual bool Step(STATE& state, int action, int& observation, double& reward) const;

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;

    virtual bool LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const;

    virtual void GeneratePreferred(const STATE& state, const HISTORY& history, std::vector<int>& actions, const STATUS& status) const;

    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
//    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, std::ostream& ostr) const;

private:
    bool IsMovable(int x, int y) const;
    void ValueIteration(std::vector<std::vector<double> >& V, std::vector<std::vector<int> >& A, double threshold);
    double inline abs(double value) {return (value < 0) ? -value : value;}

	static const int Map[10][10];
	static const int DeltaAct[5][2];
	static const int DeltaObs[8][2];

	std::vector<std::vector<double> > V;
	std::vector<std::vector<int> > A;

	mutable MEMORY_POOL<ROBOT_STATE> MemoryPool;

	ROBOT_STATE StartState = ROBOT_STATE(7,3);
	ROBOT_STATE GoalState = ROBOT_STATE(8,8);
};

#endif /* ROBOTNAVIGATION_H_ */
