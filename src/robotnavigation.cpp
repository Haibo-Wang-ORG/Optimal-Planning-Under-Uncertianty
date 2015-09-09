/*
 * robotnavigation.cpp
 *
 *  Created on: 18/07/2014
 *      Author: haibowang
 */

#include "robotnavigation.h"
#include "utils.h"
#include <time.h>

/**
 * The actual map, from human perspective, is:
 * 		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 1, 1, 1, 1, 1, 1},
		{1, 0, 0, 0, 1, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 1, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 1, 0, 1, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 1, 0, 0, 1},
		{1, 1, 1, 0, 0, 0, 1, 0, 0, 1},
		{1, 1, 1, 0, 0, 0, 1, 0, 0, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
 */
const int ROBOT_NAVIGATION::Map[10][10] = {
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 0, 0, 0, 0, 0, 0, 1},
		{1, 1, 1, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 1, 1, 1, 1, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
		{1, 1, 1, 1, 1, 0, 0, 1, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

/**
 * 5 actions encoding: stay, up, down, left, right
 */
const int ROBOT_NAVIGATION::DeltaAct[5][2] = {
		{0, 0},   	// Stay
		{0, 1},		// Up
		{0, -1}, 	// Down
		{-1, 0},	// Left
		{1, 0}, 	// Right
};

/**
 * Encoding the delta observations in anti-clockwise direction
 */
const int ROBOT_NAVIGATION::DeltaObs[8][2] = {
		{-1, -1},
		{-1, 0},
		{-1, 1},
		{0, 1},
		{1, 1},
		{1, 0},
		{1, -1},
		{0, -1},
};

ROBOT_NAVIGATION::ROBOT_NAVIGATION()
{
	NumActions = 5;
	NumObservations = 1 << 8 + 1;
	Discount = 0.95;

	UTILS::RandomSeed(time(NULL));

	V = std::vector<std::vector<double> >(10, std::vector<double>(10, -10000.0));
	A = std::vector<std::vector<int> >(10, std::vector<int>(10, -1));
	V[GoalState.X][GoalState.Y] = 100;
	A[GoalState.X][GoalState.Y] = 0;

	ValueIteration(V, A, 0.001);
//	for (int i=0; i<10; i++)
//	{
//		for (int j=0; j<10; j++)
//			std::cout << A[i][j] << " ";
//		std::cout << std::endl;
//	}

}

ROBOT_NAVIGATION::~ROBOT_NAVIGATION()
{
}

STATE* ROBOT_NAVIGATION::CreateStartState() const
{
	ROBOT_STATE* robotstate = MemoryPool.Allocate();
	std::vector<ROBOT_STATE> startStates;
	for (int i=7; i<=8; i++)
		for (int j=1; j<=4; j++)
			startStates.push_back(ROBOT_STATE(i,j));
//	startStates.push_back(ROBOT_STATE(7,3));
//	startStates.push_back(ROBOT_STATE(4,2));
//	startStates.push_back(ROBOT_STATE(2,4));
	int idx = UTILS::Random(0, startStates.size());
	robotstate->Copy(startStates[idx]);
	startStates.clear();
	return robotstate;
}

void ROBOT_NAVIGATION::FreeState(STATE* state) const
{
	ROBOT_STATE* robotstate = safe_cast<ROBOT_STATE*>(state);
	MemoryPool.Free(robotstate);
}

bool ROBOT_NAVIGATION::Step(STATE& state, int action, int& observation, double& reward) const
{
	ROBOT_STATE& robotstate = safe_cast<ROBOT_STATE&>(state);

	if (robotstate.X == GoalState.X && robotstate.Y == GoalState.Y)
	{
		observation = 1 << 8;
		reward = 100;
		return true;
	}

	/*
	 * The actions are up, down, left, right, and stay in place.
	 *
	 * The movement actions move the agent one grid cell in the desired direction with
	 * probability 0.6(or 0.9)or in one of the other directions or current cell with
	 * probability 0.1(or 0.025) each.
	 *
	 * Movements into a wall result in the robot's staying in place.
	 *
	 * Choosing to stay in place always keeps the agent in the current location.
	 */

	int idx; // The resulting moving direction index after performing action
	if (action == 0)
	{
		idx = 0;
	}
	else
	{
		double chance = UTILS::RandomDouble(0, 1.0);
		if (chance < 0.9)
			idx = action;
		else if (chance < 0.925)
			idx = (action + 1) % 5;
		else if (chance < 0.950)
			idx = (action + 2) % 5;
		else if (chance < 0.975)
			idx = (action + 3) % 5;
		else
			idx = (action + 4) % 5;
	}

	int newx = robotstate.X + ROBOT_NAVIGATION::DeltaAct[idx][0];
	int newy = robotstate.Y + ROBOT_NAVIGATION::DeltaAct[idx][1];
	if (IsMovable(newx, newy))
	{
		robotstate.X = newx;
		robotstate.Y = newy;
	} /* Otherwise the robot stays in place, that is, robotstate does not change */

	/*
	 * We assume perfect observation of the grid cells immediately surrounding the agent.
	 * As a result, the robot can observe the wall configurations in surrounding squares,
	 * but not its own actual location.
	 */
	int obs = 0;
	for (int i=0; i<8; i++)
	{
		int obsx = robotstate.X + ROBOT_NAVIGATION::DeltaObs[i][0];
		int obsy = robotstate.Y + ROBOT_NAVIGATION::DeltaObs[i][1];
		if (ROBOT_NAVIGATION::Map[obsx][obsy] == 1)
			obs += 1 << i;
	}
	observation = obs;

	// Reward and terminal state
	reward = -1;

	return false;
}

bool ROBOT_NAVIGATION::IsMovable(int x, int y) const
{
	return (ROBOT_NAVIGATION::Map[x][y] == 0) ? true : false;
}

bool ROBOT_NAVIGATION::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const
{
	ROBOT_STATE& robotstate = safe_cast<ROBOT_STATE&>(state);

	/*
	 * We assume perfect observation of the grid cells immediately surrounding the agent.
	 * As a result, the robot can observe the wall configurations in surrounding squares,
	 * but not its own actual location.
	 */
	int obs = 0;
	for (int i=0; i<8; i++)
	{
		int obsx = robotstate.X + ROBOT_NAVIGATION::DeltaObs[i][0];
		int obsy = robotstate.Y + ROBOT_NAVIGATION::DeltaObs[i][1];
		if (ROBOT_NAVIGATION::Map[obsx][obsy] == 1)
			obs += 1 << i;
	}

	return (obs == history.Back().Observation);
}

void ROBOT_NAVIGATION::GeneratePreferred(const STATE& state, const HISTORY& history, std::vector<int>& actions, const STATUS& status) const
{
	const ROBOT_STATE& robotstate = safe_cast<const ROBOT_STATE&>(state);
	actions.push_back(A[robotstate.X][robotstate.Y]);
}

STATE* ROBOT_NAVIGATION::Copy(const STATE& state) const
{
	const ROBOT_STATE& robotstate = safe_cast<const ROBOT_STATE&>(state);
	ROBOT_STATE* newstate = MemoryPool.Allocate();
	*newstate = robotstate;
	return newstate;
}

void ROBOT_NAVIGATION::Validate(const STATE& state) const
{
	const ROBOT_STATE& robotstate = safe_cast<const ROBOT_STATE&>(state);
	assert(robotstate.X > 0 && robotstate.X < 9 && robotstate.Y > 0 && robotstate.Y < 9);
}

void ROBOT_NAVIGATION::DisplayState(const STATE& state, std::ostream& ostr) const
{
	const ROBOT_STATE& robotstate = safe_cast<const ROBOT_STATE&>(state);
	ostr << "State: " << robotstate.X << " " << robotstate.Y << std::endl;
}

void ROBOT_NAVIGATION::ValueIteration(std::vector<std::vector<double> >& V, std::vector<std::vector<int> >& A, double threshold)
{
	std::vector<std::vector<double> > tempV = V;
	for (int i=1; i<=8; i++)
		for (int j=1; j<=8; j++)
		{
			if (ROBOT_NAVIGATION::Map[i][j] == 1 || (i == GoalState.X && j == GoalState.Y)) continue; // Skip the cell of wall & the goal state

			// V(s) = max_{a \in A}(R(s,a) + r*(sum_{s' \in S}(p(s'|s,a)*V(s'))))
			double reward = -1 + Discount * V[i][j]; // For action 0, i.e., action of staying in place
			for (int idx=1; idx<5; idx++) // For other four actions: up, down, left, right
			{
				// calculate the expected reward with the corresponding action
				double expectedSum = 0, p = 0;
				for (int k=1; k<5; k++)
					if (IsMovable(i+ROBOT_NAVIGATION::DeltaAct[k][0], j+ROBOT_NAVIGATION::DeltaAct[k][1]))
						if (idx == k)
						{
							expectedSum += 0.9 * V[i+ROBOT_NAVIGATION::DeltaAct[k][0]][j+ROBOT_NAVIGATION::DeltaAct[k][1]];
							p += 0.9;
						}
						else
						{
							expectedSum += 0.025 * V[i+ROBOT_NAVIGATION::DeltaAct[k][0]][j+ROBOT_NAVIGATION::DeltaAct[k][1]];
							p += 0.025;
						}
				expectedSum += (1-p) * V[i][j];

				double tempReward = -1 + Discount * expectedSum;

				if (tempReward > reward)
				{
					reward = tempReward;
					A[i][j] = idx;
				}
			}
			tempV[i][j] = reward;
		}

	double maxgap = 0;
	for (int i=1; i<=8; i++)
		for (int j=1; j<=8; j++)
			if (ROBOT_NAVIGATION::Map[i][j] == 0)
				if (abs(tempV[i][j] - V[i][j]) > maxgap)
					maxgap = abs(tempV[i][j] - V[i][j]);

	if (maxgap > threshold)
	{
		V = tempV;
		tempV.clear();
		ValueIteration(V, A, threshold);
	}
}

