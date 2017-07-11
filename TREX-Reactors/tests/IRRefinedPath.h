#pragma once
#include <string>

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/IntegerDomain.hh>

#include "../../source/MPC/source/ALMPC.h"
#include "../../source/MPC/source/LMModelLinear.h"

/*
*	CR: Common Reactor
*
*/


namespace TREX{

class IRRefinedPath
	: public TREX::transaction::TeleoReactor
{

public:

	IRRefinedPath(TREX::transaction::TeleoReactor::xml_arg_type arg);
	~IRRefinedPath();
	void handleInit();

	bool synchronize();
	void notify(TREX::transaction::Observation const & obs);

	bool hasWork();
	void resume();

private:

	std::vector<double> initialState;

	void handleRequest(TREX::transaction::goal_id const & goal);

	void dispatchGoals();
	void dispatchObservations();

	static LMModelLinear * linearModel;
	static TREX::utils::Symbol const AtPred;
	static TREX::utils::Symbol const GoingPred;
	TREX::utils::Symbol const UAVTimeline;
	static TREX::utils::Symbol const XVar;

	TREX::utils::Symbol const ActuatorTimeline;
	TREX::utils::Symbol const StateTimeline;
	TREX::utils::Symbol const PosTimeline;

	static TREX::utils::Symbol const north;
	static TREX::utils::Symbol const east;
	static TREX::utils::Symbol const down;

	static TREX::utils::Symbol const roll;
	static TREX::utils::Symbol const pitch;
	static TREX::utils::Symbol const yaw;

	static TREX::utils::Symbol const u;
	static TREX::utils::Symbol const v;
	static TREX::utils::Symbol const w;

	static TREX::utils::Symbol const p;
	static TREX::utils::Symbol const r;
	static TREX::utils::Symbol const q;
	
	TREX::transaction::goal_id pendingGoal;
	int currentTick;
	double jumpAt;

	bool planReady;
	bool needsWorkThisTick;

	int lastTickReceivedPlan;
	int nwpTick;

	LMMPC * mpc;
	double stepLength;

};



}