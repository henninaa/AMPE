#pragma once
#include <string>

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/IntegerDomain.hh>

#include "shared.h"
#include "CrudePathModule.h"

/*
*	CR: Common Reactor
*
*/


namespace TREX{

class CRCrudePath
	: public TREX::transaction::TeleoReactor
{

public:

	CRCrudePath(TREX::transaction::TeleoReactor::xml_arg_type arg);
	~CRCrudePath();

	bool synchronize();
	void notify(TREX::transaction::Observation const & obs);
	bool hasWork();
	void resume();

	void handleInit();

private:

	class UAVTimelinePair{
	public:
		UAVTimelinePair(int id, TREX::utils::Symbol timeline) : id(id) { this->timeline = timeline; }
		~UAVTimelinePair() {}

		TREX::utils::Symbol timeline;
		int id;
		std::vector<WP> path;
	};

	void addUAVTimelinePair(double n0, double e0, double d0, TREX::utils::Symbol timeline);


	void handleRequest(TREX::transaction::goal_id const & goal);

	void dispatchPlan();

	static TREX::utils::Symbol const UAVTimeline_1;
	static TREX::utils::Symbol const UAVTimeline_2;
	static TREX::utils::Symbol const UAVTimeline_3;
	static TREX::utils::Symbol const gatherDataTimeline;
	static TREX::utils::Symbol const AtPred;
	static TREX::utils::Symbol const GoingPred;

	int UAVIdCounter;
	int nodeIdCounter;

	std::vector<UAVTimelinePair> uavTimelinePairs;
	bool planReady;
	bool hasWorkThisTick;
	bool deliberationNeedsStart;
	bool const moduleSwitch = true;;
	int currentTick;
	double stepLength;
	double sampleTime;
	int planStartedAt;

	CrudePathModule module;

	void printPlan();
};


}

