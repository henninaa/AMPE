#pragma once

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>

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

private:

	void handleRequest(TREX::transaction::goal_id const & goal);

	static TREX::utils::Symbol const UAVTimeline_1;
	static TREX::utils::Symbol const UAVTimeline_2;
	static TREX::utils::Symbol const UAVTimeline_3;
	static TREX::utils::Symbol const gatherDataTimeline;
};



}