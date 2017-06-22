#pragma once


namespace TREX{

class CRCrudePath
	: public TREX::transaction::TeleoReactor
{

public:

	CRCrudePath(TREX::transaction::TeleoReactor::xml_arg_type arg);
	~CRCrudePath();


	void notify(TREX::transaction::Observation const & obs);

private:

	void handleRequest(TREX::transaction::goal_id const & goal);
	bool synchronize();

	static TREX::utils::Symbol const navigatorTimeline;
	static TREX::utils::Symbol const gatherDataTimeline;
};



}