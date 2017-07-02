#pragma once
#include <string>

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/IntegerDomain.hh>


/*
*	CR: Common Reactor
*
*/


namespace TREX{

class IRNavigator
	: public TREX::transaction::TeleoReactor
{

public:

	IRNavigator(TREX::transaction::TeleoReactor::xml_arg_type arg);
	~IRNavigator();

	bool synchronize();
	void notify(TREX::transaction::Observation const & obs);

private:

	void handleRequest(TREX::transaction::goal_id const & goal);

	void sendPosition();
	void initPosition();

	double 	n_v,e_v,d_v,
			roll_v,pitch_v,yaw_v,
			u_v,v_v,w_v,
			p_v,q_v,r_v;

	static TREX::utils::Symbol const AtPred;
	static TREX::utils::Symbol const GoingPred;
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
	double currentX;
	double jumpAt;


};



}