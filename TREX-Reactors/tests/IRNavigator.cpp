#include "IRNavigator.h"

const std::string UAVTIMELINENAME = "navigator_";


namespace
{

  /** @brief TREX log entry point */
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

  /** @brief Platform reactor declaration */
  TREX::transaction::TeleoReactor::xml_factory::declare<TREX::IRNavigator> decl("RefinedPath");

}


namespace TREX {
	void initPlugin() {
    	::s_log->syslog("plugin.lightswitch")<<"RefinedPath loaded."<<std::endl;
    // ::decl;
    }

	TREX::utils::Symbol const IRNavigator::AtPred("At");
	TREX::utils::Symbol const IRNavigator::GoingPred("Going");
	TREX::utils::Symbol const IRNavigator::XVar("XVar");
	//TREX::utils::Symbol const IRNavigator::UAVTimeline("navigator_1");


	TREX::utils::Symbol const IRNavigator::north("n");
	TREX::utils::Symbol const IRNavigator::east("e");
	TREX::utils::Symbol const IRNavigator::down("d");

	TREX::utils::Symbol const IRNavigator::roll("roll");
	TREX::utils::Symbol const IRNavigator::pitch("pitch");
	TREX::utils::Symbol const IRNavigator::yaw("yaw");

	TREX::utils::Symbol const IRNavigator::u("u");
	TREX::utils::Symbol const IRNavigator::v("v");
	TREX::utils::Symbol const IRNavigator::w("w");

	TREX::utils::Symbol const IRNavigator::p("p");
	TREX::utils::Symbol const IRNavigator::r("r");
	TREX::utils::Symbol const IRNavigator::q("q");


	IRNavigator::IRNavigator(TREX::transaction::TeleoReactor::xml_arg_type arg) 
				: TeleoReactor(arg, false), jumpAt(1100.0),	
				ActuatorTimeline("actuators_" + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number")),
				StateTimeline("state_" + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number")),
				PosTimeline("pos_" + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number"))
	{

		provide(ActuatorTimeline);
		provide(StateTimeline);
		provide(PosTimeline);

		initPosition();
		std::cout << "Navigator reactor created for UAV " << TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number") << "\n";

	}

	IRNavigator::~IRNavigator(){
		std::cout << "Refined path closed\n";
	}

	bool IRNavigator::synchronize(){

		n_v++;
		bool hasPosted = false;

		if(n_v > 5){
			TREX::transaction::Observation obs(ActuatorTimeline, AtPred);
		
			obs.restrictAttribute(north, TREX::transaction::FloatDomain(n_v));
			obs.restrictAttribute("start", TREX::transaction::IntegerDomain(2));
			obs.restrictAttribute("end", TREX::transaction::IntegerDomain(2));
			postObservation(obs);
			hasPosted = true;
		}
		if(hasPosted)
		std::cout << "posted observation: X =" << n_v << "\n";
		
		if (n_v == jumpAt)
			n_v += 100;

	}

	void IRNavigator::notify(TREX::transaction::Observation const & obs){




	}

	void IRNavigator::handleRequest(TREX::transaction::goal_id const & goal){

		std::cout << "Got request\n";
		pendingGoal = goal;
		if (goal->hasAttribute(north))
			jumpAt = goal->getAttribute(north).domain().getTypedSingleton<float, true>();


	}

	void IRNavigator::initPosition(){
		n_v = 0.0;
		e_v = 0.0;
		d_v = 0.0;
		roll_v = 0.0;
		pitch_v = 0.0;
		yaw_v = 0.0;
		u_v = 0.0;
		v_v = 0.0;
		w_v = 0.0;
		p_v = 0.0;
		q_v = 0.0;
		r_v = 0.0;
	}



}