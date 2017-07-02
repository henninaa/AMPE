#include "IRRefinedPath.h"

const std::string UAVTIMELINENAME = "navigator_";


namespace
{

  /** @brief TREX log entry point */
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

  /** @brief Platform reactor declaration */
  TREX::transaction::TeleoReactor::xml_factory::declare<TREX::IRRefinedPath> decl("RefinedPath");

}


namespace TREX {
	void initPlugin() {
    	::s_log->syslog("plugin.lightswitch")<<"RefinedPath loaded."<<std::endl;
    // ::decl;
    }

	TREX::utils::Symbol const IRRefinedPath::AtPred("At");
	TREX::utils::Symbol const IRRefinedPath::GoingPred("Going");
	TREX::utils::Symbol const IRRefinedPath::XVar("XVar");


	TREX::utils::Symbol const IRRefinedPath::north("n");
	TREX::utils::Symbol const IRRefinedPath::east("e");
	TREX::utils::Symbol const IRRefinedPath::down("d");

	TREX::utils::Symbol const IRRefinedPath::roll("roll");
	TREX::utils::Symbol const IRRefinedPath::pitch("pitch");
	TREX::utils::Symbol const IRRefinedPath::yaw("yaw");

	TREX::utils::Symbol const IRRefinedPath::u("u");
	TREX::utils::Symbol const IRRefinedPath::v("v");
	TREX::utils::Symbol const IRRefinedPath::w("w");

	TREX::utils::Symbol const IRRefinedPath::p("p");
	TREX::utils::Symbol const IRRefinedPath::r("r");
	TREX::utils::Symbol const IRRefinedPath::q("q");


	IRRefinedPath::IRRefinedPath(TREX::transaction::TeleoReactor::xml_arg_type arg) 
				: TeleoReactor(arg, false), jumpAt(1100.0),	
				UAVTimeline(UAVTIMELINENAME + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number")),
				ActuatorTimeline("actuators_" + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number")),
				StateTimeline("state_" + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number")),
				PosTimeline("pos_" + TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number"))
	{

		provide(UAVTimeline);
		currentX = 0;
		std::cout << "Refined path reactor created for UAV " << TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number") << "\n";
		

	}

	IRRefinedPath::~IRRefinedPath(){
		std::cout << "Refined path closed\n";
	}

	bool IRRefinedPath::synchronize(){

		currentX++;
		bool hasPosted = false;

		if(currentX > 5){
			TREX::transaction::Observation obs(UAVTimeline, AtPred);
		
			obs.restrictAttribute("X", TREX::transaction::FloatDomain(currentX));
			obs.restrictAttribute("start", TREX::transaction::IntegerDomain(2));
			obs.restrictAttribute("end", TREX::transaction::IntegerDomain(2));
			postObservation(obs);
			hasPosted = true;
		}
		if(hasPosted)
		std::cout << "posted observation: X =" << currentX << "\n";
		
		if (currentX == jumpAt)
			currentX += 100;

	}

	void IRRefinedPath::notify(TREX::transaction::Observation const & obs){




	}

	void IRRefinedPath::handleRequest(TREX::transaction::goal_id const & goal){

		std::cout << "Got request\n";
		pendingGoal = goal;
		if (goal->hasAttribute("X"))
			jumpAt = goal->getAttribute("X").domain().getTypedSingleton<float, true>();


	}





}