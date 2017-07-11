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
    	std::cout << "Init refined path";
    	

    }

    LMModelLinear * IRRefinedPath::linearModel = new LMModelLinear();

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
		currentTick = -1;
		std::cout << "Refined path reactor created for UAV " << TREX::utils::parse_attr<std::string>("1", TeleoReactor::xml_factory::node(arg), "uav_number") << "\n";

		planReady = false;
		needsWorkThisTick = false;
		lastTickReceivedPlan = -1;
		nwpTick = 0;
		stepLength = 0.25;


		initialState = std::vector<double>(18,0);

		initialState[10] = TREX::utils::parse_attr<float>(0.0f, TeleoReactor::xml_factory::node(arg), "init_n");
		initialState[11] = TREX::utils::parse_attr<float>(0.0f, TeleoReactor::xml_factory::node(arg), "init_e");
		initialState[12] = TREX::utils::parse_attr<float>(0.0f, TeleoReactor::xml_factory::node(arg), "init_d");
		std::cout << "With initial position: " << initialState[10]<< " " <<  initialState[11]<< " " << initialState[12]<< "\n";

	}

	IRRefinedPath::~IRRefinedPath(){
		std::cout << "Refined path closed\n";
	}

	void IRRefinedPath::handleInit(){
		mpc = new LMMPC(linearModel);
		mpc->setup(10.0, 0.25);


		mpc->initializeController(initialState);

		//mpc->addWaypoint(0.0, 0.0, 0.0, 0.0);
		//mpc->addWaypoint(18.0*15, 0.0, 0.0, 15.0);
		//mpc->addWaypoint(18.0*15, 15 * 18.0, 0.0, 30.0);
		//mpc->addWaypoint(18.0*15, 45 * 18.0, 0.0, 60.0);
		//mpc->addWaypoint(165.0, 30.0, 0.0, 9.0);
		mpc->addWaypoint(initialState[10], initialState[11], initialState[12], 0.0);
		mpc->addWaypoint(600.0, 500.0, 0.0, 38.221);
		mpc->addWaypoint(200, 800, 00.0, 65.2480);
		mpc->addWaypoint(-200.0, 800.0, 00.0, 86.869);
		mpc->addWaypoint(00.0, 00.0, 00.0, 131.4431);
		std::cout << "Waypoints inserted\n";

	}

	bool IRRefinedPath::synchronize(){

		currentTick++;

		if(planReady == true){

			//dispatchGoals();
			dispatchObservations();
			planReady = false;

			

		}
		if(!needsWorkThisTick){
				needsWorkThisTick = true;
			}
		return true;
	
	}

	void IRRefinedPath::notify(TREX::transaction::Observation const & obs){




	}

	bool IRRefinedPath::hasWork(){
		return needsWorkThisTick;
	}

	void IRRefinedPath::resume(){

		planReady = true;
		
		mpc->step(stepLength * currentTick);
		std::cout << "\n step: " << stepLength * currentTick << "\n";
		//mpc->simulate(30.0);
		if(currentTick % 50 == 0 && currentTick == 200)
			mpc->plot();

		needsWorkThisTick = false;

	}

	void IRRefinedPath::handleRequest(TREX::transaction::goal_id const & goal){

		std::cout << "Got request\n";
		pendingGoal = goal;
		if (goal->hasAttribute("X"))
			jumpAt = goal->getAttribute("X").domain().getTypedSingleton<float, true>();


		if(goal->object() == UAVTimeline){
			std::cout << "\nUAV received Goal\n";
			double n,e,d,t;
				
				int nVars = 0;

				if (goal->hasAttribute("n")){
					//std::cout << "Goal N received: " << goal->getAttribute("n").domain().getTypedSingleton<float, true>() << "\n";
					n = goal->getAttribute("n").domain().getTypedSingleton<float, true>();
					nVars++;
				}
				if(goal->hasAttribute("e")){
					//std::cout << "Goal E received: " << goal->getAttribute("e").domain().getTypedSingleton<float, true>() << "\n";
					e = goal->getAttribute("e").domain().getTypedSingleton<float, true>();
					nVars++;
				}
				if(goal->hasAttribute("d")){
					//std::cout << "Goal D received: " << goal->getAttribute("d").domain().getTypedSingleton<float, true>() << "\n";
					d = goal->getAttribute("d").domain().getTypedSingleton<float, true>();
					nVars++;
				}
				if(goal->hasAttribute("t")){
					//std::cout << "Goal t received: " << goal->getAttribute("t").domain().getTypedSingleton<float, true>() << "\n";
					t = goal->getAttribute("t").domain().getTypedSingleton<float, true>();
					nVars++;
				}
				else
					std::cout << "Observation is errornous\n";

				if (nVars == 4){

					if(lastTickReceivedPlan < currentTick){

						lastTickReceivedPlan = currentTick;
						mpc->resetWaypoints();//keepOnlyWaypointsFromTo(0, ((currentTick-1) * stepLength ));
						nwpTick = 0;
					}

					mpc->addWaypoint(n, e, d, t);
					std::cout << "\n Sjekk n:" << n << " e: " << e << " d: " << d << " t: " << t << "\n";
					nwpTick++;


					std::cout << "N waypoints received this tick: " << nwpTick;


				}
				else{
					std::cout << "\n------------------BUG------IR-HRq---------" << nVars << "\n";
				}

			}


	}

	void IRRefinedPath::dispatchGoals(){




	}

	void IRRefinedPath::dispatchObservations(){

		TREX::transaction::Observation obs = TREX::transaction::Observation(UAVTimeline, "At");
		obs.restrictAttribute("n", TREX::transaction::FloatDomain(mpc->getCurrentN(), mpc->getCurrentN()));
		obs.restrictAttribute("e", TREX::transaction::FloatDomain(mpc->getCurrentE(), mpc->getCurrentE()));
		obs.restrictAttribute("d", TREX::transaction::FloatDomain(mpc->getCurrentD(), mpc->getCurrentD()));

		postObservation(obs);
		
	}





}