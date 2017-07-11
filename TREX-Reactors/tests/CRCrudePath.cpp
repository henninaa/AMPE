#include "CRCrudePath.h"


namespace
{

  /** @brief TREX log entry point */
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

  /** @brief Platform reactor declaration */
  TREX::transaction::TeleoReactor::xml_factory::declare<TREX::CRCrudePath> decl("CrudePath");

}


namespace TREX {

	void initPlugin() {
    	::s_log->syslog("plugin.CrudePath")<<"CrudePath loaded."<<std::endl;
    // ::decl;
    }

	TREX::utils::Symbol const CRCrudePath::UAVTimeline_1("navigator_1");
	TREX::utils::Symbol const CRCrudePath::UAVTimeline_2("navigator_2");
	TREX::utils::Symbol const CRCrudePath::UAVTimeline_3("navigator_3");
	TREX::utils::Symbol const CRCrudePath::gatherDataTimeline("collectData");
	TREX::utils::Symbol const CRCrudePath::AtPred("At");
	TREX::utils::Symbol const CRCrudePath::GoingPred("Going");

	CRCrudePath::CRCrudePath(TREX::transaction::TeleoReactor::xml_arg_type arg) : TeleoReactor(arg), module(), nodeIdCounter(0), UAVIdCounter(0)
	{
		int nUAVs = TREX::utils::parse_attr<int>(1, TeleoReactor::xml_factory::node(arg), "numberOfUAVs");
		
		use(UAVTimeline_1);
		addUAVTimelinePair(-100.0, 0.0, 0.0, UAVTimeline_1);
	
		if(nUAVs >= 2){
			use(UAVTimeline_2);
			addUAVTimelinePair(100.0, 0.0, 0.0, UAVTimeline_2);
		}

		if(nUAVs >= 3){
			use(UAVTimeline_3);
			addUAVTimelinePair(0.0, 100.0, 0.0, UAVTimeline_3);

		}
		std::cout << "Crude path reactor created using " << nUAVs << " UAVS\n";

		planReady = false;
		hasWorkThisTick = false;
		currentTick = -2;
		/*
		nodeIdCounter++;
		module.addNode(100.0, 0.0, 0.0, nodeIdCounter);
		//module.addUAV(0.0, 0.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(100.0, 400.0, 00.0, nodeIdCounter);
		//module.addUAV(10.0, 0.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(400.0, 50.0, 00.0, nodeIdCounter);
		//module.addUAV(0.0, 10.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(100.0, 150.0, 0.0, nodeIdCounter);
		//nodeIdCounter++;
		//module.addNode(150.0, 200.0, 100.0, nodeIdCounter);
	*/
		nodeIdCounter++;
		module.addNode(600.0, 500.0, 0.0, nodeIdCounter);
		//module.addUAV(0.0, 0.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(200.0, 800.0, 00.0, nodeIdCounter);
		//module.addUAV(10.0, 0.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(00.0, 1100.0, 00.0, nodeIdCounter);
		//module.addUAV(0.0, 10.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(-200.0, 1100.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(-400.0, 500.0, 0.0, nodeIdCounter);
		//nodeIdCounter++;
		//module.addNode(150.0, 200.0, 100.0, nodeIdCounter);
		stepLength = 0.25;
		sampleTime = 5;//2.0;
		module.setSampleTime(sampleTime);
		planStartedAt = -3;


	}

	CRCrudePath::~CRCrudePath(){
		std::cout << "Crude path reactor closed\n";

	}

	void CRCrudePath::handleInit(){

		std::cout << "\nRunning init plan\n";
		if(moduleSwitch){
			planStartedAt = 0;
			module.runSync();
			module.updatePath();
			dispatchPlan();
		}

		std::cout << "\nPlan dispatched\n";


	}

	bool CRCrudePath::synchronize(){

		currentTick++;

		if(!hasWorkThisTick){
			deliberationNeedsStart = true;
			hasWorkThisTick = true;
		}

		if(planReady){
			planReady = false;
			//printPlan();
			dispatchPlan();
		}

		return true;
	}

	void CRCrudePath::notify(TREX::transaction::Observation const & obs){

		if (obs.hasAttribute("X")){
			std::cout << "Observation received: " << obs.getAttribute("X").domain().getTypedSingleton<float, true>() << "\n";
			if(obs.getAttribute("X").domain().getTypedSingleton<float, true>() >= 100){
				std::cout << "complete!\n";
			}
		}
		else
			std::cout << "random notify\n";

		double n,e,d;

		for(auto tl = uavTimelinePairs.begin(); tl != uavTimelinePairs.end(); tl++){
			
			int nVars = 0;

			if(obs.object() == tl->timeline){
				
				nVars = 0;


				if (obs.hasAttribute("n")){
					std::cout << "Observation N received: " << obs.getAttribute("n").domain().getTypedSingleton<float, true>() << "\n";
					n = (obs.getAttribute("n").domain().getTypedSingleton<float, true>());
					nVars++;
				}
				if(obs.hasAttribute("e")){
					std::cout << "Observation E received: " << obs.getAttribute("e").domain().getTypedSingleton<float, true>() << "\n";
					e = (obs.getAttribute("e").domain().getTypedSingleton<float, true>());

					nVars++;
				}
				if(obs.hasAttribute("d")){
					std::cout << "Observation D received: " << obs.getAttribute("d").domain().getTypedSingleton<float, true>() << "\n";
					d = (obs.getAttribute("d").domain().getTypedSingleton<float, true>() );
					
					nVars++;
				}
				else
					std::cout << "Observation is errornous\n";

				if(nVars == 3){
				std::cout << "Received observation on timeline " << tl->id << "\n";
				module.moveUAVTo(n,e,d, tl->id);

				}


			}




		}


	}

	bool CRCrudePath::hasWork(){
		return hasWorkThisTick && moduleSwitch;
	}

	void CRCrudePath::resume(){
		bool resetPt = false;

		if(deliberationNeedsStart){

			module.run();
			deliberationNeedsStart = false;
			resetPt = true;
		}

		if(module.isDone()){
			hasWorkThisTick = false;
			planReady = true;
			module.updatePath();
			std::cout << "\nAMPL complete\n";
		}

		if(resetPt){		
			planStartedAt = currentTick;
		}

		

	}

	void CRCrudePath::handleRequest(TREX::transaction::goal_id const & goal){



	}


	void CRCrudePath::addUAVTimelinePair(double n0, double e0, double d0, TREX::utils::Symbol timeline){
		
		UAVIdCounter++;
		module.addUAV(n0, e0, d0, UAVIdCounter);
		uavTimelinePairs.push_back(UAVTimelinePair(UAVIdCounter, timeline));

	}

	void CRCrudePath::dispatchPlan(){

		std::cout << "\nDispatched plan\n";
		TREX::transaction::Goal * goal;
		int pathSize = module.getPathSize();
		int timeOffset = 0;

		for(int i = 0; i < uavTimelinePairs.size(); i++){
			for(int j = 0; j < pathSize; j++){

				timeOffset = ((planStartedAt) * stepLength);// + (j* module.getSampleTime()))));
				std::cout << "\n----------------------TIME OFFSET:\n" << timeOffset << "\n";

				goal = new TREX::transaction::Goal(uavTimelinePairs[i].timeline, "At");
				goal->restrictAttribute(TREX::transaction::Variable("n", TREX::transaction::FloatDomain(module.getWaypoint(i, j).n)));
				goal->restrictAttribute(TREX::transaction::Variable("e", TREX::transaction::FloatDomain(module.getWaypoint(i, j).e)));
				goal->restrictAttribute(TREX::transaction::Variable("d", TREX::transaction::FloatDomain(module.getWaypoint(i, j).d)));
				goal->restrictAttribute(TREX::transaction::Variable("t", TREX::transaction::FloatDomain( timeOffset + (j* module.getSampleTime()))));//((planStartedAt) * stepLength) + (j* module.getSampleTime()))));

				postGoal(*goal);
				delete goal;

				if(j > 20)
					break;
			}
		}

	}

	void CRCrudePath::printPlan(){
		module.updatePath();
		int pathSize = module.getPathSize();
		
		for(int i = 0; i < uavTimelinePairs.size(); i++){


			std::cout << "\n------------------UAV nr " << i << "\n";

			for(int j = 0; j < pathSize; j++){
				
				std::cout << module.getWaypoint(i, j).n << "   " << module.getWaypoint(i, j).e << "   " << module.getWaypoint(i, j).d << "\n";
			}


		}

	}


}