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
		addUAVTimelinePair(0.0, 0.0, 0.0, UAVTimeline_1);
	
		if(nUAVs >= 2){
			use(UAVTimeline_2);
			addUAVTimelinePair(10.0, 0.0, 0.0, UAVTimeline_2);
		}

		if(nUAVs >= 3){
			use(UAVTimeline_3);
			addUAVTimelinePair(0.0, 10.0, 0.0, UAVTimeline_3);

		}
		
		nodeIdCounter++;
		module.addNode(100.0, 0.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(100.0, 1000.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(100.0, 50.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(100.0, 150.0, 0.0, nodeIdCounter);
		nodeIdCounter++;
		module.addNode(150.0, 200.0, 0.0, nodeIdCounter);

		std::cout << "Crude path reactor created using " << nUAVs << " UAVS\n";

	}

	CRCrudePath::~CRCrudePath(){
		std::cout << "Crude path reactor closed\n";

	}

	bool CRCrudePath::synchronize(){

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


	}

	void CRCrudePath::handleRequest(TREX::transaction::goal_id const & goal){



	}

	void CRCrudePath::addUAVTimelinePair(double n0, double e0, double d0, TREX::utils::Symbol timeline){
		
		UAVIdCounter++;
		module.addUAV(n0, e0, d0, UAVIdCounter);
		uavTimelinePairs.push_back(UAVTimelinePair(UAVIdCounter, timeline));

	}


}