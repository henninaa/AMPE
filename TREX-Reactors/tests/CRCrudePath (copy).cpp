#include "CRCrudePath.h"

namespace TREX {

	TREX::utils::Symbol const CRCrudePath::UAVTimeline_1("navigator_1");
	TREX::utils::Symbol const CRCrudePath::UAVTimeline_2("navigator_2");
	TREX::utils::Symbol const CRCrudePath::UAVTimeline_3("navigator_3");
	TREX::utils::Symbol const CRCrudePath::gatherDataTimeline("collectData");

	CRCrudePath::CRCrudePath(TREX::transaction::TeleoReactor::xml_arg_type arg) : TeleoReactor(arg)
	{

		use(UAVTimeline_1);
		use(UAVTimeline_2);
		use(UAVTimeline_3);

	}

	bool CRCrudePath::synchronize(){

	}

	void CRCrudePath::notify(TREX::transaction::Observation const & obs){




	}

	void CRCrudePath::handleRequest(TREX::transaction::goal_id const & goal){

	}





}