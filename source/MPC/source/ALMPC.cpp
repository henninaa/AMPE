#include "ALMPC.h"



LMMPC::LMMPC() {

}

LMMPC::~LMMPC(){

}

void LMMPC::setModel(LMModel * model){

	if (this->model != nullptr)
		delete this->model;
	this->model  = model;
}

void LMMPC::setup(double horizon, double stepLength){

	model = new LMModelLinear();

	ocp = new ACADO::OCP(0.0, horizon, horizon / stepLength);

	setupReferenceFunction();
	setupOCP();

	alg = new ACADO::RealTimeAlgorithm(*ocp, stepLength);

	controller = new ACADO::Controller(*alg, *referenceTrajectory);
}

void LMMPC::step(ACADO::DVector currentY, double currentTime){

	controller->step(currentTime, currentY);
	completeStep();	

}

void LMMPC::step(ACADO::DVector currentY, ACADO::VariablesGrid referenceTrajectory, double currentTime){

	controller->step(currentTime, currentY, referenceTrajectory);
	completeStep();

}

void LMMPC::simulate(double duration){

	ACADO::DynamicSystem dynSys(*(model->getModel()), simulatorOutput);
	ACADO::Process process(dynSys, ACADO::INT_RK45);
	simulator = new ACADO::SimulationEnvironment(0.0, duration, process, *controller);

}

	void LMMPC::setWaypoints(std::vector<Waypoint> waypoints){

		resetWaypoints();

		for(auto it = waypoints.begin(); it != waypoints.end(); it++)
			addWaypoint(*it);

	}
	void LMMPC::addWaypoint(Waypoint waypoint){

		ACADO::DVector vector(3);
		vector(0) = waypoint.x;
		vector(1) = waypoint.y;
		vector(2) = waypoint.z;
		this->waypoints.addVector(vector, waypoint.time);

	}
	void LMMPC::addWaypoint(double x, double y, double z, double time){
		addWaypoint(Waypoint(x,y,z,time));
	}
	void LMMPC::deleteWaypoint(double time){

	}
	void LMMPC::resetWaypoints(){

		waypoints = ACADO::VariablesGrid();

	} // deletes wp list 


//-----------------------------------Private functions

void LMMPC::setupReferenceFunction(){

	referenceFunction = new ACADO::Function();

	(*referenceFunction) << model->N;
	(*referenceFunction) << model->E;
	(*referenceFunction) << model->D;
	(*referenceFunction) << model->u;
	(*referenceFunction) << model->v;
	(*referenceFunction) << model->delta_t;
	(*referenceFunction) << model->delta_a;


	coefficientMatrix = new ACADO::DMatrix(5, 5);
	coefficientMatrix->setAll(0);
	(*coefficientMatrix)(0,0) = 1.0;
	(*coefficientMatrix)(1,1) = 1.0;
	(*coefficientMatrix)(2,2) = 1.0;
	(*coefficientMatrix)(3,3) = 0.0;
	(*coefficientMatrix)(4,4) = 0.0;
	(*coefficientMatrix)(5,5) = 1.0;
	(*coefficientMatrix)(6,6) = 1.0;

	referenceFunctionDimention = 7;
}


void LMMPC::setupOCP(){

	ocp->minimizeLSQ( *coefficientMatrix, *referenceFunction, ACADO::DVector(referenceFunctionDimention, 0));

	ocp->subjectTo(*(model->getModel()));

}

void LMMPC::getDubinsPath(std::vector<ACADO::DVector> wps){

	

}

void LMMPC::completeStep(){



}