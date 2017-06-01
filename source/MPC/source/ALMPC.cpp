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

void LMMPC::setup(double horizon, double stepLength, double initialX, double initialY, double initialZ){

	std::cout << "MPC constructor\n";
	model = new LMModelLinear();
	model->createModel();
	std::cout << model->getModel()->getDim() << std::endl;
	model->printAllParameters();
	std::cout << "Model made\n";

	ocp = new ACADO::OCP(0.0, horizon, horizon / stepLength);
	std::cout << "ocp made\n";

	setupReferenceFunction();
	std::cout << "ref func setup\n";
	setupOCP();
	std::cout << "ocp and ref func setup\n";

	alg = new ACADO::RealTimeAlgorithm(*ocp, stepLength);
	alg->set( ACADO::INTEGRATOR_TOLERANCE, 1e-6           );
	std::cout << "alg setup\n";

	referenceTrajectory = new ACADO::StaticReferenceTrajectory(waypoints);
	waypoints.print();
	std::cout << "rt setup\n";
	controller = new ACADO::Controller(*alg, *referenceTrajectory);
	std::cout << "controller setup\n";
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


}

void LMMPC::simulate(double duration, ACADO::DVector initialState){

	ACADO::DynamicSystem dynSys(*(model->getModel()), simulatorOutput);
	ACADO::Process process(dynSys, ACADO::INT_RK45);
	simulator = new ACADO::SimulationEnvironment(0.0, duration, process, *controller);

	if (simulator->init( initialState ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (simulator->run( ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

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

	//*referenceTrajectory = ACADO::StaticReferenceTrajectory(this->waypoints);
}
void LMMPC::addWaypoint(double x, double y, double z, double time){
	addWaypoint(Waypoint(x,y,z,time));
}
void LMMPC::deleteWaypoint(double time){

}
void LMMPC::resetWaypoints(){

	waypoints = ACADO::VariablesGrid();
	//*referenceTrajectory = ACADO::StaticReferenceTrajectory();

} // deletes wp list 


//-----------------------------------Private functions

void LMMPC::setupReferenceFunction(){

	referenceFunction = new ACADO::Function();

	(*referenceFunction) << model->N;
	(*referenceFunction) << model->E;
	(*referenceFunction) << model->D;
	(*referenceFunction) << model->u;
	(*referenceFunction) << model->v;
	(*referenceFunction) << model->delta_tDot;
	(*referenceFunction) << model->delta_aDot;


	referenceFunctionDimention = 7;

	coefficientMatrix = new ACADO::DMatrix(referenceFunctionDimention, referenceFunctionDimention);
	coefficientMatrix->setAll(0);
	(*coefficientMatrix)(0,0) = 1.0;
	(*coefficientMatrix)(1,1) = 1.0;
	(*coefficientMatrix)(2,2) = 1.0;
	(*coefficientMatrix)(3,3) = 1.0;
	(*coefficientMatrix)(4,4) = 1.0;
	(*coefficientMatrix)(5,5) = 1.0;
	(*coefficientMatrix)(6,6) = 1.0;
}


void LMMPC::setupOCP(){

	std::cout << "ocp and ref func setup11\n";
	ACADO::DVector tmpVec(referenceFunctionDimention);
	tmpVec.setAll(0.0);
	ocp->minimizeLSQ( *coefficientMatrix, *referenceFunction, tmpVec);

	std::cout << "ocp and ref func setup\n";
	ocp->subjectTo(*(model->getModel()));
	//ocp->subjectTo(-0.3 <= model->theta <= 0.3);
	//ocp->subjectTo(-0.3 <= model->phi <= 0.3);
	//ocp->subjectTo(-0.3 <= model->psi <= 0.3);

	std::cout << "ocp and ref func setup\n";
}

void LMMPC::getDubinsPath(std::vector<ACADO::DVector> wps){

	

}

void LMMPC::completeStep(){



}