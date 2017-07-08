#include "ALMPC.h"



LMMPC::LMMPC() {

	referenceTrajectory = nullptr;
	model = nullptr;
}

LMMPC::LMMPC(LMModel * model) {

	this->model = model;
	referenceTrajectory = nullptr;

}

LMMPC::~LMMPC(){

	delete model;
	delete coefficientMatrix;
	delete referenceFunction;

}

void LMMPC::setModel(LMModel * model){

	if (this->model != nullptr)
		delete this->model;
	this->model  = model;
}

void LMMPC::setup(double horizon, double stepLength, double initialX, double initialY, double initialZ){

	this->horizon = horizon;
	this->stepLength = stepLength;

	if (model == nullptr)
		setupModel();

	setupReferenceFunction();
	
	setupOCP(horizon, stepLength);

	alg = new ACADO::RealTimeAlgorithm(*ocp, stepLength);
	//alg->set( ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45           );
	//alg->set( ACADO::INTEGRATOR_TOLERANCE, 1e-6           );
    //alg->set( ACADO::MAX_NUM_ITERATIONS, 2  );
    //alg.set( ACADO::MAX_NUM_INTEGRATOR_STEPS, 10000  );

	//referenceTrajectory = new ACADO::StaticReferenceTrajectory(waypoints);
	
	controller = new ACADO::Controller(*alg);
}

ACADO::DVector LMMPC::step(double currentTime){//ACADO::DVector currentY, double currentTime1){
//
	//controller->setReferenceTrajectory(*referenceTrajectory);
	

	//controller->step(currentTime, currentY);
	//completeStep();	
	ACADO::DVector s0(17);
	s0.setAll(0.0);
	s0(5) = 0.0;
	s0(9) = 0;
	s0(12) = 0;
	

	//double currentTime = 0;
	double startTime = 0;
	double samplingTime = 0.25;
	double endTime = 10;
	int nSteps = 0;

	createReferenceTrajectory();
	//controller->setReferenceTrajectory(*referenceTrajectory);
	
	//while ( currentTime <= endTime )
	//{
		printf( "\n*** WOHO Simulation Loop No. %d (starting at time %.3f) ***\n",nSteps,currentTime );

		
		if (controller->step( currentTime,ySim.getLastVector(), referencePath ) != ACADO::SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		
		controller->getU( uCon );
		//if (controller->preparationStep( ) != ACADO::SUCCESSFUL_RETURN)
		//	exit( EXIT_FAILURE );
		//
		if (process->step( currentTime,currentTime+samplingTime,uCon ) != ACADO::SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		process->getY( ySim );
		
		this->currentTime += samplingTime;
		++nSteps;
	//}
		pureSim.addVector(ySim.getLastVector(), currentTime);
		allUs.addVector(uCon, currentTime);
	return uCon;

}

double LMMPC::getCurrentState(int state){

	return ySim.getLastVector()(state);
}

double LMMPC::getCurrentControl(int control){

	return allUs.getLastVector()(control);
}


void LMMPC::step(ACADO::DVector currentY, ACADO::VariablesGrid referenceTrajectory, double currentTime){

	controller->step(currentTime, currentY, referenceTrajectory);

	if (controller->step( currentTime, currentY, referenceTrajectory ) != ACADO::SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		controller->getU( u );
	
	//if (process.step( currentTime,currentTime+samplingTime,uCon ) != SUCCESSFUL_RETURN)
	//	exit( EXIT_FAILURE );
	//process.getY( ySim );

	completeStep();

}
void LMMPC::simulate(double duration){

	ACADO::DVector s0(17);
	s0.setAll(0.0);
	s0(5) = 0.0;
	s0(9) = 0;
	s0(12) = 0;
	//std::cout << "starting simulation..1\n";
	simulate(duration, s0);

}

void LMMPC::simulate(double duration, ACADO::DVector initialState){
	
	createReferenceTrajectory();

	/*ACADO::VariablesGrid path;
	ACADO::DVector vector = ACADO::DVector(referenceFunctionDimention);
	vector.setAll(0.0);
	vector(0) = 18.0*30.0;
	path.addVector(vector, 0.0);
	referenceTrajectory = new ACADO::StaticReferenceTrajectory(path);
	referencePath = path;*/
	//referencePath.print();

	controller->setReferenceTrajectory(*referenceTrajectory);

	simulatorOutput = new ACADO::OutputFcn;

	dynSys = new ACADO::DynamicSystem(*(model->getModel()), *simulatorOutput);
	process = new ACADO::Process(*dynSys, ACADO::INT_RK45);
	simulator = new ACADO::SimulationEnvironment(0.0, duration, *process, *controller);

	if (simulator->init( initialState ) != ACADO::SUCCESSFUL_RETURN) int dummy = 1;
		//exit( EXIT_FAILURE );
	if (simulator->run( ) != ACADO::SUCCESSFUL_RETURN) int dummy = 1;
		//exit( EXIT_FAILURE );

	//plotSimulation();

}

void LMMPC::setWaypoints(std::vector<Waypoint> waypoints){

	resetWaypoints();

	for(auto it = waypoints.begin(); it != waypoints.end(); it++)
		addWaypoint(*it);

}

void LMMPC::addWaypoint(Waypoint waypoint){

	if (waypoints.size() == 0){
		waypoints.push_back(waypoint);
	}
	else if (waypoints[waypoints.size()-1].time < waypoint.time){
		waypoints.push_back((waypoint));
	}
	else{
		auto it = waypoints.begin();
		for (it; it != waypoints.end(); it++)
			if (it->time < waypoint.time)
				break;

		waypoints.insert(it, waypoint);
	}

	
}
void LMMPC::addWaypoint(double x, double y, double z, double time){
	addWaypoint(Waypoint(x,y,z,time));
}
void LMMPC::deleteWaypoint(double time){

}
void LMMPC::resetWaypoints(){

	waypoints = std::vector<Waypoint>();
	//*referenceTrajectory = ACADO::StaticReferenceTrajectory();

} // deletes wp list 

void LMMPC::keepOnlyWaypointsFromTo(double from, double to){

	std::vector<Waypoint> tmp;

	for (auto  it = waypoints.begin(); it != waypoints.end(); it++){

		if(it->time >= from && it->time <= to)
			tmp.push_back((*it));

	}

	waypoints = tmp;
}


//-----------------------------------Private functions

void LMMPC::setupReferenceFunction(){

	referenceFunction = new ACADO::Function();

	(*referenceFunction) << model->N;
	(*referenceFunction) << model->E;
	(*referenceFunction) << model->D;
	//(*referenceFunction) << model->psi;
	(*referenceFunction) << model->delta_t;
	(*referenceFunction) << model->delta_aDot;
	(*referenceFunction) << model->delta_eDot;
	(*referenceFunction) << model->delta_tDot;
	(*referenceFunction) << model->vHat;


	referenceFunctionDimention = 8;

	coefficientMatrix = new ACADO::DMatrix(referenceFunctionDimention, referenceFunctionDimention);
	coefficientMatrix->setAll(0);

	for (int i = 0; i < referenceFunctionDimention; i++)
		(*coefficientMatrix)(i,i) = 1.0f;

	(*coefficientMatrix)(2,2) = 10.0f;
	(*coefficientMatrix)(3,3) = 1.0f;
	(*coefficientMatrix)(4,4) = 01.10f;
	(*coefficientMatrix)(5,5) = 01.10f;
	(*coefficientMatrix)(6,6) = 01.10f;

}


void LMMPC::setupOCP(double horizon, double stepLength){

	ocp = new ACADO::OCP(0.0, horizon, horizon / stepLength);

	ACADO::DVector tmpVec(referenceFunctionDimention);
	tmpVec.setAll(0.0);
	ocp->minimizeLSQ( *coefficientMatrix, *referenceFunction, tmpVec);

	ocp->subjectTo(*(model->getModel()));
	//ocp->subjectTo(-0.3 <= model->theta <= 0.3);
	//ocp->subjectTo(-0.3 <= model->phi <= 0.3);
	//ocp->subjectTo(-0.3 <= model->psi <= 0.3);
	ocp->subjectTo(-1 <= model->phiHat <= 1);
	ocp->subjectTo(-1 <= model->thetaHat <= 1);
	ocp->subjectTo(-10 <= model->delta_tHat <= 10);
	ocp->subjectTo(-0.3 <= model->delta_eHat <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_rHat <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_aHat <= 0.3);
	ocp->subjectTo(-10 <= model->delta_tDot <= 10);
	ocp->subjectTo(-0.3 <= model->delta_eDot <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_rDot <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_aDot <= 0.3);
}

void LMMPC::setupModel(){
	std::cout << "lin model\n";
	model = new LMModelLinear();
	model->createModel();
	model->printAllParameters();

}

void LMMPC::plotSimulation(){
	

	ACADO::VariablesGrid diffStates;
	if (simulator->getProcessDifferentialStates( diffStates ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	ACADO::VariablesGrid feedbackControl;
	if (simulator->getFeedbackControl( feedbackControl ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	ACADO::GnuplotWindow window;
		window.addSubplot( diffStates(10),   "n" );
		window.addSubplot( diffStates(11),   "e" );
		window.addSubplot( diffStates(12),   "d" );
		window.addSubplot( diffStates(5),   "u" );
		window.addSubplot( diffStates(0),   "v" );
		window.addSubplot( diffStates(6),   "w" );
		window.addSubplot( diffStates(15),   "delta_e" );
		window.addSubplot( diffStates(13),   "delta_a" );
		window.addSubplot( diffStates(14),   "delta_r" );
		window.addSubplot( diffStates(16),   "delta_t" );
		window.plot();
	ACADO::GnuplotWindow window2;
		window2.addSubplot( diffStates(1),   "p" );
		window2.addSubplot( diffStates(7),   "q" );
		window2.addSubplot( diffStates(2),   "r" );
		window2.addSubplot( diffStates(3),   "phi" );
		window2.addSubplot( diffStates(8),   "theta" );
		window2.addSubplot( diffStates(4),   "psi" );
		window2.plot();
	ACADO::GnuplotWindow window1;
		window1.addSubplot( feedbackControl(0),   "delta_e" );
		window1.addSubplot( feedbackControl(1),   "delta_a" );
		window1.addSubplot( feedbackControl(2),   "delta_r" );
		window1.addSubplot( feedbackControl(3),   "delta_t" );
		window1.plot();
	diffStates.print();
}

void LMMPC::getDubinsPath(std::vector<ACADO::DVector> wps){

	

}

void LMMPC::completeStep(){



}


void LMMPC::createReferenceTrajectory(){

	ACADO::VariablesGrid path;
	std::cout << waypoints.size() <<"\n";

	int nWP = waypoints.size();

	for(int i = 0; i < nWP; i++)
			std::cout << (waypoints[i].x)<< " " << (waypoints[i].y) << " " << (waypoints[i].z)<< " t: " << waypoints[i].time << "\n";


	//std::cout << "creating trajectory\n";
	double ts = 0.0;
	double te = 0.0;
	double td = 0.0;
	double angle = 0.0;
	ACADO::DVector vector = ACADO::DVector(referenceFunctionDimention);

	for (int i = 0; i < nWP-1; i++){
		ts = waypoints[i].time;
		te = waypoints[i+1].time;
		td = te - ts;
		//std::cout << "td: " << td << "\n";
		angle = atan2(waypoints[i+1].y - waypoints[i].y, waypoints[i+1].x - waypoints[i].x);
		for (double t = 0.0; t < td; t += stepLength/2){
			
			vector.setAll(0);
			vector(0) = (waypoints[i].x * ((td - t) / td) ) + (waypoints[i+1].x * (t / td));
			vector(1) = (waypoints[i].y * ((td - t) / td) ) + (waypoints[i+1].y * (t / td));
			vector(2) = (waypoints[i].z * ((td - t) / td) ) + (waypoints[i+1].z * (t / td));
			//vector(3) = angle;
			//TODO: fix angle to match revolution (n*2pi) of yaw
		
			path.addVector(vector, t + ts);
			std::cout << (waypoints[i].x * ((td - t) / td) ) + (waypoints[i+1].x * (t / td))<< " " << (waypoints[i].y * ((td - t) / td) ) + (waypoints[i+1].y * (t / td)) << " " << (waypoints[i].z * ((td - t) / td) ) + (waypoints[i+1].z * (t / td))<< " " << t + ts <<"\n";
		}
	}
	this->referencePath = path;
	//path.print();

	if (referenceTrajectory != nullptr)
		delete referenceTrajectory;

	referenceTrajectory = new ACADO::StaticReferenceTrajectory(path);

}

void LMMPC::initializeController(ACADO::DVector initialState){
	std::cout << "\nHALP\n" << std::flush;

	simulatorOutput = new ACADO::OutputFcn;
	std::cout << "\nHALP\n" << std::flush;
	//controller->step(currentTime, currentY);
	//completeStep();	
	ACADO::DVector s0(17);
	s0.setAll(0.0);
	s0(5) = 0.0;
	s0(9) = 0;
	s0(12) = 0;
	dynSys = new ACADO::DynamicSystem(*(model->getModel()), *simulatorOutput);
		std::cout << "\nDBG\n" << std::flush;
	process = new ACADO::Process(*dynSys, ACADO::INT_RK45);
	std::cout << "\nHALP\n" << std::flush;

	double currentTime = 0;
	double startTime = 0;
	double samplingTime = 0.25;
	double endTime = 10;
	int nSteps = 0;

	
	if (controller->init( 0,s0 ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	controller->getU( uCon );
		printf( "\n*** Simulation Loop No. %d (starting at time %.3f) ***\n",nSteps,currentTime );
	
	if (process->init( startTime,s0,uCon ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	process->getY( ySim );
		printf( "\n*** Simulation Loop No. %d (starting at time %.3f) ***\n",nSteps,currentTime );


		pureSim = ySim;
}

void LMMPC::setObservation(int varNumber, double value){

	ySim(ySim.getDim()-1, varNumber) = value;

}