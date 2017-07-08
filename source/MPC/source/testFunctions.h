#pragma once
#include "ALMPC.h"
#include "SimpleMPC.h"
#include "LMModelLinear.h"

void testSimulate(){

	LMMPC * mpc = new LMMPC();
	mpc->setup(10.0, 0.25);
	mpc->addWaypoint(0.0, 0.0, 0.0, 0.0);
	mpc->addWaypoint(18.0*15, 0.0, 0.0, 15.0);
	mpc->addWaypoint(18.0*15, 15 * 18.0, 0.0, 30.0);
	mpc->addWaypoint(18.0*15, 45 * 18.0, 0.0, 60.0);
	//mpc->addWaypoint(165.0, 30.0, 0.0, 9.0);
	std::cout << "Waypoints inserted\n";

	ACADO::DVector s0(17);
	s0.setAll(0.0);
	s0(5) = 0.0;
	s0(9) = 0;
	s0(12) = 0;
	std::cout << "starting simulation..1\n";
	mpc->simulate(20, s0);
}

void testSimpleSimulate(){

	SimpleModel model;
	model.createModel();

	LMMPC * mpc1 = new SimpleMPC(&model);
	LMMPC * mpc = new SimpleMPC(&model);
	mpc1->setup(10.0, 0.25);
	mpc1->addWaypoint(0.0, 0.0, 0.0, 0.0);
	mpc1->addWaypoint(18.0*10, 0.0, 0.0, 10.0);
	mpc1->addWaypoint(18.0*10, 10*18.0, 0.0, 20.0);
	mpc1->addWaypoint(10*18.0, 30*18.0, 0.0, 40.0);
	mpc->setup(10.0, 0.25);
	mpc->addWaypoint(0.0, 0.0, 0.0, 0.0);
	mpc->addWaypoint(18.0*10, 0.0, 0.0, 10.0);
	mpc->addWaypoint(18.0*10, 10*18.0, 0.0, 20.0);
	mpc->addWaypoint(10*18.0, 30*18.0, 0.0, 40.0);
	std::cout << "Waypoints inserted\n";

	ACADO::DVector s0(10);
	s0.setAll(0.0);
	s0(3) = 18.0;
	//s0(6) = 1.6;

	std::cout << "starting simulation..1\n";
	mpc->simulate(30, s0);
	//mpc1->simulate(10, s0);
}


void testFlat(){

	double horizon = 10;
	double stepLength = 0.25;
	LMModelLinear * model = new LMModelLinear;


	std::cout << "MPC constructor\n";
	model->createModel();
	std::cout << model->getModel()->getDim() << std::endl;
	model->printAllParameters();
	std::cout << "Model made\n";

	ACADO::OCP * ocp = new ACADO::OCP(0.0, horizon, horizon / stepLength);
	std::cout << "ocp made\n";

	ACADO::Function * referenceFunction = new ACADO::Function();

	(*referenceFunction) << model->N;
	(*referenceFunction) << model->E;
	(*referenceFunction) << model->D;
	(*referenceFunction) << model->u;
	(*referenceFunction) << model->v;
	(*referenceFunction) << model->delta_tDot;
	(*referenceFunction) << model->delta_aDot;


	int referenceFunctionDimention = 7;

	ACADO::DMatrix * coefficientMatrix = new ACADO::DMatrix(referenceFunctionDimention, referenceFunctionDimention);
	coefficientMatrix->setAll(0);
	(*coefficientMatrix)(0,0) = 1.0;
	(*coefficientMatrix)(1,1) = 1.0;
	(*coefficientMatrix)(2,2) = 1.0;
	(*coefficientMatrix)(3,3) = 1.0;
	(*coefficientMatrix)(4,4) = 1.0;
	(*coefficientMatrix)(5,5) = 1.0;
	(*coefficientMatrix)(6,6) = 1.0;


	std::cout << "ref func setup\n";


	std::cout << "ocp and ref func setup11\n";
	ACADO::DVector * tmpVec = new ACADO::DVector(referenceFunctionDimention);
	tmpVec->setAll(0.0);
	ocp->minimizeLSQ( *coefficientMatrix, *referenceFunction, *tmpVec);

	delete tmpVec;

	std::cout << "ocp and ref func setup\n";
	ocp->subjectTo(*(model->getModel()));
	//ocp->subjectTo(-0.3 <= model->theta <= 0.3);
	//ocp->subjectTo(-0.3 <= model->phi <= 0.3);
	//ocp->subjectTo(-0.3 <= model->psi <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_t <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_e <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_r <= 0.3);
	ocp->subjectTo(-0.3 <= model->delta_a <= 0.3);

	std::cout << "ocp and ref func setup\n";
	std::cout << "ocp and ref func setup\n";

	ACADO::RealTimeAlgorithm * alg = new ACADO::RealTimeAlgorithm(*ocp, stepLength);
	alg->set( ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45           );
	alg->set( ACADO::INTEGRATOR_TOLERANCE, 1e-6           );
    alg->set( ACADO::MAX_NUM_ITERATIONS, 20  );
    //alg.set( ACADO::MAX_NUM_INTEGRATOR_STEPS, 10000  );

	std::cout << "alg setup\n";

	ACADO::VariablesGrid waypoints;
	ACADO::DVector * vector = new ACADO::DVector(7);
	ACADO::DVector  vector1(7), vector2(7);
	vector->setAll(0);
	vector1.setAll(0);
	vector2.setAll(0);


	(*vector)(0) = 60.0;
	(*vector)(1) = 0.0;
	(*vector)(2) = -150.0;
	waypoints.addVector(*vector, 0.0);
	delete vector;

	vector1(0) = 100.0;
	vector1(1) = 20.0;
	vector1(2) = -150.0;
	waypoints.addVector(vector1, 3.0);

	vector2(0) = 150.0;
	vector2(1) = 30.0;
	vector2(2) = -150.0;
	waypoints.addVector(vector2, 6.0);


	ACADO::StaticReferenceTrajectory * referenceTrajectory = new ACADO::StaticReferenceTrajectory(waypoints);
	waypoints.print();
	std::cout << "rt setup\n";
	ACADO::Controller * controller = new ACADO::Controller(*alg, *referenceTrajectory);
	std::cout << "controller setup\n";
	ACADO::OutputFcn * simulatorOutput = new ACADO::OutputFcn ;
	ACADO::DynamicSystem dynSys(*(model->getModel()), *simulatorOutput);
	ACADO::Process * process = new ACADO::Process(dynSys, ACADO::INT_RK45);

	process->set( ACADO::INTEGRATOR_TOLERANCE, 1e-6           );
    //process.set            ( ACADO::MAX_NUM_ITERATIONS, 100  );
	ACADO::SimulationEnvironment simulator = ACADO::SimulationEnvironment(0.0, 5.0, *process, *controller);

	ACADO::DVector s0(17);
	s0.setAll(0.0);
	s0(5) = 18;
	s0(9) = -150;
	s0(12) = -150;


	if (simulator.init( s0 ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (simulator.run( ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );


}