#include "SimpleMPC.h"



void SimpleMPC::setupReferenceFunction(){

	std::cout << "\n\nkorekt func!!!!!!!!!\n\n";

	referenceFunction = new ACADO::Function();

	(*referenceFunction) << model->N;
	(*referenceFunction) << model->E;
	(*referenceFunction) << model->D;
	(*referenceFunction) << model->delta_eDot;


	referenceFunctionDimention = 4;

	coefficientMatrix = new ACADO::DMatrix(referenceFunctionDimention, referenceFunctionDimention);
	coefficientMatrix->setAll(0);

	for (int i = 0; i < referenceFunctionDimention; i++)
		(*coefficientMatrix)(i,i) = 1.0f;

	(*coefficientMatrix)(3,3) = 10000.0f;

}

void SimpleMPC::setupOCP(double horizon, double stepLength){

	ocp = new ACADO::OCP(0.0, horizon, horizon / stepLength);

	ACADO::DVector tmpVec(referenceFunctionDimention);
	tmpVec.setAll(0.0);
	ocp->minimizeLSQ( *coefficientMatrix, *referenceFunction, tmpVec);

	ocp->subjectTo(*(model->getModel()));
	//ocp->subjectTo(-0.3 <= model->theta <= 0.3);
	//ocp->subjectTo(-0.3 <= model->phi <= 0.3);
	//ocp->subjectTo(-0.3 <= model->psi <= 0.3);
	//ocp->subjectTo(-10 <= model->N + model->delta_eDot <= 10.0);
	ocp->subjectTo(-1 <= model->delta_tDot <= 1);
	ocp->subjectTo(-0.3 <= model->delta_rDot <= 0.3);
}

void SimpleMPC::setupModel(){

	model = new SimpleModel();
	model->createModel();
	model->printAllParameters();

}

void SimpleMPC::plotSimulation(){
	

	ACADO::VariablesGrid diffStates;
	if (simulator->getProcessDifferentialStates( diffStates ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	ACADO::VariablesGrid feedbackControl;
	if (simulator->getFeedbackControl( feedbackControl ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	ACADO::GnuplotWindow window;
		window.addSubplot( diffStates(0),   "n" );
		window.addSubplot( diffStates(1),   "e" );
		window.addSubplot( diffStates(2),   "d" );
		window.addSubplot( diffStates(3),   "u" );
		window.addSubplot( diffStates(4),   "v" );
		window.addSubplot( diffStates(5),   "w" );
		window.plot();
	ACADO::GnuplotWindow window2;
		window2.addSubplot( diffStates(7),   "phi" );
		window2.addSubplot( diffStates(6),   "theta" );
		window2.addSubplot( diffStates(8),   "psi" );
		window2.plot();
	ACADO::GnuplotWindow window1;
		window1.addSubplot( feedbackControl(2),   "delta_r" );
		window1.addSubplot( feedbackControl(3),   "delta_t" );
		window1.plot();
	//diffStates.print();
}

/*

void SimpleMPC::plotSimulation(){
	

	ACADO::VariablesGrid diffStates;
	if (simulator->getProcessDifferentialStates( diffStates ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	ACADO::VariablesGrid feedbackControl;
	if (simulator->getFeedbackControl( feedbackControl ) != ACADO::SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	ACADO::GnuplotWindow window;
		window.addSubplot( diffStates(0),   "n" );
		window.addSubplot( diffStates(1),   "e" );
		window.addSubplot( diffStates(2),   "d" );
		window.addSubplot( diffStates(3),   "u" );
		window.addSubplot( diffStates(4),   "v" );
		window.addSubplot( diffStates(5),   "w" );
		window.plot();
	ACADO::GnuplotWindow window2;
		window2.addSubplot( diffStates(6),   "theta" );
		window2.addSubplot( diffStates(7),   "phi" );
		window2.addSubplot( diffStates(8),   "psi" );
		window2.plot();
	ACADO::GnuplotWindow window1;
		window1.addSubplot( feedbackControl(2),   "delta_r" );
		window1.addSubplot( feedbackControl(3),   "delta_t" );
		window1.plot();
	diffStates.print();

}*/