#pragma once
#include "ALMPC.h"
#include "SimpleModel.h"
#include "iostream"


class SimpleMPC
	: public LMMPC
{
public:
	SimpleMPC() : LMMPC() {}
	SimpleMPC(LMModel * model) : LMMPC(model) {}
	~SimpleMPC() {}

private:

	void setupReferenceFunction();
	void setupOCP(double horizon, double stepLength);
	void setupModel();
	void plotSimulation();
	void createReferenceTrajectory();

	ACADO::Control epsilon;

};