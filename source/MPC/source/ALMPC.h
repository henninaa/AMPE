#pragma once
#include "LMModelLinear.h"
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>

class LMMPC
{
public:
	LMMPC();
	~LMMPC();
	class Waypoint;

	void setup(double horizon = 20, double stepLength = 0.25);
	void initializeController(ACADO::DVector initialState);
	void setModel(LMModel * model);
	void step(ACADO::DVector currentY, double currentTime);
	void step(ACADO::DVector currentY, ACADO::VariablesGrid referenceTrajectory, double currentTime);
	void simulate(double duration);

	void setWaypoints(std::vector<Waypoint> waypoints);
	void addWaypoint(Waypoint waypoint);
	void addWaypoint(double x, double y, double z, double time);
	void deleteWaypoint(double time);
	void resetWaypoints(); // deletes wp list

private:

	double horizon;
	double stepLength;
	double currentTime;

	LMModel * model;
	ACADO::DMatrix * coefficientMatrix;
	ACADO::DifferentialState X;
	ACADO::Function * referenceFunction;
	ACADO::OCP * ocp;
	ACADO::RealTimeAlgorithm * alg;
	ACADO::StaticReferenceTrajectory * referenceTrajectory;
	int referenceFunctionDimention;
	ACADO::Controller * controller;

	ACADO::VariablesGrid waypoints;

	ACADO::SimulationEnvironment * simulator;
	ACADO::OutputFcn simulatorOutput;

	void setupReferenceFunction();
	void setupOCP();
	void getDubinsPath(std::vector<ACADO::DVector> wps);
	void completeStep();

	//const double maxWaypoints = 20;
};

class LMMPC::Waypoint
{
public:
	Waypoint(double x, double y, double z, double time = -1) : x(x), y(y), z(z), time(time) {}
	~Waypoint(){};
	

	double x, y, z, time;

};