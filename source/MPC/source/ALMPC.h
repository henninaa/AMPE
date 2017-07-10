#pragma once
#include <string>
#include <vector>
#include <iostream>

#include "LMModelLinear.h"
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


class LMMPC
{
public:
	LMMPC();
	LMMPC(LMModel * model);
	~LMMPC();
	class Waypoint;

	void setup(double horizon = 20, double stepLength = 0.25, double initialX = 0, double initialY = 0, double initialZ = 50);
	void initializeController(ACADO::DVector initialState = ACADO::DVector());
	void setModel(LMModel * model);
	ACADO::DVector step(double currnetTime);//ACADO::DVector currentY, double currentTime);
	void step(ACADO::DVector currentY, ACADO::VariablesGrid referenceTrajectory, double currentTime);
	void simulate(double duration);
	void simulate(double duration, ACADO::DVector intitalState);

	void setWaypoints(std::vector<Waypoint> waypoints);
	void addWaypoint(Waypoint waypoint);
	void addWaypoint(double x, double y, double z, double time);
	void deleteWaypoint(double time);
	void resetWaypoints(); // deletes wp list
	void keepOnlyWaypointsFromTo(double from, double to);

	void setObservation(int stateNumber, double value);

	double getCurrentState(int state);
	double getCurrentControl(int control);

	double getCurrentN() { return ySim.getLastVector()(10); }
	double getCurrentE() { return ySim.getLastVector()(11); }
	double getCurrentD() { return ySim.getLastVector()(12); }

	void plot();

protected:

	double horizon;
	double stepLength;
	double currentTime;
	int nSteps;
	ACADO::DVector u; 

	LMModel * model;
	ACADO::DMatrix * coefficientMatrix;
	
	ACADO::Function * referenceFunction;
	ACADO::OCP * ocp;
	ACADO::RealTimeAlgorithm * alg;
	ACADO::StaticReferenceTrajectory * referenceTrajectory;
	int referenceFunctionDimention;
	ACADO::Controller * controller;

	ACADO::DynamicSystem *dynSys;
	ACADO::Process *process;

	std::vector<Waypoint> waypoints;
	ACADO::VariablesGrid referencePath;

	ACADO::SimulationEnvironment * simulator;
	ACADO::OutputFcn * simulatorOutput;

	virtual void setupReferenceFunction();
	virtual void setupOCP(double horizon, double sttepLenght);
	virtual void setupModel();
	virtual void plotSimulation();
	virtual void createReferenceTrajectory(double t0 = 0.0);
	void getDubinsPath(std::vector<ACADO::DVector> wps);
	void completeStep();

	//const double maxWaypoints = 20;

	ACADO::DVector uCon;
	
	ACADO::VariablesGrid ySim;
	ACADO::VariablesGrid pureSim;
	ACADO::VariablesGrid allUs;

	//----Plot variables and functions

	ACADO::GnuplotWindow windowControls;
	ACADO::GnuplotWindow windowLinStates;
	ACADO::GnuplotWindow windowAngStates;
	bool hasPlotWindow;
	bool havePloted;
	void createPlot();
	void updatePlot();

};

class LMMPC::Waypoint
{
public:
	Waypoint(double x, double y, double z, double time = -1) : x(x), y(y), z(z), time(time) {}
	~Waypoint(){};
	

	double x, y, z, time;

};

/*
ostream & operator <<(std::vector<Waypoints> &rhs){
	ostream result;
	return result;
}
*/