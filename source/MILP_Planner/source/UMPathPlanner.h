#pragma once
#include <string>
#include <vector>
#include <iostream>

#include <ampl/ampl.h>

#include "UNodeObject.h"
#include "../../Utillities/Vector3.h"

//#define MODELFOLDERPATH 	"/home/henning/Documents/Masters_Thesis/AMPE/source/MILP_Planner/build/amplModel/"
#define MODEL				"model.mod"
#define DATA				"model.dat"
#define DATACX				"chix.dat"
#define DATACY				"chiy.dat"
#define DATACZ				"chiz.dat"

class UMPathPlanner
{

public:

	UMPathPlanner(int missionTime = 100, double stepLength = 2);
	~UMPathPlanner();

	class ObjectPath
	{
	public:

		ObjectPath(unsigned int pathLength, unsigned int ID = 0) : ID(ID), path(pathLength, Vector3())
		{

		}
		~ObjectPath(){

		}

		unsigned int ID;
		std::vector<Vector3> path;
		std::vector<double> time;

		unsigned int getID() { return ID; }

		Vector3 & operator[](int i){
			return path[i];
		}
		double t(int i){
			return time[i];
		}

	};


	void setModelPath(std::string path);
	bool setup();

	void run(std::vector<std::vector<double> > waypointsInn);
	void runAsync(std::vector<std::vector<double> > waypointsInn);

	std::vector<ObjectPath> getPaths();

	ObjectPath getPathForObject(unsigned int ID);

	void setNodes(std::vector<NodeObject> nodes);
	void setNode(Vector3 pos, unsigned int ID);

	void setUAVPositions(std::vector<std::vector<double> > uavPositions);

	bool isDone() { return !ampl.isBusy(); }
	void setModelParam(std::string name, double val);
	void sabotage(double newBattery, int uavn);
private:

	ampl::AMPL ampl;
	bool isAmplRunning;

	std::vector<NodeObject> nodes;
	std::vector<ObjectPath> paths;

	std::string modelFolderPath;

	class MyInterpretIsOver  : public ampl::Runnable {
	public:
		void run() {
			std::cout << "solved";
		}
	} isOverHandler;

	void exportNodePositionsToAMPL();
	void modifyWp(std::vector<std::vector<double> > waypoints);

};






