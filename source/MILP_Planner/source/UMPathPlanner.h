#pragma once
#include <string>
#include <vector>
#include <iostream>

#include <ampl/ampl.h>

#include "UNodeObject.h"
#include "../../Utillities/Vector3.h"

#define MODELFOLDERPATH 	"amplModel/"
#define MODEL				"model.mod"
#define DATA				"model.dat"
#define DATACX				"chix.dat"
#define DATACY				"chiy.dat"
#define DATACZ				"chiz.dat"

class UMPathPlanner
{

public:

	UMPathPlanner();
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

		unsigned int getID() { return ID; }

		Vector3 & operator[](int i){
			return path[i];
		}

	};


	void setModelPath(std::string path);
	bool setup();

	void run();
	void runASynchrounous();

	std::vector<ObjectPath> getPaths();
	ObjectPath getPathForObject(unsigned int ID);

	void setNodes(std::vector<NodeObject> nodes);
	void setNode(Vector3 pos, unsigned int ID);
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

};






