#pragma once
#include <string>
#include <vector>
#include <cmath>

#include "shared.h"
#include "../../source/MILP_Planner/source/UMPathPlanner.h"


class CrudePathModule
{
public:

	CrudePathModule();
	~CrudePathModule() {}

	virtual void run();
	virtual bool isRunning(){return !planner.isDone();}
	virtual bool isDone(){return planner.isDone();}

	virtual void addUAV(double n0, double e0, double d0, int id);
	virtual bool removeUAV(int id);
	virtual bool moveUAVTo(double n, double e, double d, int id);
	virtual bool moveUAVN(double n, int id);
	virtual bool moveUAVE(double e, int id);
	virtual bool moveUAVD(double d, int id);
	virtual bool decomissionUAV(int id);
	virtual bool comissionUAV(int id);

	virtual void addNode(double n0, double e0, double d0, int id);
	virtual bool moveNodeTo(double n, double e, double d, int id);
	virtual bool removeNode(int id);

	virtual WP getWaypoint(int uavNr, int wpNr);
	virtual int getPathSize();

	virtual std::vector<WP> getPathForUAV(int id);	//Implemented here in order to use this as a test class

	virtual void updatePath() {storePath();}

	double getSampleTime() { return sampleTime; }
	void setSampleTime(double st);

protected:
	double const NodeHitDistance = 15;
	double sampleTime;
	int numberOfNodes();
	int numberOfUAVs();
	std::vector<Node> nodes;
	std::vector<UAV> UAVs;
	std::vector<UMPathPlanner::ObjectPath> path;

	void storePath();
	void checkDistances(UAV & uav);
	
	int getNumberOfActiveNodes();
	std::vector<std::vector<double> > getWaypointsVector();
	void setUAVPositions();

	UMPathPlanner planner;

};