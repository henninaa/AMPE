#pragma once
#include <string>
#include <vector>

#include "shared.h"


class CrudePathModule
{
public:

	CrudePathModule() {}
	~CrudePathModule() {}

	virtual void addUAV(double n0, double e0, double d0, int id);
	virtual bool removeUAV(int id);
	virtual bool moveUAVTo(double n, double e, double d, int id);
	virtual bool decomissionUAV(int id);
	virtual bool comissionUAV(int id);

	virtual void addNode(double n0, double e0, double d0, int id);
	virtual bool moveNodeTo(double n, double e, double d, int id);
	virtual bool removeNode(int id);


	virtual std::vector<WP> getPathForUAV(int id);	//Implemented here in order to use this as a test class

protected:

	int numberOfUAVs();
	int numberOfNodes();
	std::vector<Node> nodes;
	std::vector<UAV> UAVs;

};