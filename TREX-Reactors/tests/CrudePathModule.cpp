#include "CrudePathModule.h"


void CrudePathModule::addUAV(double n0, double e0, double d0, int id) {
	
	bool exists = false;
	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			exists = true;
			break;
		}
	if (!exists)
		UAVs.push_back(UAV(n0, e0, d0, id));
	
}



bool CrudePathModule::removeUAV(int id) {

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		UAVs.erase(it);
		return true;
	}
	else
		return false; 
}

bool CrudePathModule::moveUAVTo(double n, double e, double d, int id){

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		it->setPosition(n,e,d);
		return true;
	}
	else
		return false; 

}

bool CrudePathModule::decomissionUAV(int id){

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		it->isActive = false;
		return true;
	}
	else
		return false; 

}

bool CrudePathModule::comissionUAV(int id){

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		it->isActive = true;
		return true;
	}
	else
		return false; 

}


void CrudePathModule::addNode(double n0, double e0, double d0, int id) {
	
	bool exists = false;
	auto it = nodes.begin();
	for (it; it != nodes.end(); it++)
		if (it->id == id){
			exists = true;
			break;
		}
	if (!exists)
		nodes.push_back(Node(n0, e0, d0, id));
}

bool CrudePathModule::moveNodeTo(double n, double e, double d, int id) {

	auto it = nodes.begin();
	for (it; it != nodes.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != nodes.end()){
		it->setPosition(n, e, d);
		return true;
	}
	else
		return false;
}

bool CrudePathModule::removeNode(int id) {

	auto it = nodes.begin();
	for (it; it != nodes.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != nodes.end()){
		nodes.erase(it);
		return true;
	}
	else
		return false;
}

std::vector<WP> CrudePathModule::getPathForUAV(int id){

	int pos = 1;
	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++){
		if (it->id == id){
			break;
		}
		else
			pos++;
	}

	if (it == UAVs.end())
		return std::vector<WP>();

	int low = (numberOfNodes() * (pos-1) ) / numberOfUAVs();
	int high = (numberOfNodes() * pos) / numberOfUAVs();

	std::vector<WP> result;
	for (int i = low; i < high; i++)
		result.push_back(WP(nodes[i].n, nodes[i].e, nodes[i].d));

	return result;
}

int CrudePathModule::numberOfUAVs(){
	return UAVs.size();
}
int CrudePathModule::numberOfNodes(){
	return nodes.size();
}