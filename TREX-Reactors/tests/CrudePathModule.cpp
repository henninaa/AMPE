#include "CrudePathModule.h"

CrudePathModule::CrudePathModule() : planner(36, 5.0), sampleTime(5)
{

	planner.setModelParam("deltat", sampleTime);

}

void CrudePathModule::run(){


	std::vector<std::vector<double> > wp = std::vector<std::vector<double> >(4, std::vector<double>(3, 100));
	wp[0][0] = 0;	wp[0][1] = 0;	wp[0][2] = 100;
	wp[1][0] = 100;	wp[1][1] = 100;	wp[1][2] = 200;
	wp[2][0] = 400;	wp[2][1] = 200;	wp[2][2] = 400;
	wp[3][0] = 300;	wp[3][1] = 200;	wp[3][2] = 100;

	//checkDistances();

	setUAVPositions();

	planner.runAsync(getWaypointsVector());

	//bool fin = false;
	//std::vector<std::vector<std::vector<double> > >pathWp;


}

void CrudePathModule::runSync(){


	std::vector<std::vector<double> > wp = std::vector<std::vector<double> >(4, std::vector<double>(3, 100));
	wp[0][0] = 0;	wp[0][1] = 0;	wp[0][2] = 100;
	wp[1][0] = 100;	wp[1][1] = 100;	wp[1][2] = 200;
	wp[2][0] = 400;	wp[2][1] = 200;	wp[2][2] = 400;
	wp[3][0] = 300;	wp[3][1] = 200;	wp[3][2] = 100;

	//checkDistances();

	setUAVPositions();

	planner.run(getWaypointsVector());

	//bool fin = false;
	//std::vector<std::vector<std::vector<double> > >pathWp;


}

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
		checkDistances(*it);
		return true;
	}
	else
		return false; 

}

bool CrudePathModule::moveUAVN(double n, int id){

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		it->n = n;
		checkDistances(*it);
		return true;
	}
	else
		return false; 

}

bool CrudePathModule::moveUAVE(double e, int id){

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		it->e = e;
		checkDistances(*it);
		return true;
	}
	else
		return false; 

}

bool CrudePathModule::moveUAVD(double d, int id){

	auto it = UAVs.begin();
	for (it; it != UAVs.end(); it++)
		if (it->id == id){
			break;
		}
	if (it != UAVs.end()){
		it->d = d;
		checkDistances(*it);
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

int CrudePathModule::getNumberOfActiveNodes(){

	int result = 0;
	for(auto it = nodes.begin(); it != nodes.end(); it++)
		if (it->isActive)
			result++;

	return result;
}

std::vector<std::vector<double> > CrudePathModule::getWaypointsVector(){

	std::vector<std::vector<double> > wps = std::vector<std::vector<double> >(getNumberOfActiveNodes(), std::vector<double>(3, 0.0));

	int i = -1;
	for (auto it = nodes.begin(); it != nodes.end(); it++){

		if(!it->isActive)
			continue;
		else
			i++;

		wps[i][0] = it->n;
		wps[i][1] = it->e;
		wps[i][2] = it->d;

	}
	
	return wps;

}


void CrudePathModule::checkDistances(UAV & uav){

	double distance;

	for (auto it = nodes.begin(); it != nodes.end()-1; it++){

		distance = std::sqrt( std::pow(uav.n - it->n, 2) + std::pow(uav.e - it->e, 2) + std::pow(uav.d - it->d, 2));

		if (distance < NodeHitDistance)
			it->isActive = false;


	}

}

void CrudePathModule::setUAVPositions(){

	std::vector<std::vector<double> > pos = std::vector<std::vector<double> >(UAVs.size(), std::vector<double>(3, 0.0));

	int i = -1;
	for (auto it = UAVs.begin(); it != UAVs.end(); it++){

		if(!it->isActive)
			continue;
		else
			i++;

		pos[i][0] = it->n;
		pos[i][1] = it->e;
		pos[i][2] = it->d;

	}
	
	planner.setUAVPositions(pos);

}

void CrudePathModule::storePath(){
	//std::cout << "\n" << nodes.size() << " \n";
	path = planner.getPaths();
	//std::cout << "\ndbg\n";

}

WP CrudePathModule::getWaypoint(int uavNr, int wpNr){

	WP result(path[uavNr][wpNr][0],
	path[uavNr][wpNr][1],
	path[uavNr][wpNr][2],
	wpNr);

	return result;
}

int CrudePathModule::getPathSize(){

	if(path.size() == 0)
		return 0;
	else
		return path[0].path.size();

}

	
void CrudePathModule::setSampleTime(double st){

	sampleTime = st;
	planner.setModelParam("deltat", st);

}