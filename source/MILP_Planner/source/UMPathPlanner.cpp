#include "UMPathPlanner.h"



UMPathPlanner::UMPathPlanner() : ampl(), isAmplRunning(false), nodes()
{

	setModelPath(MODELFOLDERPATH);

}

UMPathPlanner::~UMPathPlanner(){



}

bool UMPathPlanner::setup(){

	ampl.read(modelFolderPath + MODEL);
	ampl.readData(modelFolderPath + DATA);
	ampl.readData(modelFolderPath + DATACX);
	ampl.readData(modelFolderPath + DATACY);
	ampl.readData(modelFolderPath + DATACZ);

}


void UMPathPlanner::setModelPath(std::string path){
	this->modelFolderPath = path;
}

std::vector<UMPathPlanner::ObjectPath> UMPathPlanner::getPaths(){

	if (ampl.isBusy())
		return this->paths;

	//std::vector<std::vector<std::vector<double> > > path;

	ampl::Variable amplVarx = ampl.getVariable("x");
	ampl::Variable amplVary = ampl.getVariable("y");
	ampl::Variable amplVarz = ampl.getVariable("z");
	ampl::DataFrame datax = amplVarx.getValues();
	ampl::DataFrame datay = amplVary.getValues();
	ampl::DataFrame dataz = amplVarz.getValues();

	int n = datax.getRowByIndex(datax.getNumRows()-1)[1].dbl(); //get length of path
	int p = datax.getRowByIndex(datax.getNumRows()-1)[0].dbl();	//get number of uavs

	//path = std::vector<std::vector<std::vector<double> > > (p, std::vector<std::vector<double> >(n, std::vector<double>(3, 0)));

	paths = std::vector<ObjectPath>(p, ObjectPath(n));


	std::cout << n << " " << p;

	int a = datax.getNumRows()/p -1;
	for (int i = 0; i < a; i++) {	
		
		for(int k = 0; k < p; k++){

			//path[k][i][0] = datax.getRowByIndex(i + n*k)[2].dbl();
			//path[k][i][1] = datay.getRowByIndex(i + n*k)[2].dbl();
			//path[k][i][2] = dataz.getRowByIndex(i + n*k)[2].dbl();

			paths[i][k].setValue(
				datax.getRowByIndex(i + n*k)[2].dbl(),
				datay.getRowByIndex(i + n*k)[2].dbl(),
				dataz.getRowByIndex(i + n*k)[2].dbl()
								);

		}
		
	}

	//this->path = path;
	return paths;

}

void UMPathPlanner::setNodes(std::vector<NodeObject> nodes)
{
	this->nodes = nodes;
}


void UMPathPlanner::setNode(Vector3 pos, unsigned int ID)
{

	auto it  = nodes.begin();

	for (it; it != nodes.end(); it++){

		if (it->getID() == ID)
			break;

	}

	if (it == nodes.end()){
		nodes.push_back(NodeObject(pos, ID));
	}
	else{
		it->setPos(pos);
	}

}


void UMPathPlanner::exportNodePositionsToAMPL(){

	int n = nodes.size();
	int nc = 3;

	if (n==0) return;

	ampl::Parameter waypoints = ampl.getParameter("waypoints");
	ampl::Parameter W = ampl.getParameter("W");

	double colIndices[] = { 1,2,3 };
	double* rowIndices = new double[n];
	double* waypointData = new double[n*nc];


	for (int i = 0; i < n; i++) {
		rowIndices[i] = i + 1;

		for (int j = 0; j < nc; j++)
			waypointData[j + i * nc] = nodes[i][j];
	}

	ampl::Variant wn(n);

	waypoints.setValues(n, rowIndices, 3, colIndices, waypointData, false);
	W.set(wn);
}