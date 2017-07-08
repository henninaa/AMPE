#include "LMModel.h"

LMModel::LMModel(int parameterSet){

	std::string baseLoc = UAVPARAMLOCATION;
	switch (parameterSet){
		case X8PARAMETERS:
			loadParameters(baseLoc + "params_X8.txt");
		break;
		case ZAGIPARAMETERS:
			loadParameters(baseLoc + "params_Zagi.txt");
		break;
		case AEROSONDEPARAMETERS:
			loadParameters(baseLoc + "params_Aerosonde.txt");
		break;
	}

}


LMModel::~LMModel(){


}

void LMModel::createDiffState(std::string name){

	differentialStates[name] = new ACADO::DifferentialState();
}

void LMModel::addDiffState(std::string name, ACADO::DifferentialState * state){

	differentialStates[name] = state;
}

void LMModel::createDIntermediateState(std::string name){

	intermediateStates[name] = new ACADO::IntermediateState();
}

void LMModel::addDIntermediateState(std::string name, ACADO::IntermediateState * state){

	intermediateStates[name] = state;
}

double LMModel::param(std::string param){

	std::map<std::string, double>::iterator it = parameters.find(param);

	if (it == parameters.end()){
		std::cout<< "\nError finding parameter: " << param << std::endl;
		return 0.0;
	}

	return it->second;

}

void LMModel::loadParameters(std::string filename){

	std::ifstream file;
	file.open(filename);

	if (file.fail()){
		std::cout << "\nError loading: " << filename << std::endl;
		return;
	}

	std::string content;
	while (std::getline(file, content)){
		if (content.size() > 1)
			insertParameter(content);
	}
	file.close();

}

void LMModel::printAllParameters(){

	std::cout << "\nPARAMS:\n";
	for (auto it = parameters.begin(); it != parameters.end(); it++)
		std::cout << it->first << ": " << it->second << std::endl;

}

void LMModel::insertParameter(std::string content){

	std::string name = "";
	std::string valueS = "";

	enum _state {nameState, valueState} loopState = nameState;

	for (auto it = content.begin(); it != content.end(); it++){

		switch (loopState){
			case nameState:

				if (*it == ' '){
					loopState = valueState;
				}
				else{
					name += *it;
				}

			break;
			case valueState:


				if (isdigit(*it) || *it == '-' || *it == '.' || *it == ','){

					valueS += *it;

				}


			break;
		}

	}

	if (name.size() > 0 && valueS.size() > 0)
		parameters[name] = std::stod(valueS);

}

void LMModel::setParameter(std::string name, double value){
	parameters[name] = value;
}