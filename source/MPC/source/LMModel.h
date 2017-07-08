#pragma once
#include <map>
#include <string>
#include <map>
#include <iostream>
//acado
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>

//#define PARAMLOCATION "/home/henning/Documents/Masters_Thesis/AMPE/source/MPC/source/"
#define X8PARAMETERS 0x1000
#define ZAGIPARAMETERS 0x1001
#define AEROSONDEPARAMETERS 0x1002

class LMModel
{
public:
	LMModel(int parameterSet);
	~LMModel();
	
	virtual bool createModel() { std::cout << "feil\n";return false; }
	ACADO::DifferentialEquation * getModel() { return &model; }
	void printAllParameters();

	//ACADO::DifferentialState N, E, D, uHat, vHat, wHat, phiHat, thetaHat, psiHat, pHat, rHat, qHat, hHat,  delta_eHat, delta_aHat, delta_rHat, delta_tHat;
	//ACADO::IntermediateState u, v, w, theta, phi, psi, p, r, q, h,  delta_e, delta_a, delta_r, delta_t;
	//ACADO::Control delta_eDot, delta_aDot, delta_rDot, delta_tDot;

	ACADO::DifferentialState vHat, pHat, rHat, phiHat, psiHat, uHat, wHat, qHat, thetaHat, hHat, N, E, D, delta_aHat, delta_rHat, delta_eHat, delta_tHat;
	ACADO::IntermediateState u, v, w, theta, phi, psi, p, r, q, h,  delta_e, delta_a, delta_r, delta_t;
	ACADO::Control delta_aDot, delta_rDot, delta_eDot, delta_tDot;

protected:

	//double m, J_x, J_y, J_z, J_xy, S, b, C_L_0, C_L_0, C_D_0, C_m_0, C_L_alpha, C_D_alpha, C_m_alpha, 
	double const g = 9.81;
	double const windN = 0.0;
	double const windY = 0.0;

	std::map<std::string, ACADO::DifferentialState *> differentialStates;
	std::map<std::string, ACADO::IntermediateState *> intermediateStates;
	std::map < std::string, double> parameters;

	double param(std::string param);

	ACADO::DifferentialEquation model;

	void createDiffState(std::string name);
	void addDiffState(std::string name, ACADO::DifferentialState * state);
	void createDIntermediateState(std::string name);
	void addDIntermediateState(std::string name, ACADO::IntermediateState * state);


	void createLateralModel();
	void createVModel();

	void loadParameters(std::string filename);
	void insertParameter(std::string content);
	void setParameter(std::string name, double value);
};