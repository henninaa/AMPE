#pragma once
#include <map>
#include <string>
#include <math.h>
//acado
#include <acado_optimal_control.hpp>
//internal includes
#include "LMModel.h"

class LMModelLinear :
	public LMModel
{
public:
	LMModelLinear() : LMModel(X8PARAMETERS) {}
	~LMModelLinear() {}

	bool createModel();

	ACADO::DifferentialState N, E, D, uHat, vHat, wHat, thetaHat, phiHat, psiHat, pHat, rHat, qHat, hHat,  delta_eHat, delta_aHat, delta_rHat, delta_tHat;
	ACADO::IntermediateState u, v, w, theta, phi, psi, p, r, q, h,  delta_e, delta_a, delta_r, delta_t;
	ACADO::Control delta_eDot, delta_aDot, delta_rDot, delta_tDot;
private:

	double					Y_v, Y_p, Y_r, Y_delta_a, Y_delta_r,
							L_v, L_p, L_r, L_delta_a, L_delta_r,
							N_v, N_p, N_r, N_delta_a, N_delta_r,
							X_u, X_w, X_q, X_delta_e, X_delta_t,
							Z_u, Z_w, Z_q, Z_delta_e, Z_delta_t,
							M_u, M_w, M_q, M_delta_e, M_delta_t;

	double uStar, vStar, wStar, thetaStar, phiStar, psiStar, pStar, qStar, rStar, alphaStar, betaStar, delta_aStar, delta_rStar, delta_eStar, delta_tStar, hStar; //Trim
	double V_aStar;
	//other parameters can be found in LMModel.h

	void setEquilibriumStates();

	void setConstaintsAsParameters();
};