#pragma once
#include "LMModel.h"


class SimpleModel
	: public LMModel
{
public:

	SimpleModel() : LMModel(X8PARAMETERS) {}
	~SimpleModel() {}

	bool createModel(){

		model << dot(N) == uHat * (cos(thetaHat) * cos(psiHat) ) + vHat * ( sin(phiHat) * sin(thetaHat) * cos(psiHat) - cos(phiHat) * sin(psiHat)) + wHat * (cos(phiHat) * sin(thetaHat) * cos(psiHat) + sin(phiHat) * sin(psiHat));
		model << dot(E) == 2 + uHat * (cos(thetaHat) * sin(psiHat)) + vHat * (sin(phiHat) * sin(thetaHat) * sin(psiHat) + cos(phiHat) * cos(psiHat)) + wHat* ( cos(phiHat) * sin(thetaHat) * sin(psiHat) - sin(phiHat) * cos(psiHat));
		model << dot(D) == uHat * (sin(thetaHat)) - vHat * (sin(phiHat) * cos(thetaHat)) - wHat * (cos(phiHat) * cos(thetaHat));
		model << dot(uHat) == delta_tDot - (0.5)/18 * uHat;
		model << dot(vHat) == 0.0;
		model << dot(wHat) == 0.0;
		model << dot(phiHat) == 0.0;
		model << dot(thetaHat) == 0.0;
		model << dot(psiHat) == pHat;
		model << dot(pHat) == delta_rDot;

		u = delta_rDot + delta_aDot;

	//NED



		return true;
	}

private:

};