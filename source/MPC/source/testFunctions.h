#pragma once
#include "ALMPC.h"

void testSimulate(){

	LMMPC * mpc = new LMMPC();
	mpc->addWaypoint(60.0, 0.0, 50.0, 0.0);
	mpc->addWaypoint(110.0, 20.0, 50.0, 4.0);
	mpc->addWaypoint(135.0, 45.0, 50.0, 5.0);
	mpc->setup();
	std::cout << "Waypoints inserted\n";

	ACADO::DVector s0(17);
	s0.setAll(0.0);
	s0(5) = 30;
	s0(9) = 50;
	s0(12) = 50;
	std::cout << "starting simulation...\n";
	mpc->simulate(5.0, s0);
}