#include "LMModelLinear.h"


bool LMModelLinear::createModel(){

	setParameter("Rho", 		param("J_x") * param("J_z") - pow(param("J_xz"),2));

	setParameter("Rho_1", 		( ( param("J_xz") * ( param("J_x") - param("J_y") + param("J_z") ) ) / ( param("Rho") ) ) );

	setParameter("Rho_2",		( ( param("J_z") * ( param("J_z") - param("J_y") ) + pow(param("J_xz"),2) ) / ( param("Rho") ) ) );

	setParameter("Rho_3", 		( ( param("J_z") ) ) / ( param("Rho") ) );

	setParameter("Rho_4", 		( ( param("J_xz") ) ) / ( param("Rho") ) );

	setParameter("Rho_5", 		( (  param("J_z") - param("J_x")  ) / ( param("J_y") ) ) );

	setParameter("Rho_6", 		( ( param("J_xz") ) ) / ( param("J_y") ) );

	setParameter("Rho_7", 		( ( ( param("J_x") - param("J_y") ) * param("J_x") + pow(param("J_xz"), 2) )  / ( param("Rho") ) ) );

	setParameter("Rho_8", 		( ( param("J_x") ) ) / ( param("Rho") ) );

	setParameter( "C_p_0" , 		param("Rho_3") * param("C_l_0") + param("Rho_4") * param("C_n_0") );

	setParameter( "C_p_beta" , 		param("Rho_3") * param("C_l_beta") + param("Rho_4") * param("C_n_beta") );

	setParameter( "C_p_p" , 		param("Rho_3") * param("C_l_p") + param("Rho_4") * param("C_n_p") );

	setParameter( "C_p_r" , 		param("Rho_3") * param("C_l_r") + param("Rho_4") * param("C_n_r") );

	setParameter( "C_p_delta_r" , 	param("Rho_3") * param("C_l_delta_r") + param("Rho_4") * param("C_n_delta_r") );

	setParameter( "C_p_delta_a" , 	param("Rho_3") * param("C_l_delta_a") + param("Rho_4") * param("C_n_delta_a") );

	setParameter( "C_r_0" , 		param("Rho_4") * param("C_l_0") + param("Rho_8") * param("C_n_0") );

	setParameter( "C_r_0" , 		param("Rho_4") * param("C_l_0") + param("Rho_8") * param("C_n_0") );

	setParameter( "C_r_beta" , 		param("Rho_4") * param("C_l_beta") + param("Rho_8") * param("C_n_beta") );

	setParameter( "C_r_p" , 		param("Rho_4") * param("C_l_p") + param("Rho_8") * param("C_n_p") );

	setParameter( "C_r_r" , 		param("Rho_4") * param("C_l_r") + param("Rho_8") * param("C_n_r") );

	setParameter( "C_r_delta_r" , 	param("Rho_4") * param("C_l_delta_r") + param("Rho_8") * param("C_n_delta_r") );

	setParameter( "C_r_delta_a" , 	param("Rho_4") * param("C_l_delta_a") + param("Rho_8") * param("C_n_delta_a") );

	setParameter( "C_r_0" , 		param("Rho_4") * param("C_l_0") + param("Rho_8") * param("C_n_0") );

	double alpha = 					alphaStar;
	double C_L = 					param("C_L_0") + param("C_L_alpha") * alpha;
	double C_D = 					param("C_D_0") + param("C_D_alpha") * alpha;

	setParameter( "C_X_0" , 0 );
	setParameter( "C_Z_0" , 0 );

	setParameter( "C_X_q" , 		param("C_D_q") * cos(alpha) + param("C_L_q") * sin(alpha));
	setParameter( "C_Z_q" , 		param("C_D_q") * sin(alpha) + param("C_L_q") * cos(alpha));

	setParameter( "C_X_delta_e" ,	param("C_D_delta_e") * cos(alpha) + param("C_L_delta_e") * sin(alpha));
	setParameter( "C_Z_delta_e" , 	param("C_D_delta_e") * sin(alpha) + param("C_L_delta_e") * cos(alpha));

	setParameter( "C_X_alpha" , 	param("C_D_alpha") * cos(alpha) + param("C_L_alpha") * sin(alpha));
	setParameter( "C_Z_alpha" , 	param("C_D_alpha") * sin(alpha) + param("C_L_alpha") * cos(alpha));


	//------------------------------Intermadiate Latteral p. 81

//Y_v
	Y_v = 			( (param("rho") * param("S") * param("b") * vStar) / (4 * param("m") * V_aStar) ) * ( param("C_Y_p") * pStar + param("C_Y_r") * rStar ) 
					+ ( (param("rho") * param("S") * vStar) / ( param("m") ) ) * ( param("C_Y_0") + param("C_Y_beta") * betaStar + param("C_Y_delta_a") * delta_aStar + param("C_Y_delta_r") * delta_rStar )
					+ ( param("rho") * param("S") * param("C_Y_beta") ) / (2 * param("m") ) * sqrt(pow(uStar, 2) * pow(wStar, 2)) ;


//Y_p
	Y_p = 			wStar + ( (param ("rho") * V_aStar * param("S") * param("b")) / (4 * param("m")) ) * param("C_Y_p");

//Y_r
	Y_r =			- uStar + ( (param ("rho") * V_aStar * param("S") * param("b")) / (4 * param("m")) ) * param("C_Y_r");

//Y_delta_a
	Y_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") ) / ( 2 * param("m")) ) * param("C_Y_delta_a");

//Y_delta_r
	Y_delta_r = 	( ( param("rho") * pow(V_aStar,2) * param("S") ) / ( 2 * param("m")) ) * param("C_Y_delta_r");

//L_v
	L_v = 			( (param("rho") * param("S") * pow(param("b"),2) * vStar )  / ( 4 * V_aStar ) ) * ( param("C_p_p") * pStar + param("C_p_r") * rStar )
					+ (param("rho") * param("S") * param("b") * vStar ) * ( param("C_p_0") + param("C_p_beta") * betaStar + param("C_p_delta_a") * delta_aStar + param("C_p_delta_r") * delta_rStar )
					+ ((param("rho") * param("S") * param("b") * param("C_p_beta") ) / (2 * param("m")) / 2 ) * ( sqrt(pow(uStar, 2) * pow(wStar, 2)) );

//L_p
	L_p = 			param("Rho_1") * qStar + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_p_p") );

//L_p
	L_r = 			param("Rho_2") * qStar + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_p_r") );

//L_delta_a
	L_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_p_delta_a");

//L_delta_r
	L_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_p_delta_r");

//N_v
	N_v = 			( (param("rho") * param("S") * pow(param("b"),2) * vStar )  / ( 4 * V_aStar ) ) * ( param("C_r_p") * pStar + param("C_r_r") * rStar )
					+ (param("rho") * param("S") * param("b") * vStar ) * ( param("C_r_0") + param("C_r_beta") * betaStar + param("C_r_delta_a") * delta_aStar + param("C_r_delta_r") * delta_rStar )
					+ ((param("rho") * param("S") * param("b") * param("C_r_beta") ) / (2 * param("m")) / 2 ) * ( sqrt(pow(uStar, 2) * pow(wStar, 2)) );

//N_p
	N_p = 			param("Rho_7") * qStar + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_r_p") );

//N_p
	N_r = 			param("Rho_1") * qStar + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_r_r") );

//N_delta_a
	N_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_r_delta_a");

//N_delta_r
	N_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_r_delta_r");


	//------------------------------Intermediate Londitudinal p.86

//X_u
	X_u = 	( ( uStar * param("rho") * param("S") ) / ( param("m") ) ) * ( param("C_X_0") + param("C_X_alpha") * alphaStar + param("C_X_delta_e") * delta_eStar)
			- ( ( param("rho") * param("S") * wStar * param("C_X_alpha") ) / ( 2 * param("m") ) )
			+ ( ( param("rho") * param("S") * param("c") * param("C_X_q") * uStar * qStar ) / ( 4 * param("m") * V_aStar ) )
			- ( ( param("rho") * param("S_prop") * param("C_prop") * uStar ) / ( param("m") ) );

//X_w
	X_w = 	-qStar
			+ ( ( wStar * param("rho") * param("S") ) / ( param("m") ) ) * ( param("C_X_0") + param("C_X_alpha") * alphaStar + param("C_X_delta_e") * delta_eStar)
			+ ( ( param("rho") * param("S") * param("c") * param("C_X_q") * wStar * qStar ) / ( 4 * param("m") * V_aStar ) )
			+ ( ( param("rho") * param("S") * param("C_X_alpha") * uStar ) / ( 2 * param("m") ) )
			- ( ( param("rho") * param("S_prop") * param("C_prop") * wStar ) / ( param("m") ) );

//X_q
	X_q = 	-wStar
			+ ( ( param("rho") * V_aStar * param("S") * param("C_X_q") * param("c") ) / ( 4 * param("m") ) );

//X_delta_e
	X_delta_e = ( ( param("rho") * pow(V_aStar,2) * param("S") * param("C_X_delta_e") ) / ( 2 * param("m") ) );

//X_delta_t
	X_delta_t = ( ( param("rho") * param("S_prop") * param("C_prop") * pow(param("k_motor"), 2) * delta_tStar ) / ( param("m") ) );


//Z_u
	Z_u = 	qStar
			+ ( ( uStar * param("rho") * param("S") ) / ( param("m") ) ) * ( param("C_Z_0") + param("C_Z_alpha") * alphaStar + param("C_Z_delta_e") * delta_eStar)
			- ( ( param("rho") * param("S") * wStar * param("C_Z_alpha") ) / ( 2 * param("m") ) )
			+ ( ( param("rho") * param("S") * param("c") * param("C_Z_q") * uStar * qStar ) / ( 4 * param("m") * V_aStar ) );

//Z_w
	Z_w = 	( ( wStar * param("rho") * param("S") ) / ( param("m") ) ) * ( param("C_Z_0") + param("C_Z_alpha") * alphaStar + param("C_Z_delta_e") * delta_eStar)
			+ ( ( param("rho") * param("S") * param("c") * param("C_Z_q") * wStar * qStar ) / ( 4 * param("m") * V_aStar ) )
			+ ( ( param("rho") * param("S") * param("C_Z_alpha") * uStar ) / ( 2 * param("m") ) );

//Z_q
	Z_q = 	uStar
			+ ( ( param("rho") * V_aStar * param("S") * param("C_Z_q") * param("c") ) / ( 4 * param("m") ) );

//Z_delta_e
	Z_delta_e = ( ( param("rho") * pow(V_aStar,2) * param("S") * param("C_Z_delta_e") ) / ( 2 * param("m") ) );

//M_u
	M_u = 	( ( uStar * param("rho") * param("S") * param("c") ) / ( param("J_y") ) ) * ( param("C_m_0") + param("C_m_alpha") * alphaStar + param("C_m_delta_e") * delta_eStar)
			- ( ( param("rho") * param("S") * wStar * param("C_m_alpha") * param("c") ) / ( 2 * param("J_y") ) )
			+ ( ( param("rho") * param("S") * pow(param("c"), 2) * param("C_m_q") * uStar * qStar ) / ( 4 * param("J_y") * V_aStar ) );

//M_w
	M_w = 	( ( wStar * param("rho") * param("S") ) / ( param("m") ) ) * ( param("C_m_0") + param("C_m_alpha") * alphaStar + param("C_m_delta_e") * delta_eStar)
			+ ( ( param("rho") * param("S") * param("c") * param("C_m_q") * wStar * qStar ) / ( 4 * param("J_y") * V_aStar ) )
			+ ( ( param("rho") * param("S") * param("C_m_alpha") * uStar ) / ( 2 * param("J_y") ) );

//M_q
	M_q = 	-wStar
			+ ( ( param("rho") * V_aStar * param("S") * param("C_m_q") * pow(param("c"), 2) ) / ( 4 * param("m") ) );

//M_delta_e
	M_delta_e = ( ( param("rho") * pow(V_aStar,2) * param("c") * param("S") * param("C_m_delta_e") ) / ( 2 * param("J_y") ) );

	//------------------------------Latteral

	model << dot(v) == Y_v * v + Y_p * p + Y_r * r + g * cos(thetaStar) * cos(phiStar) * phi + Y_delta_a * delta_a + Y_delta_r * delta_r;

	model << dot(p) == L_v * v + L_p * p + L_r * r + L_delta_a * delta_a + L_delta_r * delta_r;

	model << dot(r) == N_v * v + N_p * p + N_r * r + N_delta_a * delta_a + N_delta_r * delta_r;

	model << dot(phi) == p + ( cos(phiStar) * tan(thetaStar) ) * r + ( qStar * cos(phiStar) * tan(thetaStar) - rStar * sin (phiStar) * tan (thetaStar)) * phi;

	model << dot(psi) == ( cos(phiStar) * (1 / cos(thetaStar)) ) * r + ( pStar * cos(phiStar) * (1 / cos(thetaStar))) - rStar * sin (phiStar) * (1 / cos(thetaStar)) * phi;




	//------------------------------Longditudinal

	model << dot(u) == X_u * u + X_w * w + X_q * q + (- g * cos(thetaStar) * theta) + X_delta_e * delta_t + X_delta_t * delta_t;
	
	model << dot(w) == Z_u * u + Z_w * w + Z_q * q + (- g * sin(thetaStar) * theta) + Z_delta_e;
	
	model << dot(q) == M_u * u + M_w * w + M_q * q + M_delta_e * delta_t;

	model << dot(theta) == q;

	model << dot(h) == sin(thetaStar) * u + (- cos(thetaStar)) * v + (uStar * cos(thetaStar) + wStar * sin(thetaStar) ) * theta;



	//NED

	model << dot(N) == u * (cos(theta) * cos(psi) ) + v * ( sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) + w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
	model << dot(E) == u * (cos(theta) * sin(psi)) + v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) + w * ( cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
	model << dot(D) ==  u * (sin(theta)) - v * (sin(phi) * cos(theta)) - w * (cos(phi) * cos(theta));


	//Controll
	model << dot(delta_a) == delta_aDot;
	model << dot(delta_a) == delta_aDot;
	model << dot(delta_e) == delta_eDot;
	model << dot(delta_t) == delta_tDot;


	return true;
}