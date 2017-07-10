#include "LMModelLinear.h"

LMModelLinear::LMModelLinear() : LMModel(X8PARAMETERS) {
	createModel();
}

bool LMModelLinear::createModel(){

	std::cout << "\nModel Created\n" << std::flush;

	setEquilibriumStates();

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

	setParameter( "C_r_beta" , 		param("Rho_4") * param("C_l_beta") + param("Rho_8") * param("C_n_beta") );

	setParameter( "C_r_p" , 		param("Rho_4") * param("C_l_p") + param("Rho_8") * param("C_n_p") );

	setParameter( "C_r_r" , 		param("Rho_4") * param("C_l_r") + param("Rho_8") * param("C_n_r") );

	setParameter( "C_r_delta_r" , 	param("Rho_4") * param("C_l_delta_r") + param("Rho_8") * param("C_n_delta_r") );

	setParameter( "C_r_delta_a" , 	param("Rho_4") * param("C_l_delta_a") + param("Rho_8") * param("C_n_delta_a") );

	double alpha = 					alphaStar;
	double C_L = 					param("C_L_0") + param("C_L_alpha") * alpha;
	double C_D = 					param("C_D_0") + param("C_D_alpha") * alpha;

	setParameter( "C_X_0" , 0 );
	setParameter( "C_Z_0" , 0 );

	setParameter( "C_X_q" , 		param("C_D_q") * cos(alpha) + param("C_L_q") * sin(alpha));
	setParameter( "C_Z_q" , 		param("C_D_q") * sin(alpha) + param("C_L_q") * cos(alpha));

	setParameter( "C_X_delta_e" ,	param("C_D_delta_e") * cos(alpha) + param("C_L_delta_e") * sin(alpha));
	setParameter( "C_Z_delta_e" , 	param("C_D_delta_e") * sin(alpha) + param("C_L_delta_e") * cos(alpha));

	setParameter( "C_X_alpha" , 	0.0);//param("C_D_alpha") * cos(alpha) + param("C_L_alpha") * sin(alpha));
	setParameter( "C_Z_alpha" , 	0.0);//param("C_D_alpha") * sin(alpha) + param("C_L_alpha") * cos(alpha));


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
	L_r = 			(-param("Rho_2") * qStar) + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_p_r") );

//L_delta_a
	L_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_p_delta_a");

//L_delta_r
	L_delta_r = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_p_delta_r");

//N_v
	N_v = 			( (param("rho") * param("S") * pow(param("b"),2) * vStar )  / ( 4 * V_aStar ) ) * ( param("C_r_p") * pStar + param("C_r_r") * rStar )
					+ (param("rho") * param("S") * param("b") * vStar ) * ( param("C_r_0") + param("C_r_beta") * betaStar + param("C_r_delta_a") * delta_aStar + param("C_r_delta_r") * delta_rStar )
					+ ((param("rho") * param("S") * param("b") * param("C_r_beta") ) / (2 * param("m")) / 2 ) * ( sqrt(pow(uStar, 2) * pow(wStar, 2)) );

//N_p
	N_p = 			param("Rho_7") * qStar + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_r_p") );

//N_p
	N_r = 			(-param("Rho_1") * qStar) + ( ( param("rho") * param("S") * V_aStar * pow(param("b"), 2) ) / ( 4 ) ) * ( param("C_r_r") );

//N_delta_a
	N_delta_a = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_r_delta_a");

//N_delta_r
	N_delta_r = 	( ( param("rho") * pow(V_aStar,2) * param("S") * param("b") ) / ( 2 ) ) * param("C_r_delta_r");


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

	//intermediate States:

	u = uHat + uStar;
	v = vHat + vStar;
	w = wHat + wStar;
	theta = thetaHat + thetaStar;
	phi = phiHat + phiStar;
	psi = psiHat + psiStar;
	p = pHat + pStar;
	r = rHat + rStar;
	q = qHat + qStar;
	h = hHat + hStar;
	delta_a = delta_aHat + delta_aStar;
	delta_r = delta_rHat + delta_rStar;
	delta_e = delta_eHat + delta_eStar;
	delta_t = delta_tHat + delta_tStar;


	//------------------------------Latteral

	model << dot(vHat) == Y_v * vHat + Y_p * pHat + Y_r * rHat + g * cos(thetaStar) * cos(phiStar) * phiHat + Y_delta_a * delta_aHat + Y_delta_r * delta_rHat;

	model << dot(pHat) == L_v * vHat + L_p * pHat + L_r * rHat + L_delta_a * delta_aHat + L_delta_r * delta_rHat;

	model << dot(rHat) == N_v * vHat + N_p * pHat + N_r * rHat + N_delta_a * delta_aHat + N_delta_r * delta_rHat;

	model << dot(phiHat) == pHat + ( cos(phiStar) * tan(thetaStar) ) * rHat + ( qStar * cos(phiStar) * tan(thetaStar) - rStar * sin (phiStar) * tan (thetaStar)) * phiHat;

	model << dot(psiHat) == ( cos(phiStar) * (1 / cos(thetaStar)) ) * rHat + ( pStar * cos(phiStar) * (1 / cos(thetaStar))) - rStar * sin (phiStar) * (1 / cos(thetaStar)) * phiHat;




	//------------------------------Longditudinal

	model << dot(uHat) == X_u * uHat + X_w * wHat + X_q * qHat + (- g * cos(thetaStar) * thetaHat) + X_delta_e * delta_eHat + X_delta_t * delta_tHat;
	
	model << dot(wHat) == Z_u * uHat + Z_w * wHat + Z_q * qHat + (- g * sin(thetaStar) * thetaHat) + Z_delta_e * delta_eHat;
	
	model << dot(qHat) == M_u * uHat + M_w * wHat + M_q * qHat + M_delta_e * delta_e;

	model << dot(thetaHat) == qHat;

	model << dot(hHat) == sin(thetaStar) * uHat + (- cos(thetaStar)) * vHat + (uStar * cos(thetaStar) + wStar * sin(thetaStar) ) * thetaHat;



	//NED

	model << dot(N) == windN + u * (cos(theta) * cos(psi) ) + v * ( sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) + w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
	model << dot(E) == windY + u * (cos(theta) * sin(psi)) + v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) + w * ( cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
	model << dot(D) == u * (sin(theta)) - v * (sin(phi) * cos(theta)) - w * (cos(phi) * cos(theta));


	//Controll
	model << dot(delta_aHat) == delta_aDot;
	model << dot(delta_rHat) == delta_rDot;
	model << dot(delta_eHat) == delta_eDot;
	model << dot(delta_tHat) == delta_tDot;

	setConstaintsAsParameters();
	return true;
}

void LMModelLinear::setEquilibriumStates(){

	uStar = 18.0168;
	vStar = 0.0001;
	wStar = 0.8366;
	thetaStar = (2.65868 / 180) * 3.14159265359;
	phiStar = 0.0;
	psiStar = 0.0;
	pStar = 0.0;
	qStar = 0.0;
	rStar = 0.0;
	alphaStar = 0.0;
	betaStar = 0.0;
	delta_aStar = 0.0000;
	delta_rStar = 0.0;
	delta_eStar = 0.0039;//0.0079;
	delta_tStar = 0.1240;
	V_aStar = 19.0;
	hStar = 0.0;

}

void LMModelLinear::setConstaintsAsParameters(){

	setParameter("Y_v", Y_v);
	setParameter("Y_p", Y_p);
	setParameter("Y_r", Y_r);
	setParameter("Y_delta_a", Y_delta_a);
	setParameter("Y_delta_r", Y_delta_r);
	setParameter("L_v", L_v);
	setParameter("L_p", L_p);
	setParameter("L_r", L_r);
	setParameter("L_delta_a", L_delta_a);
	setParameter("L_delta_r", L_delta_r);
	setParameter("N_v", N_v);
	setParameter("N_p", N_p);
	setParameter("N_r", N_r);
	setParameter("N_delta_a", N_delta_a);
	setParameter("N_delta_r", N_delta_r);
	setParameter("X_u", X_u);
	setParameter("X_w", X_w);
	setParameter("X_q", X_q);
	setParameter("X_delta_e", X_delta_e);
	setParameter("X_delta_t", X_delta_t);
	setParameter("Z_u", Z_u);
	setParameter("Z_w", Z_w);
	setParameter("Z_q", Z_q);
	setParameter("Z_delta_e", Z_delta_e);
	setParameter("Z_delta_t", Z_delta_t);
	setParameter("M_u", M_u);
	setParameter("M_w", M_w);
	setParameter("M_q", M_q);
	setParameter("M_delta_e", M_delta_e);
	setParameter("M_delta_t", M_delta_t);
	setParameter("thetaStar", thetaStar);
	setParameter("V_aStar", V_aStar);


}

