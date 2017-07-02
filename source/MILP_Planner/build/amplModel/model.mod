
param np; 	#total number of vehicles

param N; 	# optimization horizon

param deltat; 	# Sampletime

param Vmax;
param Vmin;

set Dim := 1..3;		#ENU


param initPos1{i in Dim};
param initPos2{i in Dim};

param x_min;
param x_max;
param y_min;
param y_max;
param h_min;
param h_max;

#test params:
param goal{i in Dim};
param goal2{i in 1..2, j in 1..4};  #row first then collumn

param Dvel;
param chix{k in 1..Dvel, l in 1..Dvel/2};
param chiy{k in 1..Dvel, l in 1..Dvel/2};
param chiz{k in 1..Dvel, l in 1..Dvel/2};

param alphavel;			#alpha for velocity estimation
param Mvel;

param rweight{j in Dim};				#acceleration weighting

param W;
param dwp;
param Mwp;

param waypoints{w in 1..W, j in Dim};  #--- actual waypoints.

param M_finnish;
param gamma_finnish;  		# positive weight for J_finnish

param M_con;

param dx;	#
param dy;	#	-anti-collition safety distance
param dz;	#

param MCol; # Big M andti-collision

param cx;	#
param cy;	#	-connectivity distance
param cz;	#

param MConDim; # big M for casting conectivityDimentional to bool

#-------- Vars

var v {p in 1..np, i in 0..N, row in Dim};		#velocity in vehicle, time, row
var V {p in 1..np, i in 0..N};		#absolute velocity in vehicle, time, row

var pos {p in 1..np, i in 0..N, row in Dim};

var Jacc; 					#The penalizing var for acceleration
var wacc{p in 1..np, i in 0..N, j in Dim};					#Acceleration vector

var theta{w in 1..W};
var theta_finnish{p in 1..np};
var eta_finnish;			# max finnishing time
var  J_finnish;					# finnishing time pennalizer;
 
var x{p in 1..np, i in 0..N};		# 
var y{p in 1..np, i in 0..N};		#		-Used to read in teleo reactor
var z{p in 1..np, i in 0..N};		#

var lambda_sensor{p in 1..np, i in 0..N, t in 1..W};

var UAV_distances{p in 1..np, q in 1..np, i in 0..N};

 
#--- binary vars

var bvel{p in 1..np, i in 0..N, k in 1..Dvel, l in 1..Dvel/2} binary;

var bwp{p in 1..np, i in 0..N, w in 1..W} binary;

var bsensor{p in 1..np, i in 0..N} binary;

var bCol{p in 1..np-1, q in 2..np, i in 1..N, row in 1..3, col in 1..2}; #big M anti-collision 

var bconnectivityDimentional{p in 1..np, q in 1..np, i in 0..N, d in Dim} binary;
var bconnectivity{p in 1..np, q in 1..np, i in 0..N} binary;

#-------- model

#objective goes here:
minimize objective: J_finnish + Jacc;

#constraints:

#initialPosition

subject to initPos1constraint{r in Dim}:
pos[1,0,r] = initPos1[r];

subject to initPos2constraint{r in Dim}:
pos[2,0,r] = initPos2[r];

#UAV model

subject to model{p in 1..np, i in 0..N-1, row in Dim}:
pos[p, i+1, row] = pos[p,i,row] + deltat * v[p,i,row];

#velocity constraints

/*
subject to poorVEstimate{p in 1..np, i in 0..N}:
V[p,i] = sum{row in Dim}(v[p,i,row]);*/

subject to Vestimate1{p in 1..np, i in 0..N-1, k in 1..Dvel, l in 1..Dvel/2}:
V[p,i] >= v[p,i,1] * chix[k,l] + v[p,i,2] * chiy[k,l] + v[p,i,3] * chiz[k,l];

subject to Vestimate2{p in 1..np, i in 0..N-1, k in 1..Dvel, l in 1..Dvel/2}:
alphavel*(v[p,i,1] * chix[k,l] + v[p,i,2] * chiy[k,l] + v[p,i,3] * chiz[k,l]) >= V[p,i] - Mvel * (1 - bvel[p,i,k,l]);

subject to Vestimate3{p in 1..np, i in 0..N-1}:
sum{k in 1..Dvel, l in 1..Dvel/2}( bvel[p,i,k,l] ) = 1;

subject to velocityConstraintsMax{p in 1..np, i in 0..N}:
V[p,i] <= Vmax * (1 - bwp[p,i,W]);

subject to velocityConstraintsMin{p in 1..np, i in 0..N}:
V[p,i] >= Vmin * (1 - bwp[p,i,W]);


#position constraints

subject to posx1{p in 1..np, i in 0..N}:
pos[p,i,1] >= x_min;

subject to posx2{p in 1..np, i in 0..N}:
pos[p,i,1] <= x_max;

subject to posy1{p in 1..np, i in 0..N}:
pos[p,i,2] >= y_min;

subject to posy2{p in 1..np, i in 0..N}:
pos[p,i,2] <= y_max;

subject to posh1{p in 1..np, i in 0..N}:
pos[p,i,3] >= h_min;

subject to posh2{p in 1..np, i in 0..N}:
pos[p,i,3] <= h_max;


subject to accelerationConstraint1:
Jacc = sum{p in 1..np, i in 0..N-2, j in Dim}(rweight[j] * wacc[p,i,j]);

subject to accelerationConstraint2{p in 1..np,i in 0..N-2, j in Dim}:
(v[p,i, j] - v[p, i+1, j]) <= wacc[p, i, j];

subject to accelerationConstraint3{p in 1..np,i in 0..N-2, j in Dim}:
-(v[p,i, j] - v[p, i+1, j]) <= wacc[p, i, j];

#------ Task assignment

subject to waypointHitP{p in  1..np, i in 1..N, w in 1..W, j in Dim}:
pos[p,i,j] - waypoints[w,j] - dwp <= Mwp * (1 - bwp[p,i,w]);

subject to waypointHitN{p in  1..np, i in 1..N, w in 1..W, j in Dim}:
 - pos[p,i,j] + waypoints[w,j] - dwp <= Mwp * (1 - bwp[p,i,w]);
 
 subject to wpOnce{w in 1..W-1}:
 sum{p in 1..np, i in 1..N}(bwp[p,i,w]) = 1;
 
  subject to AllToLast{ p in 1..np}:
 sum{i in 1..N}(bwp[p,i,W]) >= 1;
 
 /*
 subject to bStayTrue{p in 1..np, i in 1..N-1, w in 1..W}:
 bwp[p, i+1, W] >= bwp[p,i,w];
 */
 /*
 subject to timeBeforeWP{w in 1..W-1}:
 theta[w] = sum{p in 1..np, i in 1..N}(i*bwp[p,i,w]);
 */
 
 subject to finnishTime1{p in 1..np, i in 0..N}:
 theta_finnish[p] <= M_finnish * (1 - bwp[p,i,W]) + i*bwp[p,i,W];
 
 subject to finnishTime2{p in 1..np, i in 0..N}:
 theta_finnish[p] >= (i + 1) * (1 - bwp[p,i,W]);
 
 subject to timePennalty1{p in 1..np}:
 eta_finnish >= theta_finnish[p];
 
 subject to timePennalty2:
 J_finnish = gamma_finnish * eta_finnish;
 
 

 #Set reactor readable vars

 subject to xPos{p in 1..np, i in 0..N}:
 x[p,i] = pos[p,i,1];
 subject to yPos{p in 1..np, i in 0..N}:
 y[p,i] = pos[p,i,2];
 subject to zPos{p in 1..np, i in 0..N}:
 z[p,i] = pos[p,i,3];


#anti collision constraints

subject to antiCollisionX1{p in 1..np-1, q in p+1..np, i in 1..N} :
pos[p,i,1] - pos[q,i,1] >= dx - MCol * bCol[p,q,i,1,1];
subject to antiCollisionX2{p in 1..np-1, q in p+1..np, i in 1..N} :
pos[p,i,1] - pos[q,i,1] <= MCol * bCol[p,q,i,1,2] - dx;

subject to antiCollisionY1{p in 1..np-1, q in p+1..np, i in 1..N} :
pos[p,i,2] - pos[q,i,2] >= dy - MCol * bCol[p,q,i,2,1];
subject to antiCollisionY2{p in 1..np-1, q in p+1..np, i in 1..N} :
pos[p,i,2] - pos[q,i,2] <= MCol * bCol[p,q,i,2,2] - dy;

subject to antiCollisionZ1{p in 1..np-1, q in p+1..np, i in 1..N} :
pos[p,i,3] - pos[q,i,3] >= dz - MCol * bCol[p,q,i,3,1];
subject to antiCollisionZ2{p in 1..np-1, q in p+1..np, i in 1..N} :
pos[p,i,3] - pos[q,i,3] <= MCol * bCol[p,q,i,3,2] - dz;

subject to antiCollisionSum{p in 1..np-1, q in p+1..np, i in 1..N} :
sum{row in 1..3, col in 1..2}(bCol[p,q,i,row,col]) <= 5;


 #Data gathering constraints

subject to gather1{p in 1..np, i in 1..N, t in 1..W-1}:
lambda_sensor[p,i,t] <= sum{k in 1..i}(bwp[p,k,t]);

subject to gather2{p in 1..np, i in 1..N, t in 1..W-1}:
lambda_sensor[p,i,t] <= sum{k in i..N}(bwp[p,k,t]);

subject to gather3{p in 1..np, t in 1..W-1}:
sum{i in 1..N}(lambda_sensor[p,i,t]) = sum{i in 1..N}(i * bwp[p,i,1]) - sum{i in 1..N}((i + 1) * bwp[p,i,W]);

subject to sensorActive{p in 1..np, i in 1..N}:
sum{t in 1..W}(lambda_sensor[p,i,t]) = bsensor[p,i];

#Dataflow constraints




#connectivity constraints

#subject to connectivityDistance{p in 1..np, q in 1..np, i in 0..N}:
#UAV_distance[p,q,i] = 