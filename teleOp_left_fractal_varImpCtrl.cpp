//  (C) 2001-2018 Force Dimension
//  All Rights Reserved.
//
//  Version 3.7.3

// TELEMANIPULATION PROJECT
// author: KEYHAN KOUHKILOUI

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "dhdc.h"
#include "drdc.h"

#include "/home/anymal/keyhan_teleOp_ws/external/Eigen/Eigen/Dense"
#include "/home/anymal/keyhan_teleOp_ws/external/Eigen/Eigen/Eigenvalues"
#include "/home/anymal/keyhan_teleOp_ws/external/Eigen/unsupported/Eigen/MatrixFunctions"

#define REFRESH_INTERVAL  0.005   // sec
#define _USE_MATH_DEFINES

// HOLDING GRIPPER FOR FIC ACTIVATION
float gripperAct = 1;

Eigen::MatrixXf inertiaMat(6,6);
Eigen::VectorXf jointAngles(DHD_MAX_DOF);

Eigen::VectorXf curPos(3);
Eigen::VectorXf curOri(3);
Eigen::VectorXf desPos(3);
Eigen::VectorXf desOri(3);

Eigen::VectorXf curVelLinear(3);
Eigen::VectorXf curVelAngular(3);

Eigen::VectorXf posError(3);
Eigen::VectorXf oriError(3);
Eigen::VectorXf velLinearError(3);
Eigen::VectorXf velAngularError(3);

Eigen::VectorXf prePosError(3);
Eigen::VectorXf preOriError(3);
Eigen::VectorXf preVelLinearError(3);
Eigen::VectorXf preVelAngularError(3);

Eigen::VectorXf curThumbPos(3);
Eigen::VectorXf curFingerPos(3);
Eigen::VectorXf thumbFingerPosError(3);
Eigen::VectorXf gripperGapVec(1);

Eigen::MatrixXf kConstPos(3,3);
Eigen::MatrixXf kConstOri(3,3);
Eigen::MatrixXf kConstThumbFing(3,3);
Eigen::MatrixXf kVarPos(3,3);
Eigen::MatrixXf kVarOri(3,3);
Eigen::MatrixXf kTotalDesPos(3,3);
Eigen::MatrixXf kTotalDesOri(3,3);
Eigen::MatrixXf kTotal6x6(6,6);

Eigen::VectorXf Beta(3);
Eigen::VectorXf BetaOri(3);

Eigen::MatrixXf dConstPos(3,3);
Eigen::MatrixXf dConstOri(3,3);
Eigen::MatrixXf dConstThumbFing(3,3);

Eigen::VectorXf F_haptic(3);
Eigen::VectorXf Tau_haptic(3);
Eigen::VectorXf F_haptic_thumbFing(3);

Eigen::VectorXf Pmid3x1(3);
Eigen::VectorXf Pmax3x1(3);
Eigen::VectorXf Pmax3x1_Ori(3);
Eigen::VectorXf Pmid3x1_Ori(3);
Eigen::MatrixXf K1_3x3(3,3);
Eigen::MatrixXf K2_3x3(3,3);
Eigen::MatrixXf K1_Ori_3x3(3,3);
Eigen::MatrixXf K2_Ori_3x3(3,3);
Eigen::MatrixXf Kc_3x3(3,3);
Eigen::MatrixXf Kc_Ori_3x3(3,3);
Eigen::MatrixXf KdMax(3,3);
Eigen::MatrixXf KdMax_Ori(3,3);
Eigen::VectorXf maxDistVec_3x1(3);
Eigen::VectorXf maxDistVec_Ori_3x1(3);

Eigen::VectorXf Fmax(3);
Eigen::VectorXf maxDisp(3);
Eigen::VectorXf TauMax(3);
Eigen::VectorXf maxDisp_Ori(3);
Eigen::MatrixXf KmaxSystem(3,3);
Eigen::VectorXf beta(3);
Eigen::MatrixXf KmaxSystem_Ori(3,3);
Eigen::VectorXf beta_Ori(3);

Eigen::VectorXf FkCtrl(3);
Eigen::VectorXf Fmid_Ctrl(3);
Eigen::VectorXf TauCtrl(3);
Eigen::VectorXf TauMid_Ctrl(3);
Eigen::VectorXf FdCtrl(3);
Eigen::VectorXf Tau_dCtrl(3);

class teleOp{

public:

	// TEST CLASS
	void printSomething();

	// CONTROLLER CLASSES
	void calculate_Kmax_Beta_Pos_Ori(Eigen::VectorXf &F_MAX, Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &TAU_MAX, Eigen::VectorXf &MAX_DISP_ORI, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::VectorXf &BETA, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, Eigen::VectorXf &BETA_ORI, int i);
	void get_K_Total_Pos_Ori(Eigen::VectorXf &BETA, Eigen::MatrixXf &K_VAR_POS, Eigen::MatrixXf &K_TOTAL_DES_POS, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_VAR_ORI, Eigen::MatrixXf &K_TOTAL_DES_ORI, int i);
	void calculate_F_Max(Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &F_MAX, int i);
	void calculate_Tau_Max(Eigen::VectorXf &MAX_DISP_ORI, Eigen::VectorXf &TAU_MAX, int i);
	void calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_POS(Eigen::MatrixXf &K1, Eigen::MatrixXf &K2, Eigen::MatrixXf &KdMAX, Eigen::VectorXf &PMAX, Eigen::VectorXf &PMID, Eigen::VectorXf &MAX_DISP_VEC, int i);
	void calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_ORI(Eigen::MatrixXf &K1_ORI, Eigen::MatrixXf &K2_ORI, Eigen::MatrixXf &KdMAX_ORI, Eigen::VectorXf &PMAX_ORI, Eigen::VectorXf &PMID_ORI, Eigen::VectorXf &MAX_DISP_VEC_ORI, int i);
	void calculate_Kc_constDampLinear_FmidCtrl_Fctrl(Eigen::MatrixXf &Kc, Eigen::MatrixXf &constDampLinear, Eigen::VectorXf &F_MID_CTRL, Eigen::VectorXf &F_CTRL, int i);
	void calculate_Kc_constDampAngular_FmidCtrl_Fctrl_ORI(Eigen::MatrixXf &Kc_ORI, Eigen::MatrixXf &constDampAngular, Eigen::VectorXf &F_MID_CTRL_ORI, Eigen::VectorXf &F_CTRL_ORI, int i);

	template <typename T> static int sgn (const T &val, const T &eps = T(0)){
		return (T(eps) < val) - (val < T(-eps));
	} 
};

	
	void teleOp::printSomething(){
		//std::cout << "Class Working!!!" << std::endl;
		printf ("Class Working!!!\n");
	}

	void teleOp::calculate_Kmax_Beta_Pos_Ori(Eigen::VectorXf &F_MAX, Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &TAU_MAX, Eigen::VectorXf &MAX_DISP_ORI, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::VectorXf &BETA, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, Eigen::VectorXf &BETA_ORI, int i){

	K_MAX_SYSTEM(i,i) = F_MAX[i]/MAX_DISP[i];
	BETA[i] = sqrt(log(K_MAX_SYSTEM(i,i) - kConstPos(i,i))/pow(MAX_DISP[i], 2));
	if (std::isnan(BETA[i])){
		BETA[i] = 0;
	}

	K_MAX_SYSTEM_ORI(i,i) = TAU_MAX[i]/MAX_DISP_ORI[i];
	BETA_ORI[i] = sqrt(log(K_MAX_SYSTEM_ORI(i,i) - kConstOri(i,i))/pow(MAX_DISP_ORI[i], 2));
	if (std::isnan(BETA_ORI[i])){
		BETA_ORI[i] = 0;
	}
}

	void teleOp::get_K_Total_Pos_Ori(Eigen::VectorXf &BETA, Eigen::MatrixXf &K_VAR_POS, Eigen::MatrixXf &K_TOTAL_DES_POS, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_VAR_ORI, Eigen::MatrixXf &K_TOTAL_DES_ORI, int i){
	
	K_VAR_POS(i,i) = exp(pow(BETA[i] * posError[i], 2)) - 1;
	K_TOTAL_DES_POS(i,i) = kConstPos(i,i) + K_VAR_POS(i,i);

	K_VAR_ORI(i,i) = exp(pow(BETA_ORI[i] * oriError[i], 2)) - 1;
	K_TOTAL_DES_ORI(i,i) = kConstOri(i,i) + K_VAR_ORI(i,i);
}
		
	void teleOp::calculate_F_Max(Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &F_MAX, int i){
	
	if (MAX_DISP[i] >= 0.075){
		F_MAX[i] = 20;
	}else if (MAX_DISP[i] >= 0.05 && MAX_DISP[i] < 0.075){
		F_MAX[i] = 18;
	}else if (MAX_DISP[i] >= 0.01 && MAX_DISP[i] < 0.05){
		F_MAX[i] = 16;
	}
}

	void teleOp::calculate_Tau_Max(Eigen::VectorXf &MAX_DISP_ORI, Eigen::VectorXf &TAU_MAX, int i){

	if (MAX_DISP_ORI[i] >= 0.086){ 
		TAU_MAX[i] = 0.3;
	}else if (MAX_DISP_ORI[i] >= 0.016 && MAX_DISP_ORI[i] < 0.086){
		TAU_MAX[i] = 0.125;
	}
}

	void teleOp::calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_POS(Eigen::MatrixXf &K1, Eigen::MatrixXf &K2, Eigen::MatrixXf &KdMAX, Eigen::VectorXf &PMAX, Eigen::VectorXf &PMID, Eigen::VectorXf &MAX_DISP_VEC, int i){

	if (fabs(posError[i]) > fabs(prePosError[i])){
		MAX_DISP_VEC[i] = curPos[i] - desPos[i];
		KdMAX(i,i) = kConstPos(i,i) + kVarPos(i,i);
		//std::cout << "Condition ==> 1" << std::endl;
	}else{
		MAX_DISP_VEC[i] = MAX_DISP_VEC[i]; 
		KdMAX(i,i) = KdMAX(i,i);
		//std::cout << "Condition ==> 2" << std::endl;
	}

	PMAX[i] = desPos[i] + MAX_DISP_VEC[i];
	PMID[i] = desPos[i] + (PMAX[i] - desPos[i])/2;

	K1(i,i) = 2 * KdMAX(i,i);
	K2(i,i) = 2 * KdMAX(i,i);
}

	
	void teleOp::calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_ORI(Eigen::MatrixXf &K1_ORI, Eigen::MatrixXf &K2_ORI, Eigen::MatrixXf &KdMAX_ORI, Eigen::VectorXf &PMAX_ORI, Eigen::VectorXf &PMID_ORI, Eigen::VectorXf &MAX_DISP_VEC_ORI, int i){
		
	if (fabs(oriError[i]) > fabs(preOriError[i])){
		MAX_DISP_VEC_ORI[i] = curOri[i] - desOri[i];
		KdMAX_ORI(i,i) = kConstOri(i,i) + kVarOri(i,i);
	}else{
		MAX_DISP_VEC_ORI[i] = MAX_DISP_VEC_ORI[i]; 
		KdMAX_ORI(i,i) = KdMAX_ORI(i,i);
	}

	PMAX_ORI[i] = desOri[i] + MAX_DISP_VEC_ORI[i];
	PMID_ORI[i] = desOri[i] + (PMAX_ORI[i] - desOri[i])/2;

	K1_ORI(i,i) = 2 * KdMAX_ORI(i,i);
	K2_ORI(i,i) = 2 * KdMAX_ORI(i,i);
}

	
	void teleOp::calculate_Kc_constDampLinear_FmidCtrl_Fctrl(Eigen::MatrixXf &Kc, Eigen::MatrixXf &constDampLinear, Eigen::VectorXf &F_MID_CTRL, Eigen::VectorXf &F_CTRL, int i){

	if (sgn(posError[i], 0.1f) == sgn(velLinearError[i], 0.5f)){
		Kc.setZero();
		constDampLinear(i,i) = 0;
		F_MID_CTRL[i] = 0;
	}else{
		Kc = K2_3x3;
		F_MID_CTRL = Kc * (Pmid3x1 - curPos);
		F_CTRL[i] = 0;
	}
}

	void teleOp::calculate_Kc_constDampAngular_FmidCtrl_Fctrl_ORI(Eigen::MatrixXf &Kc_ORI, Eigen::MatrixXf &constDampAngular, Eigen::VectorXf &F_MID_CTRL_ORI, Eigen::VectorXf &F_CTRL_ORI, int i){

		if (sgn(oriError[i], 10.1f) == sgn(velAngularError[i], 10.5f)){
		Kc_ORI.setZero();
		constDampAngular(i,i) = 0;
		F_MID_CTRL_ORI[i] = 0;
	}else{
		Kc_ORI = K2_Ori_3x3;
		F_MID_CTRL_ORI = Kc_ORI * (Pmid3x1_Ori - curOri);
		F_CTRL_ORI[i] = 0;
	}
}

int main (int  argc, char **argv) {
	
	teleOp teleOp;

	// PARAMETERS INITIALIZATION

	inertiaMat.setZero();
	jointAngles.setZero();

	curPos.setZero(); 
	curOri.setZero();
	desPos.setZero();  
	desOri.setZero();

	curVelLinear.setZero();
	curVelAngular.setZero();

	posError.setZero();
	oriError.setZero();
	velLinearError.setZero();
	velAngularError.setZero();

	prePosError.setZero();
	preOriError.setZero();

	curThumbPos.setZero();
	curFingerPos.setZero();
	thumbFingerPosError.setZero();
	gripperGapVec.setZero();

	kConstPos.setIdentity();
	kConstOri.setIdentity();
	kConstThumbFing.setIdentity();
	kVarPos.setIdentity();
	kVarOri.setIdentity();
	kTotalDesPos.setIdentity();
	kTotalDesOri.setIdentity();
	kTotal6x6.setIdentity();
	dConstPos.setIdentity();
	dConstOri.setIdentity();
	dConstThumbFing.setIdentity();

	Beta.setZero();
	BetaOri.setZero();

	F_haptic.setZero();
	Tau_haptic.setZero();
	F_haptic_thumbFing.setZero();

	Pmid3x1.setZero();
	Pmax3x1.setZero();
	Pmid3x1_Ori.setZero();
	Pmax3x1_Ori.setZero();
	K1_3x3.setIdentity();
	K2_3x3.setIdentity();
	K1_Ori_3x3.setIdentity();
	K2_Ori_3x3.setIdentity();
	Kc_3x3.setIdentity();
	Kc_Ori_3x3.setIdentity();
	KdMax.setIdentity();
	KdMax_Ori.setIdentity();
	maxDistVec_3x1.setZero();
	maxDistVec_Ori_3x1.setZero();
 
	Fmax.setZero();
	maxDisp.setZero();
	TauMax.setZero();
	maxDisp_Ori.setZero();
	KmaxSystem.setIdentity();
	beta.setZero();
	KmaxSystem_Ori.setIdentity();
	beta_Ori.setZero();

	FkCtrl.setZero();
	Fmid_Ctrl.setZero();
	TauCtrl.setZero();
	TauMid_Ctrl.setZero();
	FdCtrl.setZero();
	Tau_dCtrl.setZero();

	maxDisp.setConstant(0.025);
	maxDisp_Ori.setConstant(0.174533);
// 0.349066 ==> 20 degree
// 0.3054326 ==> 17.5 degree
// 0.261799 ==> 15 degree
// 0.2181662 ==> 12.5 degree
// 0.174533 ==> 10 degree
// 0.1309 ==> 7.5 degree
// 0.0872665 ==> 5 degree
// 0.0436332 ==> 2.5 degree
// 0.0174533 ==> 1 degree

	// PARAMETERS FOR RETRIEVING REQUIRED INFORMATION
	double gripperGap;
	double px, py, pz;
	double oxRad, oyRad, ozRad;
  double fx, fy, fz;
	double tauX, tauY, tauZ;
	double f_fingThumbX, f_fingThumbY, f_fingThumbZ;
	double thumbPosX, thumbPosY, thumbPosZ;
	double fingerPosX, fingerPosY, fingerPosZ;
 	double velLinearX, velLinearY, velLinearZ;
	double velAngularX, velAngularY, velAngularZ; 

	double k_fingThumb = 25, d_fingThumb = 5;

	double t1,t0  = dhdGetTime ();

  int done   = 0;
	int ctrlSelect = 1;

  // message
  int major, minor, release, revision;
  dhdGetSDKVersion (&major, &minor, &release, &revision);
  printf ("\n");
  printf ("Force Dimension - Gravity Compensation Example %d.%d.%d.%d\n", major, minor, release, revision);
  printf ("(C) 2001-2018 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // open the first available device
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  // identify device
  printf ("%s device detected\n\n", dhdGetSystemName());

  // display instructions
  printf ("press 'q' to quit\n\n");

  // enable force
  dhdEnableForce (DHD_ON);

	// TEST CLASS
	//teleOp.printSomething();

  // haptic loop
  while (!done) {

	// DESIRED POSISTION AND ORIENTATION OF HAPTIC DEVICE
	desPos[0] = 0; desPos[1] = 0; desPos[2] = 0; desOri[0] = 0; desOri[1] = 0; desOri[2] = 0;

	// CURRENT POSITION AND ORIENTATION OF HAPTIC DEVICE
	curPos[0] = px; curPos[1] = py; curPos[2] = pz; curOri[0] = oxRad; curOri[1] = oyRad; curOri[2] = ozRad;

	// CURRENT LINEAR AND ANGULAR VELOCITY OF HAPTIC DEVICE
	curVelLinear[0] = velLinearX; curVelLinear[1] = velLinearY; curVelLinear[2] = velLinearZ; curVelAngular[0] = velAngularX; curVelAngular[1] = velAngularY; curVelAngular[2] = velAngularZ;

	// CURRENT THUMB AND FINGER POSITION OF HPTIC DEVICE
	curThumbPos[0] = thumbPosX; curThumbPos[1] = thumbPosY; curThumbPos[2] = thumbPosZ; curFingerPos[0] = fingerPosX; curFingerPos[2] = fingerPosY; curFingerPos[2] = fingerPosZ;

	// CURRENT GRIPPER GAP
	gripperGapVec[0] = gripperGap;

	// START OF MAIN FOR LOOP
	for (int i = 0; i < 3; i++){

	// DEFINING END-EFFECTOR POSE ERROR
	posError[i] = desPos[i] - curPos[i];
	oriError[i] = desOri[i] - curOri[i];
	thumbFingerPosError[i] = curThumbPos[i] - curFingerPos[i];
	
	// DEFINING END-EFFECTOR VELOCITY ERRORS (LINEAR AND ANGULAR)	
	velLinearError[i] = - curVelLinear[i];
	velAngularError[i] = - curVelAngular[i];
			
	// SETTING CONSTANT STIFFNESS AND DAMPING
	kConstPos(i,i) = 100; kConstOri(i,i) = 0.1;
	dConstPos(i,i) = 0; dConstOri(i,i) = 0.01;

	kConstThumbFing(i,i) = 25;
	dConstThumbFing(i,i) = 5;

	// F_MAX AND TAU_MAX CALCULATION
	teleOp.calculate_F_Max(maxDisp, Fmax, i);
	teleOp.calculate_Tau_Max(maxDisp_Ori, TauMax, i);

	// K_SYSTEM_MAX, MAXIMUM DISPLACEMENT AND BETA CALCULATION FOR BOTH END-EFFECTOR POSITION AND ORIENTATION 	
	teleOp.calculate_Kmax_Beta_Pos_Ori(Fmax, maxDisp, TauMax, maxDisp_Ori, KmaxSystem, beta, KmaxSystem_Ori, beta_Ori, i);

	// K_TOTAL AND K_VAR CALCULATION FOR BOTH POSITION AND ORIENTATION
	teleOp.get_K_Total_Pos_Ori(beta, kVarPos, kTotalDesPos, beta_Ori, kVarOri, kTotalDesOri, i);

	// APPLYING THE SYSTEM PHYSICAL LIMITATION CONDITION
	if (kConstPos(i,i) + kVarPos(i,i) > KmaxSystem(i,i) && beta[i] > 0){
		kVarPos(i,i) = Fmax[i]/fabs(posError[i] - kConstPos(i,i));
	}else if (kTotalDesPos (i,i) > KmaxSystem(i,i)){
		kVarPos(i,i) = 0;
	}

	if (kConstOri(i,i) + kVarOri(i,i) > KmaxSystem_Ori(i,i) && beta_Ori[i] > 0){
		kVarOri(i,i) = TauMax[i]/fabs(oriError[i] - kConstOri(i,i));
	}else if (kTotalDesOri (i,i) > KmaxSystem_Ori(i,i)){
		kVarOri(i,i) = 0;
	}

	// ATTRACTIVE FORCE FOR POSITION AND ORINETATION CTRL CALCULATION
	FkCtrl[i] = (kConstPos(i,i) + kVarPos(i,i)) * posError[i];
	TauCtrl[i] = (kConstOri(i,i) + kVarOri(i,i)) * oriError[i];

	// DAMPING FORCES FOR LINEAR AND ANGULAR VELOCITIES - DEFINITIONS
	FdCtrl[i] = dConstPos(i,i) * velLinearError[i];
	Tau_dCtrl[i] = dConstOri(i,i) * velAngularError[i];

	// // // // // // // // // // // // // // // // // // // // // // // // PROPOSED METHOD - END-EFFECTOR POSITION // // // // // // // // // // // // // // // // // // // // // // // //
	teleOp.calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_POS(K1_3x3, K2_3x3, KdMax, Pmax3x1, Pmid3x1, maxDistVec_3x1, i);

	// // // // // // // // // // // // // // // // // // // // // // // // PROPOSED METHOD - END-EFFECTOR ORIENTATION // // // // // // // // // // // // // // // // // // // // // // // 
	teleOp.calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_ORI(K1_Ori_3x3, K2_Ori_3x3, KdMax_Ori, Pmax3x1_Ori, Pmid3x1_Ori, maxDistVec_Ori_3x1, i);

	// // // // // // // // // // // // // // // // // // DEFINING THE CONDITIONS FOR THE PROPOSED METHOD - END-EFFECTOR POSITION // // // // // // // // // // // // // // // // // // // //
	teleOp.calculate_Kc_constDampLinear_FmidCtrl_Fctrl(Kc_3x3, dConstPos, Fmid_Ctrl, FkCtrl, i);

	// // // // // // // // // // // // // // // // // // DEFINING THE CONDITIONS FOR THE PROPOSED METHOD - END-EFFECTOR ORIENTATION // // // // // // // // // // // // // // // // // // //
	teleOp.calculate_Kc_constDampAngular_FmidCtrl_Fctrl_ORI(Kc_Ori_3x3, dConstOri, TauMid_Ctrl, TauCtrl, i);

	// CONDITION ON GENERATED FORCE
	if (FkCtrl[i] >= Fmax[i]){
		FkCtrl[i] = Fmax[i];
		Fmid_Ctrl[i] = 0;
	}

	if (Fmid_Ctrl[i] >= Fmax[i]){
		Fmid_Ctrl[i] = Fmax[i];
		FkCtrl[i] = 0;
	}

	// CONDITION ON GENERATED TORQUES
	if (TauCtrl[i] >= TauMax[i]){
		TauCtrl[i] = TauMax[i];
		TauMid_Ctrl[i] = 0;
	}

	if (TauMid_Ctrl[i] >= TauMax[i]){
		TauMid_Ctrl[i] = TauMax[i];
		TauCtrl[i] = 0;
	}


	// TOTAL SYSTEM STIFFNESS PROFILE
	if (teleOp.sgn(posError[i], 0.5f) == teleOp.sgn(velLinearError[i], 0.5f)){
		kTotal6x6(i,i) = kConstPos(i,i) + kVarPos(i,i);
	}else{
		kTotal6x6(i,i) = Kc_3x3(i,i);
	}

	if (teleOp.sgn(oriError[i], 0.5f) == teleOp.sgn(velAngularError[i], 0.5f)){
		kTotal6x6(i+3,i+3) = kConstOri(i,i) + kVarOri(i,i);
	}else{
		kTotal6x6(i+3,i+3) = Kc_Ori_3x3(i,i);
	}

		// CONTROL PARAMETERS FOR TRADITIONAL CTRL
		Beta[i] = 1;
		BetaOri[i] = 0.01;
		// UNCOMMENT FOR TRADITIONAL IMPEDANCE CONTROL 
		/*kVarPos(i,i) = exp(pow(Beta[i] * (posError[i]),2)) - 1;
		kVarOri(i,i) = exp(pow(BetaOri[i] * (oriError[i]),2)) - 1;*/
					
		// TRADITIONAL CTRL
		F_haptic[i] = (kConstPos(i,i) + kVarPos(i,i)) * posError[i] + FdCtrl[i];
		Tau_haptic[i] = (kConstOri(i,i) + kVarOri(i,i)) *  oriError[i] + Tau_dCtrl[i];
		F_haptic_thumbFing[i] = kConstThumbFing(i,i) * thumbFingerPosError[i];
		
		if (thumbFingerPosError[i] >= 0){
			F_haptic_thumbFing[i] = - F_haptic_thumbFing[i];
		}else{
			F_haptic_thumbFing[i] = F_haptic_thumbFing[i];
		}

		//if()

	 // HAPTIC FORCE AND TORQUE CTRL SELECTION
    if (ctrlSelect == 0){
         
        // TRADITIONAL CTRL
        if (dhdSetForceAndTorqueAndGripperForce (F_haptic[0], F_haptic[1], F_haptic[2], Tau_haptic[0], Tau_haptic[1], Tau_haptic[2], 1.0) < DHD_NO_ERROR) {
        //if (dhdSetForceAndTorqueAndGripperForce (FkCtrl[0], FkCtrl[1], FkCtrl[2], Tau_haptic[0], Tau_haptic[1], Tau_haptic[2], 1.0) < DHD_NO_ERROR) {
        printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        done = 1;
    }
     
    }else{
         
        if (gripperAct == 1){           
            // COMMENT/UNCOMMENT IF USING GRIPPER FOR SWITCHING BETWEEN GRAVITY COMPENSATION AND CONTROLLER ACTIVATION
            if (gripperGapVec[0] <= -0.001){
                 
                // GRAVITY COMPENSATION + CONTROLLER NOT ACTIVE (GRIPPER CASE)
                if (dhdSetForceAndTorqueAndGripperForce (0, 0, 0, 0, 0, 0, F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]) < DHD_NO_ERROR) {
        printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        done = 1;
            }
            }else{
           
                // PROPOSED METHOD + CONTROLLER ACTIVE (GRIPPER CASE), (DAMPING FORCE ==> FdCtrl[i] (ADD, IF NEEDED))
                if (dhdSetForceAndTorqueAndGripperForce (FkCtrl[0] + Fmid_Ctrl[0] + FdCtrl[0], FkCtrl[1] + Fmid_Ctrl[1] + FdCtrl[1], FkCtrl[2] + Fmid_Ctrl[2] + FdCtrl[2], TauCtrl[0] + TauMid_Ctrl[0] + Tau_dCtrl[0], TauCtrl[1] + TauMid_Ctrl[1] + Tau_dCtrl[1], TauCtrl[2] + TauMid_Ctrl[2] + Tau_dCtrl[2], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]) < DHD_NO_ERROR) {
        printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        done = 1;
        }
        }
    }else{
 
            // COMMENT/UNCOMMENT IF (NOT) USING GRIPPER FOR SWITCHING BETWEEN GRAVITY COMPENSATION AND CONTROLLER ACTIVATION
            // PROPOSED METHOD (DAMPING FORCE ==> FdCtrl[i] (ADD, IF NEEDED))
            if (dhdSetForceAndTorqueAndGripperForce (FkCtrl[0] + Fmid_Ctrl[0] + FdCtrl[0], FkCtrl[1] + Fmid_Ctrl[1] + FdCtrl[1], FkCtrl[2] + Fmid_Ctrl[2] + FdCtrl[2], TauCtrl[0] + TauMid_Ctrl[0] + Tau_dCtrl[0], TauCtrl[1] + TauMid_Ctrl[1] + Tau_dCtrl[1], TauCtrl[2] + TauMid_Ctrl[2] + Tau_dCtrl[2], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]) < DHD_NO_ERROR) {
        printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        done = 1;
        }
        }
             
    }
} // END OF MAIN FOR LOOP

			prePosError = posError;
			preOriError = oriError;
			preVelLinearError = velLinearError;
			preVelAngularError = velAngularError;

    // display refresh rate and position at 10Hz
    t1 = dhdGetTime ();
    if ((t1-t0) > REFRESH_INTERVAL) {

      // update timestamp
      t0 = t1;

			// retrieve thumb position 
      if (dhdGetGripperThumbPos (&thumbPosX, &thumbPosY, &thumbPosZ) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

			// retrieve finger position   
      if (dhdGetGripperFingerPos (&fingerPosX, &fingerPosY, &fingerPosZ) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }   

			// retrieve gripper gap  
      if (dhdGetGripperGap (&gripperGap) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }   

      // retrieve position
      if (dhdGetPosition (&px, &py, &pz) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

			// retrieve orientation Rad // dhdGetOrientationRad
      if (dhdGetOrientationRad (&oxRad, &oyRad, &ozRad) < DHD_NO_ERROR) {
        printf ("error: cannot read position (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

			// retrieve linear velocity
      if (dhdGetLinearVelocity (&velLinearX, &velLinearY, &velLinearZ) < DHD_NO_ERROR) {
        printf ("error: cannot read linear velocity (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

			// retrieve angular velocity //
      if (dhdGetAngularVelocityRad (&velAngularX, &velAngularY, &velAngularZ) < DHD_NO_ERROR) {
        printf ("error: cannot read linear velocity (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

      // retrieve force
      if (dhdGetForceAndTorque(&fx, &fy, &fz, &tauX, &tauY, &tauZ) < DHD_NO_ERROR) {
        printf ("error: cannot read force (%s)\n", dhdErrorGetLastStr());
        done = 1;
      }

      // display status
   //printf ("posError (%+0.03f %+0.03f %+0.03f) m | angleError (%+0.03f %+0.03f %+0.03f) rad | vel (%+0.03f %+0.03f %+0.03f) m/s | force (%+0.01f %+0.01f %+0.01f) N | torque (%+0.01f %+0.01f %+0.01f) Nm  |  frequency %0.02f kHz\r", 0.000 - px, 0.000 - py, 0.000 - pz, 0.000 - oxRad, 0.000 - oyRad, 0.000 - ozRad, velLinearX, velLinearY, velLinearZ, fx, fy, fz, tauX, tauY, tauZ, dhdGetComFreq());

    printf ("posError (%+0.03f %+0.03f %+0.03f) m | angleError (%+0.03f %+0.03f %+0.03f) rad | vel (%+0.03f %+0.03f %+0.03f) m/s | force (%+0.01f %+0.01f %+0.01f) N | torque (%+0.01f %+0.01f %+0.01f) Nm  |  frequency %0.02f kHz\r", 0.000 - px, 0.000 - py, 0.000 - pz, 0.000 - oxRad, 0.000 - oyRad, 0.000 - ozRad, velLinearX, velLinearY, velLinearZ, FkCtrl[0] + Fmid_Ctrl[0], FkCtrl[1] + Fmid_Ctrl[1], FkCtrl[2] + Fmid_Ctrl[2], TauCtrl[0] + TauMid_Ctrl[0] + Tau_dCtrl[0], TauCtrl[1] + TauMid_Ctrl[1] + Tau_dCtrl[1], TauCtrl[2] + TauMid_Ctrl[2] + Tau_dCtrl[2], dhdGetComFreq());

			//std::cout << "gripperGapVec ==> " << gripperGapVec[0] << std::endl;
      // user input
      if (dhdKbHit() && dhdKbGet() == 'q') done = 1;
    }
  }

	
  // close the connection
  dhdClose ();

  // happily exit
  printf ("\ndone.\n");
  return 0;
}
