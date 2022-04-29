// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // NEW PART OF THE CODE // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <atomic>
#include <cmath>
#include <array>
#include <chrono>
//#include "/home/advr/WorkspaceUoE/src/uoe_franka_control/include/uoe_franka_control/CircularBuffer.hpp"
#include "/home/advr/WorkspaceUoE/src/uoe_franka_control/include/uoe_franka_control/TimeSeriesBuffer.hpp"

#include <omd/opto.h>
#include <omd/sensorconfig.h>
#include <omd/optopackage.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/rate_limiting.h>

#include <dhdc.h>
#include <drdc.h>

#include <uoe_maths/Angle.h>
#include <uoe_maths/Clamp.h>
#include <uoe_maths/Deadband.h>
#include <uoe_maths/AxisAngle.h>
#include <uoe_maths/SplineCubic.hpp>
#include <uoe_maths/SplineQuintic.hpp>
#include <uoe_maths/FilterIIR.hpp>
#include <uoe_maths/Deadband.h>
#include <uoe_plot/Plot.hpp>
#include <uoe_model/Model.hpp>
#include <uoe_viewer_model/ViewerModel.hpp>
#include <uoe_viewer_model/DrawModel.h>
#include <uoe_utils/Filesystem.h>

#include "/home/advr/ForceDimension/sdk-3.7.3/external/Eigen/Eigen/Dense"
#include "/home/advr/ForceDimension/sdk-3.7.3/external/Eigen/Eigen/Eigenvalues"
#include "/home/advr/ForceDimension/sdk-3.7.3/external/Eigen/unsupported/Eigen/MatrixFunctions"

#define REFRESH_INTERVAL  0.001
#define _USE_MATH_DEFINES

#include <RhIO.hpp>

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // FRACTAL IMPEDANCE CONTROL FOR HAPTIC DEVICE ==> SIGMA 7 // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

//TEST
Eigen::Vector3d dualArmPos;
double inputRequestStage = 0;

// SOME VISUALIZATION REQUIRED PARAMETERS
double color1 = 0.0;
int count = 0;
Eigen::Vector3d targetPosSphere;
std::atomic<bool> goToMode(false);
std::atomic<bool> abortMotionKey(false);

// DATA PRITNING GLOBAL VARIABLES
double timeControllerGlobalVar;
Eigen::VectorXd tauCmdEigenGlobalVar(7);
Eigen::VectorXd readJointPosGlobalVar(7);
Eigen::VectorXd wrenchStiffnessGlobalVar(6);

// DEMO_exp 
float virtConstraintSize = 0.1;
float virtConstraintRotSize = 0.0872665;
double trajectoryActPhase = 0;

// 1.5708 ==> 90 degrees
// 1.0472 ==> 60 degrees
// 0.785398 ==> 45 degrees
// 0.523599==> 30 degrees
// 0.349066 ==> 20 degrees
// 0.3054326 ==> 17.5 degrees
// 0.261799 ==> 15 degrees
// 0.2181662 ==> 12.5 degrees
// 0.174533 ==> 10 degrees
// 0.1309 ==> 7.5 degrees
// 0.0872665 ==> 5 degrees
// 0.0436332 ==> 2.5 degrees
// 0.0174533 ==> 1 degree

// // // // // // // // // // // // // // // TIME DELAT ATTEMPT // // // // // // // // // // // // // // //
static double weightedAverage(
    double w1, const double& v1, 
    double w2, const double& v2)
{
    return w1*v1 + w2*v2;
}

Eigen::VectorXd MtoShapticPos_Raw(3);
Eigen::VectorXd MtoShapticPos_Delayed(3);
Eigen::VectorXd FeedBackForce_Raw(3);
Eigen::VectorXd FeedBackForce_Delayed(3);

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

// // // // // // // // // // // // // // // LOW-BANDWIDTH AND TIME-DELAY IMPLEMENTATION // // // // // // // // // // // // // // //
// DEMO
int countBuff = 0;
int vecSize = 1000;
int timeDelayBuffSize = 1000;

// FRANKA
Eigen::VectorXf buffVec_PosX(vecSize);
Eigen::VectorXf buffVec_PosY(vecSize); 
Eigen::VectorXf buffVec_PosZ(vecSize); 
Eigen::VectorXf buffVec_OriX(vecSize); 
Eigen::VectorXf buffVec_OriY(vecSize); 
Eigen::VectorXf buffVec_OriZ(vecSize); 
Eigen::VectorXf buffVec_6x1(6);

Eigen::VectorXf buffVec_Vel_PosX(vecSize);
Eigen::VectorXf buffVec_Vel_PosY(vecSize); 
Eigen::VectorXf buffVec_Vel_PosZ(vecSize); 
Eigen::VectorXf buffVec_Vel_OriX(vecSize); 
Eigen::VectorXf buffVec_Vel_OriY(vecSize); 
Eigen::VectorXf buffVec_Vel_OriZ(vecSize); 
Eigen::VectorXf buffVec_Vel_6x1(6);

Eigen::VectorXf velLinearError_Cur_Buff_Sel_Franka(3);

Eigen::VectorXf delayCurPosVec(3);
Eigen::VectorXf delayCurVelLinearVec(3);

// SIGMA 7
Eigen::VectorXf buffVec_PosX_Sigma(vecSize);
Eigen::VectorXf buffVec_PosY_Sigma(vecSize); 
Eigen::VectorXf buffVec_PosZ_Sigma(vecSize); 
Eigen::VectorXf buffVec_OriX_Sigma(vecSize); 
Eigen::VectorXf buffVec_OriY_Sigma(vecSize); 
Eigen::VectorXf buffVec_OriZ_Sigma(vecSize); 
Eigen::VectorXf buffVec_6x1_Sigma(6);

Eigen::VectorXf buffVec_Vel_PosX_Sigma(vecSize);
Eigen::VectorXf buffVec_Vel_PosY_Sigma(vecSize); 
Eigen::VectorXf buffVec_Vel_PosZ_Sigma(vecSize); 
Eigen::VectorXf buffVec_Vel_OriX_Sigma(vecSize); 
Eigen::VectorXf buffVec_Vel_OriY_Sigma(vecSize); 
Eigen::VectorXf buffVec_Vel_OriZ_Sigma(vecSize); 
Eigen::VectorXf buffVec_Vel_6x1_Sigma(6);

Eigen::VectorXf velLinearError_Cur_Buff_Sel(3);

//MASTER TO SLAVE POSITION SIGNAL WITH LOWBANDWIDTH
Eigen::VectorXf buffVec_MtoS_PosX(vecSize);
Eigen::VectorXf buffVec_MtoS_PosY(vecSize); 
Eigen::VectorXf buffVec_MtoS_PosZ(vecSize); 
Eigen::VectorXf buffVec_MtoS_OriX(vecSize); 
Eigen::VectorXf buffVec_MtoS_OriY(vecSize); 
Eigen::VectorXf buffVec_MtoS_OriZ(vecSize); 
Eigen::VectorXf buffVec_MtoS_6x1(6);

//SLAVE TO MASTER WRENCH SIGNAL WITH LOWBANDWIDTH
Eigen::VectorXf buffVec_Force_X(vecSize);
Eigen::VectorXf buffVec_Force_Y(vecSize); 
Eigen::VectorXf buffVec_Force_Z(vecSize); 
Eigen::VectorXf buffVec_Tau_X(vecSize); 
Eigen::VectorXf buffVec_Tau_Y(vecSize); 
Eigen::VectorXf buffVec_Tau_Z(vecSize); 
Eigen::VectorXf buffVec_Wrench_6x1(6);

//MASTER TO SLAVE POSITION SIGNAL WITH LOWBANDWIDTH & TIME-DELAY
Eigen::VectorXf buffVec_MtoS_PosX_lowBandWidth_timeDelay(vecSize);
Eigen::VectorXf buffVec_MtoS_PosY_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_MtoS_PosZ_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_MtoS_OriX_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_MtoS_OriY_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_MtoS_OriZ_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_MtoS_6x1_lowBandWidth_timeDelay(6);

//SLAVE TO MASTER WRENCH SIGNAL WITH LOWBANDWIDTH
Eigen::VectorXf buffVec_Force_X_lowBandWidth_timeDelay(vecSize);
Eigen::VectorXf buffVec_Force_Y_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_Force_Z_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_Tau_X_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_Tau_Y_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_Tau_Z_lowBandWidth_timeDelay(vecSize); 
Eigen::VectorXf buffVec_Wrench_6x1_lowBandWidth_timeDelay(6);

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

// 1.5708 ==> 90 degrees
// 1.0472 ==> 60 degrees
// 0.785398 ==> 45 degrees
// 0.523599 ==> 30 degrees
// 0.349066 ==> 20 degrees
// 0.3054326 ==> 17.5 degrees
// 0.261799 ==> 15 degrees
// 0.2181662 ==> 12.5 degrees
// 0.174533 ==> 10 degrees
// 0.1309 ==> 7.5 degrees
// 0.0872665 ==> 5 degrees
// 0.0436332 ==> 2.5 degrees
// 0.0174533 ==> 1 degree

// DEMO
float frankaDesPosHapticCtl = 1;

// DEMO 
float gripperAct = 1;
float hapticVirtConstFeedBackAct = 1;
float hapticOptoForceSensorAct = 1;

//DEMO
int lowBandwidth_Franka_FIC_Act = 0;
int lowBandwidth_Sigma_FIC_Act = 0;
// std_lowBandwidth_timeDelay_TeleOp_FIC_Act = 0 ==> STANDARD HIGH FREQUENCY CONTROL
// std_lowBandwidth_timeDelay_TeleOp_FIC_Act = 1 ==> LOW-BANDWIDTH FEEDBACK CONTROL BY CHANGING THE VECSIZE VALUE
// std_lowBandwidth_timeDelay_TeleOp_FIC_Act = 2 ==> TIME-DELAY FEEDBACK CONTROL BY SETTING THE SIZE OF BUFFER
// std_lowBandwidth_timeDelay_TeleOp_FIC_Act = 3 ==> LOW-BANDWIDTH FEEDBACK CONTROL BY CHANGING THE VECSIZE VALUE + TIME-DELAY FEEDBACK CONTROL BY SETTING THE SIZE OF BUFFER
int std_lowBandwidth_timeDelay_TeleOp_FIC_Act = 0;

//DEMO
int initial_configuration_Franka;

// DEMO
float optoFT_forceCoef = 0.5;
float optoFT_torqueCoef = 0.25;


double goTime = 5.0;

// DEMO 
float fractalImpedanceAct = 1;

double scalingFactorPos = 1;
double scalingFactorOri = 1;

Eigen::VectorXf curPos(3);
Eigen::VectorXf curOri(3);
Eigen::VectorXf desPos(3);
Eigen::VectorXf desOri(3);

Eigen::VectorXd hapticPos(3);
Eigen::VectorXd hapticOri(3);

Eigen::VectorXd MtoShapticPos(3);
Eigen::VectorXd MtoShapticOri(3);


Eigen::VectorXf curVelLinear(3);
Eigen::VectorXf curVelAngular(3);
Eigen::VectorXf velLinearErrorSquared(6);

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

Eigen::VectorXf potEnergy_6x1(6);

Eigen::VectorXf Wrench_FIC_Sigma(6);
Eigen::VectorXf preWrench_FIC_Sigma(6);
Eigen::VectorXf deltaWrench_Sigma(6);

Eigen::VectorXd Wrench_FIC_Sigma_doubleType(6);
Eigen::VectorXd preWrench_FIC_Sigma_doubleType(6);
Eigen::VectorXd deltaWrench_Sigma_doubleType(6);

Eigen::VectorXd div_conv_pos_ori_sigma(6);
std::atomic<double> div_conv_posX(0.0);
std::atomic<double> div_conv_posY(0.0);
std::atomic<double> div_conv_posZ(0.0);
std::atomic<double> div_conv_oriX(0.0);
std::atomic<double> div_conv_oriY(0.0);
std::atomic<double> div_conv_oriZ(0.0);

// SIGMA 7 FIC
class teleOp{

public:

	// TEST CLASS
	void printSomething();

	// CONTROLLER CLASSES
	void calculate_Kmax_Beta_Pos_Ori(Eigen::VectorXf &F_MAX, Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &TAU_MAX, Eigen::VectorXf &MAX_DISP_ORI, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::VectorXf &BETA, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, Eigen::VectorXf &BETA_ORI, int i);
	void get_K_Total_Pos_Ori(Eigen::VectorXf &BETA, Eigen::MatrixXf &K_VAR_POS, Eigen::MatrixXf &K_TOTAL_DES_POS, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_VAR_ORI, Eigen::MatrixXf &K_TOTAL_DES_ORI, int i);
	void get_K_Total_Pos_Ori_V2(Eigen::VectorXf &BETA, Eigen::MatrixXf &K_VAR_POS, Eigen::MatrixXf &K_TOTAL_DES_POS, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_VAR_ORI, Eigen::MatrixXf &K_TOTAL_DES_ORI, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, int i);
	void calculate_F_Max(Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &F_MAX, int i);
	void calculate_Tau_Max(Eigen::VectorXf &MAX_DISP_ORI, Eigen::VectorXf &TAU_MAX, int i);
	void calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_POS(Eigen::MatrixXf &K1, Eigen::MatrixXf &K2, Eigen::MatrixXf &KdMAX, Eigen::VectorXf &PMAX, Eigen::VectorXf &PMID, Eigen::VectorXf &MAX_DISP_VEC, Eigen::VectorXf &BETA, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &F_MAX,int i);
	void calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_ORI(Eigen::MatrixXf &K1_ORI, Eigen::MatrixXf &K2_ORI, Eigen::MatrixXf &KdMAX_ORI, Eigen::VectorXf &PMAX_ORI, Eigen::VectorXf &PMID_ORI, Eigen::VectorXf &MAX_DISP_VEC_ORI, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, Eigen::VectorXf &MAX_DISP_ORI, Eigen::VectorXf &TAU_MAX, int i);
	void calculate_Kc_constDampLinear_FmidCtrl_Fctrl(Eigen::MatrixXf &Kc, Eigen::MatrixXf &constDampLinear, Eigen::VectorXf &F_MID_CTRL, Eigen::VectorXf &F_CTRL, int i);
	void calculate_Kc_constDampAngular_FmidCtrl_Fctrl_ORI(Eigen::MatrixXf &Kc_ORI, Eigen::MatrixXf &constDampAngular, Eigen::VectorXf &F_MID_CTRL_ORI, Eigen::VectorXf &F_CTRL_ORI, int i);

	template <typename T> static int sgn (const T &val, const T &eps = T(0)){
		return (T(eps) < val) - (val < T(-eps));
	} 
};

	
void teleOp::printSomething(){
	printf ("TeleOp Class Working!!!\n");
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

void teleOp::get_K_Total_Pos_Ori_V2(Eigen::VectorXf &BETA, Eigen::MatrixXf &K_Var_Pos, Eigen::MatrixXf &K_TOTAL_DES_POS, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_VAR_ORI, Eigen::MatrixXf &K_TOTAL_DES_ORI, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, int i) {

  K_Var_Pos(i,i) = (K_MAX_SYSTEM(i,i) - kConstPos(i,i)) * fabs(tanh(pow(0.75 * BETA[i] * posError[i],2)));
  K_TOTAL_DES_POS(i,i) = kConstPos(i,i) + K_Var_Pos(i,i);

  K_VAR_ORI(i,i) = (K_MAX_SYSTEM_ORI(i,i) - kConstOri(i,i)) * fabs(tanh(pow(0.75 * BETA_ORI[i] * oriError[i],2)));
  K_TOTAL_DES_ORI(i,i) = kConstOri(i,i) + K_VAR_ORI(i,i);

}
		
void teleOp::calculate_F_Max(Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &F_MAX, int i){
	
	if (MAX_DISP[i] >= 0.075){
		F_MAX[i] = 20;
	}else if (MAX_DISP[i] >= 0.05 && MAX_DISP[i] < 0.075){
		F_MAX[i] = 17.5;
	}else if (MAX_DISP[i] >= 0.01 && MAX_DISP[i] < 0.05){
		F_MAX[i] = 15;
	}
	
	/*if (MAX_DISP[i] >= 0.075){
		F_MAX[i] = 20;
	}else if (MAX_DISP[i] >= 0.05 && MAX_DISP[i] < 0.075){
		F_MAX[i] = 18;
	}else if (MAX_DISP[i] >= 0.01 && MAX_DISP[i] < 0.05){
		F_MAX[i] = 16;
	}*/
}

void teleOp::calculate_Tau_Max(Eigen::VectorXf &MAX_DISP_ORI, Eigen::VectorXf &TAU_MAX, int i){

	if (MAX_DISP_ORI[i] >= 0.086){ 
		TAU_MAX[i] = 0.3;
	}else if (MAX_DISP_ORI[i] >= 0.016 && MAX_DISP_ORI[i] < 0.086){
		TAU_MAX[i] = 0.125;
	}
}

void teleOp::calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_POS(Eigen::MatrixXf &K1, Eigen::MatrixXf &K2, Eigen::MatrixXf &KdMAX, Eigen::VectorXf &PMAX, Eigen::VectorXf &PMID, Eigen::VectorXf &MAX_DISP_VEC, Eigen::VectorXf &BETA, Eigen::MatrixXf &K_MAX_SYSTEM, Eigen::VectorXf &MAX_DISP, Eigen::VectorXf &F_MAX,int i){

	//LOW-BANDWIDTH TEST
	if (fabs(posError[i]) > fabs(prePosError[i])){
		if (lowBandwidth_Sigma_FIC_Act == 0){
			MAX_DISP_VEC[i] = curPos[i] - desPos[i];
		}else{
			MAX_DISP_VEC[i] = buffVec_6x1_Sigma[i] - desPos[i];
		}
		KdMAX(i,i) = kConstPos(i,i) + kVarPos(i,i);
		//std::cout << "Condition ==> 1" << std::endl;

	}else{
		MAX_DISP_VEC[i] = MAX_DISP_VEC[i]; 
		KdMAX(i,i) = KdMAX(i,i);
		//std::cout << "Condition ==> 2" << std::endl;

	}

	PMAX[i] = desPos[i] + MAX_DISP_VEC[i];
	PMID[i] = desPos[i] + (PMAX[i] - desPos[i])/2;

		//DEFINING K1_3x3 AND K2_3x3 w.r.t MAXIMUM STIFFNESS AT THE RELEASED POINT
		if(fabs(posError[i]) <= maxDisp[i]){

				K1(i, i) = (2 * pow(maxDisp[i], 2) * (pow(F_MAX[i]/maxDisp[i] - kConstPos(i,i), (pow(posError[i], 2)/pow(maxDisp[i], 2)))) - 2 * pow(maxDisp[i], 2) + 2 * kConstPos(i,i) * log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]) * fabs(pow(posError[i], 2)))/(pow(MAX_DISP_VEC[i], 2) * log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]));
				K2(i, i) = (2 * pow(maxDisp[i], 2) * (pow(F_MAX[i]/maxDisp[i] - kConstPos(i,i), (pow(posError[i], 2)/pow(maxDisp[i], 2)))) - 2 * pow(maxDisp[i], 2) + 2 * kConstPos(i,i) * log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]) * fabs(pow(posError[i], 2)))/(pow(MAX_DISP_VEC[i], 2) * log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]));
				

     }else{

			K1(i, i) = - ((2 * pow(maxDisp[i], 2))/(log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i])) - 2 * kConstPos(i,i) * pow(maxDisp[i], 2) + 4 * F_MAX[i] * (maxDisp[i] - fabs(posError[i])) - (2 * maxDisp[i] * (F_MAX[i] - kConstPos(i,i) * maxDisp[i]))/(log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i])))/pow(MAX_DISP_VEC[i], 2);
			K2(i, i) = - ((2 * pow(maxDisp[i], 2))/(log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i])) - 2 * kConstPos(i,i) * pow(maxDisp[i], 2) + 4 * F_MAX[i] * (maxDisp[i] - fabs(posError[i])) - (2 * maxDisp[i] * (F_MAX[i] - kConstPos(i,i) * maxDisp[i]))/(log((F_MAX[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i])))/pow(MAX_DISP_VEC[i], 2);

        }

	/*K1(i,i) = 2 * KdMAX(i,i);
	K2(i,i) = 2 * KdMAX(i,i);*/
}

	
void teleOp::calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_ORI(Eigen::MatrixXf &K1_ORI, Eigen::MatrixXf &K2_ORI, Eigen::MatrixXf &KdMAX_ORI, Eigen::VectorXf &PMAX_ORI, Eigen::VectorXf &PMID_ORI, Eigen::VectorXf &MAX_DISP_VEC_ORI, Eigen::VectorXf &BETA_ORI, Eigen::MatrixXf &K_MAX_SYSTEM_ORI, Eigen::VectorXf &MAX_DISP_ORI, Eigen::VectorXf &TAU_MAX, int i){
		
	if (fabs(oriError[i]) > fabs(preOriError[i])){
		MAX_DISP_VEC_ORI[i] = curOri[i] - desOri[i];
		KdMAX_ORI(i,i) = kConstOri(i,i) + kVarOri(i,i);
		//std::cout << "Condition ==> 1" << std::endl;
	}else{
		MAX_DISP_VEC_ORI[i] = MAX_DISP_VEC_ORI[i]; 
		KdMAX_ORI(i,i) = KdMAX_ORI(i,i);
		//std::cout << "Condition ==> 2" << std::endl;
	}

	PMAX_ORI[i] = desOri[i] + MAX_DISP_VEC_ORI[i];
	PMID_ORI[i] = desOri[i] + (PMAX_ORI[i] - desOri[i])/2;


	// DEFINING K1_Ori_3x3 AND K2_Ori_3x3 w.r.t MAXIMUM STIFFNESS AT THE RELEASED POINT
	if(fabs(oriError[i]) <= maxDisp_Ori[i]){

		K1_ORI(i, i) = (2 * pow(maxDisp_Ori[i], 2) * (pow(TAU_MAX[i]/maxDisp_Ori[i] - kConstOri(i,i), (pow(oriError[i], 2)/pow(maxDisp_Ori[i], 2)))) - 2 * pow(maxDisp_Ori[i], 2) + 2 * kConstOri(i,i) * log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]) * fabs(pow(oriError[i], 2)))/(pow(MAX_DISP_VEC_ORI[i] + 1, 2) * log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]));
		K2_ORI(i, i) = (2 * pow(maxDisp_Ori[i], 2) * (pow(TAU_MAX[i]/maxDisp_Ori[i] - kConstOri(i,i), (pow(oriError[i], 2)/pow(maxDisp_Ori[i], 2)))) - 2 * pow(maxDisp_Ori[i], 2) + 2 * kConstOri(i,i) * log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]) * fabs(pow(oriError[i], 2)))/(pow(MAX_DISP_VEC_ORI[i] + 1, 2) * log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]));
            
   }else{

		K1_ORI(i, i) = - ((2 * pow(maxDisp_Ori[i], 2))/(log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i])) - 2 * kConstOri(i,i) * pow(maxDisp_Ori[i], 2) + 4 * TAU_MAX[i] * (maxDisp_Ori[i] - fabs(oriError[i])) - (2 * maxDisp_Ori[i] * (TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i]))/(log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i])))/pow(MAX_DISP_VEC_ORI[i] + 1, 2);
		K2_ORI(i, i) = - ((2 * pow(maxDisp_Ori[i], 2))/(log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i])) - 2 * kConstOri(i,i) * pow(maxDisp_Ori[i], 2) + 4 * TAU_MAX[i] * (maxDisp_Ori[i] - fabs(oriError[i])) - (2 * maxDisp_Ori[i] * (TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i]))/(log((TAU_MAX[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i])))/pow(MAX_DISP_VEC_ORI[i] + 1, 2);

}

	/*K1_ORI(i,i) = 2 * KdMAX_ORI(i,i);
	K2_ORI(i,i) = 2 * KdMAX_ORI(i,i);*/
}

	
void teleOp::calculate_Kc_constDampLinear_FmidCtrl_Fctrl(Eigen::MatrixXf &Kc, Eigen::MatrixXf &dConstDampLinear, Eigen::VectorXf &F_MID_CTRL, Eigen::VectorXf &F_CTRL, int i){

	if (lowBandwidth_Sigma_FIC_Act == 0){
			velLinearError_Cur_Buff_Sel = velLinearError;
	}else{
			velLinearError_Cur_Buff_Sel = buffVec_Vel_6x1_Sigma;
	}
	//LOW-BANDWIDTH TEST
	if (sgn(posError[i], 0.01f) == sgn(velLinearError_Cur_Buff_Sel[i], 0.25f)){
	//if (sgn(posError[i], 0.01f) == sgn(buffVec_Vel_6x1_Sigma[i], 0.25f)){
		Kc.setZero();
		dConstDampLinear(i,i) = 0;
		F_MID_CTRL[i] = 0;
		div_conv_pos_ori_sigma[i] = 0;

	}else{
		//LOW-BANDWIDTH TEST
		Kc = K2_3x3;
		if (lowBandwidth_Sigma_FIC_Act == 0){
			F_MID_CTRL = Kc * (Pmid3x1 - curPos);
		}else{
			F_MID_CTRL = Kc * (Pmid3x1 - buffVec_6x1_Sigma);
		}
		F_CTRL[i] = 0;
		div_conv_pos_ori_sigma[i] = 1;

	}
}

void teleOp::calculate_Kc_constDampAngular_FmidCtrl_Fctrl_ORI(Eigen::MatrixXf &Kc_ORI, Eigen::MatrixXf &dConstDampAngular, Eigen::VectorXf &F_MID_CTRL_ORI, Eigen::VectorXf &F_CTRL_ORI, int i){

	if (sgn(oriError[i], 10.0f) == sgn(velAngularError[i], 10.0f)){
		Kc_ORI.setZero();
		dConstDampAngular(i,i) = 0;
		F_MID_CTRL_ORI[i] = 0;
		div_conv_pos_ori_sigma[i+3] = 0;

	}else{
		Kc_ORI = K2_Ori_3x3;
		F_MID_CTRL_ORI = Kc_ORI * (Pmid3x1_Ori - curOri);
		F_CTRL_ORI[i] = 0;
		div_conv_pos_ori_sigma[i+3] = 1;

	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // FRACTAL IMPEDANCE CONTROL FOR 7-DOF FRANKA PANDA // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

// DEMO_exp
float virtConstraintSize_Franka = 0.05;
float virtConstraintRotSize_Franka = 0.0872665;
 
// 1.5708 ==> 90 degrees
// 1.0472 ==> 60 degrees
// 0.785398 ==> 45 degrees
// 0.523599==> 30 degrees
// 0.349066 ==> 20 degrees
// 0.3054326 ==> 17.5 degrees
// 0.261799 ==> 15 degrees
// 0.2181662 ==> 12.5 degrees
// 0.174533 ==> 10 degrees
// 0.1309 ==> 7.5 degrees
// 0.0872665 ==> 5 degrees
// 0.0436332 ==> 2.5 degrees
// 0.0174533 ==> 1 degree

// END-EFFECTOR CURRENT AND DESIRED POSE
Eigen::VectorXf curPos3x1_Franka(3);
Eigen::VectorXf desPos3x1_Franka(3); 
Eigen::VectorXf curOri3x1_Franka(3); 
Eigen::VectorXf desOri3x1_Franka(3); 
Eigen::VectorXf curVel3x1_Franka(3);
Eigen::VectorXf desVel3x1_Franka(3);

// END-EFFECTOR POSE ERROR
Eigen::VectorXf position_error_Franka(3); 
Eigen::VectorXf angular_error_Franka(3);
Eigen::VectorXf posError_oriError_6x1_Franka(6);

// END-EFFECTOR LINEAR AND ANGULAR VELOCITY ERROR
Eigen::VectorXf velLinearError_Franka(3);
Eigen::VectorXf velAngularError_Franka(3);
Eigen::VectorXf endEffectorVelError_Franka(6);
Eigen::VectorXf velLinearErrorSquared_Franka(6);

// IMPEDANCE CONTROL GAIN: STIFFNESS
Eigen::MatrixXf kConstPos_Franka(3,3);
Eigen::MatrixXf kVarPos_Franka(3,3);
Eigen::MatrixXf kTotalDesPos_Franka(3,3);
Eigen::MatrixXf kConstOri_Franka(3,3); 			
Eigen::MatrixXf kVarOri_Franka(3,3);
Eigen::MatrixXf kTotalDesOri_Franka(3,3);
Eigen::MatrixXf kTotal6x6_Franka(6,6);

// IMPEDANCE CONTROL GAIN: DAMPING	
Eigen::MatrixXf dConstLinear_Franka(3,3); 
Eigen::MatrixXf dVarPos_Franka(3,3);
Eigen::MatrixXf dConstAngular_Franka(3,3); 
Eigen::MatrixXf dVarOri_Franka(3,3);
Eigen::MatrixXf dTotal6x6_Franka(6,6);
Eigen::MatrixXf dVar6x6_Franka(6,6); 
Eigen::MatrixXf dConst6x6_Franka(6,6); 
Eigen::MatrixXf dCritical6x6_Franka(6,6); 

// DESIRED STIFFNESS AND DAMPING INITIALIZATION
Eigen::VectorXf Stiffness_Des_Franka(6); 
Eigen::VectorXf Damping_Des_Franka(6);

// KdMAX - ELEMENTS
Eigen::MatrixXf KdMax_Franka(3,3);
Eigen::MatrixXf KdMax_Ori_Franka(3,3);

// NULL-SPACE - ELEMENTS (CURRENT JOINTS: 7-DoF)
Eigen::VectorXf curJntPos7x1_Franka(7);
Eigen::VectorXf desJntPos7x1_Franka(7);
Eigen::VectorXf curJntVel7x1_Franka(7);
Eigen::VectorXf desJntVel7x1_Franka(7);

// NULL-SPACE - STIFFNESS - ELEMENTS
Eigen::MatrixXf kConstNull_Franka(7,7);
Eigen::MatrixXf kVarNull_Franka(7,7);

// NULL-SPACE - DAMPING - ELEMENTS
Eigen::MatrixXf dConstNull_Franka(7,7);
Eigen::MatrixXf dVarNull_Franka(7,7);

//REQUIRED PARAMETERS FOR NULL SPACE CONTROL
Eigen::MatrixXd IdentityMat_7x7(7,7);
Eigen::MatrixXd pseudoInvJac(7,6);

// CALCULATION OF PREVIOUS VALUES - ELEMENTS
Eigen::VectorXf preEndEff_PosError3x1_Franka(3);
Eigen::VectorXf preEndEff_VelError3x1_Franka(3);
Eigen::VectorXf preEndEff_OriError3x1_Franka(3);
Eigen::VectorXf preEndEff_VelOriError3x1_Franka(3);
Eigen::VectorXd tauCmdEigen_Pre(7);
Eigen::VectorXd deltaTau(7);
Eigen::VectorXf preEndEff_DesPos3x1_Franka(3);

// NEW DESIRED POSE FOR HAPTIC CONTROL
Eigen::VectorXf newEndEff_DesPos3x1_Franka(3);
Eigen::MatrixXd newTargetCartMat_Franka(3,3);
Eigen::MatrixXd preTargetCartMat_Franka(3,3);

// NULL-SPACE CTRL PLUS TORQUE SATURATION - ELEMENTS
Eigen::VectorXf nullSpaceCtrl_Franka(7);
Eigen::VectorXf torqueSaturation_Franka(7);

// SYSTEM ENERGY - POTENTIAL AND KINETIC INITIALIZATION
double potentialEnergy_Franka, kineticEnergy_Franka;
Eigen::VectorXf total_system_energy_Franka(3);
Eigen::VectorXf energy_vector_Franka(3);

// PROPOSED METHOD - POSITION - ELEMENTS
Eigen::VectorXf maxVel3x1_Franka(3);
Eigen::VectorXf Pmax3x1_Franka(3);
Eigen::VectorXf Pmid3x1_Franka(3);
Eigen::VectorXf maxDistVec_3x1_Franka(3);
Eigen::MatrixXf K1_3x3_Franka(3,3);
Eigen::MatrixXf K2_3x3_Franka(3,3);
Eigen::MatrixXf Kc_3x3_Franka(3,3);
Eigen::VectorXf F_midPointErrorCtrl_Franka(3);
Eigen::VectorXf Fk_Franka(3);

// PROPOSED METHOD - ORIENTATION - ELEMENTS
Eigen::VectorXf maxVel3x1_Ori_Franka(3);
Eigen::VectorXf Pmax3x1_Ori_Franka(3);
Eigen::VectorXf Pmid3x1_Ori_Franka(3);
Eigen::VectorXf maxDistVec_Ori_3x1_Franka(3);
Eigen::MatrixXf K1_Ori_3x3_Franka(3,3);
Eigen::MatrixXf K2_Ori_3x3_Franka(3,3);
Eigen::MatrixXf Kc_Ori_3x3_Franka(3,3);
Eigen::VectorXf F_midPointErrorCtrl_Ori_Franka(3);
Eigen::VectorXf Fk_Ori_Franka(3);

// DAMPING FORCES FOR LINEAR AND ANGULAR VELOCITIES - REQUIRED ELEMENTS
Eigen::VectorXf Fd_Pos_Franka(3);
Eigen::VectorXf Fd_Ori_Franka(3);

// Fmax & BETA & MAXIMUM DISPLACEMENT (POSITION) & KmaxSystem
Eigen::VectorXf Fmax_Franka(3);
Eigen::VectorXf maxDisp_Franka(3);
Eigen::VectorXf beta_Franka(3);
Eigen::MatrixXf KmaxSystem_Franka(3,3);

// TauMax &  BETA_Ori & MAXIMUM DISPLACEMENT (ORIENTATION) & KmaxSystem_Ori
Eigen::VectorXf TauMax_Franka(3);
Eigen::VectorXf maxDisp_Ori_Franka(3);
Eigen::VectorXf beta_Ori_Franka(3);
Eigen::MatrixXf KmaxSystem_Ori_Franka(3,3);

// Fmax & TauMax SLOPE INITIALIZATION
double slopeFmax_Franka, slopeTauMax_Ori_Franka;

// ONLINE BOX SHAPING SWITCH
double onlineBoxShapingSwitch_Franka;
	
// END-EFFECTOR TASK SPACE INERTIA & ESTIMATED JOINT TORQUES
Eigen::MatrixXf lambda_c_Franka(6,6);
Eigen::VectorXf estimated_torque_var_Franka(7);

// EXTRA ELEMENTS
Eigen::VectorXf desPos_curPos_Franka(6);
Eigen::VectorXf Fctrl_XYZ_Franka(3);

// FRACTAL IMPEDANCE CONTROL END-EFFECTOR FORCES AND TORQUES
Eigen::VectorXf F_FIC_Franka(3);
Eigen::VectorXf Tau_FIC_Franka(3);
Eigen::VectorXf Wrench_FIC_Franka(6);
Eigen::VectorXd Wrench_FIC_Franka_doubleType(6);
Eigen::VectorXf tauCtrlNull_Franka(7);
Eigen::VectorXd tauCtrlNull_Franka_doubleType(7);
Eigen::VectorXd NullSpaceCtrl(7);

//TORQUE SATURATION REQUIRED ELEMENTS
double delta_tau_max = 0.5;
Eigen::VectorXd tmp(7);
Eigen::VectorXd buffer(7);
Eigen::VectorXd buffer_wrench(6);
Eigen::VectorXd estimated_torques(7);
Eigen::VectorXd estimated_wrenches(6);

Eigen::VectorXf potEnergy_6x1_Franka(6);

Eigen::VectorXd div_conv_pos_ori_franka(6);
std::atomic<double> div_conv_posX_franka(0.0);
std::atomic<double> div_conv_posY_franka(0.0);
std::atomic<double> div_conv_posZ_franka(0.0);
std::atomic<double> div_conv_oriX_franka(0.0);
std::atomic<double> div_conv_oriY_franka(0.0);
std::atomic<double> div_conv_oriZ_franka(0.0);

double sinr_cosp, cosr_cosp, roll, sinp, pitch, siny_cosp, cosy_cosp, yaw;
Eigen::VectorXf Fdamping(3);


// FRANKA FIC
class frankaFIC{

public:

	// TEST CLASS
	void printSomething_Franka();

	// CONTROLLER CLASSES
	void calculate_K_Max_Beta_Pos_Ori_Franka(Eigen::VectorXf &F_MAX, Eigen::VectorXf &Max_Disp, Eigen::VectorXf &Tau_MAX, Eigen::VectorXf &Max_Disp_Ori, Eigen::MatrixXf &K_Max_System, Eigen::VectorXf &Beta ,Eigen::MatrixXf &K_Max_System_Ori, Eigen::VectorXf &Beta_Ori, int i);
  void get_K_Total_Pos_Ori_Franka(Eigen::VectorXf &Beta, Eigen::MatrixXf &K_Var_Pos, Eigen::MatrixXf &K_Total_Des_Pos, Eigen::VectorXf &Beta_Ori, Eigen::MatrixXf &K_Var_Ori, Eigen::MatrixXf &K_Total_Des_Ori, int i);
	void get_K_Total_Pos_Ori_Franka_V2(Eigen::VectorXf &Beta, Eigen::MatrixXf &K_Var_Pos, Eigen::MatrixXf &K_Total_Des_Pos, Eigen::VectorXf &Beta_Ori, Eigen::MatrixXf &K_Var_Ori, Eigen::MatrixXf &K_Total_Des_Ori, Eigen::MatrixXf &K_Max_System, Eigen::MatrixXf &K_Max_System_Ori, int i);    
	void calculate_F_Max_Franka(Eigen::VectorXf &Max_Disp, Eigen::VectorXf &F_Max, int i);
  void calculate_Tau_Max_Franka(Eigen::VectorXf &Max_Disp_Ori, Eigen::VectorXf &Tau_Max, int i);
  void boxShaping_Online_Franka(Eigen::VectorXf &Max_Disp, int i);
  void calculation_K1_K2_Kd_Max_PMax_PMid_Max_Dist_Vec_Franka(Eigen::MatrixXf &K1, Eigen::MatrixXf &K2, Eigen::MatrixXf &Kd_Max, Eigen::VectorXf &Pmax, Eigen::VectorXf &Pmid, Eigen::VectorXf &Max_Dist_Vec, Eigen::VectorXf &Beta, Eigen::MatrixXf &K_Max_System, Eigen::VectorXf &Max_Disp, Eigen::VectorXf &F_Max, int i);
  void calculation_K1_K2_Kd_Max_PMax_PMid_Max_Dist_Vec_Ori_Franka(Eigen::MatrixXf &K1_Ori, Eigen::MatrixXf &K2_Ori, Eigen::MatrixXf &Kd_Max_Ori, Eigen::VectorXf &Pmax_Ori, Eigen::VectorXf &Pmid_Ori, Eigen::VectorXf &Max_Dist_Vec_Ori, Eigen::VectorXf &Beta_Ori, Eigen::MatrixXf &K_Max_System_Ori, Eigen::VectorXf &Max_Disp_Ori, Eigen::VectorXf &Tau_Max, int i);
  void calculation_Kc_DampConstLiner_FMidCtrl_FkCtrl_Franka(Eigen::MatrixXf &Kc, Eigen::MatrixXf &DampConstLinear, Eigen::VectorXf &FMidCtrl, Eigen::VectorXf &FkCtrl, int i);
  void calculation_Kc_DampConstAngular_FMidCtrl_FkCtrl_Ori_Franka(Eigen::MatrixXf &Kc_Ori, Eigen::MatrixXf &DampConstAngular, Eigen::VectorXf &FMidCtrl_Ori, Eigen::VectorXf &FkCtrl_Ori, int i);

	template <typename T> static int sgn (const T &val, const T &eps = T(0)){
		return (T(eps) < val) - (val < T(-eps));
	} 

};

void frankaFIC::printSomething_Franka(){
	printf ("Franka Class Working!!!\n");
	}

void frankaFIC::calculate_K_Max_Beta_Pos_Ori_Franka(Eigen::VectorXf &F_MAX, Eigen::VectorXf &Max_Disp, Eigen::VectorXf &Tau_MAX, Eigen::VectorXf &Max_Disp_Ori, Eigen::MatrixXf &K_Max_System, Eigen::VectorXf &Beta ,Eigen::MatrixXf &K_Max_System_Ori, Eigen::VectorXf &Beta_Ori, int i) {

  K_Max_System(i,i) = F_MAX[i]/Max_Disp[i];    
	Beta[i] = sqrt(log(K_Max_System(i,i))/pow(Max_Disp[i],2));
	if (std::isnan(Beta[i])){
		Beta[i] = 0;
	}

  K_Max_System_Ori(i,i) = Tau_MAX[i]/Max_Disp_Ori[i];
  Beta_Ori[i] = sqrt(log(K_Max_System_Ori(i,i))/pow(Max_Disp_Ori[i],2));
	if (std::isnan(Beta_Ori[i])){
		Beta_Ori[i] = 0;
	}
}

void frankaFIC::get_K_Total_Pos_Ori_Franka(Eigen::VectorXf &Beta, Eigen::MatrixXf &K_Var_Pos, Eigen::MatrixXf &K_Total_Des_Pos, Eigen::VectorXf &Beta_Ori, Eigen::MatrixXf &K_Var_Ori, Eigen::MatrixXf &K_Total_Des_Ori, int i) {

  K_Var_Pos(i,i) = exp(pow(Beta[i] * (position_error_Franka[i]), 2));
  K_Total_Des_Pos(i,i) = kConstPos_Franka(i,i) + K_Var_Pos(i,i);

  K_Var_Ori(i, i) = exp(pow(Beta_Ori[i] * (angular_error_Franka[i]),2));
  K_Total_Des_Ori(i,i) = kConstOri_Franka(i,i) + K_Var_Ori(i,i);
}

void frankaFIC::get_K_Total_Pos_Ori_Franka_V2(Eigen::VectorXf &Beta, Eigen::MatrixXf &K_Var_Pos, Eigen::MatrixXf &K_Total_Des_Pos, Eigen::VectorXf &Beta_Ori, Eigen::MatrixXf &K_Var_Ori, Eigen::MatrixXf &K_Total_Des_Ori, Eigen::MatrixXf &K_Max_System, Eigen::MatrixXf &K_Max_System_Ori, int i) {

  K_Var_Pos(i,i) = (K_Max_System(i,i) - kConstPos_Franka(i,i)) * fabs(tanh(pow(0.75 * Beta[i] * position_error_Franka[i],2)));
  K_Total_Des_Pos(i,i) = kConstPos_Franka(i,i) + K_Var_Pos(i,i);

  K_Var_Ori(i,i) = (K_Max_System_Ori(i,i) - kConstOri_Franka(i,i)) * fabs(tanh(pow(0.75 * Beta_Ori[i] * angular_error_Franka[i],2)));
  K_Total_Des_Ori(i,i) = kConstOri_Franka(i,i) + K_Var_Ori(i,i);
}

void frankaFIC::calculate_F_Max_Franka(Eigen::VectorXf &Max_Disp, Eigen::VectorXf &F_Max, int i){

////////////////	////////////////	// CALIBRATED //	////////////////	////////////////

	if (Max_Disp[i] >= 0.075){
		F_Max[i] = 20;
	}else if (Max_Disp[i] >= 0.05 && Max_Disp[i] < 0.075){
		F_Max[i] = 17.5;
	}else if (Max_Disp[i] >= 0.01 && Max_Disp[i] < 0.05){
		F_Max[i] = 15;
	}else if (Max_Disp[i] >= 0.003 && Max_Disp[i] < 0.01){
		F_Max[i] = 10;
	}else if (Max_Disp[i] >= 0.001 && Max_Disp[i] < 0.003){
		F_Max[i] = 7.5;
	}else if (Max_Disp[i] >= 0.0005 && Max_Disp[i] < 0.001){
		F_Max[i] = 5;
	}
	
	/*if (i == 0){
		if (Max_Disp[i] >= 0.003){
			F_Max[i] = 30;		
		}else if(Max_Disp[i] >= 0.001 && Max_Disp[i] < 0.003) {
			F_Max[i] = 15;
		}else {
			F_Max[i] = 12.5;
		}
		}else if (i == 1){

		if (Max_Disp[i] >= 0.09){
			F_Max[i] = 30;		
		}else if(Max_Disp[i] >= 0.05 && Max_Disp[i] < 0.09) {
			F_Max[i] = 25;
		}else if(Max_Disp[i] >= 0.04 && Max_Disp[i] < 0.05) {
			F_Max[i] = 22.5;
		}else if(Max_Disp[i] >= 0.03 && Max_Disp[i] < 0.04) {
			F_Max[i] = 20;
		}else if(Max_Disp[i] >= 0.02 && Max_Disp[i] < 0.03) {
			F_Max[i] = 18;
		}else if(Max_Disp[i] >= 0.007 && Max_Disp[i] < 0.02) {
			F_Max[i] = 12.5;
		}else if(Max_Disp[i] >= 0.006 && Max_Disp[i] < 0.007) {
			F_Max[i] = 10;
		}else if(Max_Disp[i] >= 0.004 && Max_Disp[i] < 0.006) {
			F_Max[i] = 6;
		}else if(Max_Disp[i] >= 0.003 && Max_Disp[i] < 0.004) {
			F_Max[i] = 5;
		}else if(Max_Disp[i] >= 0.002 && Max_Disp[i] < 0.003) {
			F_Max[i] = 4;
		}else if(Max_Disp[i] >= 0.001 && Max_Disp[i] < 0.002) {
			F_Max[i] = 3;
		}else {
			F_Max[i] = 2;
		}
		}else{

		if (Max_Disp[i] >= 0.08){
			F_Max[i] = 30;		
		}else if(Max_Disp[i] >= 0.05 && Max_Disp[i] < 0.08) {
			F_Max[i] = 28;
		}else if(Max_Disp[i] >= 0.04 && Max_Disp[i] < 0.05) {
			F_Max[i] = 26;
		}else if(Max_Disp[i] >= 0.03 && Max_Disp[i] < 0.04) {
			F_Max[i] = 24;
		}else if(Max_Disp[i] >= 0.02 && Max_Disp[i] < 0.03) {
			F_Max[i] = 20;
		}else if(Max_Disp[i] >= 0.007 && Max_Disp[i] < 0.02) {
			F_Max[i] = 12.5;
		}else if(Max_Disp[i] >= 0.006 && Max_Disp[i] < 0.007) {
			F_Max[i] = 10;
		}else if(Max_Disp[i] >= 0.004 && Max_Disp[i] < 0.006) {
			F_Max[i] = 6;
		}else if(Max_Disp[i] >= 0.003 && Max_Disp[i] < 0.004) {
			F_Max[i] = 5;
		}else if(Max_Disp[i] >= 0.002 && Max_Disp[i] < 0.003) {
			F_Max[i] = 4;
		}else if(Max_Disp[i] >= 0.001 && Max_Disp[i] < 0.002) {
			F_Max[i] = 3;
		}else {
			F_Max[i] = 2;
		}

}*/
////////////////	////////////////	////////////////	////////////////	////////////////	
}

void frankaFIC::calculate_Tau_Max_Franka(Eigen::VectorXf &Max_Disp_Ori, Eigen::VectorXf &Tau_Max, int i){

////////////////	////////////////	// CALIBRATED //	////////////////	////////////////
	if (i == 0){	
		Tau_Max[i] = 30;
		}
	else{
		if (Max_Disp_Ori[i] >= 0.261799){
			Tau_Max[i] = 20;
		}else if (Max_Disp_Ori[i] >= 0.0872665 && Max_Disp_Ori[i] < 0.261799){
			Tau_Max[i] = 15;
		}else {
			Tau_Max[i] = 5;
	}
  }
////////////////	////////////////	////////////////	////////////////	////////////////

}

/*void frankaFIC::boxShaping_Online_Franka(Eigen::VectorXf &Max_Disp, int i){
	
	
		if (currentTime >= time0 + 1){	
		Max_Disp[i] = Max_Disp[i] - 0.0025;
		if (i == 2){
			time0 = currentTime;
		}    	
    	if(Max_Disp[i] <= 0.01) {
    		Max_Disp[i] = 0.01;
    	}
	}
}*/

void frankaFIC::calculation_K1_K2_Kd_Max_PMax_PMid_Max_Dist_Vec_Franka(Eigen::MatrixXf &K1, Eigen::MatrixXf &K2, Eigen::MatrixXf &Kd_Max, Eigen::VectorXf &Pmax, Eigen::VectorXf &Pmid, Eigen::VectorXf &Max_Dist_Vec, Eigen::VectorXf &Beta, Eigen::MatrixXf &K_Max_System, Eigen::VectorXf &Max_Disp, Eigen::VectorXf &F_Max, int i) {

	//LOW-BANDWIDTH TEST
	// DEFINING END-EFFECTOR MAXIMUM DISPLACEMENT (POSITION) & KdMax 
	if (fabs(position_error_Franka[i]) > fabs(preEndEff_PosError3x1_Franka[i])) {
		if (lowBandwidth_Franka_FIC_Act == 0){
        Max_Dist_Vec[i] = curPos3x1_Franka[i] - desPos3x1_Franka[i];
		}else{
				Max_Dist_Vec[i] = (buffVec_6x1[i]) - desPos3x1_Franka[i];
		}
        Kd_Max(i,i) = kConstPos_Franka(i,i) + kVarPos_Franka(i, i);
				//std::cout << "Condition ==> 1" << std::endl;
    } else{

        Max_Dist_Vec[i] = Max_Dist_Vec[i];
        Kd_Max(i,i) = Kd_Max(i,i);
				//std::cout << "Condition ==> 2" << std::endl;
    }

	// DEFINING Pmax3x1 and Pmid3x1 (POSITION)
  Pmax[i] = desPos3x1_Franka[i] + Max_Dist_Vec[i];
  Pmid[i] = desPos3x1_Franka[i] + (Pmax[i] - desPos3x1_Franka[i])/2;


  // DEFINING K1_3x3 AND K2_3x3 w.r.t MAXIMUM STIFFNESS AT THE RELEASED POINT
	if(fabs(position_error_Franka[i]) <= maxDisp_Franka[i]){

		K1(i, i) = (2 * pow(maxDisp_Franka[i], 2) * (pow(F_Max[i]/maxDisp_Franka[i] - kConstPos_Franka(i,i), (pow(position_error_Franka[i], 2)/pow(maxDisp_Franka[i], 2)))) - 2 * pow(maxDisp_Franka[i], 2) + 2 * kConstPos_Franka(i,i) * log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]) * fabs(pow(position_error_Franka[i], 2)))/(pow(Max_Dist_Vec[i], 2) * log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]));
		K2(i, i) = (2 * pow(maxDisp_Franka[i], 2) * (pow(F_Max[i]/maxDisp_Franka[i] - kConstPos_Franka(i,i), (pow(position_error_Franka[i], 2)/pow(maxDisp_Franka[i], 2)))) - 2 * pow(maxDisp_Franka[i], 2) + 2 * kConstPos_Franka(i,i) * log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]) * fabs(pow(position_error_Franka[i], 2)))/(pow(Max_Dist_Vec[i], 2) * log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]));
            
   }else{

		K1(i, i) = - ((2 * pow(maxDisp_Franka[i], 2))/(log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i])) - 2 * kConstPos_Franka(i,i) * pow(maxDisp_Franka[i], 2) + 4 * F_Max[i] * (maxDisp_Franka[i] - fabs(position_error_Franka[i])) - (2 * maxDisp_Franka[i] * (F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i]))/(log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i])))/pow(Max_Dist_Vec[i], 2);
		K2(i, i) = - ((2 * pow(maxDisp_Franka[i], 2))/(log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i])) - 2 * kConstPos_Franka(i,i) * pow(maxDisp_Franka[i], 2) + 4 * F_Max[i] * (maxDisp_Franka[i] - fabs(position_error_Franka[i])) - (2 * maxDisp_Franka[i] * (F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i]))/(log((F_Max[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i])))/pow(Max_Dist_Vec[i], 2);

	}

	/*K1(i, i) = 2 * Kd_Max(i, i);
  K2(i, i) = 2 * Kd_Max(i, i);*/

}

void frankaFIC::calculation_K1_K2_Kd_Max_PMax_PMid_Max_Dist_Vec_Ori_Franka(Eigen::MatrixXf &K1_Ori, Eigen::MatrixXf &K2_Ori, Eigen::MatrixXf &Kd_Max_Ori, Eigen::VectorXf &Pmax_Ori, Eigen::VectorXf &Pmid_Ori, Eigen::VectorXf &Max_Dist_Vec_Ori, Eigen::VectorXf &Beta_Ori, Eigen::MatrixXf &K_Max_System_Ori, Eigen::VectorXf &Max_Disp_Ori, Eigen::VectorXf &Tau_Max, int i) {

	// DEFINING END-EFFECTOR MAXIMUM DISPLACEMENT (ORIENTATION) & KdMax_Ori 
    if (fabs(angular_error_Franka[i]) > fabs(preEndEff_OriError3x1_Franka[i])) {
        Max_Dist_Vec_Ori[i] = curOri3x1_Franka[i] - desOri3x1_Franka[i];
        Kd_Max_Ori(i,i) = kConstOri_Franka(i,i) + kVarOri_Franka(i, i);
				//std::cout << "Condition ==> 1" << std::endl;
    } else{

        Max_Dist_Vec_Ori[i] = Max_Dist_Vec_Ori[i];
        Kd_Max_Ori(i,i) = Kd_Max_Ori(i,i);
				//std::cout << "Condition ==> 2" << std::endl;
    }

    // DEFINING Pmax3x1 and Pmid3x1 (ORIENTATION)
		Pmax_Ori[i] = desOri3x1_Franka[i] + Max_Dist_Vec_Ori[i];
    Pmid_Ori[i] = desOri3x1_Franka[i] + (Pmax_Ori[i] - desOri3x1_Franka[i])/2;

	// DEFINING K1_Ori_3x3 AND K2_Ori_3x3 w.r.t MAXIMUM STIFFNESS AT THE RELEASED POINT
	if(fabs(angular_error_Franka[i]) <= maxDisp_Ori_Franka[i]){

		K1_Ori(i, i) = (2 * pow(maxDisp_Ori_Franka[i], 2) * (pow(Tau_Max[i]/maxDisp_Ori_Franka[i] - kConstOri_Franka(i,i), (pow(angular_error_Franka[i], 2)/pow(maxDisp_Ori_Franka[i], 2)))) - 2 * pow(maxDisp_Ori_Franka[i], 2) + 2 * kConstOri_Franka(i,i) * log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]) * fabs(pow(angular_error_Franka[i], 2)))/(pow(Max_Dist_Vec_Ori[i] + 1, 2) * log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]));
		K2_Ori(i, i) = (2 * pow(maxDisp_Ori_Franka[i], 2) * (pow(Tau_Max[i]/maxDisp_Ori_Franka[i] - kConstOri_Franka(i,i), (pow(angular_error_Franka[i], 2)/pow(maxDisp_Ori_Franka[i], 2)))) - 2 * pow(maxDisp_Ori_Franka[i], 2) + 2 * kConstOri_Franka(i,i) * log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]) * fabs(pow(angular_error_Franka[i], 2)))/(pow(Max_Dist_Vec_Ori[i] + 1, 2) * log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]));

  }else{

		K1_Ori(i, i) = - ((2 * pow(maxDisp_Ori_Franka[i], 2))/(log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i])) - 2 * kConstOri_Franka(i,i) * pow(maxDisp_Ori_Franka[i], 2) + 4 * Tau_Max[i] * (maxDisp_Ori_Franka[i] - fabs(angular_error_Franka[i])) - (2 * maxDisp_Ori_Franka[i] * (Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori[i]))/(log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i])))/pow(Max_Dist_Vec_Ori[i] + 1, 2);
		K2_Ori(i, i) = - ((2 * pow(maxDisp_Ori_Franka[i], 2))/(log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i])) - 2 * kConstOri_Franka(i,i) * pow(maxDisp_Ori_Franka[i], 2) + 4 * Tau_Max[i] * (maxDisp_Ori_Franka[i] - fabs(angular_error_Franka[i])) - (2 * maxDisp_Ori_Franka[i] * (Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori[i]))/(log((Tau_Max[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i])))/pow(Max_Dist_Vec_Ori[i] + 1, 2);

  }

    /*K1_Ori(i, i) = 2 * Kd_Max_Ori(i, i);
    K2_Ori(i, i) = 2 * Kd_Max_Ori(i, i);*/
}

void frankaFIC::calculation_Kc_DampConstLiner_FMidCtrl_FkCtrl_Franka(Eigen::MatrixXf &Kc, Eigen::MatrixXf &DampConstLinear, Eigen::VectorXf &FMidCtrl, Eigen::VectorXf &FkCtrl, int i) {
					
	if (lowBandwidth_Franka_FIC_Act == 0){
			velLinearError_Cur_Buff_Sel_Franka = velLinearError_Franka;
	}else{
			velLinearError_Cur_Buff_Sel_Franka = buffVec_Vel_6x1;
	}
	//LOW-BANDWIDTH TEST	
	if(sgn(position_error_Franka[i], 0.01f) == sgn(velLinearError_Cur_Buff_Sel_Franka[i], 0.25f)){
	//if(sgn(position_error_Franka[i], 0.01f) == sgn(buffVec_Vel_6x1[i], 0.25f)){
		 Kc.setZero();
     DampConstLinear(i,i) = 0;
     FMidCtrl[i] = 0;
		 div_conv_pos_ori_franka[i] = 0;

  }else {
		 //LOW-BANDWIDTH TEST
     Kc = K2_3x3_Franka;
	if (lowBandwidth_Franka_FIC_Act == 0){
     FMidCtrl = Kc * (Pmid3x1_Franka - curPos3x1_Franka);
	}else{
		 FMidCtrl = Kc * (Pmid3x1_Franka - (buffVec_6x1));
	}
     FkCtrl[i] = 0;
		 div_conv_pos_ori_franka[i] = 1;
	}
}

void frankaFIC::calculation_Kc_DampConstAngular_FMidCtrl_FkCtrl_Ori_Franka(Eigen::MatrixXf &Kc_Ori, Eigen::MatrixXf &DampConstAngular, Eigen::VectorXf &FMidCtrl_Ori, Eigen::VectorXf &FkCtrl_Ori, int i) {
            
	if(sgn(angular_error_Franka[i], 0.1f) == sgn(velAngularError_Franka[i], 2.5f)){
     Kc_Ori.setZero();
     DampConstAngular(i,i) = 0;
     FMidCtrl_Ori[i] = 0;
		 div_conv_pos_ori_franka[i+3] = 0;

   }else {
     Kc_Ori = K2_Ori_3x3_Franka;
     FMidCtrl_Ori = Kc_Ori * (Pmid3x1_Ori_Franka - curOri3x1_Franka);
     FkCtrl_Ori[i] = 0;
		 div_conv_pos_ori_franka[i+3] = 1;
   }    
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // TORQUE SATURATION CLASS // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

/*
template < typename T > class TorqueSaturation {

public:

	TorqueSaturation(typename T::Scalar delta_tau_max, T current) {
	  this->delta_tau_max = delta_tau_max;
    buffer = current;
  }

  void saturateTorqueRate(const T &torque_desired, T &torque_output) {
  	torque_output = torque_desired.array() - buffer.array();
    float scale = delta_tau_max / torque_output.norm();
    if (scale < 1) {
       torque_output *= scale;
    }
    		torque_output += buffer;
    		buffer = torque_output;
   	}
private:
    T buffer;
    typename T::Scalar delta_tau_max;
};
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

/* Global configuration parameters */
const double hapticGainLinVel = 15.0;
const double hapticGainAngVel = 0.01;
const double hapticLinMaxEffort = 20.0;
const double hapticAngMaxEffort = 0.4;

const double hapticLinDeadband = 0.0;
const double hapticAngDeadband = 0.0;

// DEMO
const double MtoShapticLinDeadband = 0.00;
const double MtoShapticAngDeadband = 0.00;

const double frankaLinVelGain = 3.0;
const double frankaLinVelClamp = 0.3;
const double frankaAngVelGain = 0.3;
const double frankaAngVelClamp = 0.3;

// SYNCRONIZATION PROCEDURE TEST
std::atomic<long> cnt(0);

/* Global state parameters */
std::atomic<double> hapticGainPosLinX(200.0);
std::atomic<double> hapticGainPosLinY(200.0);
std::atomic<double> hapticGainPosLinZ(200.0);
std::atomic<double> hapticGainPosAngX(0.3);
std::atomic<double> hapticGainPosAngY(0.3);
std::atomic<double> hapticGainPosAngZ(0.3);

/* Global haptic device and franka state for thread communication */
std::atomic<bool> hapticIsDone(false);
std::atomic<double> hapticReadLinPosX(0.0);
std::atomic<double> hapticReadLinPosY(0.0);
std::atomic<double> hapticReadLinPosZ(0.0);
std::atomic<double> hapticReadAngPosX(0.0);
std::atomic<double> hapticReadAngPosY(0.0);
std::atomic<double> hapticReadAngPosZ(0.0);
std::atomic<double> hapticReadGripRatio(0.0);
std::atomic<double> MtoShapticReadLinPosX(0.0);
std::atomic<double> MtoShapticReadLinPosY(0.0);
std::atomic<double> MtoShapticReadLinPosZ(0.0);
std::atomic<double> MtoShapticReadAngPosX(0.0);
std::atomic<double> MtoShapticReadAngPosY(0.0);
std::atomic<double> MtoShapticReadAngPosZ(0.0);
std::atomic<bool> frankaIsDone(false);
std::atomic<bool> frankaIsControl(true);
std::atomic<double> frankaReadJoint1(0.0);
std::atomic<double> frankaReadJoint2(0.0);
std::atomic<double> frankaReadJoint3(0.0);
std::atomic<double> frankaReadJoint4(0.0);
std::atomic<double> frankaReadJoint5(0.0);
std::atomic<double> frankaReadJoint6(0.0);
std::atomic<double> frankaReadJoint7(0.0);
std::atomic<double> frankaTargetPosLinX(0.0);
std::atomic<double> frankaTargetPosLinY(0.0);
std::atomic<double> frankaTargetPosLinZ(0.0);
std::atomic<double> frankaTargetPosAngX(0.0);
std::atomic<double> frankaTargetPosAngY(0.0);
std::atomic<double> frankaTargetPosAngZ(0.0);

/* GLOBAL OPTO FORCE STATE PARAMETERS */
std::atomic<bool> isOver(false);
std::atomic<double> optoForceX(0.0);
std::atomic<double> optoForceY(0.0);
std::atomic<double> optoForceZ(0.0);
std::atomic<double> optoTorqueX(0.0);
std::atomic<double> optoTorqueY(0.0);
std::atomic<double> optoTorqueZ(0.0);

// FOR SPHERE POSITION CHANGE
std::atomic<double> frankaTargetPosLinXSphere(0.0);
std::atomic<double> frankaTargetPosLinYSphere(0.0);
std::atomic<double> frankaTargetPosLinZSphere(0.0);
std::atomic<double> frankaTargetPosLinSphereSize(0.0075);

//TODO XXX
std::atomic<double> frankaReadExternalWrenchLinX1(0.0);
std::atomic<double> frankaReadExternalWrenchLinX2(0.0);
std::atomic<double> frankaReadExternalWrenchLinX3(0.0);
std::atomic<double> frankaReadExternalWrenchLinX4(0.0);

//TODO XXX
std::atomic<double> frankaReadExcitationCmd(0.0);
std::atomic<double> frankaReadExcitationForce(0.0);
std::atomic<double> frankaReadExcitationDelta(0.0);
std::atomic<double> frankaReadExcitationVel(0.0);

//TODO XXX
std::atomic<double> frankaRead1(0.0);
std::atomic<double> frankaRead2(0.0);
std::atomic<double> frankaRead3(0.0);
std::atomic<double> frankaRead4(0.0);

// PLOTING THE FRACTAL IMPEDANCE WRENCH
std::atomic<double> wrenchFIC_Fx(0.0);
std::atomic<double> wrenchFIC_Fy(0.0);
std::atomic<double> wrenchFIC_Fz(0.0);
std::atomic<double> wrenchFIC_Tau_x(0.0);
std::atomic<double> wrenchFIC_Tau_y(0.0);
std::atomic<double> wrenchFIC_Tau_z(0.0);

// PLOTING THE FRACTAL IMPEDANCE WRENCH - FEEDBACK ON SIGMA 7
std::atomic<double> feedBack_wrenchFIC_Fx(0.0);
std::atomic<double> feedBack_wrenchFIC_Fy(0.0);
std::atomic<double> feedBack_wrenchFIC_Fz(0.0);
std::atomic<double> feedBack_wrenchFIC_Tau_x(0.0);
std::atomic<double> feedBack_wrenchFIC_Tau_y(0.0);
std::atomic<double> feedBack_wrenchFIC_Tau_z(0.0);

// PLOTING THE ROBOT TORQUES
std::atomic<double> tau_1(0.0);
std::atomic<double> tau_2(0.0);
std::atomic<double> tau_3(0.0);
std::atomic<double> tau_4(0.0);
std::atomic<double> tau_5(0.0);
std::atomic<double> tau_6(0.0);
std::atomic<double> tau_7(0.0);

// PLOTING THE ROBOT TORQUES
std::atomic<double> deltaTau_1(0.0);
std::atomic<double> deltaTau_2(0.0);
std::atomic<double> deltaTau_3(0.0);
std::atomic<double> deltaTau_4(0.0);
std::atomic<double> deltaTau_5(0.0);
std::atomic<double> deltaTau_6(0.0);
std::atomic<double> deltaTau_7(0.0);

// PLOTING THE SIGMA 7 LINEAR AND ANGULAR VELOCITY ERRORS
std::atomic<double> velLinear_X(0.0);
std::atomic<double> velLinear_Y(0.0);
std::atomic<double> velLinear_Z(0.0);
std::atomic<double> velAngular_X(0.0);
std::atomic<double> velAngular_Y(0.0);
std::atomic<double> velAngular_Z(0.0);

// PLOTING THE FRANKA LINEAR AND ANGULAR VELOCITY ERRORS
std::atomic<double> Franka_velLinear_X(0.0);
std::atomic<double> Franka_velLinear_Y(0.0);
std::atomic<double> Franka_velLinear_Z(0.0);
std::atomic<double> Franka_velAngular_X(0.0);
std::atomic<double> Franka_velAngular_Y(0.0);
std::atomic<double> Franka_velAngular_Z(0.0);

// GUI NEW TARGET END-EFFECTOR POSITION
Eigen::Vector3d newTargetEndEffPosUI;

/* Haptic device main thread */
static void mainHaptic() {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  int deviceCount;
  int deviceID0;
  int deviceID1;
  
  deviceCount = dhdGetDeviceCount ();
  
  if (deviceCount < 1) {
    printf ("NOTE: no device detected\n");
  }else if (deviceCount < 2) {
    printf ("NOTE: 1 device detected\n");
  }else if (deviceCount < 3){
  	printf ("NOTE: 2 devices detected\n");
  }
  
  if ((deviceID0 = drdOpenID (0)) < 0) {
    printf ("error: %s\n", dhdErrorGetLastStr ());
  }
 
  // open the second available device
  if ((deviceID1 = drdOpenID (1)) < 0) {
    printf ("error: %s\n", dhdErrorGetLastStr ());
  }
  
  std:: cout << "Number of devices: " << deviceCount << std::endl;
  std:: cout << "Haptic Right ID: " << deviceID0 << std::endl << "Haptic Left ID: " << deviceID1 << std::endl;
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
	uoe::FilterIIR<Eigen::Vector3f> filterPmid_3x1;
	filterPmid_3x1.init(mkfilter::FilterType_t::Butterworth, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 20.0, 0.0);

	uoe::FilterIIR<Eigen::Vector3f> filterPmax_3x1;
	filterPmax_3x1.init(mkfilter::FilterType_t::Butterworth, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 20.0, 0.0);

	uoe::FilterIIR<Eigen::Vector3f> filterPmid_Ori_3x1;
	filterPmid_Ori_3x1.init(mkfilter::FilterType_t::Butterworth, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 20.0, 0.0);

	uoe::FilterIIR<Eigen::Vector3f> filterPmax_Ori_3x1;
	filterPmax_Ori_3x1.init(mkfilter::FilterType_t::Butterworth, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 20.0, 0.0);
*/

	// init(mkfilter::FilterType_t filter, mkfilter::PassType_t pass, double ripple, unsigned int order, double freqSampling, double freqCorner1, double freqCorner2)
    
	//Version verbose
  int major, minor, release, revision;
  dhdGetSDKVersion(&major, &minor, &release, &revision);
  std::cout << "Force Dimension SDK: " << major << "," << minor << "," << release << "," << revision << std::endl;
    
  //Open first haptic device
  /*if (drdOpen() < 0) {
      std::cerr << "Error opening haptic device: " << dhdErrorGetLastStr() << std::endl;
      hapticIsDone.store(true);
      return;
  }*/
    
  //Check DRD support
  if (!drdIsSupported()) {
      dhdSleep(1.0);
      drdClose();
      std::cerr << "Unsupported haptive device: " << dhdErrorGetLastStr() << std::endl;
      hapticIsDone.store(true);
      return;
   }
   std::cout << "Haptic device detected: " << dhdGetSystemName() << std::endl;
   std::cout << "Haptic device ID: " << dhdGetDeviceID() << std::endl;

   //Perform auto-initialization and start robotic control loop
   if (!drdIsInitialized () && drdAutoInit () < 0) {
       dhdSleep(1.0);
       drdClose();
       std::cerr << "Failed to auto-initialize haptive device: " << dhdErrorGetLastStr() << std::endl;
       hapticIsDone.store(true);
       return;
    }
    else if (drdStart () < 0) {
       dhdSleep(1.0);
       drdClose();
       std::cerr << "Fail start haptic control thread: " << dhdErrorGetLastStr() << std::endl;
       hapticIsDone.store(true);
       return;
    }
    
	//Workspace center
  double poseCenter[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //Move to center
  drdMoveTo(poseCenter);
    
  //Enable orientation and gripper regulation
  drdRegulatePos(false);
  drdRegulateRot(false);
  drdRegulateGrip(false);

	// // // // // // // // // // // // // // // // NEW PART OF THE CODE // // // // // // // // // // // // // // // //
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

	// DEMO_exp
	//maxDisp.setConstant(virtConstraintSize);
	maxDisp_Ori.setConstant(virtConstraintRotSize);

	maxDisp[0] = 0.05;
	maxDisp[1] = 0.05;
	maxDisp[2] = 0.05;

	teleOp teleOp;

	// TEST CLASS
	//teleOp.printSomething();
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

  //Main control loop
  std::cout << "Starting haptic control loop" << std::endl;
  while (!hapticIsDone.load()) {

  //Synchronize with regulation thread
  drdWaitForTick();

	// // // // // // // // // // // // // // // // NEW PART OF THE CODE // // // // // // // // // // // // // // // //
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

	//LOW-BANDWIDTH TEST
	buffVec_6x1_Sigma[0] = buffVec_PosX_Sigma[0];
	buffVec_6x1_Sigma[1] = buffVec_PosY_Sigma[0];
	buffVec_6x1_Sigma[2] = buffVec_PosZ_Sigma[0];
	buffVec_6x1_Sigma[3] = buffVec_OriX_Sigma[0];
	buffVec_6x1_Sigma[4] = buffVec_OriY_Sigma[0];
	buffVec_6x1_Sigma[5] = buffVec_OriZ_Sigma[0];

  buffVec_Vel_6x1_Sigma[0] = buffVec_Vel_PosX_Sigma[0];
  buffVec_Vel_6x1_Sigma[1] = buffVec_Vel_PosY_Sigma[0];
  buffVec_Vel_6x1_Sigma[2] = buffVec_Vel_PosZ_Sigma[0];
  buffVec_Vel_6x1_Sigma[3] = buffVec_Vel_OriX_Sigma[0];
  buffVec_Vel_6x1_Sigma[4] = buffVec_Vel_OriY_Sigma[0];
  buffVec_Vel_6x1_Sigma[5] = buffVec_Vel_OriZ_Sigma[0];


	buffVec_Wrench_6x1[0] = buffVec_Force_X[0];
	buffVec_Wrench_6x1[1] = buffVec_Force_Y[0];
	buffVec_Wrench_6x1[2] = buffVec_Force_Z[0];

	buffVec_Wrench_6x1_lowBandWidth_timeDelay[0] = buffVec_Force_X_lowBandWidth_timeDelay[0];
	buffVec_Wrench_6x1_lowBandWidth_timeDelay[1] = buffVec_Force_Y_lowBandWidth_timeDelay[0];
	buffVec_Wrench_6x1_lowBandWidth_timeDelay[2] = buffVec_Force_Z_lowBandWidth_timeDelay[0];

	// PLOTING SIGMA 7 LINEAR AND ANGULAR VELOCITIES
	velLinear_X.store(curVelLinear[0]);
	velLinear_Y.store(curVelLinear[1]);
	velLinear_Z.store(curVelLinear[2]);
	velAngular_X.store(curVelAngular[0]);
	velAngular_Y.store(curVelAngular[1]);
	velAngular_Z.store(curVelAngular[2]);

	// START OF MAIN FOR LOOP
	for (int i = 0; i < 3; i++){

	// DEFINING END-EFFECTOR POSE ERROR
	//LOW-BANDWIDTH TEST
	if (lowBandwidth_Sigma_FIC_Act == 0){
		posError[i] = desPos[i] - curPos[i];
	}else{
		posError[i] = desPos[i] - buffVec_6x1_Sigma[i];
	}
	oriError[i] = desOri[i] - curOri[i];
	thumbFingerPosError[i] = curThumbPos[i] - curFingerPos[i];
	
	// DEFINING END-EFFECTOR VELOCITY ERRORS (LINEAR AND ANGULAR)	
	velLinearError[i] = - curVelLinear[i];
	velAngularError[i] = - curVelAngular[i];
			
	// SETTING CONSTANT STIFFNESS AND DAMPING
  // DEMO
	kConstPos(i,i) = 100; kConstOri(i,i) = 0.1;
	dConstPos(i,i) = scalingFactorPos * 2.5; dConstOri(i,i) = 0.01;

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
		kVarPos(i,i) = Fmax[i]/fabs(posError[i]) - kConstPos(i,i);
	}

	if (kConstOri(i,i) + kVarOri(i,i) > KmaxSystem_Ori(i,i) && beta_Ori[i] > 0){
		kVarOri(i,i) = TauMax[i]/fabs(oriError[i]) - kConstOri(i,i);
	}

	// ATTRACTIVE FORCE FOR POSITION AND ORINETATION CTRL CALCULATION
	FkCtrl[i] = (kConstPos(i,i) + kVarPos(i,i)) * posError[i];
	TauCtrl[i] = (kConstOri(i,i) + kVarOri(i,i)) * oriError[i];

	// DAMPING FORCES FOR LINEAR AND ANGULAR VELOCITIES - DEFINITIONS
	//LOW-BANDWIDTH TEST
	if (lowBandwidth_Sigma_FIC_Act == 0){
		FdCtrl[i] = dConstPos(i,i) * velLinearError[i];
	}else{
		FdCtrl[i] = dConstPos(i,i) * buffVec_Vel_6x1_Sigma[i];
	}
	Tau_dCtrl[i] = dConstOri(i,i) * velAngularError[i];

/*
	// FILTERING Pmid3x1
	filterPmid_3x1.update(Pmid3x1);
  Pmid3x1 = filterPmid_3x1.value();

	// FILTERING Pmax3x1
	filterPmax_3x1.update(Pmax3x1);
  Pmax3x1 = filterPmax_3x1.value();

	// FILTERING Pmid3x1_Ori
	filterPmid_Ori_3x1.update(Pmid3x1_Ori);
  Pmid3x1_Ori = filterPmid_Ori_3x1.value();

	// FILTERING Pmax3x1_Ori
	filterPmax_Ori_3x1.update(Pmax3x1_Ori);
  Pmax3x1_Ori = filterPmax_Ori_3x1.value();
*/

	// // // // // // // // // // // // // // // // // // // // // // // // PROPOSED METHOD - END-EFFECTOR POSITION // // // // // // // // // // // // // // // // // // // // // // // //
	teleOp.calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_POS(K1_3x3, K2_3x3, KdMax, Pmax3x1, Pmid3x1, maxDistVec_3x1, beta, KmaxSystem, maxDisp, Fmax, i);

	// // // // // // // // // // // // // // // // // // // // // // // // PROPOSED METHOD - END-EFFECTOR ORIENTATION // // // // // // // // // // // // // // // // // // // // // // // 
	teleOp.calculate_K1_K2_KdMax_Pmax_Pmid_MaxDistVec_ORI(K1_Ori_3x3, K2_Ori_3x3, KdMax_Ori, Pmax3x1_Ori, Pmid3x1_Ori, maxDistVec_Ori_3x1, beta_Ori, KmaxSystem_Ori, maxDisp_Ori, TauMax, i);

	// // // // // // // // // // // // // // // // // // DEFINING THE CONDITIONS FOR THE PROPOSED METHOD - END-EFFECTOR POSITION // // // // // // // // // // // // // // // // // // // //
	teleOp.calculate_Kc_constDampLinear_FmidCtrl_Fctrl(Kc_3x3, dConstPos, Fmid_Ctrl, FkCtrl, i);

	// // // // // // // // // // // // // // // // // // DEFINING THE CONDITIONS FOR THE PROPOSED METHOD - END-EFFECTOR ORIENTATION // // // // // // // // // // // // // // // // // // //
	teleOp.calculate_Kc_constDampAngular_FmidCtrl_Fctrl_ORI(Kc_Ori_3x3, dConstOri, TauMid_Ctrl, TauCtrl, i);

	// // // // // // // // // // // // // // // // // // // // // //  // // DEFINING THE WRENCH VECTOR OF HAPTIC DEVICE // // // // // // // // // // // // // // // // // // // // // // //
	Wrench_FIC_Sigma[i] = FkCtrl[i] + Fmid_Ctrl[i] + FdCtrl[i];
	Wrench_FIC_Sigma[i+3] = TauCtrl[i] + TauMid_Ctrl[i] + Tau_dCtrl[i];
	Wrench_FIC_Sigma_doubleType[i] = (double) Wrench_FIC_Sigma[i];
	Wrench_FIC_Sigma_doubleType[i+3] = (double) Wrench_FIC_Sigma[i+3];
	
/*
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
*/

	// TOTAL SYSTEM STIFFNESS PROFILE
	//LOW-BANDWIDTH TEST
	if (lowBandwidth_Sigma_FIC_Act == 0){
		velLinearError_Cur_Buff_Sel = velLinearError;
	}else{
		velLinearError_Cur_Buff_Sel = buffVec_Vel_6x1_Sigma;
	}
	if (teleOp.sgn(posError[i], 0.005f) == teleOp.sgn(velLinearError_Cur_Buff_Sel[i], 0.005f)){
	//if (teleOp.sgn(posError[i], 0.005f) == teleOp.sgn(buffVec_Vel_6x1_Sigma[i], 0.005f)){
		kTotal6x6(i,i) = kConstPos(i,i) + kVarPos(i,i);
	}else{
		kTotal6x6(i,i) = Kc_3x3(i,i);
	}

	if (teleOp.sgn(oriError[i], 0.005f) == teleOp.sgn(velAngularError[i], 0.005f)){
		kTotal6x6(i+3,i+3) = kConstOri(i,i) + kVarOri(i,i);
	}else{
		kTotal6x6(i+3,i+3) = Kc_Ori_3x3(i,i);
	}

	// ENERGY EQUATION ALONG/AROUND EACH AXIS
	if (fabs(posError[i] <= maxDisp[i])){
            potEnergy_6x1[i] = (pow(maxDisp[i], 2) * (pow(Fmax[i]/maxDisp[i] - kConstPos(i,i), (pow(posError[i], 2)/pow(maxDisp[i], 2)))) - pow(maxDisp[i], 2) + kConstPos(i,i) * log((Fmax[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]) * fabs(pow(posError[i], 2)))/(2 * log((Fmax[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]));
  }else{
            potEnergy_6x1[i] = (kConstPos(i,i) * pow(maxDisp[i], 2))/2 - pow(maxDisp[i], 2)/(2 * log((Fmax[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i])) - Fmax[i] * (maxDisp[i] - fabs(posError[i])) + (maxDisp[i] * (Fmax[i] - kConstPos(i,i) * maxDisp[i]))/(2 * log((Fmax[i] - kConstPos(i,i) * maxDisp[i])/maxDisp[i]));
  }

  if (fabs(oriError[i]) <= maxDisp_Ori[i]){
            potEnergy_6x1[i+3] = (pow(maxDisp_Ori[i], 2) * (pow(TauMax[i]/maxDisp_Ori[i] - kConstOri(i,i), (pow(oriError[i], 2)/pow(maxDisp_Ori[i], 2)))) - pow(maxDisp_Ori[i], 2) + kConstOri(i,i) * log((TauMax[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]) * fabs(pow(oriError[i], 2)))/(2 * log((TauMax[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]));
  }else{
            potEnergy_6x1[i+3] = (kConstOri(i,i) * pow(maxDisp_Ori[i], 2))/2 - pow(maxDisp_Ori[i], 2)/(2 * log((TauMax[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i])) - TauMax[i] * (maxDisp_Ori[i] - fabs(oriError[i])) + (maxDisp_Ori[i] * (TauMax[i] - kConstOri(i,i) * maxDisp_Ori[i]))/(2 * log((TauMax[i] - kConstOri(i,i) * maxDisp_Ori[i])/maxDisp_Ori[i]));
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
} // END OF MAIN FOR LOOP

	// CALCULATION OF DELTA WRENCH FOR HAPTIC DEVICE
	deltaWrench_Sigma = Wrench_FIC_Sigma - preWrench_FIC_Sigma;

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
}
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

	//Retrieve current position
  Eigen::Vector3d cartPos;
  Eigen::Vector3d cartAng;
  double gripPos;
  Eigen::Matrix3d cartMat;
  double tmpMat[3][3];
  drdGetPositionAndOrientation(&(cartPos.x()), &(cartPos.y()), &(cartPos.z()), &(cartAng.x()), &(cartAng.y()), &(cartAng.z()), &gripPos, tmpMat);
        
	//Conversion back to Eigen
  cartMat(0,0) = tmpMat[0][0];
  cartMat(0,1) = tmpMat[0][1];
  cartMat(0,2) = tmpMat[0][2];
  cartMat(1,0) = tmpMat[1][0];
  cartMat(1,1) = tmpMat[1][1];
  cartMat(1,2) = tmpMat[1][2];
  cartMat(2,0) = tmpMat[2][0];
  cartMat(2,1) = tmpMat[2][1];
  cartMat(2,2) = tmpMat[2][2];

  //Compute filtered state
  hapticReadLinPosX.store(uoe::Deadband(cartPos.x(), hapticLinDeadband, true));
  hapticReadLinPosY.store(uoe::Deadband(cartPos.y(), hapticLinDeadband, true));
  hapticReadLinPosZ.store(uoe::Deadband(cartPos.z(), hapticLinDeadband, true));
  Eigen::Vector3d tmpPosAxis = uoe::MatrixToAxis(cartMat);
  hapticReadAngPosX.store(uoe::Deadband(tmpPosAxis.x(), hapticAngDeadband, true));
  hapticReadAngPosY.store(uoe::Deadband(tmpPosAxis.y(), hapticAngDeadband, true));
  hapticReadAngPosZ.store(uoe::Deadband(tmpPosAxis.z(), hapticAngDeadband, true));

	MtoShapticReadLinPosX.store(uoe::Deadband(cartPos.x(), MtoShapticLinDeadband, true));
  MtoShapticReadLinPosY.store(uoe::Deadband(cartPos.y(), MtoShapticLinDeadband, true));
  MtoShapticReadLinPosZ.store(uoe::Deadband(cartPos.z(), MtoShapticLinDeadband, true));
  MtoShapticReadAngPosX.store(uoe::Deadband(tmpPosAxis.x(), MtoShapticAngDeadband, true));
  MtoShapticReadAngPosY.store(uoe::Deadband(tmpPosAxis.y(), MtoShapticAngDeadband, true));
  MtoShapticReadAngPosZ.store(uoe::Deadband(tmpPosAxis.z(), MtoShapticAngDeadband, true));
        
	//std::cout << MtoShapticReadLinPosX << "   " << cartPos.x() << std::endl << std::endl;
	//Compute gripper state between [0:1]
  double gripPosRatio = 1.0 - uoe::ClampRange(gripPos/0.02, 0.0, 1.0);
  hapticReadGripRatio.store(gripPosRatio);

  //Retrieve current velocity
  Eigen::Vector3d cartVelLin;
  Eigen::Vector3d cartVelAng;
  double gripVel;
  drdGetVelocity(&(cartVelLin.x()), &(cartVelLin.y()), &(cartVelLin.z()), &(cartVelAng.x()), &(cartVelAng.y()), &(cartVelAng.z()), &gripVel);
        
	//Retrieve current force applied by human
  Eigen::Vector3d forceRead;
  dhdGetForce(&(forceRead.x()), &(forceRead.y()), &(forceRead.z()));
        
	//Compute linear control effort
  Eigen::Vector3d forceLinSpring;
	forceLinSpring.x() = -hapticGainPosLinX.load()*cartPos.x();
  forceLinSpring.y() = -hapticGainPosLinY.load()*cartPos.y();
  forceLinSpring.z() = -hapticGainPosLinZ.load()*cartPos.z();
  Eigen::Vector3d forceAngSpring;
  forceAngSpring.x() = hapticGainPosAngX.load()*uoe::MatrixToAxis(cartMat.transpose()).x();
  forceAngSpring.y() = hapticGainPosAngY.load()*uoe::MatrixToAxis(cartMat.transpose()).y();
  forceAngSpring.z() = hapticGainPosAngZ.load()*uoe::MatrixToAxis(cartMat.transpose()).z();
  for (size_t i=0;i<3;i++) {
      if (forceLinSpring(i) > hapticLinMaxEffort) {
          forceLinSpring(i) = hapticLinMaxEffort;
      }
      if (forceLinSpring(i) < -hapticLinMaxEffort) {
          forceLinSpring(i) = -hapticLinMaxEffort;
      }
  }
  for (size_t i=0;i<3;i++) {
       if (forceAngSpring(i) > hapticAngMaxEffort) {
           forceAngSpring(i) = hapticAngMaxEffort;
       }
       if (forceAngSpring(i) < -hapticAngMaxEffort) {
           forceAngSpring(i) = -hapticAngMaxEffort;
       }
  }
  
	Eigen::Vector3d forceLinDamping = -hapticGainLinVel*cartVelLin;
  Eigen::Vector3d forceAngDamping = -hapticGainAngVel*cartVelAng;
  Eigen::Vector3d forceLinEffort = forceLinSpring + forceLinDamping;
  Eigen::Vector3d forceAngEffort = forceAngSpring + forceAngDamping;

	int res;

if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 0){
	if (gripperAct == 1){ 
	      
      // HOLDING THE GRIPPER FOR GRAVITY COMPENSATION ACTIVATION
      if (gripperGapVec[0] < 0.001){
          if (hapticVirtConstFeedBackAct == 0){
					  res = drdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
					  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(0 - 1 * Wrench_FIC_Franka_doubleType[0], 0 - 1 * Wrench_FIC_Franka_doubleType[1], 0 - 1 * Wrench_FIC_Franka_doubleType[2], 0 - 0.015 * Wrench_FIC_Franka_doubleType[3], 0 - 0.015 * Wrench_FIC_Franka_doubleType[4], 0 - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					  }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(0 + optoFT_forceCoef * optoForceY.load(), 0 + optoFT_forceCoef * optoForceX.load(), 0 - optoFT_forceCoef * optoForceZ.load(), 0 - optoFT_torqueCoef * optoTorqueX.load(), 0 -  optoFT_torqueCoef * optoTorqueY.load(), 0 - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
            }
					}
					
			}else{
					if (hapticVirtConstFeedBackAct == 0){
						//WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					    }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] + optoFT_forceCoef * optoForceY.load(), Wrench_FIC_Sigma_doubleType[1] + optoFT_forceCoef * optoForceX.load(), Wrench_FIC_Sigma_doubleType[2] - optoFT_forceCoef * optoForceZ.load(), Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
          }
				}
			}
			
			}else{
						
					if (hapticVirtConstFeedBackAct == 0){
					  //WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						if (hapticOptoForceSensorAct == 0){
						  //WITH FIC HAPTIC FORCE FEDDBACK - FLIPPED CONTACT WRENCH
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						}else if (hapticOptoForceSensorAct == 1){
						  //WITH OPTO FT SENSOR FORCE FEDDBACK
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] + optoFT_forceCoef * optoForceY.load(), Wrench_FIC_Sigma_doubleType[1] + optoFT_forceCoef * optoForceX.load(), Wrench_FIC_Sigma_doubleType[2] - optoFT_forceCoef * optoForceZ.load(), Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						
					}

						
		}
	}
}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 1){
	
	//LOW-BANDWIDTH TEST
	if (gripperAct == 1){ 
	      
      // HOLDING THE GRIPPER FOR GRAVITY COMPENSATION ACTIVATION
      if (gripperGapVec[0] < 0.001){
          if (hapticVirtConstFeedBackAct == 0){
					  res = drdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
					  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(0 - 1 * Wrench_FIC_Franka_doubleType[0], 0 - 1 * Wrench_FIC_Franka_doubleType[1], 0 - 1 * Wrench_FIC_Franka_doubleType[2], 0 - 0.015 * Wrench_FIC_Franka_doubleType[3], 0 - 0.015 * Wrench_FIC_Franka_doubleType[4], 0 - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					  }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(0 - buffVec_Wrench_6x1[0], 0 - buffVec_Wrench_6x1[1], 0 - buffVec_Wrench_6x1[2], 0 - optoFT_torqueCoef * optoTorqueX.load(), 0 -  optoFT_torqueCoef * optoTorqueY.load(), 0 - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
            }
					}
					
			}else{
					if (hapticVirtConstFeedBackAct == 0){
						//WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					    }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - buffVec_Wrench_6x1[0], Wrench_FIC_Sigma_doubleType[1] - buffVec_Wrench_6x1[1], Wrench_FIC_Sigma_doubleType[2] - buffVec_Wrench_6x1[2], Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
          }
				}
			}
			
			}else{
						
					if (hapticVirtConstFeedBackAct == 0){
					  //WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						if (hapticOptoForceSensorAct == 0){
						  //WITH FIC HAPTIC FORCE FEDDBACK - FLIPPED CONTACT WRENCH
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						}else if (hapticOptoForceSensorAct == 1){
						  //WITH OPTO FT SENSOR FORCE FEDDBACK
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - buffVec_Wrench_6x1[0], Wrench_FIC_Sigma_doubleType[1] - buffVec_Wrench_6x1[1], Wrench_FIC_Sigma_doubleType[2] - buffVec_Wrench_6x1[2], Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						
					}

						
		}
	}
}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 2){
	if (gripperAct == 1){ 
	      
      // HOLDING THE GRIPPER FOR GRAVITY COMPENSATION ACTIVATION
      if (gripperGapVec[0] < 0.001){
          if (hapticVirtConstFeedBackAct == 0){
					  res = drdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
					  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(0 - 1 * Wrench_FIC_Franka_doubleType[0], 0 - 1 * Wrench_FIC_Franka_doubleType[1], 0 - 1 * Wrench_FIC_Franka_doubleType[2], 0 - 0.015 * Wrench_FIC_Franka_doubleType[3], 0 - 0.015 * Wrench_FIC_Franka_doubleType[4], 0 - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					  }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(0 - FeedBackForce_Delayed[0], 0 - FeedBackForce_Delayed[1], 0 - FeedBackForce_Delayed[2], 0 - optoFT_torqueCoef * optoTorqueX.load(), 0 -  optoFT_torqueCoef * optoTorqueY.load(), 0 - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
            }
					}
					
			}else{
					if (hapticVirtConstFeedBackAct == 0){
						//WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					    }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - FeedBackForce_Delayed[0], Wrench_FIC_Sigma_doubleType[1] - FeedBackForce_Delayed[1], Wrench_FIC_Sigma_doubleType[2] - FeedBackForce_Delayed[2], Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
          }
				}
			}
			
			}else{
						
					if (hapticVirtConstFeedBackAct == 0){
					  //WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						if (hapticOptoForceSensorAct == 0){
						  //WITH FIC HAPTIC FORCE FEDDBACK - FLIPPED CONTACT WRENCH
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						}else if (hapticOptoForceSensorAct == 1){
						  //WITH OPTO FT SENSOR FORCE FEDDBACK
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - FeedBackForce_Delayed[0], Wrench_FIC_Sigma_doubleType[1] - FeedBackForce_Delayed[1], Wrench_FIC_Sigma_doubleType[2] - FeedBackForce_Delayed[2], Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						
					}

						
		}
	}
}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 3){
	
	//LOW-BANDWIDTH TEST
	if (gripperAct == 1){ 
	      
      // HOLDING THE GRIPPER FOR GRAVITY COMPENSATION ACTIVATION
      if (gripperGapVec[0] < 0.001){
          if (hapticVirtConstFeedBackAct == 0){
					  res = drdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
					  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(0 - 1 * Wrench_FIC_Franka_doubleType[0], 0 - 1 * Wrench_FIC_Franka_doubleType[1], 0 - 1 * Wrench_FIC_Franka_doubleType[2], 0 - 0.015 * Wrench_FIC_Franka_doubleType[3], 0 - 0.015 * Wrench_FIC_Franka_doubleType[4], 0 - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					  }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(0 - buffVec_Wrench_6x1_lowBandWidth_timeDelay[0], 0 - buffVec_Wrench_6x1_lowBandWidth_timeDelay[1], 0 - buffVec_Wrench_6x1_lowBandWidth_timeDelay[2], 0 - optoFT_torqueCoef * optoTorqueX.load(), 0 -  optoFT_torqueCoef * optoTorqueY.load(), 0 - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
            }
					}
					
			}else{
					if (hapticVirtConstFeedBackAct == 0){
						//WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						  if (hapticOptoForceSensorAct == 0){
						    //WITH FIC HAPTIC FORCE FEDDBACK
						    res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					    }else if (hapticOptoForceSensorAct == 1){
					      //WITH OPTO FT SENSOR FORCE FEDDBACK
					      res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - buffVec_Wrench_6x1_lowBandWidth_timeDelay[0], Wrench_FIC_Sigma_doubleType[1] - buffVec_Wrench_6x1_lowBandWidth_timeDelay[1], Wrench_FIC_Sigma_doubleType[2] - buffVec_Wrench_6x1_lowBandWidth_timeDelay[2], Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
          }
				}
			}
			
			}else{
						
					if (hapticVirtConstFeedBackAct == 0){
					  //WITHOUT HAPTIC FORCE FEEDBACK
						res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0], Wrench_FIC_Sigma_doubleType[1], Wrench_FIC_Sigma_doubleType[2], Wrench_FIC_Sigma_doubleType[3], Wrench_FIC_Sigma_doubleType[4], Wrench_FIC_Sigma_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
					}else{
						if (hapticOptoForceSensorAct == 0){
						  //WITH FIC HAPTIC FORCE FEDDBACK - FLIPPED CONTACT WRENCH
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5], F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						}else if (hapticOptoForceSensorAct == 1){
						  //WITH OPTO FT SENSOR FORCE FEDDBACK
						  res = drdSetForceAndTorqueAndGripperForce(Wrench_FIC_Sigma_doubleType[0] - buffVec_Wrench_6x1_lowBandWidth_timeDelay[0], Wrench_FIC_Sigma_doubleType[1] - buffVec_Wrench_6x1_lowBandWidth_timeDelay[1], Wrench_FIC_Sigma_doubleType[2] - buffVec_Wrench_6x1_lowBandWidth_timeDelay[2], Wrench_FIC_Sigma_doubleType[3] - optoFT_torqueCoef * optoTorqueX.load(), Wrench_FIC_Sigma_doubleType[4] - optoFT_torqueCoef * optoTorqueY.load(), Wrench_FIC_Sigma_doubleType[5] - optoFT_torqueCoef * optoTorqueZ.load(), F_haptic_thumbFing[0] + F_haptic_thumbFing[1] + F_haptic_thumbFing[2]);
						
					}

						
	}
}
}
	
	
	//LOW-BANDWIDTH TEST
	prePosError = posError;
	preOriError = oriError;

	if (lowBandwidth_Sigma_FIC_Act == 0){
		preVelLinearError = velLinearError;
	}else{
		preVelLinearError = buffVec_Vel_6x1_Sigma;
	}
	preVelAngularError = velAngularError;
	preWrench_FIC_Sigma = Wrench_FIC_Sigma;

	buffVec_PosX_Sigma[countBuff] = curPos[0];
  buffVec_PosY_Sigma[countBuff] = curPos[1];
  buffVec_PosZ_Sigma[countBuff] = curPos[2];

  buffVec_Vel_PosX_Sigma[countBuff] = velLinearError[0];
  buffVec_Vel_PosY_Sigma[countBuff] = velLinearError[1];
  buffVec_Vel_PosZ_Sigma[countBuff] = velLinearError[2];

	buffVec_Force_X[countBuff] = optoFT_forceCoef * optoForceX.load();
	buffVec_Force_Y[countBuff] = optoFT_forceCoef * optoForceY.load();
	buffVec_Force_Z[countBuff] = optoFT_forceCoef * optoForceZ.load();

	buffVec_Force_X_lowBandWidth_timeDelay[countBuff] = FeedBackForce_Delayed[0];
	buffVec_Force_Y_lowBandWidth_timeDelay[countBuff] = FeedBackForce_Delayed[1];
	buffVec_Force_Z_lowBandWidth_timeDelay[countBuff] = FeedBackForce_Delayed[2];

	countBuff++;

	if (countBuff == vecSize){
			countBuff = 0;
	}

 
  if (res < DHD_NO_ERROR) {
      std::cout << "Error: cannot set haptic force: " << dhdErrorGetLastStr() << std::endl;
      hapticIsDone.store(true);
   }
  }
  std::cout << "Stopping haptic control loop" << std::endl;



  //Stop regulation
  drdStop();

  //Close haptic device
  drdClose();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void mainOptoForce()
{
    OptoDAQ daq;
    OptoPorts ports;
    OPort* portlist = ports.listPorts(true);
    if (ports.getLastSize() == 0) {
        std::cerr << "Error listing optoforce device" << std::endl;
        isOver.store(true);
        return;
    }

    std::cout << "Opening OptoForce device: " 
        << portlist[1].name << " " 
        << portlist[1].deviceName << " " 
        << portlist[1].serialNumber << std::endl;
    if (!daq.open(portlist[1])) {
        std::cerr << "Error opening optoforce device" << std::endl;
        isOver.store(true);
        return;
    }
    if (daq.getVersion() != _95 && daq.getVersion() != _64) {
        std::cerr << "Error optoforce device is 3d not 6d" << std::endl;
        isOver.store(true);
        return;
    }

    SensorConfig sensorConfig;
    sensorConfig.setSpeed(1000);
    sensorConfig.setFilter(0);
    sensorConfig.setMode(1);
    if (!daq.sendConfig(sensorConfig)) {
        std::cerr << "Error setting config to optoforce device" << std::endl;
        isOver.store(true);
        return;
    }

    daq.zeroAll();

    OptoPackage6D pack6D;
    uoe::FilterIIR<Eigen::Vector3d> filterForce;
    uoe::FilterIIR<Eigen::Vector3d> filterTorque;
    filterForce.init(
        mkfilter::FilterType_t::Butterworth,
        mkfilter::PassType_t::Lowpass,
        0.0, 1, 1000.0, 5.0, 0.0);
    filterTorque.init(
        mkfilter::FilterType_t::Butterworth,
        mkfilter::PassType_t::Lowpass,
        0.0, 1, 1000.0, 5.0, 0.0);
    std::cout << "Starting optoforce loop" << std::endl;
    while (!isOver.load()) {
        int size = daq.read6D(pack6D,false);
        if (size < 0) {
            std::cerr << "Error reading optoforce device: " << size << std::endl;
            isOver.store(true);
            return;
        }
        if (size > 0) {
            //Low pass filtering
            Eigen::Vector3d dataForce;
            Eigen::Vector3d dataTorque;
            dataForce(0) = (double)(pack6D.Fx)/13.55;
            dataForce(1) = (double)(pack6D.Fy)/14.36;
            dataForce(2) = (double)(pack6D.Fz)/2.0;
            dataTorque(0) = (double)(pack6D.Tx)/3000;
            dataTorque(1) = (double)(pack6D.Ty)/3000;
            dataTorque(2) = (double)(pack6D.Tz)/3000;
            
            // ORIGINIAL VALUES - WORKING!
            /*
            dataForce(0) = (double)(pack6D.Fx)/13.55;
            dataForce(1) = (double)(pack6D.Fy)/14.36;
            dataForce(2) = (double)(pack6D.Fz)/2.0;
            dataTorque(0) = (double)(pack6D.Tx)/263.89;
            dataTorque(1) = (double)(pack6D.Ty)/249.77;
            dataTorque(2) = (double)(pack6D.Tz)/414.74;
            */
            
            
            filterForce.update(dataForce);
            filterTorque.update(dataTorque);
            //Conversion from sensor counts to Newton with scaling
            //ratio found in sensor datasheet. May need recalibration to be
            //truly accurate.
            optoForceX.store(filterForce.value()(0));
            optoForceY.store(filterForce.value()(1));
            optoForceZ.store(filterForce.value()(2));
            optoTorqueX.store(filterTorque.value()(0));
            optoTorqueY.store(filterTorque.value()(1));
            optoTorqueZ.store(filterTorque.value()(2));
        }
    }
    std::cout << "Stopping optoforce loop" << std::endl;

    daq.close();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Franka arm main thread */
static void mainFranka(const char* frankaIP){

	try {

    //Connection to robot
    std::cout << "Connecting to Franka arm: " << frankaIP << std::endl;
    franka::Robot robot(frankaIP);
    //Load kinematics and dynamics model
    franka::Model model = robot.loadModel();

    //Default robot configuration
    robot.setCollisionBehavior(
    	{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    //robot.setJointImpedance({{1000, 1000, 1000, 1000, 1000, 1000, 1000}});
    //robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

		//robot.setJointImpedance({{100, 100, 100, 100, 100, 100, 100}});
		//robot.setCartesianImpedance({{100, 100, 100, 10, 10, 10}});
        
    //Define and configure end effector frame transformations
    std::array<double, 16> transformFlange2EE_Array;
    std::array<double, 16> transformEE2K_Array;
    auto transformFlange2EE_Eigen(Eigen::Map<Eigen::Matrix4d>(transformFlange2EE_Array.data()));
    auto transformEE2K_Eigen(Eigen::Map<Eigen::Matrix4d>(transformEE2K_Array.data()));
    transformFlange2EE_Eigen.setIdentity();
    transformEE2K_Eigen.setIdentity();
    /*transformFlange2EE_Eigen.block(0, 0, 3, 3) = Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();*/
    transformFlange2EE_Eigen.block(0, 0, 3, 3) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transformFlange2EE_Eigen.block(0, 3, 3, 1) = Eigen::Vector3d(0.0125, 0.0, 0.159);
    
    //Set end effector configuration (in column major format)

	  robot.setEE(transformFlange2EE_Array);
    robot.setK(transformEE2K_Array);

    //Define and configure end effector inertia
    const double handMass = 0.065;
    const std::array<double, 3> handFlange2CoM_Array = {{0.0, 0.0, 0.058}};
    const std::array<double, 9> handInertia_Array = {{0.000136 , 0.0, 0.0, 0.0, 0.000136 , 0.0, 0.0, 0.0, 5e-6}};
    robot.setLoad(handMass, handFlange2CoM_Array, handInertia_Array);
        
    //Move the robot to initial posture
    std::cout << "Franka go to initial posture..." << std::endl;
	  std::cin.ignore();

		//DEMO_exp
		//INITIAL CONFIGURATION FOR HUMAN ROBOT INTERATION 
		//std::array<double, 7> postureGoal = {{0.0, -M_PI_4, 0.0, -3.0*M_PI_4, 0.0, M_PI_2, M_PI_4}}; 
		//initial_configuration_Franka = 0;

		//INITIAL CONFIGURATION FOR BILATERAL TELEOPERATION-drilling task
		//std::array<double, 7> postureGoal = {{0.450967, 0.0406032, 0.401888, -1.64982, -0.0350874, 1.70505, 1.64923}};
		//initial_configuration_Franka = 1;
		
		//INITIAL CONFIGURATION FOR DUAL-ARM HUMAN ROBOT COLLABORATION 
		std::array<double, 7> postureGoal = {{0.0, -M_PI_4, 0.0, -3.0*M_PI_4, M_PI_2, M_PI_2, M_PI_4}}; 
		initial_configuration_Franka = 2;
		
    std::array<double, 7> postureStart;
    std::array<uoe::SplineQuintic, 7> splines;
    double time = 0.0;
    double timeLength = 5.0;
    robot.control([&postureGoal, &postureStart, &splines, &time, &timeLength] 
		(const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions         
		{
    	//Update time
      time += period.toSec();
      //First iteration initialization
      if (time == 0.0) {
          postureStart = robot_state.q_d;
          for (size_t i=0;i<7;i++) {
               splines[i] = uoe::SplineQuintic();
               splines[i].addPoint(0.0, postureStart[i]);
               splines[i].addPoint(1.0, postureGoal[i]);
          }
      }

      //Compute smooth trajectory
      double phase = (time-1.0)/timeLength;
      if (phase < 0.0) {
          phase = 0.0;
      }
      if (phase > 1.0) {
          phase = 1.0;
      }
      std::array<double, 7> postureCmd;
      for (size_t i=0;i<7;i++) {
           postureCmd[i] = splines[i].pos(phase);
      }

      //Terminal condition
      if (time > timeLength+2.0) {
          return franka::MotionFinished(franka::JointPositions(postureCmd));
      } else {
          return franka::JointPositions(postureCmd);
  	    }
      });
      std::cout << "Initial posture reached" << std::endl;

      //State for motion generation and torque control
			int gripperPressed = 0;
      double timeController = 0.0;
      uoe::FilterIIR<Eigen::Vector6d> filterTargetVel;
      uoe::FilterIIR<Eigen::Vector6d> filterErrorVel;
      uoe::FilterIIR<double> filterGripRatio;
      Eigen::Vector3d targetCartPos = Eigen::Vector3d::Zero();
      Eigen::Matrix3d targetCartMat = Eigen::Matrix3d::Identity();

			// VECTORS FOR DESIRED END-EFFECTOR POSE AND JOINT POSES
			Eigen::Matrix3d targetCartMat_FIC = Eigen::Matrix3d::Identity();
			Eigen::Matrix3d desCartOriMat = Eigen::Matrix3d::Identity();
			Eigen::VectorXd desiredJointPos(7);
				
			Eigen::Vector6d wrenchIntegral = Eigen::Vector6d::Zero();
      //Filter initialization
      filterTargetVel.init(mkfilter::FilterType_t::Bessel, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 2.0, 0.0);
      filterErrorVel.init(mkfilter::FilterType_t::Butterworth, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 20.0, 0.0);
      filterGripRatio.init(mkfilter::FilterType_t::Butterworth, mkfilter::PassType_t::Lowpass, 0.0, 2, 1000.0, 2.0, 0.0);

			//Goto mode splines
			std::array<uoe::SplineQuintic, 3> splinesGotoMode;
			bool lastGotoMode = false;

      //Cartesian force control callback implementation
      auto callbackControl = [&model, &timeController, &filterTargetVel, &filterErrorVel, &filterGripRatio, &targetCartPos, &desCartOriMat, &targetCartMat, &targetCartMat_FIC, &desiredJointPos, &wrenchIntegral, &gripperPressed, &splinesGotoMode, &lastGotoMode]
           (const franka::RobotState& state, franka::Duration period) -> franka::Torques
        {
         //Control parameters
         double frankaControlMaxEffortLinX = 10.0;
         double frankaControlMaxEffortLinY = 10.0;
         double frankaControlMaxEffortLinZ = 10.0;
         double frankaControlMaxEffortAngX = 2.5;
         double frankaControlMaxEffortAngY = 2.5;
         double frankaControlMaxEffortAngZ = 2.5;
         double frankaControlMaxErrorLinX = 0.01;
         double frankaControlMaxErrorLinY = 0.01;
         double frankaControlMaxErrorLinZ = 0.01;
         double frankaControlMaxErrorAngX = uoe::DegToRad(5.72);
         double frankaControlMaxErrorAngY = uoe::DegToRad(5.72);
         double frankaControlMaxErrorAngZ = uoe::DegToRad(5.72);
         double frankaControlDampingRatioLin = 1.0;
         double frankaControlDampingRatioAng = 0.0;
         double frankaControlTargetRatioMaxError = 1.5;
         double frankaControlIntegralTimeLinX = 0.2;
         double frankaControlIntegralTimeLinY = 0.2;
         double frankaControlIntegralTimeLinZ = 0.2;
         double frankaControlIntegralTimeAngX = 0.2;
         double frankaControlIntegralTimeAngY = 0.2;
         double frankaControlIntegralTimeAngZ = 0.2;
            
         //Compute model joint Coriolis torque vector
         std::array<double, 7> coriolisArray = model.coriolis(state);

         //Compute model end effector Jacobian in base frame
         std::array<double, 42> jacobianArray = model.zeroJacobian(
         franka::Frame::kEndEffector, state);

         //Compute model gravity vector
         std::array<double, 7> gravityArray = model.gravity(state);
            
         //Mapping state to Eigen
         Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolisEigen(coriolisArray.data());
         Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobianEigen(jacobianArray.data());
         Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravityEigen(gravityArray.data());
         Eigen::Map<const Eigen::Matrix<double, 7, 1>> readJointPos(state.q.data());
         Eigen::Map<const Eigen::Matrix<double, 7, 1>> readJointVel(state.dq.data());
         Eigen::Map<const Eigen::Matrix<double, 7, 1>> readJointTau(state.tau_J.data());
         Eigen::Affine3d readCartTransform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
         Eigen::Vector3d readCartPos(readCartTransform.translation());
         Eigen::Matrix3d readCartMat(readCartTransform.linear());
         Eigen::Vector6d readCartVel = jacobianEigen*readJointVel;
            
         //Joint torque to Cartesian wrench
         auto decompositionJac = jacobianEigen.transpose().fullPivHouseholderQr();
         Eigen::Vector6d readCartwrench = decompositionJac.solve(readJointTau-gravityEigen);
         
         Eigen::Matrix3d newTargetCartMat;
         newTargetCartMat.setIdentity();
         if(initial_configuration_Franka == 0 || initial_configuration_Franka == 1){
         	newTargetCartMat(1,1) = 0;
         	newTargetCartMat(1,2) = 0;
         	newTargetCartMat(2,1) = 0;
         	newTargetCartMat(2,2) = 1;
         }else if(initial_configuration_Franka == 2){
         	newTargetCartMat(1,1) = 0;
         	newTargetCartMat(1,2) = -1;
         	newTargetCartMat(2,1) = 1;
         	newTargetCartMat(2,2) = 0;
         }
         
            
         //State Cartesian pose initialization
         if (timeController <= 0.0) {
             targetCartPos = readCartPos;
						 desCartOriMat = newTargetCartMat;
             targetCartMat = newTargetCartMat;
					   targetCartMat_FIC = newTargetCartMat;
						 desiredJointPos = readJointPos;
					   IdentityMat_7x7.setIdentity();
						 gripperPressed = 0;
 					}

         //Time update
         timeController += period.toSec();
         timeControllerGlobalVar = timeController;
          //
				 //SPLINE FOR NEW TARGET BUILT BY UI
			   if (lastGotoMode == false && goToMode.load() == true){
				     splinesGotoMode[0] = uoe::SplineQuintic();
             splinesGotoMode[0].addPoint(timeController, readCartPos(0));
             splinesGotoMode[0].addPoint(timeController + goTime, frankaTargetPosLinXSphere.load());
						 splinesGotoMode[1] = uoe::SplineQuintic();
						 splinesGotoMode[1].addPoint(timeController, readCartPos(1));
						 splinesGotoMode[1].addPoint(timeController + goTime, frankaTargetPosLinYSphere.load());
	   				 splinesGotoMode[2] = uoe::SplineQuintic();
	   				 splinesGotoMode[2].addPoint(timeController, readCartPos(2));
	   				 splinesGotoMode[2].addPoint(timeController + goTime, frankaTargetPosLinZSphere.load());
			   }
					
         //Filter haptic gripper button
         filterGripRatio.update(hapticReadGripRatio.load());
            
         //Compute target cartesian velocity
         Eigen::Vector6d targetCartVel = Eigen::Vector6d::Zero();

         //Bound absolute Cartesian velocity
         if (frankaIsControl.load()) {

						// ORIGINAL CODE
						/*targetCartVel(0) = uoe::ClampAbsolute(frankaLinVelGain*hapticReadLinPosX.load(), frankaLinVelClamp);
              targetCartVel(1) = uoe::ClampAbsolute(frankaLinVelGain*hapticReadLinPosY.load(), frankaLinVelClamp);
              targetCartVel(2) = uoe::ClampAbsolute(frankaLinVelGain*hapticReadLinPosZ.load(), frankaLinVelClamp);*/
						//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//
					
						//MODIFIED ONE (IN SYNC WITH HAPTIC SIGMA 7 (POSITION ALONG) X-Y-Z COORDINATES)
            targetCartVel(2) = uoe::ClampAbsolute(frankaLinVelGain*-hapticReadLinPosX.load(), frankaLinVelClamp);
            targetCartVel(1) = uoe::ClampAbsolute(frankaLinVelGain*hapticReadLinPosY.load(), frankaLinVelClamp);
            targetCartVel(0) = uoe::ClampAbsolute(frankaLinVelGain*hapticReadLinPosZ.load(), frankaLinVelClamp);
						//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//

						// ORIGINAL CODE
            /*targetCartVel(3) = uoe::ClampAbsolute(frankaAngVelGain*hapticReadAngPosX.load(), frankaAngVelClamp);
              targetCartVel(4) = uoe::ClampAbsolute(frankaAngVelGain*hapticReadAngPosY.load(), frankaAngVelClamp);
              targetCartVel(5) = uoe::ClampAbsolute(frankaAngVelGain*hapticReadAngPosZ.load(), frankaAngVelClamp);*/
						//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//
						
						//MODIFIED ONE (IN SYNC WITH HAPTIC SIGMA 7 (ORIENTATION AROUND) X-Y-Z COORDINATES)
					  targetCartVel(5) = uoe::ClampAbsolute(frankaAngVelGain*-hapticReadAngPosX.load(), frankaAngVelClamp);
            targetCartVel(4) = uoe::ClampAbsolute(frankaAngVelGain*hapticReadAngPosY.load(), frankaAngVelClamp);
            targetCartVel(3) = uoe::ClampAbsolute(frankaAngVelGain*hapticReadAngPosZ.load(), frankaAngVelClamp);
						//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//	//
					}

          //Convert Cartesian velocity end effector local frame to world frame
          targetCartVel.segment(0, 3) = readCartMat*targetCartVel.segment(0, 3);
          targetCartVel.segment(3, 3) = readCartMat*targetCartVel.segment(3, 3);
            
          //Target velocity filtering
          filterTargetVel.update(targetCartVel);
          targetCartVel = filterTargetVel.value();

					// SETTING THE VIRTUAL CONSTRAINT SIZES
					//maxDisp_Franka.setConstant(virtConstraintSize_Franka);
					maxDisp_Ori_Franka.setConstant(virtConstraintRotSize_Franka);

					// DEMO_exp 
					maxDisp_Franka[0] = 0.025;
					maxDisp_Franka[1] = 0.025;
					maxDisp_Franka[2] = 0.025;
										
					// INITIALIZATION OF DESIRED AND CURRENT CONFIGURATION OF THE END-EFFECTOR FOR THE FRACTAL IMPEDANCE CONTROL
					// DEMO
					buffVec_6x1[0] = buffVec_PosX[0];
					buffVec_6x1[1] = buffVec_PosY[0];
					buffVec_6x1[2] = buffVec_PosZ[0];
					buffVec_6x1[3] = buffVec_OriX[0];
					buffVec_6x1[4] = buffVec_OriY[0];
					buffVec_6x1[5] = buffVec_OriZ[0];

          buffVec_Vel_6x1[0] = buffVec_Vel_PosX[0];
          buffVec_Vel_6x1[1] = buffVec_Vel_PosY[0];
          buffVec_Vel_6x1[2] = buffVec_Vel_PosZ[0];
          buffVec_Vel_6x1[3] = buffVec_Vel_OriX[0];
          buffVec_Vel_6x1[4] = buffVec_Vel_OriY[0];
          buffVec_Vel_6x1[5] = buffVec_Vel_OriZ[0];

					buffVec_MtoS_6x1[0] = buffVec_MtoS_PosX[0];
					buffVec_MtoS_6x1[1] = buffVec_MtoS_PosY[0];
					buffVec_MtoS_6x1[2] = buffVec_MtoS_PosZ[0];

					buffVec_MtoS_6x1_lowBandWidth_timeDelay[0] = buffVec_MtoS_PosX_lowBandWidth_timeDelay[0];
					buffVec_MtoS_6x1_lowBandWidth_timeDelay[1] = buffVec_MtoS_PosY_lowBandWidth_timeDelay[0];
					buffVec_MtoS_6x1_lowBandWidth_timeDelay[2] = buffVec_MtoS_PosZ_lowBandWidth_timeDelay[0];

					delayCurPosVec[0] = buffVec_6x1[0];
					delayCurPosVec[1] = buffVec_6x1[1];
					delayCurPosVec[2] = buffVec_6x1[2];

					delayCurVelLinearVec[0] = buffVec_Vel_6x1[0];
					delayCurVelLinearVec[1] = buffVec_Vel_6x1[1];
					delayCurVelLinearVec[2] = buffVec_Vel_6x1[2];

					curPos3x1_Franka[0] = readCartPos(0);
				  curPos3x1_Franka[1] = readCartPos(1); 
				  curPos3x1_Franka[2] = readCartPos(2);  

					//DEMO INITIAL CONFIGURATION
					if (frankaDesPosHapticCtl == 0){
						if (initial_configuration_Franka == 0){
							//INITIAL CONFIGURATION FOR HUMAN ROBOT INTERATION 
				 			desPos3x1_Franka[0] = +0.316;
							desPos3x1_Franka[1] = -0.009;
				 			// desPos3x1_Franka[1] = -0.009 + 0.1 * sin(0.0625 * 2 * 3.14 * timeControllerGlobalVar);
				 			desPos3x1_Franka[2] = +0.431;
				 			
				 			
						}else if (initial_configuration_Franka == 1){
							//INITIAL CONFIGURATION FOR BILATERAL TELEOPERATION
							desPos3x1_Franka[0] = +0.401;
							desPos3x1_Franka[1] = +0.426;
				 			desPos3x1_Franka[2] = +0.426;
						}else if (initial_configuration_Franka == 2){
							//INITIAL CONFIGURATION FOR BILATERAL TELEOPERATION
							// 0.311781 -0.362779  0.429069
				 			desPos3x1_Franka[0] = +0.316;
							desPos3x1_Franka[1] = +0.250;
				 			// desPos3x1_Franka[1] = -0.009 + 0.1 * sin(0.0625 * 2 * 3.14 * timeControllerGlobalVar);
				 			desPos3x1_Franka[2] = +0.70;
						}
						
					
						
					}else{

						for (int i = 0; i < 3; i++){
							if (preEndEff_DesPos3x1_Franka[i] == 0){
								if (initial_configuration_Franka == 0){
									//INITIAL CONFIGURATION FOR HUMAN ROBOT INTERATION 
									desPos3x1_Franka[0] = +0.316;
				 					desPos3x1_Franka[1] = -0.009;
									// desPos3x1_Franka[1] = -0.009 + 0.1 * sin(0.0625 * 2 * 3.14 * timeControllerGlobalVar);
				 					desPos3x1_Franka[2] = +0.431;
				 					

								}else if (initial_configuration_Franka == 1){
									//INITIAL CONFIGURATION FOR BILATERAL TELEOPERATION
									desPos3x1_Franka[0] = +0.401;
									desPos3x1_Franka[1] = +0.426;
				 					desPos3x1_Franka[2] = +0.426;
								}else if (initial_configuration_Franka == 2){
							//INITIAL CONFIGURATION FOR BILATERAL TELEOPERATION
							// 0.311781 -0.362779  0.429069
				 			desPos3x1_Franka[0] = +0.316;
							desPos3x1_Franka[1] = +0.250;
				 			// desPos3x1_Franka[1] = -0.009 + 0.1 * sin(0.0625 * 2 * 3.14 * timeControllerGlobalVar);
				 			desPos3x1_Franka[2] = +0.70;
						}
								
							
								preEndEff_DesPos3x1_Franka = desPos3x1_Franka;
							}else{
								desPos3x1_Franka[0] = preEndEff_DesPos3x1_Franka[0];
								desPos3x1_Franka[1] = preEndEff_DesPos3x1_Franka[1];
								desPos3x1_Franka[2] = preEndEff_DesPos3x1_Franka[2];
							}
						}

						/*for (int i = 0; i < 3; i++){
							for (int j = 0; j < 3; j++){
								if (preTargetCartMat_Franka(i,j) == 0){
									preTargetCartMat_Franka(i,j) = desCartOriMat(i,j);
								
								}else{
									desCartOriMat(i,j) = preTargetCartMat_Franka(i,j);
								}
							}
						}*/
					}

					//std::cout << preEndEff_DesPos3x1_Franka.transpose() << std::endl << std::endl;
					//std::cout << preTargetCartMat_Franka << std::endl << std::endl;
					

					//desOri3x1_Franka[0] = +0.000;
					//desOri3x1_Franka[1] = -1.571;
					//desOri3x1_Franka[2] = +0.000;

					Eigen::Vector3d desOri3x1_Franka_double;
					desOri3x1_Franka_double = uoe::MatrixToAxis(desCartOriMat);

					desOri3x1_Franka[0] = desOri3x1_Franka_double(0);
					desOri3x1_Franka[1] = desOri3x1_Franka_double(1);
					desOri3x1_Franka[2] = desOri3x1_Franka_double(2);

					// TELEOPERATION w.r.t HAPTIC POSITION/ORIENTATION 
					hapticPos[0] = scalingFactorPos * hapticReadLinPosX.load();
					hapticPos[1] = scalingFactorPos * hapticReadLinPosY.load();
					hapticPos[2] = scalingFactorPos * hapticReadLinPosZ.load();

					hapticOri[0] = scalingFactorOri * hapticReadAngPosX.load();
					hapticOri[1] = scalingFactorOri * hapticReadAngPosY.load();
					hapticOri[2] = scalingFactorOri * hapticReadAngPosZ.load();

					MtoShapticPos[0] = scalingFactorPos * MtoShapticReadLinPosX.load();
					MtoShapticPos[1] = scalingFactorPos * MtoShapticReadLinPosY.load();
					MtoShapticPos[2] = scalingFactorPos * MtoShapticReadLinPosZ.load();

					//std::cout << MtoShapticReadLinPosX.load() << " ---  " << MtoShapticPos[0] << std::endl << std::endl;

					MtoShapticOri[0] = scalingFactorOri * MtoShapticReadAngPosX.load();
					MtoShapticOri[1] = scalingFactorOri * MtoShapticReadAngPosY.load();
					MtoShapticOri[2] = scalingFactorOri * MtoShapticReadAngPosZ.load();

          //Integrate target Cartesian velocity to pose
          //targetCartPos += period.toSec()*targetCartVel.segment(0, 3);
					//targetCartMat = uoe::AxisToMatrix(period.toSec()*targetCartVel.segment(3, 3))*targetCartMat;

					//LOW-BANDWIDTH TEST
					// DIRECT POSITION AND ORIENTATION MAPPING 
					if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 0){
							targetCartPos[0] = desPos3x1_Franka[0] + MtoShapticPos[0];
							targetCartPos[1] = desPos3x1_Franka[1] + MtoShapticPos[1];
							targetCartPos[2] = desPos3x1_Franka[2] + MtoShapticPos[2];
					
					}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 1){
							targetCartPos[0] = desPos3x1_Franka[0] + buffVec_MtoS_6x1[0];
							targetCartPos[1] = desPos3x1_Franka[1] + buffVec_MtoS_6x1[1];
							targetCartPos[2] = desPos3x1_Franka[2] + buffVec_MtoS_6x1[2];

					}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 2){
							targetCartPos[0] = desPos3x1_Franka[0] + MtoShapticPos_Delayed[0];
							targetCartPos[1] = desPos3x1_Franka[1] + MtoShapticPos_Delayed[1];
							targetCartPos[2] = desPos3x1_Franka[2] + MtoShapticPos_Delayed[2];

					}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 3){
							targetCartPos[0] = desPos3x1_Franka[0] + buffVec_MtoS_6x1_lowBandWidth_timeDelay[0];
							targetCartPos[1] = desPos3x1_Franka[1] + buffVec_MtoS_6x1_lowBandWidth_timeDelay[1];
							targetCartPos[2] = desPos3x1_Franka[2] + buffVec_MtoS_6x1_lowBandWidth_timeDelay[2];
					}

					targetCartMat = uoe::AxisToMatrix((uoe::MatrixToAxis(desCartOriMat)) + (uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri))));

					//targetCartMat = desCartOriMat + uoe::AxisToMatrix(hapticOri)* readCartMat;

					//CORRECT
					//std::cout << (uoe::MatrixToAxis(desCartOriMat)).transpose() << std::endl << std::endl;

					//CORRECT
					//std::cout << uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri)).transpose() << std::endl << std::endl;

					//CORRECT
					//std::cout << (uoe::MatrixToAxis(desCartOriMat)).transpose() + (uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri))).transpose() << std::endl << std::endl;

					//CORRECT
					//std::cout << uoe::AxisToMatrix((uoe::MatrixToAxis(desCartOriMat)) + (uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri)))) << std::endl << std::endl;

					// TESTING
					if (gripperGapVec[0] >= 0.001){
							gripperPressed = 0;
							//std::cout << "new desired position ==> " << (preEndEff_DesPos3x1_Franka).transpose() << std::endl << std::endl;
							//std::cout << "new desired orientation ==> " << preTargetCartMat_Franka << std::endl << std::endl;
						
					}else{
							gripperPressed = 1;
							
							//LOW-BANDWIDTH TEST
							/*newEndEff_DesPos3x1_Franka[0] = curPos3x1_Franka[0] + period.toSec() * hapticPos(0);
							newEndEff_DesPos3x1_Franka[1] = curPos3x1_Franka[1] + period.toSec() * hapticPos(1);
							newEndEff_DesPos3x1_Franka[2] = curPos3x1_Franka[2] + period.toSec() * hapticPos(2);*/ // WRONG!

							if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 0){
									newEndEff_DesPos3x1_Franka[0] = preEndEff_DesPos3x1_Franka[0] + period.toSec() * MtoShapticPos(0);
									newEndEff_DesPos3x1_Franka[1] = preEndEff_DesPos3x1_Franka[1] + period.toSec() * MtoShapticPos(1);
									newEndEff_DesPos3x1_Franka[2] = preEndEff_DesPos3x1_Franka[2] + period.toSec() * MtoShapticPos(2);
							
							}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 1){
									newEndEff_DesPos3x1_Franka[0] = preEndEff_DesPos3x1_Franka[0] + period.toSec() * buffVec_MtoS_6x1(0);
									newEndEff_DesPos3x1_Franka[1] = preEndEff_DesPos3x1_Franka[1] + period.toSec() * buffVec_MtoS_6x1(1);
									newEndEff_DesPos3x1_Franka[2] = preEndEff_DesPos3x1_Franka[2] + period.toSec() * buffVec_MtoS_6x1(2);
							}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 2){
									newEndEff_DesPos3x1_Franka[0] = preEndEff_DesPos3x1_Franka[0] + period.toSec() * MtoShapticPos_Delayed(0);
									newEndEff_DesPos3x1_Franka[1] = preEndEff_DesPos3x1_Franka[1] + period.toSec() * MtoShapticPos_Delayed(1);
									newEndEff_DesPos3x1_Franka[2] = preEndEff_DesPos3x1_Franka[2] + period.toSec() * MtoShapticPos_Delayed(2);
							}else if (std_lowBandwidth_timeDelay_TeleOp_FIC_Act == 3){
									newEndEff_DesPos3x1_Franka[0] = preEndEff_DesPos3x1_Franka[0] + period.toSec() * buffVec_MtoS_6x1_lowBandWidth_timeDelay(0);
									newEndEff_DesPos3x1_Franka[1] = preEndEff_DesPos3x1_Franka[1] + period.toSec() * buffVec_MtoS_6x1_lowBandWidth_timeDelay(1);
									newEndEff_DesPos3x1_Franka[2] = preEndEff_DesPos3x1_Franka[2] + period.toSec() * buffVec_MtoS_6x1_lowBandWidth_timeDelay(2);
							}

							
							//std::cout << "new desired position ==> " << (newEndEff_DesPos3x1_Franka).transpose() << std::endl << std::endl;

							/*newTargetCartMat_Franka = uoe::AxisToMatrix(uoe::MatrixToAxis(readCartMat) + period.toSec() * uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri)));*/ // ==> WRONG!

							// *** targetCartMat = uoe::AxisToMatrix((uoe::MatrixToAxis(desCartOriMat)) + (uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri)))); ***

							newTargetCartMat_Franka = uoe::AxisToMatrix((uoe::MatrixToAxis(preTargetCartMat_Franka)) + period.toSec() * (uoe::MatrixToAxis(uoe::AxisToMatrix(hapticOri))));//+  uoe::MatrixToAxis(uoe::AxisToMatrix(period.toSec() * MtoShapticOri)));

							//newTargetCartMat_Franka = uoe::AxisToMatrix((uoe::MatrixToAxis(preTargetCartMat_Franka)) + period.toSec() * (uoe::MatrixToAxis(uoe::AxisToMatrix(MtoShapticOri))));

							//std::cout << "new desired orientation ==> " << newTargetCartMat_Franka << std::endl << std::endl;

							for (int i = 0; i < 3; i++){
								if (fabs (Wrench_FIC_Franka_doubleType[i]) >= Fmax_Franka[i]){
									newEndEff_DesPos3x1_Franka[i] = preEndEff_DesPos3x1_Franka[i];
								}

								if (fabs (Wrench_FIC_Franka_doubleType[i+3]) >= TauMax_Franka[i]){
									newTargetCartMat_Franka = preTargetCartMat_Franka;
								}
							}
					}
					
					if (goToMode.load() == true){
					    newEndEff_DesPos3x1_Franka[0] = splinesGotoMode[0].pos(timeController);
					    newEndEff_DesPos3x1_Franka[1] = splinesGotoMode[1].pos(timeController);
					    newEndEff_DesPos3x1_Franka[2] = splinesGotoMode[2].pos(timeController);
					    if (timeController >= splinesGotoMode[0].max()) {
					        goToMode.store(false);
					    }
					}
					
					lastGotoMode = goToMode.load();
					
					readJointPosGlobalVar = readJointPos;

					// POSITION MAPPING WITH CONTROL TIMING
					//targetCartPos += period.toSec() * hapticPos;
					//targetCartMat = uoe::AxisToMatrix(period.toSec() * hapticOri)*targetCartMat;

          //Export joint position
          frankaReadJoint1.store(readJointPos(0));
          frankaReadJoint2.store(readJointPos(1));
          frankaReadJoint3.store(readJointPos(2));
          frankaReadJoint4.store(readJointPos(3));
          frankaReadJoint5.store(readJointPos(4));
          frankaReadJoint6.store(readJointPos(5));
          frankaReadJoint7.store(readJointPos(6));
            
          //Compute tracking Cartesian position and velocity vector
          Eigen::Vector6d errorCartPos = Eigen::Vector6d::Zero();
					Eigen::Vector3d errorCartOri = Eigen::Vector3d::Zero();
          errorCartPos.segment(0, 3) = targetCartPos - readCartPos;
          errorCartPos.segment(3, 3) = uoe::MatrixToAxis(targetCartMat*readCartMat.transpose());
					errorCartOri = uoe::MatrixToAxis(targetCartMat_FIC*readCartMat.transpose());
          Eigen::Vector6d errorCartVel = targetCartVel - readCartVel;

					Eigen::VectorXd targetCartOri(3);
					targetCartOri = (uoe::MatrixToAxis(targetCartMat));
					
					// CONDITIONS FOR NOT GOING BEYOND THE BOUNDARIES (WORKING ON IT!!!)
						/*
						for (int i = 0; i < 3; i++){
							if (targetCartPos(i) >= desPos3x1_Franka(i) + maxDisp_Franka[i]){
								targetCartPos(i) = desPos3x1_Franka[i] + maxDisp_Franka[i];						
							}else if(targetCartPos(i) <= desPos3x1_Franka[i] - maxDisp_Franka[i]){
								targetCartPos(i) = desPos3x1_Franka[i] - maxDisp_Franka[i];
							}

							if (targetCartOri[i] >= desOri3x1_Franka[i] + maxDisp_Ori_Franka[i]){
								targetCartOri[i] = desOri3x1_Franka[i] + maxDisp_Ori_Franka[i];
								targetCartMat = uoe::AxisToMatrix(targetCartOri);
							}else if (targetCartOri[i] <= desOri3x1_Franka[i] - maxDisp_Ori_Franka[i]){
								targetCartOri[i] = desOri3x1_Franka[i] - maxDisp_Ori_Franka[i];
								targetCartMat = uoe::AxisToMatrix(targetCartOri);
							}
						}*/

            //Export target position
            frankaTargetPosLinX.store(targetCartPos(0));
            frankaTargetPosLinY.store(targetCartPos(1));
            frankaTargetPosLinZ.store(targetCartPos(2));
            frankaTargetPosAngX.store(errorCartOri(0));
            frankaTargetPosAngY.store(errorCartOri(1));
            frankaTargetPosAngZ.store(errorCartOri(2));

            //Compute filtering
            filterErrorVel.update(errorCartVel);

            //Initialize and configure Cartesian impedance matrices
            Eigen::Matrix<double,6,6> matCartStiffness = Eigen::Matrix<double,6,6>::Zero();
            Eigen::Matrix<double,6,6> matCartDamping = Eigen::Matrix<double,6,6>::Zero();
            Eigen::Matrix<double,6,6> matCartIntegral = Eigen::Matrix<double,6,6>::Zero();
            matCartStiffness(0,0) = frankaControlMaxEffortLinX/frankaControlMaxErrorLinX;
            matCartStiffness(1,1) = frankaControlMaxEffortLinY/frankaControlMaxErrorLinY;
            matCartStiffness(2,2) = frankaControlMaxEffortLinZ/frankaControlMaxErrorLinZ;
            matCartStiffness(3,3) = frankaControlMaxEffortAngX/frankaControlMaxErrorAngX;
            matCartStiffness(4,4) = frankaControlMaxEffortAngY/frankaControlMaxErrorAngY;
            matCartStiffness(5,5) = frankaControlMaxEffortAngZ/frankaControlMaxErrorAngZ;
            matCartDamping(0,0) = 2.0*frankaControlDampingRatioLin*std::sqrt(matCartStiffness(0,0));
            matCartDamping(1,1) = 2.0*frankaControlDampingRatioLin*std::sqrt(matCartStiffness(1,1));
            matCartDamping(2,2) = 2.0*frankaControlDampingRatioLin*std::sqrt(matCartStiffness(2,2));
            matCartDamping(3,3) = 2.0*frankaControlDampingRatioAng*std::sqrt(matCartStiffness(3,3));
            matCartDamping(4,4) = 2.0*frankaControlDampingRatioAng*std::sqrt(matCartStiffness(4,4));
            matCartDamping(5,5) = 2.0*frankaControlDampingRatioAng*std::sqrt(matCartStiffness(5,5));
            
            //Compute impedance control Cartesian wrench
            Eigen::Vector6d wrenchStiffness = matCartStiffness*errorCartPos;
            Eigen::Vector6d wrenchDamping = matCartDamping*filterErrorVel.value();
            wrenchIntegral += period.toSec()*matCartIntegral*errorCartPos;
            wrenchStiffnessGlobalVar = wrenchStiffness;

						// // // // // // // // // // // // // // // // NEW PART OF THE CODE // // // // // // // // // // // // // // // //

						frankaFIC frankaFIC;
										
						// TEST CLASS (FRANKA)
						//frankaFIC.printSomething_Franka();

						// TESTING MY CONTROLLER //
						// // // // // // // // START OF FRACTAL IMPEDANCE CONTROLLER FOR LOOP // // // // // // // //
						for (int i = 0; i < 3; i++){ 
							
							//LOW-BANDWIDTH TEST
							//curPos3x1_Franka[i] = readCartPos(i); 
							//desPos3x1_Franka[i] = targetCartPos(i);
							if (lowBandwidth_Franka_FIC_Act == 0){
								position_error_Franka[i] = desPos3x1_Franka[i] - curPos3x1_Franka[i];
							}else{
								position_error_Franka[i] = desPos3x1_Franka[i] - (buffVec_6x1[i]);
							}

							curOri3x1_Franka[i] = (uoe::MatrixToAxis(readCartMat))[i];
							desOri3x1_Franka[i] = (uoe::MatrixToAxis(targetCartMat))[i]; 
							//angular_error_Franka[i] = desOri3x1_Franka[i] - curOri3x1_Franka[i];
							//angular_error_Franka[i] = errorCartPos(i+3);
							angular_error_Franka[i] = errorCartOri(i);

							velLinearError_Franka[i] = targetCartVel[i] - readCartVel[i];
	 						velAngularError_Franka[i] = targetCartVel[i+3] - readCartVel[i+3];
	 						
	 						// PLOTING SIGMA 7 LINEAR AND ANGULAR VELOCITIES
	            Franka_velLinear_X.store(velLinearError_Franka[0]);
	            Franka_velLinear_Y.store(velLinearError_Franka[1]);
	            Franka_velLinear_Z.store(velLinearError_Franka[2]);
	            Franka_velAngular_X.store(velAngularError_Franka[0]);
	            Franka_velAngular_Y.store(velAngularError_Franka[1]);
	            Franka_velAngular_Z.store(velAngularError_Franka[2]);

              // DEMO
							kConstPos_Franka(i,i) = 100;
							kConstOri_Franka(i,i) = 5;
							dConstLinear_Franka(i,i) = scalingFactorPos * 2.5;
							dConstAngular_Franka(i,i) = 1.25;

							frankaFIC.calculate_F_Max_Franka (maxDisp_Franka, Fmax_Franka, i);
        			frankaFIC.calculate_Tau_Max_Franka (maxDisp_Ori_Franka, TauMax_Franka, i);

							// K_SYSTEM_MAX, MAXIMUM DISPLACEMENT AND BETA CALCULATION FOR BOTH END-EEFECTOR POSITION AND ORIENTATION
        			frankaFIC.calculate_K_Max_Beta_Pos_Ori_Franka (Fmax_Franka, maxDisp_Franka, TauMax_Franka, maxDisp_Ori_Franka, KmaxSystem_Franka, beta_Franka, KmaxSystem_Ori_Franka, beta_Ori_Franka, i);

							// K_TOTAL AND K_VAR CALCULATION FOR BOTH POSITION AND OREINTATION
        			frankaFIC.get_K_Total_Pos_Ori_Franka (beta_Franka, kVarPos_Franka, kTotalDesPos_Franka, beta_Ori_Franka, kVarOri_Franka, kTotalDesOri_Franka, i);

							if (kTotalDesPos_Franka(i,i) > KmaxSystem_Franka(i,i) && beta_Franka[i] > 0){
        					kVarPos_Franka(i,i) = Fmax_Franka[i]/fabs(position_error_Franka[i]) - kConstPos_Franka(i,i);
							}//else if(kTotalDesPos_Franka(i,i) > KmaxSystem_Franka(i,i)){
                	//kVarPos_Franka(i,i) = 0;
        			//}

        			if (kTotalDesOri_Franka(i,i) > KmaxSystem_Ori_Franka(i,i)&& beta_Ori_Franka[i] > 0){
           				kVarOri_Franka(i,i) = TauMax_Franka[i]/fabs(angular_error_Franka[i]) - kConstOri_Franka(i,i);
        			}//else if(kTotalDesOri_Franka(i,i) > KmaxSystem_Ori_Franka(i,i)){
                	//kVarOri_Franka(i,i) = 0;
        			//}

	    				// ATTRACTIVE FORCE FOR POSITION AND ORIENTATION CTRL CALCULATION
        			Fk_Franka[i] = (kConstPos_Franka(i,i) + kVarPos_Franka(i,i)) * (position_error_Franka[i]);
        			Fk_Ori_Franka[i] = (kConstOri_Franka(i,i) + kVarOri_Franka(i, i)) * angular_error_Franka[i];

        			// DAMPING FORCES FOR LINEAR AND ANGULAR VELOCITIES - DEFINITIONS
							//LOW-BANDWIDTH TEST
							if (lowBandwidth_Franka_FIC_Act == 0){
        				Fd_Pos_Franka[i] = dConstLinear_Franka(i,i) * velLinearError_Franka[i];
							}else{
								Fd_Pos_Franka[i] = dConstLinear_Franka(i,i) * buffVec_Vel_6x1[i];
							}
        			Fd_Ori_Franka[i] = dConstAngular_Franka(i,i) * velAngularError_Franka[i];

							//////////////////////// PROPOSED METHOD - END EFFECTOR POSITION ////////////////////////
        			frankaFIC.calculation_K1_K2_Kd_Max_PMax_PMid_Max_Dist_Vec_Franka (K1_3x3_Franka, K2_3x3_Franka, KdMax_Franka, Pmax3x1_Franka, Pmid3x1_Franka, maxDistVec_3x1_Franka, beta_Franka, KmaxSystem_Franka, maxDisp_Franka, Fmax_Franka, i);
							
        			//////////////////////// PROPOSED METHOD - END EFFECTOR ORIENTATION ////////////////////////
        			frankaFIC.calculation_K1_K2_Kd_Max_PMax_PMid_Max_Dist_Vec_Ori_Franka (K1_Ori_3x3_Franka, K2_Ori_3x3_Franka, KdMax_Ori_Franka, Pmax3x1_Ori_Franka, Pmid3x1_Ori_Franka, maxDistVec_Ori_3x1_Franka, beta_Ori_Franka, KmaxSystem_Ori_Franka, maxDisp_Ori_Franka, TauMax_Franka, i);
							
							//////////////////////// DEFINING THE CONDITIONS FOR THE PROPOSED METHOD - END EFFECTOR POSITION ////////////////////////
        			frankaFIC.calculation_Kc_DampConstLiner_FMidCtrl_FkCtrl_Franka (Kc_3x3_Franka, dConstLinear_Franka, F_midPointErrorCtrl_Franka, Fk_Franka, i);
        			
        			//////////////////////// DEFINING THE CONDITIONS FOR THE PROPOSED METHOD - END EFFECTOR ORIENTATION ////////////////////////
        			frankaFIC.calculation_Kc_DampConstAngular_FMidCtrl_FkCtrl_Ori_Franka(Kc_Ori_3x3_Franka, dConstAngular_Franka, F_midPointErrorCtrl_Ori_Franka, Fk_Ori_Franka, i);

							// FRACTAL IMPEDANCE CONTROL END-EFFECTOR FORCES AND TORQUES
							F_FIC_Franka[i] = Fk_Franka[i] + F_midPointErrorCtrl_Franka[i] + Fd_Pos_Franka[i];
							Tau_FIC_Franka[i] = Fk_Ori_Franka[i] + F_midPointErrorCtrl_Ori_Franka[i] + Fd_Ori_Franka[i];

							Wrench_FIC_Franka[i] = F_FIC_Franka[i];
							Wrench_FIC_Franka[i+3] = Tau_FIC_Franka[i];

							// TOTAL SYSTEM STIFFNESS PROFILE
							if (lowBandwidth_Franka_FIC_Act == 0){
								velLinearError_Cur_Buff_Sel_Franka = velLinearError_Franka;
							}else{
								velLinearError_Cur_Buff_Sel_Franka = buffVec_Vel_6x1;
							}
							//LOW-BANDWIDTH TEST
							if (frankaFIC.sgn(position_error_Franka[i], 0.005f) == frankaFIC.sgn(velLinearError_Cur_Buff_Sel_Franka[i], 0.005f)){
							//if (frankaFIC.sgn(position_error_Franka[i], 0.005f) == frankaFIC.sgn(buffVec_Vel_6x1[i], 0.005f)){
								kTotal6x6_Franka(i,i) = kConstPos_Franka(i,i) + kVarPos_Franka(i,i);
							}else{
								kTotal6x6_Franka(i,i) = Kc_3x3_Franka(i,i);
							}

							if (frankaFIC.sgn(angular_error_Franka[i], 0.005f) == frankaFIC.sgn(velAngularError_Franka[i], 0.005f)){
								kTotal6x6_Franka(i+3,i+3) = kConstOri_Franka(i,i) + kVarOri_Franka(i,i);
							}else{
								kTotal6x6_Franka(i+3,i+3) = Kc_Ori_3x3_Franka(i,i);
							}

							// ENERGY EQUATION ALONG/AROUND EACH AXIS
							if (fabs(position_error_Franka[i] <= maxDisp_Franka[i])){
            		potEnergy_6x1_Franka[i] = (pow(maxDisp_Franka[i], 2) * (pow(Fmax_Franka[i]/maxDisp_Franka[i] - kConstPos_Franka(i,i), (pow(position_error_Franka[i], 2)/pow(maxDisp_Franka[i], 2)))) - pow(maxDisp_Franka[i], 2) + kConstPos_Franka(i,i) * log((Fmax_Franka[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]) * fabs(pow(position_error_Franka[i], 2)))/(2 * log((Fmax_Franka[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]));
  						}else{
            		potEnergy_6x1_Franka[i] = (kConstPos_Franka(i,i) * pow(maxDisp_Franka[i], 2))/2 - pow(maxDisp_Franka[i], 2)/(2 * log((Fmax_Franka[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i])) - Fmax_Franka[i] * (maxDisp_Franka[i] - fabs(position_error_Franka[i])) + (maxDisp_Franka[i] * (Fmax_Franka[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i]))/(2 * log((Fmax_Franka[i] - kConstPos_Franka(i,i) * maxDisp_Franka[i])/maxDisp_Franka[i]));
  						}

  					if (fabs(angular_error_Franka[i]) <= maxDisp_Ori_Franka[i]){
            	potEnergy_6x1_Franka[i+3] = (pow(maxDisp_Ori_Franka[i], 2) * (pow(TauMax_Franka[i]/maxDisp_Ori_Franka[i] - kConstOri_Franka(i,i), (pow(angular_error_Franka[i], 2)/pow(maxDisp_Ori_Franka[i], 2)))) - pow(maxDisp_Ori_Franka[i], 2) + kConstOri_Franka(i,i) * log((TauMax_Franka[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]) * fabs(pow(angular_error_Franka[i], 2)))/(2 * log((TauMax_Franka[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]));
  					}else{
            	potEnergy_6x1_Franka[i+3] = (kConstOri_Franka(i,i) * pow(maxDisp_Ori_Franka[i], 2))/2 - pow(maxDisp_Ori_Franka[i], 2)/(2 * log((TauMax_Franka[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i])) - TauMax_Franka[i] * (maxDisp_Ori_Franka[i] - fabs(angular_error_Franka[i])) + (maxDisp_Ori_Franka[i] * (TauMax_Franka[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i]))/(2 * log((TauMax_Franka[i] - kConstOri_Franka(i,i) * maxDisp_Ori_Franka[i])/maxDisp_Ori_Franka[i]));
  					}

							} // // // // // // // // END OF FRACTAL IMPEDANCE CONTROLLER FOR LOOP // // // // // // // //

							// ATTEMPT FOR NULLSPACE CONTROL
							for(int i = 0; i < 7; i++){
								kConstNull_Franka(i,i) = 50;
								dConstNull_Franka(i,i) = 5;
								IdentityMat_7x7(i,i) = 1;

								tauCtrlNull_Franka(i) = kConstNull_Franka(i,i) * (desiredJointPos(i) - readJointPos(i)) - dConstNull_Franka(i,i) * readJointVel(i);
								tauCtrlNull_Franka_doubleType[i] = (double) tauCtrlNull_Franka[i];
							}
							
							//pseudoInvJac = ((jacobianEigen.transpose() * jacobianEigen).inverse()) * jacobianEigen.transpose();
							pseudoInvJac = jacobianEigen.transpose() * (jacobianEigen * jacobianEigen.transpose()).inverse();
							NullSpaceCtrl = (IdentityMat_7x7 - pseudoInvJac * jacobianEigen) * tauCtrlNull_Franka_doubleType;

							// DOUBLE TYPE OF FRACTAL IMPEDANCE WRENCH
							for (int i = 0; i < 6; i++){
								Wrench_FIC_Franka_doubleType[i] = (double) Wrench_FIC_Franka[i];
							}

						//std::cout << "FIC Wrench Franka Left ==> " << Wrench_FIC_Franka_doubleType.transpose() << std::endl;					

            //Compute desired control torque in joint space
            std::array<double, 7> cmdTauArray;
            Eigen::Map<Eigen::Matrix<double, 7, 1>> tauCmdEigen(cmdTauArray.data());
            tauCmdEigen.setZero();

						// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ON SCREEN PRINITING SECTION // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //
						//printf ("desPos (%+0.03f %+0.03f %+0.03f) m | curPos (%+0.03f %+0.03f %+0.03f) m | posError (%+0.03f %+0.03f %+0.03f) m | desOri (%+0.03f %+0.03f %+0.03f) rad | curOri (%+0.03f %+0.03f %+0.03f) rad | oriError (%+0.03f %+0.03f %+0.03f) rad \r", desPos3x1_Franka[0], desPos3x1_Franka[1], desPos3x1_Franka[2] , curPos3x1_Franka[0], curPos3x1_Franka[1], curPos3x1_Franka[2], position_error_Franka[0], position_error_Franka[1], position_error_Franka[2], desOri3x1_Franka[0], desOri3x1_Franka[1], desOri3x1_Franka[2] , curOri3x1_Franka[0], curOri3x1_Franka[1], curOri3x1_Franka[2], angular_error_Franka[0], angular_error_Franka[1], angular_error_Franka[2]);

						// CONTROLLER ELEMENTS PRINTING FOR TESTING
						//printf ("Fmax (%+0.03f %+0.03f %+0.03f) N | TauMax (%+0.03f %+0.03f %+0.03f) Nm\r", Fmax_Franka[0], Fmax_Franka[1], Fmax_Franka[2], TauMax_Franka[0], TauMax_Franka[1], TauMax_Franka[2]);
						//printf ("KmaxSystem_Franka (%+0.03f %+0.03f %+0.03f) N/m \r", KmaxSystem_Franka(0,0), KmaxSystem_Franka(1,1), KmaxSystem_Franka(2,2));
						//printf ("Fk_Franka (%+0.03f %+0.03f %+0.03f) N | Fk_Ori_Franka (%+0.03f %+0.03f %+0.03f) Nm |\r", Fk_Franka[0], Fk_Franka[1], Fk_Franka[2], Fk_Ori_Franka[0], Fk_Ori_Franka[1], Fk_Ori_Franka[2]);
						//printf ("posError - prePosError (%+0.03f %+0.03f %+0.03f) m \r", position_error_Franka[0] - preEndEff_PosError3x1_Franka[0], position_error_Franka[1] - preEndEff_PosError3x1_Franka[1], position_error_Franka[2] - preEndEff_PosError3x1_Franka[2]);
						//printf ("F_FIC_Franka (%+0.03f %+0.03f %+0.03f) N | Tau_FIC_Franka (%+0.03f %+0.03f %+0.03f) Nm\r", F_FIC_Franka[0], F_FIC_Franka[1], F_FIC_Franka[2], Tau_FIC_Franka[0], Tau_FIC_Franka[1], Tau_FIC_Franka[2]);
						
						//printf ("F_FIC_Franka (%+0.03f %+0.03f %+0.03f) N | Tau_FIC_Franka (%+0.03f %+0.03f %+0.03f) Nm\r", Wrench_FIC_Franka_doubleType[0], Wrench_FIC_Franka_doubleType[1], Wrench_FIC_Franka_doubleType[2], Wrench_FIC_Franka_doubleType[3], Wrench_FIC_Franka_doubleType[4], Wrench_FIC_Franka_doubleType[5]);

						//printf ("delta_torque (%+0.03f %+0.03f %+0.03f %+0.03f %+0.03f %+0.03f %+0.03f) \r", tauCmdEigen[0] - tauCmdEigen_Pre[0], tauCmdEigen[1] - tauCmdEigen_Pre[1], tauCmdEigen[2] - tauCmdEigen_Pre[2], tauCmdEigen[3] - tauCmdEigen_Pre[3], tauCmdEigen[4] - tauCmdEigen_Pre[4], tauCmdEigen[5] - tauCmdEigen_Pre[5], tauCmdEigen[6] - tauCmdEigen_Pre[6]);

						//printf ("cur_pre_posError (%+0.03f) \r", position_error_Franka[0] - preEndEff_PosError3x1_Franka[0]); 

						//printf ("NullSpaceCtrl Trq (%+0.03f %+0.03f %+0.03f %+0.03f %+0.03f %+0.03f %+0.03f) Nm \r", NullSpaceCtrl[0], NullSpaceCtrl[1], NullSpaceCtrl[2], NullSpaceCtrl[3], NullSpaceCtrl[4], NullSpaceCtrl[5], NullSpaceCtrl[6]);
						//printf ("jntPosError (%+0.03f %+0.03f %+0.03f %+0.03f %+0.03f %+0.03f %+0.03f) Nm \r", desiredJointPos[0] - readJointPos[0], desiredJointPos[1] - readJointPos[1], desiredJointPos[2] - readJointPos[2], desiredJointPos[3] - readJointPos[3], desiredJointPos[4] - readJointPos[4], desiredJointPos[5] - readJointPos[5], desiredJointPos[6] - readJointPos[6]);
						
						// END-EFFECTOR DESIRED/CURRENT LINEAR VELOCITY AND LINEAR VELOCITY ERROR PRINTING
						//printf ("velLinearError (%+0.03f %+0.03f %+0.03f) m/s | velAngularError (%+0.03f %+0.03f %+0.03f) rad/s \r", velLinearError_Franka[0], velLinearError_Franka[1], velLinearError_Franka[2] ,velAngularError_Franka[0], velAngularError_Franka[1], velAngularError_Franka[2]);
						
						// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 

            //Clamp stiffness control wrench
            wrenchStiffness(0) = uoe::ClampAbsolute(wrenchStiffness(0), frankaControlMaxEffortLinX);
            wrenchStiffness(1) = uoe::ClampAbsolute(wrenchStiffness(1), frankaControlMaxEffortLinY);
            wrenchStiffness(2) = uoe::ClampAbsolute(wrenchStiffness(2), frankaControlMaxEffortLinZ);
            wrenchStiffness(3) = uoe::ClampAbsolute(wrenchStiffness(3), frankaControlMaxEffortAngX);
            wrenchStiffness(4) = uoe::ClampAbsolute(wrenchStiffness(4), frankaControlMaxEffortAngY);
            wrenchStiffness(5) = uoe::ClampAbsolute(wrenchStiffness(5), frankaControlMaxEffortAngZ);

						//Clamp integral control wrench
            wrenchIntegral(0) = uoe::ClampRange(wrenchIntegral(0), -frankaControlMaxEffortLinX-wrenchStiffness(0), frankaControlMaxEffortLinX-wrenchStiffness(0));
            wrenchIntegral(1) = uoe::ClampRange(wrenchIntegral(1), -frankaControlMaxEffortLinY-wrenchStiffness(1), frankaControlMaxEffortLinY-wrenchStiffness(1));
            wrenchIntegral(2) = uoe::ClampRange(wrenchIntegral(2), -frankaControlMaxEffortLinZ-wrenchStiffness(2), frankaControlMaxEffortLinZ-wrenchStiffness(2));
            wrenchIntegral(3) = uoe::ClampRange(wrenchIntegral(3), -frankaControlMaxEffortAngX-wrenchStiffness(3), frankaControlMaxEffortAngX-wrenchStiffness(3));
            wrenchIntegral(4) = uoe::ClampRange(wrenchIntegral(4), -frankaControlMaxEffortAngY-wrenchStiffness(4), frankaControlMaxEffortAngY-wrenchStiffness(4));
            wrenchIntegral(5) = uoe::ClampRange(wrenchIntegral(5), -frankaControlMaxEffortAngZ-wrenchStiffness(5), frankaControlMaxEffortAngZ-wrenchStiffness(5));

            //TODO XXX
            frankaRead1.store(100.0*errorCartPos(2));
            frankaRead2.store(wrenchStiffness(2));
            frankaRead3.store(wrenchIntegral(2));
            frankaRead4.store(wrenchStiffness(2) + wrenchIntegral(2));

						// PLOTING THE FRACTAL IMPEDANCE CONTROL WRNECHES
						wrenchFIC_Fx.store(Wrench_FIC_Franka_doubleType[0]);
						wrenchFIC_Fy.store(Wrench_FIC_Franka_doubleType[1]);
						wrenchFIC_Fz.store(Wrench_FIC_Franka_doubleType[2]);
						wrenchFIC_Tau_x.store(Wrench_FIC_Franka_doubleType[3]);
						wrenchFIC_Tau_y.store(Wrench_FIC_Franka_doubleType[4]);
						wrenchFIC_Tau_z.store(Wrench_FIC_Franka_doubleType[5]);
						
						// PLOTING THE FRACTAL IMPEDANCE CONTROL WRNECHES - FEEDBACK ON SIGMA 7
						// , , , , , 
						feedBack_wrenchFIC_Fx.store(Wrench_FIC_Sigma_doubleType[0] - 1 * Wrench_FIC_Franka_doubleType[0]);
						feedBack_wrenchFIC_Fy.store(Wrench_FIC_Sigma_doubleType[1] - 1 * Wrench_FIC_Franka_doubleType[1]);
						feedBack_wrenchFIC_Fz.store(Wrench_FIC_Sigma_doubleType[2] - 1 * Wrench_FIC_Franka_doubleType[2]);
						feedBack_wrenchFIC_Tau_x.store(Wrench_FIC_Sigma_doubleType[3] - 0.015 * Wrench_FIC_Franka_doubleType[3]);
						feedBack_wrenchFIC_Tau_y.store(Wrench_FIC_Sigma_doubleType[4] - 0.015 * Wrench_FIC_Franka_doubleType[4]);
						feedBack_wrenchFIC_Tau_z.store(Wrench_FIC_Sigma_doubleType[5] - 0.015 * Wrench_FIC_Franka_doubleType[5]);

						//std::cout << "DesPose_3x1_Error ==> " << (desPos3x1_Franka - preEndEff_DesPos3x1_Franka).transpose() << std::endl << std::endl;

						if (fractalImpedanceAct == 0){

							// WITHOUT VIRTUAL CONSTRAINTS
							tauCmdEigen = coriolisEigen + jacobianEigen.transpose()*(wrenchStiffness + wrenchDamping + wrenchIntegral) + NullSpaceCtrl;

							deltaTau = tauCmdEigen - tauCmdEigen_Pre;
							//std::cout << "delta commanded torque ==> " << deltaTau.transpose() << std::endl << std::endl;

						}else{

							// WITH VIRTUAL CONSTRAINTS
            	tauCmdEigen = coriolisEigen + jacobianEigen.transpose()*(wrenchStiffness /*+ wrenchDamping + wrenchIntegral*/  + Wrench_FIC_Franka_doubleType) + NullSpaceCtrl;

							deltaTau = tauCmdEigen - tauCmdEigen_Pre;

							//std::cout << "delta commanded torque ==> " << deltaTau.transpose() << std::endl << std::endl;

							for (int i = 0; i < 7; i++){
								deltaTau(i) = uoe::ClampAbsolute(deltaTau(i), 165 * period.toSec());
							}

							tauCmdEigen = deltaTau + tauCmdEigen_Pre;
							
							}
							
							tauCmdEigenGlobalVar = tauCmdEigen;
							cnt = cnt + 1;
							
							//std::cout << cnt << std::endl;
							//std::cout << "Interactive Force Left: " << optoForceX.load() << "," << optoForceY.load() << "," <<  optoForceZ.load() << std::endl << std::endl;
    				
				// PLOTTING THE SWITCHING CONDITION FOR HAPTIC DEVICE
				div_conv_posX.store(div_conv_pos_ori_sigma[0]);
				div_conv_posY.store(div_conv_pos_ori_sigma[1]);
				div_conv_posZ.store(div_conv_pos_ori_sigma[2]);
				div_conv_oriX.store(div_conv_pos_ori_sigma[3]);
				div_conv_oriY.store(div_conv_pos_ori_sigma[4]);
				div_conv_oriZ.store(div_conv_pos_ori_sigma[5]);

				// PLOTTING THE SWITCHING CONDITION FOR FRANKA PANDA
				div_conv_posX_franka.store(div_conv_pos_ori_franka[0]);
				div_conv_posY_franka.store(div_conv_pos_ori_franka[1]);
				div_conv_posZ_franka.store(div_conv_pos_ori_franka[2]);
				div_conv_oriX_franka.store(div_conv_pos_ori_franka[3]);
				div_conv_oriY_franka.store(div_conv_pos_ori_franka[4]);
				div_conv_oriZ_franka.store(div_conv_pos_ori_franka[5]);

				//PLOTING THE ROBOT TORQUES
				tau_1.store(tauCmdEigen[0]);
				tau_2.store(tauCmdEigen[1]);
				tau_3.store(tauCmdEigen[2]);
				tau_4.store(tauCmdEigen[3]);
				tau_5.store(tauCmdEigen[4]);
				tau_6.store(tauCmdEigen[5]);
				tau_7.store(tauCmdEigen[6]);

				//PLOTING THE ROBOT TORQUES
				deltaTau_1.store(deltaTau[0]/period.toSec());
				deltaTau_2.store(deltaTau[1]/period.toSec());
				deltaTau_3.store(deltaTau[2]/period.toSec());
				deltaTau_4.store(deltaTau[3]/period.toSec());
				deltaTau_5.store(deltaTau[4]/period.toSec());
				deltaTau_6.store(deltaTau[5]/period.toSec());
				deltaTau_7.store(deltaTau[6]/period.toSec());


				// PREVIOUS VALUES OF ROBOT JOINT TORQUES
				tauCmdEigen_Pre = tauCmdEigen;
		
				// END-EFFECTOR POSE AND VEL ERROR IN THE PREVIOUS TIME STEP
				//LOW-BANDWIDTH TEST
				preEndEff_PosError3x1_Franka = position_error_Franka;
				if (lowBandwidth_Franka_FIC_Act == 0){
		    	preEndEff_VelError3x1_Franka = velLinearError_Franka;
				}else{
					preEndEff_VelError3x1_Franka = buffVec_Vel_6x1;
				}
		    preEndEff_OriError3x1_Franka = angular_error_Franka;
		    preEndEff_VelOriError3x1_Franka = velAngularError_Franka;
				preEndEff_DesPos3x1_Franka = newEndEff_DesPos3x1_Franka;
				preTargetCartMat_Franka = newTargetCartMat_Franka;

				// DEMO
				
				buffVec_PosX[countBuff] = curPos3x1_Franka[0];
    		buffVec_PosY[countBuff] = curPos3x1_Franka[1];
    		buffVec_PosZ[countBuff] = curPos3x1_Franka[2];

    		buffVec_Vel_PosX[countBuff] = velLinearError_Franka[0];
    		buffVec_Vel_PosY[countBuff] = velLinearError_Franka[1];
    		buffVec_Vel_PosZ[countBuff] = velLinearError_Franka[2];

				buffVec_MtoS_PosX[countBuff] = MtoShapticPos[0];
				buffVec_MtoS_PosY[countBuff] = MtoShapticPos[1];
				buffVec_MtoS_PosZ[countBuff] = MtoShapticPos[2];

				buffVec_MtoS_PosX_lowBandWidth_timeDelay[countBuff] = MtoShapticPos_Delayed[0];
				buffVec_MtoS_PosY_lowBandWidth_timeDelay[countBuff] = MtoShapticPos_Delayed[1];
				buffVec_MtoS_PosZ_lowBandWidth_timeDelay[countBuff] = MtoShapticPos_Delayed[2];
    		
				countBuff++;

				if (countBuff == vecSize){
					countBuff = 0;
				}

				/*
    		buffVec_OriX[count] = angular_error[0];
    		buffVec_OriY[count] = angular_error[1];
    		buffVec_OriZ[count] = angular_error[2];

				buffVec_Vel_OriX[count] = velAngularError[0];
    		buffVec_Vel_OriY[count] = velAngularError[1];
    		buffVec_Vel_OriZ[count] = velAngularError[2];
				*/

				//std::cout << "Left Arm curJointPose: " << readJointPos.transpose() << std::endl << std::endl;
				//std::cout << "Left Arm curPose: " << curPos3x1_Franka.transpose() << std::endl << std::endl; 

				
				

				//Terminal condition
        if (frankaIsDone.load()) {
            return franka::MotionFinished(franka::Torques(cmdTauArray));
        } else {
            return franka::Torques(cmdTauArray);
        }
		};

        robot.control(callbackControl, false, 1000.0);
    	} catch (const franka::Exception& e) {
        std::cout << "Franka Exception: " << e.what() << std::endl;
        return;
    	}
}

// LOW-BANDWIDTH TEST
// DATA PRINTING
void mainDataPrint(){
    long lastCnt = 0; 
    while (!frankaIsDone.load()) {
      if (timeControllerGlobalVar > 0){
        if (cnt > lastCnt){
          std::ofstream bilateral_telemanipulation_ctrl_output;
          bilateral_telemanipulation_ctrl_output.open("leftArm_bilateral_telemanipulation_ctrl_output.txt", std::ios_base::app);
			    bilateral_telemanipulation_ctrl_output <<  timeControllerGlobalVar << "," << tauCmdEigenGlobalVar[0] << "," << tauCmdEigenGlobalVar[1] << "," << tauCmdEigenGlobalVar[2] <<
    	    "," << tauCmdEigenGlobalVar[3] << "," << tauCmdEigenGlobalVar[4] << "," << tauCmdEigenGlobalVar[5] << "," << tauCmdEigenGlobalVar[6] <<
			    "," << readJointPosGlobalVar[0] << "," << readJointPosGlobalVar[1] << "," << readJointPosGlobalVar[2] <<
    	    "," << readJointPosGlobalVar[3] << "," << readJointPosGlobalVar[4] << "," << readJointPosGlobalVar[5] << "," << readJointPosGlobalVar[6] <<
			    "," << position_error_Franka[0] << "," << position_error_Franka[1] << "," << position_error_Franka[2] << 
			    "," << desPos3x1_Franka[0] << "," << desPos3x1_Franka[1]  << "," << desPos3x1_Franka[2] <<
    	    "," << curPos3x1_Franka[0] << "," << curPos3x1_Franka[1] << "," << curPos3x1_Franka[2]  <<
    	    "," << angular_error_Franka[0] << "," << angular_error_Franka[1] << "," << angular_error_Franka[2] <<
    	    "," << desOri3x1_Franka[0] << "," << desOri3x1_Franka[1] << "," << desOri3x1_Franka[2] <<
    	    "," << curOri3x1_Franka[0] << "," << curOri3x1_Franka[1] << "," << curOri3x1_Franka[2] <<
			    "," << velLinearError_Franka[0] << "," << velLinearError_Franka[1] << "," << velLinearError_Franka[2] <<
			    "," << velAngularError_Franka[0] << "," << velAngularError_Franka[1] << "," << velAngularError_Franka[2] <<
    	    "," << kConstPos_Franka(0,0) + kVarPos_Franka(0,0) << "," << kConstPos_Franka(1,1) + kVarPos_Franka(1,1) << "," << kConstPos_Franka(2,2) + kVarPos_Franka(2,2) <<
    	    "," << kConstOri_Franka(0,0) + kVarOri_Franka(0,0)<< "," << kConstOri_Franka(1,1) + kVarOri_Franka(1,1) << "," << kConstOri_Franka(2,2) + kVarOri_Franka(2,2) <<
    	    "," << wrenchStiffnessGlobalVar[0] << "," << wrenchStiffnessGlobalVar[1] << "," << wrenchStiffnessGlobalVar[2] <<
    	    "," << wrenchStiffnessGlobalVar[3] << "," << wrenchStiffnessGlobalVar[4] << "," << wrenchStiffnessGlobalVar[5] <<
			    "," << Wrench_FIC_Franka_doubleType[0] << "," << Wrench_FIC_Franka_doubleType[1] << "," << Wrench_FIC_Franka_doubleType[2] <<
    	    "," << Wrench_FIC_Franka_doubleType[3] << "," << Wrench_FIC_Franka_doubleType[4] << "," << Wrench_FIC_Franka_doubleType[5] <<
    	    "," << potEnergy_6x1_Franka[0] << "," << potEnergy_6x1_Franka[1] << "," << potEnergy_6x1_Franka[2] <<
    	    "," << potEnergy_6x1_Franka[3] << "," << potEnergy_6x1_Franka[4] << "," << potEnergy_6x1_Franka[5] <<
			    "," << newEndEff_DesPos3x1_Franka[0] << "," << newEndEff_DesPos3x1_Franka[1] << "," << newEndEff_DesPos3x1_Franka[2] << 
			    "," << posError[0] << "," << posError[1] << "," << posError[2] <<
    	    "," << desPos[0] << "," << desPos[1]  << "," << desPos[2] <<
    	    "," << curPos[0] << "," << curPos[1] << "," << curPos[2] <<
    	    "," << oriError[0] << "," << oriError[1] << "," << oriError[2] <<
    	    "," << desOri[0] << "," << desOri[1] << "," << desOri[2] <<
    	    "," << curOri[0] << "," << curOri[1] << "," << curOri[2] <<
			    "," << velLinearError[0] << "," << velLinearError[1] << "," << velLinearError[2] <<
			    "," << velAngularError[0] << "," << velAngularError[1] << "," << velAngularError[2] <<
    	    "," << kConstPos(0,0) + kVarPos(0,0) << "," << kConstPos(1,1) + kVarPos(1,1) << "," << kConstPos(2,2) + kVarPos(2,2) <<
    	    "," << kConstOri(0,0) + kVarOri(0,0) << "," << kConstOri(1,1) + kVarOri(1,1) << "," << kConstOri(2,2) + kVarOri(2,2) <<
    	    "," << Wrench_FIC_Sigma_doubleType[0] << "," << Wrench_FIC_Sigma_doubleType[1] << "," << Wrench_FIC_Sigma_doubleType[2] <<
    	    "," << Wrench_FIC_Sigma_doubleType[3] << "," << Wrench_FIC_Sigma_doubleType[4] << "," << Wrench_FIC_Sigma_doubleType[5] <<
    	    "," << potEnergy_6x1[0] << "," << potEnergy_6x1[1] << "," << potEnergy_6x1[2] <<
    	    "," << potEnergy_6x1[3] << "," << potEnergy_6x1[4] << "," << potEnergy_6x1[5] << 
    	    "," << optoForceX.load() << "," << optoForceY.load() << "," <<  optoForceZ.load() << 
    	    "," << optoTorqueX.load() << "," << optoTorqueY.load() << "," << optoTorqueZ.load() << 
    	    "," << frankaTargetPosLinXSphere.load() << "," << frankaTargetPosLinYSphere.load() << "," << frankaTargetPosLinZSphere.load() << 
					"," << buffVec_6x1[0] << "," << buffVec_6x1[1] << "," << buffVec_6x1[2] << 
					"," << buffVec_Vel_6x1[0] << "," << buffVec_Vel_6x1[1] << "," << buffVec_Vel_6x1[2] << 
					"," << buffVec_6x1_Sigma[0] << "," << buffVec_6x1_Sigma[1] << "," << buffVec_6x1_Sigma[2] << 
					"," << buffVec_Vel_6x1_Sigma[0] << "," << buffVec_Vel_6x1_Sigma[1] << "," << buffVec_Vel_6x1_Sigma[2] << 
					"," << MtoShapticPos[0] << "," << MtoShapticPos[1] << "," << MtoShapticPos[2] << 
					"," << buffVec_MtoS_6x1[0] << "," << buffVec_MtoS_6x1[1] << "," << buffVec_MtoS_6x1[2] << 
					"," << optoFT_forceCoef * optoForceX.load() << "," << optoFT_forceCoef * optoForceX.load() << "," << optoFT_forceCoef * optoForceZ.load() << 
					"," << buffVec_Wrench_6x1[0] << "," << buffVec_Wrench_6x1[1] << "," << buffVec_Wrench_6x1[2] << 
					"," << MtoShapticPos_Delayed[0] << "," << MtoShapticPos_Delayed[1] << "," << MtoShapticPos_Delayed[2] << 
					"," << FeedBackForce_Delayed[0] << "," << FeedBackForce_Delayed[1] << "," << FeedBackForce_Delayed[2] << 
					"," << buffVec_MtoS_6x1_lowBandWidth_timeDelay[0] << "," << buffVec_MtoS_6x1_lowBandWidth_timeDelay[1] << "," << buffVec_MtoS_6x1_lowBandWidth_timeDelay[2] << 
					"," << buffVec_Wrench_6x1_lowBandWidth_timeDelay[0] << "," << buffVec_Wrench_6x1_lowBandWidth_timeDelay[1] << "," << buffVec_Wrench_6x1_lowBandWidth_timeDelay[2] << std::endl;
    	    lastCnt = cnt;
		    }
		  }
		}
}

void mainGUI(){

	//Model initialization
  uoe::Model model(uoe::SystemResolvePath("package://uoe_franka_control/urdf/panda_arm.urdf"));
    
  //Viewer initialization
  uoe::ViewerModel viewer;
  viewer.update();
  viewer.setViewMode(uoe::ViewerModel::Mesh);
    
  //Visualization loop
  while (!frankaIsDone.load()) {
  
	//Posture update
  model.setDOFPos("panda_joint1", frankaReadJoint1.load());
  model.setDOFPos("panda_joint2", frankaReadJoint2.load());
  model.setDOFPos("panda_joint3", frankaReadJoint3.load());
  model.setDOFPos("panda_joint4", frankaReadJoint4.load());
  model.setDOFPos("panda_joint5", frankaReadJoint5.load());
  model.setDOFPos("panda_joint6", frankaReadJoint6.load());
  model.setDOFPos("panda_joint7", frankaReadJoint7.load());
  model.updateState();
  
  //End effector position
  Eigen::Vector3d posHand = model.position("panda_hand", "ROOT");
  Eigen::Vector3d posInfo1 = posHand + Eigen::Vector3d(0.2, 0.2, 0.1);
  Eigen::Vector3d posInfo2 = posHand + Eigen::Vector3d(0.2, 0.2, -0.1);
  Eigen::Vector3d posInfo3 = Eigen::Vector3d(-0.5, -0.5, 0.5);
  Eigen::Vector3d posInfo4 = Eigen::Vector3d(-0.5, 0.5, 0.5);
  
  //Target end effector position
  Eigen::Vector3d targetPosHand(frankaTargetPosLinX.load(), frankaTargetPosLinY.load(), frankaTargetPosLinZ.load());
 	Eigen::Vector3d targetOriHand(frankaTargetPosAngX.load(), frankaTargetPosAngY.load(), frankaTargetPosAngZ.load());
        
	//Retrieve haptic control 
  Eigen::Vector3d controlLin(hapticReadLinPosX.load(), hapticReadLinPosY.load(), hapticReadLinPosZ.load());
  Eigen::Vector3d controlAng(hapticReadAngPosX.load(), hapticReadAngPosY.load(), hapticReadAngPosZ.load());
        
	//Retrieve haptic stiffness
  Eigen::Vector3d stiffnessLin(hapticGainPosLinX.load(), hapticGainPosLinY.load(), hapticGainPosLinZ.load());
  Eigen::Vector3d stiffnessAng(hapticGainPosAngX.load(), hapticGainPosAngY.load(), hapticGainPosAngZ.load());
        
	//Drawing
  viewer.drawGround(0.1, 10);
  DrawModel(viewer, model);
  viewer.drawFrame(model.position("panda_hand", "ROOT"), model.orientation("panda_hand", "ROOT"));
  //viewer.drawFrame(posInfo1, Eigen::Matrix3d::Identity());
  viewer.drawArrow(targetPosHand, controlLin, scalingFactorPos*controlLin.norm(), 1.0, 1.0, 0.0);
  //viewer.drawFrame(posInfo2, Eigen::Matrix3d::Identity());
  viewer.drawArrow(targetPosHand, controlAng, (scalingFactorOri/3)*controlAng.norm(), 1.0, 0.0, 1.0);
  //viewer.drawFrame(posInfo3, Eigen::Matrix3d::Identity());
  viewer.drawFrame(targetPosHand, Eigen::Matrix3d::Identity());
	viewer.drawSphere(model.position("panda_hand", "ROOT"), 0.0075, 1.0, 0.75, 0.0);
	//viewer.drawSphere(targetPosHand, 0.0075, 0.0, 1.0, 1.0);
		
	Eigen::Vector3d endEffPosGroundLevelProj;
	endEffPosGroundLevelProj[0] = 0; endEffPosGroundLevelProj[1] = 0; endEffPosGroundLevelProj[2] = targetPosHand[2];
	
	/*for (int i = 0; i <= 1000; i++){
		color1 = color1 + 0.001 * i;
		if (color1 == 1000){
			for (int i = 0; i <= 1000; i++){
				color1 = color1 - 0.001 * i;
			}
		}
	}*/

	viewer.drawSphere(targetPosHand - endEffPosGroundLevelProj, 0.0075, 1.0, 0.0, 0.0);

	Eigen::Vector3d boxCenter;
	boxCenter[0] = 0.2; boxCenter[1] = -0.5; boxCenter[2] = 0.18;	
	//viewer.drawBox(0.15, 0.245, 0.05, boxCenter, Eigen::Matrix3d::Identity(), 1.0, 1.0, 0.0);

	Eigen::Vector3d L1Start; Eigen::Vector3d L1End; double L1Thickness = 5.0;
	Eigen::Vector3d L2Start; Eigen::Vector3d L2End; double L2Thickness = 5.0;
	Eigen::Vector3d L3Start; Eigen::Vector3d L3End; double L3Thickness = 5.0;
	Eigen::Vector3d L4Start; Eigen::Vector3d L4End;	double L4Thickness = 5.0;

	L1Start[0] = 0; L1Start[1] = 0; L1Start[2] = 0;
	L1End[0] = 0; L1End[1] = -0.5; L1End[2] = 0;
	//viewer.drawLine(L1Start, L1End,  L1Thickness, 0, 1.0, 0);

	L2Start[0] = 0; L2Start[1] = -0.5; L2Start[2] = 0;
	L2End[0] = 0.2; L2End[1] = -0.5; L2End[2] = 0;
	//viewer.drawLine(L2Start, L2End,  L2Thickness, 1.0, 0, 0);

	L3Start[0] = 0.2; L3Start[1] = -0.5; L3Start[2] = 0;
	L3End[0] = 0.2; L3End[1] = -0.5; L3End[2] = 0.18;
	//viewer.drawLine(L3Start, L3End,  L3Thickness, 0, 0, 1.0);

	L4Start[0] = targetPosHand[0] ; L4Start[1] = targetPosHand[1] ; L4Start[2] = 0;
	L4End[0] = targetPosHand[0]; L4End[1] = targetPosHand[1]; L4End[2] = targetPosHand[2];
	//viewer.drawLine(L4Start, L4End,  L4Thickness, 0.5, 0.5, 0.5);

  //viewer.drawEllipsoid(posInfo3, Eigen::Matrix3d::Identity(), 0.001*stiffnessLin, 0.4, 0.4, 1.0);
  //viewer.drawFrame(posInfo4, Eigen::Matrix3d::Identity());
  //viewer.drawEllipsoid(posInfo4, Eigen::Matrix3d::Identity(), 0.5*stiffnessAng, 0.4, 0.4, 1.0);
  
	if (frankaIsControl.load()) {
      viewer.drawCylinder(Eigen::Vector3d(0.0, 0.0, 0.0), 0.2, 0.1, 1.0, 0.4, 0.4);
  }

	//Target End-effector for UI use INITIALIZATION by pressing NUM #5
	if (viewer.isKeyPressed(sf::Keyboard::H)) {
	if (initial_configuration_Franka == 0){
     		frankaTargetPosLinXSphere.store(+0.316);
      	frankaTargetPosLinYSphere.store(-0.009);
      	frankaTargetPosLinZSphere.store(+0.431);
      }else if(initial_configuration_Franka == 1){
      	frankaTargetPosLinXSphere.store(+0.401);
      	frankaTargetPosLinYSphere.store(+0.426);
      	frankaTargetPosLinZSphere.store(+0.426);
      }else if(initial_configuration_Franka == 2){
      	frankaTargetPosLinXSphere.store(+0.316);
      	frankaTargetPosLinYSphere.store(+0.250);
      	frankaTargetPosLinZSphere.store(+0.70);
      }
      
	}
	
		if (viewer.isKeyPressed(sf::Keyboard::Home)) {
			if(initial_configuration_Franka == 2){
      	frankaTargetPosLinXSphere.store(+0.316);
      	frankaTargetPosLinYSphere.store(0.250);
      	frankaTargetPosLinZSphere.store(+0.70);
     }
      
	}

  //User keyborad inputs
	//Target End-effector for UI use value load
	//Eigen::Vector3d targetPosSphere(frankaTargetPosLinXSphere.load(), frankaTargetPosLinYSphere.load(), frankaTargetPosLinZSphere.load());
	
	targetPosSphere[0] = frankaTargetPosLinXSphere.load();
	targetPosSphere[1] = frankaTargetPosLinYSphere.load();
	targetPosSphere[2] = frankaTargetPosLinZSphere.load();
	//Target End-effector PosX++ by pressing NUM #8 
	if (viewer.isKeyPressed(sf::Keyboard::Y)) {
			frankaTargetPosLinXSphere.store(frankaTargetPosLinXSphere.load() - 0.005);
			std::cout << "Moving along ==> +X " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinXSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosX-- by pressing NUM #2
	if (viewer.isKeyPressed(sf::Keyboard::N)) {
			frankaTargetPosLinXSphere.store(frankaTargetPosLinXSphere.load() + 0.005);
			std::cout << "Moving along ==> -X " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinXSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosY-- by pressing NUM #4
	if (viewer.isKeyPressed(sf::Keyboard::G)) {
			frankaTargetPosLinYSphere.store(frankaTargetPosLinYSphere.load() - 0.005);
			std::cout << "Moving along ==> +Y " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinYSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosY++ by pressing NUM #6
	if (viewer.isKeyPressed(sf::Keyboard::J)) {
			frankaTargetPosLinYSphere.store(frankaTargetPosLinYSphere.load() + 0.005);
			std::cout << "Moving along ==> -Y " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinYSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosZ++ by pressing NUM #9
	if (viewer.isKeyPressed(sf::Keyboard::U)) {
			frankaTargetPosLinZSphere.store(frankaTargetPosLinZSphere.load() + 0.005);
			std::cout << "Moving along ==> +Z " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinZSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosZ-- by pressing NUM #3
	if (viewer.isKeyPressed(sf::Keyboard::M)) {
			frankaTargetPosLinZSphere.store(frankaTargetPosLinZSphere.load() - 0.005);
			std::cout << "Moving along ==> -Z " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinZSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	if (viewer.isKeyPressed(sf::Keyboard::Space)) {
			//Eigen::Vector3d newTargetEndEffPosUI;
			newTargetEndEffPosUI[0] = frankaTargetPosLinXSphere.load();
			newTargetEndEffPosUI[1] = frankaTargetPosLinYSphere.load();
			newTargetEndEffPosUI[2] = frankaTargetPosLinZSphere.load();
			std::cout << "Left Arm Current Target End-effector Sphere Position ==> " << newTargetEndEffPosUI.transpose() << std::endl;
			std::cout << "Left Arm Franka Current End-Effector Position ==> " << curPos3x1_Franka.transpose() << std::endl;
			std::cout << "Left Arm Franka Current End-Effector Joint Position ==> " << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	if (viewer.isKeyPressed(sf::Keyboard::LShift) && goToMode.load() == false){
			goToMode.store(true);
			std::cout << "Space Key Pressed? " << goToMode << std::endl << std::endl;
	}
	
	if (viewer.isKeyPressed(sf::Keyboard::LControl)){
			goToMode.store(false);
			std::cout << "Abort Key Pressed? " << goToMode << std::endl << std::endl;
	}
	/*
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////// DUAL-ARM CONFIGURATION CONTROL ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
		if (viewer.isKeyPressed(sf::Keyboard::F1)) {
			frankaTargetPosLinXSphere.store(frankaTargetPosLinXSphere.load() - 0.005);
			std::cout << "Moving along ==> -X " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinXSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosX-- by pressing NUM #2
	if (viewer.isKeyPressed(sf::Keyboard::F2)) {
			frankaTargetPosLinXSphere.store(frankaTargetPosLinXSphere.load() + 0.005);
			std::cout << "Moving along ==> +X " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinXSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosY-- by pressing NUM #4
	if (viewer.isKeyPressed(sf::Keyboard::F3)) {
			frankaTargetPosLinYSphere.store(frankaTargetPosLinYSphere.load() + 0.005);
			std::cout << "Moving along ==> -Y " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinYSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosY++ by pressing NUM #6
	if (viewer.isKeyPressed(sf::Keyboard::F4)) {
			frankaTargetPosLinYSphere.store(frankaTargetPosLinYSphere.load() - 0.005);
			std::cout << "Moving along ==> +Y " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinYSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosZ++ by pressing NUM #9
	if (viewer.isKeyPressed(sf::Keyboard::F5)) {
			frankaTargetPosLinZSphere.store(frankaTargetPosLinZSphere.load() + 0.005);
			std::cout << "Moving along ==> -Z " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinZSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}

	//Target End-effector PosZ-- by pressing NUM #3
	if (viewer.isKeyPressed(sf::Keyboard::F6)) {
			frankaTargetPosLinZSphere.store(frankaTargetPosLinZSphere.load() - 0.005);
			std::cout << "Moving along ==> +Z " << "Target Sphere PosX Value ==> "<< frankaTargetPosLinZSphere.load() << std::endl << std::endl;
			//std::cout << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}
	
			if (viewer.isKeyPressed(sf::Keyboard::F8)) {
			inputRequestStage += 1;
			std::cout << "Input Request: "  << inputRequestStage << std::endl <<std::endl;
			if (inputRequestStage == 1){
				frankaTargetPosLinZSphere.store(0.525);
			}
			if (inputRequestStage == 2){
				frankaTargetPosLinYSphere.store(0.50);
			}
			if (inputRequestStage == 3){
				frankaTargetPosLinYSphere.store(0.50);
				frankaTargetPosLinZSphere.store(0.70);
			}
			if (inputRequestStage == 4){
				frankaTargetPosLinYSphere.store(0.50);
				frankaTargetPosLinXSphere.store(0.7);
				
			}
			if (inputRequestStage == 5){
				frankaTargetPosLinYSphere.store(0.50);
				frankaTargetPosLinZSphere.store(0.225);

			}
			if (inputRequestStage == 6){
				frankaTargetPosLinYSphere.store(0.50);
			}
			if (inputRequestStage == 7){
				frankaTargetPosLinYSphere.store(0.50);
				frankaTargetPosLinZSphere.store(0.7);
			}	
			if (inputRequestStage == 8){
				frankaTargetPosLinYSphere.store(0.50);
				frankaTargetPosLinXSphere.store(0.316);
			}
			if (inputRequestStage == 9){
				frankaTargetPosLinYSphere.store(0.50);
				frankaTargetPosLinZSphere.store(0.525);
			}
	}
	
	if (viewer.isKeyPressed(sf::Keyboard::F7)) {
			inputRequestStage -= 1;
			std::cout << "Input Request: "  << inputRequestStage << std::endl <<std::endl;
			if (inputRequestStage <= 0){
				inputRequestStage = 0;
			}
	}*/
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////// DUAL-ARM CONFIGURATION CONTROL ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	if (viewer.isKeyPressed(sf::Keyboard::Numpad1)) {
		inputRequestStage = 1;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinZSphere.store(0.525);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad2)){
		inputRequestStage = 2;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad3)){
		inputRequestStage = 3;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
		frankaTargetPosLinZSphere.store(0.70);
		
		//Drilling
		//frankaTargetPosLinYSphere.store(0.50);
		//frankaTargetPosLinZSphere.store(0.550);
		//frankaTargetPosLinXSphere.store(0.7);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad4)){
		inputRequestStage = 4;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
		frankaTargetPosLinXSphere.store(0.7);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad5)){
		inputRequestStage = 5;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
		frankaTargetPosLinZSphere.store(0.225);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad6)){
		inputRequestStage = 6;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad7)){
		inputRequestStage = 7;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
		frankaTargetPosLinZSphere.store(0.7);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad8)){
		inputRequestStage = 8;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
		frankaTargetPosLinXSphere.store(0.316);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Numpad9)){
		inputRequestStage = 9;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.50);
		frankaTargetPosLinZSphere.store(0.525);
	}
	if (viewer.isKeyPressed(sf::Keyboard::Delete)){
		inputRequestStage = 0;
		std::cout << "Stage: "  << inputRequestStage << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.250);
		//frankaTargetPosLinZSphere.store(0.70);
	}
		if (viewer.isKeyPressed(sf::Keyboard::PageUp)){
		//inputRequestStage = 0;
		trajectoryActPhase = 1;
		std::cout << "Stage: "  << "Trajectory Tracking Activated ==> "<< trajectoryActPhase << std::endl <<std::endl;
		frankaTargetPosLinZSphere.store(0.70);
		frankaTargetPosLinYSphere.store(0.250 - 0.1 * sin(0.0625 * 2 * 3.14 * timeControllerGlobalVar));
		goToMode.store(true);
	}
	

	if (viewer.isKeyPressed(sf::Keyboard::PageDown)){
		trajectoryActPhase = 0;
		//inputRequestStage = -1;
		std::cout << "Stage: "  << "Trajectory Tracking Deactivated ==> " << trajectoryActPhase << std::endl <<std::endl;
		frankaTargetPosLinYSphere.store(0.250);
		goToMode.store(true);
	}
	
	if (viewer.isKeyPressed(sf::Keyboard::Tab)) {
			//Eigen::Vector3d newTargetEndEffPosUI;
			newTargetEndEffPosUI[0] = frankaTargetPosLinXSphere.load();
			newTargetEndEffPosUI[1] = frankaTargetPosLinYSphere.load();
			newTargetEndEffPosUI[2] = frankaTargetPosLinZSphere.load();
			std::cout << "Left Arm Current Target End-effector Sphere Position ==> " << newTargetEndEffPosUI.transpose() << std::endl;
			std::cout << "Left Arm Franka Current End-Effector Position ==> " << curPos3x1_Franka.transpose() << std::endl;
			std::cout << "Left Arm Franka Current End-Effector Joint Position ==> " << readJointPosGlobalVar.transpose() << std::endl << std::endl; 
	}
	
	if (viewer.isKeyPressed(sf::Keyboard::Insert) && goToMode.load() == false){
			goToMode.store(true);
			if (curPos3x1_Franka[1] > 0.55){
				goToMode.store(false);
			}
			std::cout << "Start Key Pressed? " << goToMode << std::endl << std::endl;
	}
	
	if (viewer.isKeyPressed(sf::Keyboard::End)){
			goToMode.store(false);
			std::cout << "Abort Key Pressed? " << goToMode << std::endl << std::endl;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
	//abortMotionKey


	viewer.drawSphere(targetPosSphere, frankaTargetPosLinSphereSize.load(), 1.0, 1.0, 1.0);
	
	//Screen update
  if (!viewer.update(false)) {
  	break;
  }
 }
}

// TIME DELAY THREAD
void timeDelay()
{
				//DEMO
				//Create a buffer of size 1000 elements
    		uoe::TimeSeriesBuffer<double, weightedAverage> buffer_MtoShapticPos_PosX(timeDelayBuffSize);
				uoe::TimeSeriesBuffer<double, weightedAverage> buffer_MtoShapticPos_PosY(timeDelayBuffSize);	
				uoe::TimeSeriesBuffer<double, weightedAverage> buffer_MtoShapticPos_PosZ(timeDelayBuffSize);		
				uoe::TimeSeriesBuffer<double, weightedAverage> buffer_FeedBackForce_Fx(timeDelayBuffSize);
				uoe::TimeSeriesBuffer<double, weightedAverage> buffer_FeedBackForce_Fy(timeDelayBuffSize);	
				uoe::TimeSeriesBuffer<double, weightedAverage> buffer_FeedBackForce_Fz(timeDelayBuffSize);
			
				while (!frankaIsDone.load()) {
				        
        //Scheduling
        //std::this_thread::sleep_for (std::chrono::milliseconds(100));

        //Generate a stream of data
        MtoShapticPos_Raw = MtoShapticPos;
				FeedBackForce_Raw[0] = optoFT_forceCoef * optoForceX.load();
				FeedBackForce_Raw[1] = optoFT_forceCoef * optoForceY.load();
				FeedBackForce_Raw[2] = optoFT_forceCoef * optoForceZ.load();		

        //Append to buffer
        buffer_MtoShapticPos_PosX.nextPoint().value = MtoShapticPos[0];
        buffer_MtoShapticPos_PosX.nextPoint().time = timeControllerGlobalVar;
        buffer_MtoShapticPos_PosX.push();

        buffer_MtoShapticPos_PosY.nextPoint().value = MtoShapticPos[1];
        buffer_MtoShapticPos_PosY.nextPoint().time = timeControllerGlobalVar;
        buffer_MtoShapticPos_PosY.push();

        buffer_MtoShapticPos_PosZ.nextPoint().value = MtoShapticPos[2];
        buffer_MtoShapticPos_PosZ.nextPoint().time = timeControllerGlobalVar;
        buffer_MtoShapticPos_PosZ.push();

        buffer_FeedBackForce_Fx.nextPoint().value = optoFT_forceCoef * optoForceX.load();
        buffer_FeedBackForce_Fx.nextPoint().time = timeControllerGlobalVar;
        buffer_FeedBackForce_Fx.push();

        buffer_FeedBackForce_Fy.nextPoint().value = optoFT_forceCoef * optoForceY.load();
        buffer_FeedBackForce_Fy.nextPoint().time = timeControllerGlobalVar;
        buffer_FeedBackForce_Fy.push();

        buffer_FeedBackForce_Fz.nextPoint().value = optoFT_forceCoef * optoForceZ.load();
        buffer_FeedBackForce_Fz.nextPoint().time = timeControllerGlobalVar;
        buffer_FeedBackForce_Fz.push();

        //Look up an old value to simulate delay
        MtoShapticPos_Delayed[0] = buffer_MtoShapticPos_PosX.getInterpolation(timeControllerGlobalVar - 1.0).value;
				MtoShapticPos_Delayed[1] = buffer_MtoShapticPos_PosY.getInterpolation(timeControllerGlobalVar - 1.0).value;
				MtoShapticPos_Delayed[2] = buffer_MtoShapticPos_PosZ.getInterpolation(timeControllerGlobalVar - 1.0).value;
				FeedBackForce_Delayed[0] = buffer_FeedBackForce_Fx.getInterpolation(timeControllerGlobalVar - 1.0).value;
				FeedBackForce_Delayed[1] = buffer_FeedBackForce_Fy.getInterpolation(timeControllerGlobalVar - 1.0).value;
				FeedBackForce_Delayed[2] = buffer_FeedBackForce_Fz.getInterpolation(timeControllerGlobalVar - 1.0).value;
			
        //std::cout << "MtoShapticPos_Raw = " << MtoShapticPos_Raw.transpose() << ",   MtoShapticPos_Delayed = " << MtoShapticPos_Delayed.transpose() << std::endl << std::endl;
				//std::cout << "FeedBackForce_Raw = " << FeedBackForce_Raw.transpose() << ",   FeedBackForce_Delayed = " << FeedBackForce_Delayed.transpose() << std::endl << std::endl;
	}
}
// // // // // // // //

/* Control the Franka Cartesian End-effector wrench with the Sigma 7 haptic device */
int main(int argc, char** argv) {
	
	// Time Delay Thread attempt
	std::thread timeDelayThread(timeDelay);

	// DEMO
	//Start haptic Right thread
  //std::thread hapticThreadRight(mainHaptic);
  
  // Start Opto Force Sensor thread
  std::thread threadOptoForce(mainOptoForce);

	//Start haptic Left thread
  //std::thread hapticThreadLeft(mainHaptic);
    
  //Start franka Right thread
  //std::thread frankaRightThread([](){ mainFranka("frankaright"); });

	//Start franka Left thread
	std::thread frankaLeftThread([](){ mainFranka("frankaleft"); });
	
	// TEST: data printing thread
	std::thread mainDataPrintThread(mainDataPrint);

  //Start vizualization thread
  std::thread guiThread(mainGUI);
        
  //Visualization and interface thread
  uoe::Plot plot;
  double timeStart = drdGetTime();
  while (true) {
  	//Check user keyboard inputs
    if (dhdKbHit()) {
        if (dhdKbGet() == 'q') {
            break;
         }
    }
   // DEMO
   //Plot real time
   double timeNow = drdGetTime();
   /*plot.add(
    	"frankaRead1", frankaRead1.load(),
      "frankaRead2", frankaRead2.load(),
      "frankaRead3", frankaRead3.load(),
      "frankaRead4", frankaRead4.load(),
      "t", timeNow - timeStart
     );*/

		// // // // // PLOTING SIGMA 7 LINEAR AND ANGULAR VELOCITIES // // // // //
    /*plot.add(
    	//"velLinear_X", velLinear_X.load(),
      //"velLinear_Y", velLinear_Y.load(),
      "velLinear_Z", velLinear_Z.load(),
      //"velAngular_X", velAngular_X.load(),
			//"velAngular_Y", velAngular_Y.load(),
			//"velAngular_Z", velAngular_Z.load(),
      "t", timeNow - timeStart
    );*/
    
    // // // // // PLOTING SIGMA 7 LINEAR AND ANGULAR VELOCITIES // // // // //
    /*plot.add(
    	"Franka_velLinear_X", Franka_velLinear_X.load(),
      "Franka_velLinear_Y", Franka_velLinear_Y.load(),
      "Franka_velLinear_Z", Franka_velLinear_Z.load(),
      "Franka_velAngular_X", Franka_velAngular_X.load(),
			"Franka_velAngular_Y", Franka_velAngular_Y.load(),
			"Franka_velAngular_Z", Franka_velAngular_Z.load(),
      "t", timeNow - timeStart
    );*/

		// // // // // PLOTING FIC WRENCHES // // // // //
    /*plot.add(
    	"wrenchFIC_Fx", wrenchFIC_Fx.load(),
      "wrenchFIC_Fy", wrenchFIC_Fy.load(),
      "wrenchFIC_Fz", wrenchFIC_Fz.load(),
      "wrenchFIC_Tau_x", wrenchFIC_Tau_x.load(),
			"wrenchFIC_Tau_y", wrenchFIC_Tau_y.load(),
			"wrenchFIC_Tau_z", wrenchFIC_Tau_z.load(),
      "t", timeNow - timeStart
    );*/
    
    // // // // // MEASURED FORCE/TORQUE OPTO FORCE // // // // //
    /*plot.add(
            "forceX", optoForceX.load(),
            "forceY", optoForceY.load(),
            "forceZ", optoForceZ.load(),
            "torqueX", optoTorqueX.load() * 100.0,
            "torqueY", optoTorqueY.load() * 100.0,
            "torqueZ", optoTorqueZ.load() * 100.0,
            "t", timeNow - timeStart
        );*/
    
    // // // // // PLOTING FIC WRENCHES - FEEDBACK ON SIGMA 7 // // // // //
    /*plot.add(
    	"feedBack_wrenchFIC_Fx", feedBack_wrenchFIC_Fx.load(),
      "feedBack_wrenchFIC_Fy", feedBack_wrenchFIC_Fy.load(),
      "feedBack_wrenchFIC_Fz", feedBack_wrenchFIC_Fz.load(),
      "feedBack_wrenchFIC_Tau_x", feedBack_wrenchFIC_Tau_x.load(),
			"feedBack_wrenchFIC_Tau_y", feedBack_wrenchFIC_Tau_y.load(),
			"feedBack_wrenchFIC_Tau_z", feedBack_wrenchFIC_Tau_z.load(),
      "t", timeNow - timeStart
    );*/

		// // // // // PLOTING HAPTIC DEVICE SWITCHING CONDITIONS // // // // //
   /*plot.add(
    	"div_conv_posX", div_conv_posX.load(),
      "div_conv_posY", div_conv_posY.load(),
      "div_conv_posZ", div_conv_posZ.load(),
      "div_conv_oriX", div_conv_oriX.load(),
			"div_conv_oriY", div_conv_oriY.load(),
			"div_conv_oriZ", div_conv_oriZ.load(),
      "t", timeNow - timeStart
    );*/

		// // // // // PLOTING FRANKA PANDA SWITCHING CONDITIONS // // // // //
    /*plot.add(
    	"div_conv_posX_franka", div_conv_posX_franka.load(),
      "div_conv_posY_franka", div_conv_posY_franka.load(),
      "div_conv_posZ_franka", div_conv_posZ_franka.load(),
      "div_conv_oriX_franka", div_conv_oriX_franka.load(),
			"div_conv_oriY_franka", div_conv_oriY_franka.load(),
			"div_conv_oriZ_franka", div_conv_oriZ_franka.load(),
      "t", timeNow - timeStart
    );*/

		// // // // // PLOTING ROBOT TORQUES // // // // //
    /*plot.add(
    		"tau_1", tau_1.load(),
				"tau_2", tau_2.load(),
				"tau_3", tau_3.load(),
				"tau_4", tau_4.load(),
				"tau_5", tau_5.load(),
				"tau_6", tau_6.load(),
				"tau_7", tau_7.load(),
				"t", timeNow - timeStart
      );*/

    /*plot.add(
    		"deltaTau_1", deltaTau_1.load(),
				"deltaTau_2", deltaTau_2.load(),
				"deltaTau_3", deltaTau_3.load(),
				"deltaTau_4", deltaTau_4.load(),
				"deltaTau_5", deltaTau_5.load(),
				"deltaTau_6", deltaTau_6.load(),
				"deltaTau_7", deltaTau_7.load(),
				"t", timeNow - timeStart
      );*/
				// // // // // // // // // // // // // // // // //
      plot.filterLowerThan("t", timeNow-timeStart-10.0);
      plot.plot("t", "all").stream();
      /*
      plot.add(
      	"frankaReadExcitationCmd", frankaReadExcitationCmd.load(),
        "frankaReadExcitationForce", frankaReadExcitationForce.load(),
        "frankaReadExcitationDelta", frankaReadExcitationDelta.load(),
        "frankaReadExcitationVel", frankaReadExcitationVel.load(),
        "t", timeNow - timeStart
      );
      plot.filterLowerThan("t", timeNow-timeStart-60.0);
      plot.plot("t", "all").stream();
      */
      /*
      plot.add(
      	"frankaReadExternalWrenchLinX1", frankaReadExternalWrenchLinX1.load(),
        "frankaReadExternalWrenchLinX2", frankaReadExternalWrenchLinX2.load(),
        "frankaReadExternalWrenchLinX3", frankaReadExternalWrenchLinX3.load(),
        "frankaReadExternalWrenchLinX4", frankaReadExternalWrenchLinX4.load(),
      	"t", timeNow - timeStart
      );
      plot.filterLowerThan("t", timeNow-timeStart-5.0);
      plot.plot("t", "all").stream();
      */
      /*
      plot.add(
      	"hapticReadLinPosX", hapticReadLinPosX.load(),
        "hapticReadLinPosY", hapticReadLinPosY.load(),
        "hapticReadLinPosZ", hapticReadLinPosZ.load(),
        "hapticReadAngPosX", hapticReadAngPosX.load(),
        "hapticReadAngPosY", hapticReadAngPosY.load(),
        "hapticReadAngPosZ", hapticReadAngPosZ.load(),
        "t", timeNow - timeStart
      );
      plot.filterLowerThan("t", timeNow-timeStart-5.0);
      plot.plot("t", "all").stream();
      */
            
  //Scheduling
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
 }
  plot.plot("t", "all").show();
  plot.closeWindow();
  plot.clear();
   
	// DEMO
  //Stop and wait for franka thread to finish
  frankaIsDone.store(true);
  //frankaRightThread.join();
	frankaLeftThread.join();
  guiThread.join();
  mainDataPrintThread.join();
	timeDelayThread.join();
    
  //Stop and wait for haptic thread to finish
  //hapticIsDone.store(true);
  //hapticThreadRight.join();
  
	//hapticThreadLeft.join();

  return 0;
}

