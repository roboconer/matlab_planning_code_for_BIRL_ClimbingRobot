/*****************************************************************************
 *        SPLINE                                                             *
 *        SCUT, 2012                                                         *
 *        Author :                                                           *
 *        Version number :  0.10                                             *
 *        Date :            2012-03-24                                       *
 *****************************************************************************/
#include "TaskControl.h"
#include "Setup.h"
#include <math.h>



#define POS_NUM  21

SplineStruct g_P[POS_NUM];

void Set_SplineData()
{
	int i;
	for(i=0; i<POS_NUM; i++)
	{
		g_P[i].T = 1000; // ms
		g_P[i].Line = i+1; // 行号
	}

//1
g_P[0].P[0] = 0.000;
g_P[0].P[1] =-0.373;
g_P[0].P[2] =0.524;
g_P[0].P[3] =-0.151;
g_P[0].P[4] =0.000;
g_P[0].V[0] = 0.000;
g_P[0].V[1] =0.000;
g_P[0].V[2] =0.000;
g_P[0].V[3] =0.000;
g_P[0].V[4] =0.000;
g_P[0].A[0] = -0.003;
g_P[0].A[1] =-0.041;
g_P[0].A[2] =0.077;
g_P[0].A[3] =-0.036;
g_P[0].A[4] =0.000;
//2
g_P[1].P[0] = -0.113;
g_P[1].P[1] =-0.652;
g_P[1].P[2] =0.954;
g_P[1].P[3] =-0.302;
g_P[1].P[4] =0.000;
g_P[1].V[0] = -0.085;
g_P[1].V[1] =-0.147;
g_P[1].V[2] =0.200;
g_P[1].V[3] =-0.053;
g_P[1].V[4] =0.000;
g_P[1].A[0] = -0.042;
g_P[1].A[1] =-0.037;
g_P[1].A[2] =0.030;
g_P[1].A[3] =0.008;
g_P[1].A[4] =0.000;
//3
g_P[2].P[0] = -0.241;
g_P[2].P[1] =-0.842;
g_P[2].P[2] =1.197;
g_P[2].P[3] =-0.355;
g_P[2].P[4] =0.000;
g_P[2].V[0] = -0.111;
g_P[2].V[1] =-0.141;
g_P[2].V[2] =0.166;
g_P[2].V[3] =-0.025;
g_P[2].V[4] =0.000;
g_P[2].A[0] = -0.000;
g_P[2].A[1] =0.046;
g_P[2].A[2] =-0.084;
g_P[2].A[3] =0.038;
g_P[2].A[4] =0.000;
//4
g_P[3].P[0] = -0.382;
g_P[3].P[1] =-0.991;
g_P[3].P[2] =1.353;
g_P[3].P[3] =-0.362;
g_P[3].P[4] =0.000;
g_P[3].V[0] = -0.118;
g_P[3].V[1] =-0.103;
g_P[3].V[2] =0.096;
g_P[3].V[3] =0.008;
g_P[3].V[4] =0.000;
g_P[3].A[0] = -0.010;
g_P[3].A[1] =0.015;
g_P[3].A[2] =-0.028;
g_P[3].A[3] =0.014;
g_P[3].A[4] =0.000;
//5
g_P[4].P[0] = -0.536;
g_P[4].P[1] =-1.108;
g_P[4].P[2] =1.451;
g_P[4].P[3] =-0.343;
g_P[4].P[4] =0.000;
g_P[4].V[0] = -0.127;
g_P[4].V[1] =-0.082;
g_P[4].V[2] =0.059;
g_P[4].V[3] =0.023;
g_P[4].V[4] =0.000;
g_P[4].A[0] = -0.005;
g_P[4].A[1] =0.020;
g_P[4].A[2] =-0.031;
g_P[4].A[3] =0.010;
g_P[4].A[4] =0.000;
//6
g_P[5].P[0] = -0.697;
g_P[5].P[1] =-1.194;
g_P[5].P[2] =1.503;
g_P[5].P[3] =-0.308;
g_P[5].P[4] =0.000;
g_P[5].V[0] = -0.131;
g_P[5].V[1] =-0.058;
g_P[5].V[2] =0.027;
g_P[5].V[3] =0.031;
g_P[5].V[4] =0.000;
g_P[5].A[0] = -0.002;
g_P[5].A[1] =0.017;
g_P[5].A[2] =-0.021;
g_P[5].A[3] =0.004;
g_P[5].A[4] =0.000;
//7
g_P[6].P[0] = -0.862;
g_P[6].P[1] =-1.254;
g_P[6].P[2] =1.522;
g_P[6].P[3] =-0.267;
g_P[6].P[4] =0.000;
g_P[6].V[0] = -0.132;
g_P[6].V[1] =-0.038;
g_P[6].V[2] =0.004;
g_P[6].V[3] =0.034;
g_P[6].V[4] =0.000;
g_P[6].A[0] = 0.001;
g_P[6].A[1] =0.015;
g_P[6].A[2] =-0.015;
g_P[6].A[3] =-0.000;
g_P[6].A[4] =0.000;
//8
g_P[7].P[0] = -1.026;
g_P[7].P[1] =-1.291;
g_P[7].P[2] =1.517;
g_P[7].P[3] =-0.226;
g_P[7].P[4] =0.000;
g_P[7].V[0] = -0.130;
g_P[7].V[1] =-0.021;
g_P[7].V[2] =-0.011;
g_P[7].V[3] =0.032;
g_P[7].V[4] =0.000;
g_P[7].A[0] = 0.003;
g_P[7].A[1] =0.012;
g_P[7].A[2] =-0.009;
g_P[7].A[3] =-0.003;
g_P[7].A[4] =0.000;
//9
g_P[8].P[0] = -1.186;
g_P[8].P[1] =-1.308;
g_P[8].P[2] =1.498;
g_P[8].P[3] =-0.190;
g_P[8].P[4] =0.000;
g_P[8].V[0] = -0.126;
g_P[8].V[1] =-0.008;
g_P[8].V[2] =-0.018;
g_P[8].V[3] =0.027;
g_P[8].V[4] =0.000;
g_P[8].A[0] = 0.004;
g_P[8].A[1] =0.008;
g_P[8].A[2] =-0.004;
g_P[8].A[3] =-0.005;
g_P[8].A[4] =0.000;
//10
g_P[9].P[0] = -1.340;
g_P[9].P[1] =-1.313;
g_P[9].P[2] =1.473;
g_P[9].P[3] =-0.160;
g_P[9].P[4] =0.000;
g_P[9].V[0] = -0.121;
g_P[9].V[1] =-0.000;
g_P[9].V[2] =-0.020;
g_P[9].V[3] =0.020;
g_P[9].V[4] =0.000;
g_P[9].A[0] = 0.003;
g_P[9].A[1] =0.005;
g_P[9].A[2] =0.001;
g_P[9].A[3] =-0.006;
g_P[9].A[4] =0.000;
//11
g_P[10].P[0] = -1.489;
g_P[10].P[1] =-1.310;
g_P[10].P[2] =1.450;
g_P[10].P[3] =-0.140;
g_P[10].P[4] =0.000;
g_P[10].V[0] = -0.118;
g_P[10].V[1] =0.005;
g_P[10].V[2] =-0.017;
g_P[10].V[3] =0.012;
g_P[10].V[4] =0.000;
g_P[10].A[0] = 0.002;
g_P[10].A[1] =0.003;
g_P[10].A[2] =0.004;
g_P[10].A[3] =-0.007;
g_P[10].A[4] =0.000;
//12
g_P[11].P[0] = -1.635;
g_P[11].P[1] =-1.302;
g_P[11].P[2] =1.433;
g_P[11].P[3] =-0.130;
g_P[11].P[4] =0.000;
g_P[11].V[0] = -0.116;
g_P[11].V[1] =0.007;
g_P[11].V[2] =-0.010;
g_P[11].V[3] =0.003;
g_P[11].V[4] =0.000;
g_P[11].A[0] = 0.000;
g_P[11].A[1] =0.001;
g_P[11].A[2] =0.006;
g_P[11].A[3] =-0.008;
g_P[11].A[4] =0.000;
//13
g_P[12].P[0] = -1.780;
g_P[12].P[1] =-1.293;
g_P[12].P[2] =1.425;
g_P[12].P[3] =-0.133;
g_P[12].P[4] =0.000;
g_P[12].V[0] = -0.117;
g_P[12].V[1] =0.008;
g_P[12].V[2] =-0.002;
g_P[12].V[3] =-0.007;
g_P[12].V[4] =0.000;
g_P[12].A[0] = -0.002;
g_P[12].A[1] =0.001;
g_P[12].A[2] =0.007;
g_P[12].A[3] =-0.008;
g_P[12].A[4] =0.000;
//14
g_P[13].P[0] = -1.929;
g_P[13].P[1] =-1.281;
g_P[13].P[2] =1.428;
g_P[13].P[3] =-0.147;
g_P[13].P[4] =0.000;
g_P[13].V[0] = -0.120;
g_P[13].V[1] =0.011;
g_P[13].V[2] =0.006;
g_P[13].V[3] =-0.017;
g_P[13].V[4] =0.000;
g_P[13].A[0] = -0.004;
g_P[13].A[1] =0.003;
g_P[13].A[2] =0.005;
g_P[13].A[3] =-0.008;
g_P[13].A[4] =0.000;
//15
g_P[14].P[0] = -2.082;
g_P[14].P[1] =-1.265;
g_P[14].P[2] =1.439;
g_P[14].P[3] =-0.175;
g_P[14].P[4] =0.000;
g_P[14].V[0] = -0.126;
g_P[14].V[1] =0.016;
g_P[14].V[2] =0.011;
g_P[14].V[3] =-0.027;
g_P[14].V[4] =0.000;
g_P[14].A[0] = -0.006;
g_P[14].A[1] =0.006;
g_P[14].A[2] =0.002;
g_P[14].A[3] =-0.007;
g_P[14].A[4] =0.000;
//16
g_P[15].P[0] = -2.244;
g_P[15].P[1] =-1.239;
g_P[15].P[2] =1.452;
g_P[15].P[3] =-0.213;
g_P[15].P[4] =0.000;
g_P[15].V[0] = -0.133;
g_P[15].V[1] =0.026;
g_P[15].V[2] =0.009;
g_P[15].V[3] =-0.035;
g_P[15].V[4] =0.000;
g_P[15].A[0] = -0.006;
g_P[15].A[1] =0.010;
g_P[15].A[2] =-0.004;
g_P[15].A[3] =-0.006;
g_P[15].A[4] =0.000;
//17
g_P[16].P[0] = -2.416;
g_P[16].P[1] =-1.196;
g_P[16].P[2] =1.457;
g_P[16].P[3] =-0.261;
g_P[16].P[4] =0.000;
g_P[16].V[0] = -0.141;
g_P[16].V[1] =0.044;
g_P[16].V[2] =-0.003;
g_P[16].V[3] =-0.040;
g_P[16].V[4] =0.000;
g_P[16].A[0] = -0.007;
g_P[16].A[1] =0.018;
g_P[16].A[2] =-0.015;
g_P[16].A[3] =-0.003;
g_P[16].A[4] =0.000;
//18
g_P[17].P[0] = -2.596;
g_P[17].P[1] =-1.128;
g_P[17].P[2] =1.440;
g_P[17].P[3] =-0.312;
g_P[17].P[4] =0.000;
g_P[17].V[0] = -0.145;
g_P[17].V[1] =0.066;
g_P[17].V[2] =-0.026;
g_P[17].V[3] =-0.041;
g_P[17].V[4] =0.000;
g_P[17].A[0] = 0.000;
g_P[17].A[1] =0.018;
g_P[17].A[2] =-0.020;
g_P[17].A[3] =0.002;
g_P[17].A[4] =0.000;
//19
g_P[18].P[0] = -2.782;
g_P[18].P[1] =-1.023;
g_P[18].P[2] =1.380;
g_P[18].P[3] =-0.358;
g_P[18].P[4] =0.000;
g_P[18].V[0] = -0.154;
g_P[18].V[1] =0.108;
g_P[18].V[2] =-0.079;
g_P[18].V[3] =-0.029;
g_P[18].V[4] =0.000;
g_P[18].A[0] = -0.014;
g_P[18].A[1] =0.049;
g_P[18].A[2] =-0.066;
g_P[18].A[3] =0.017;
g_P[18].A[4] =0.000;
//20
g_P[19].P[0] = -2.966;
g_P[19].P[1] =-0.871;
g_P[19].P[2] =1.251;
g_P[19].P[3] =-0.380;
g_P[19].P[4] =0.000;
g_P[19].V[0] = -0.126;
g_P[19].V[1] =0.118;
g_P[19].V[2] =-0.112;
g_P[19].V[3] =-0.006;
g_P[19].V[4] =0.000;
g_P[19].A[0] = 0.059;
g_P[19].A[1] =-0.033;
g_P[19].A[2] =0.014;
g_P[19].A[3] =0.020;
g_P[19].A[4] =0.000;
//21
g_P[20].P[0] = -3.142;
g_P[20].P[1] =-0.654;
g_P[20].P[2] =1.003;
g_P[20].P[3] =-0.349;
g_P[20].P[4] =0.000;
g_P[20].V[0] = 0.000;
g_P[20].V[1] =-0.000;
g_P[20].V[2] =0.000;
g_P[20].V[3] =-0.000;
g_P[20].V[4] =0.000;
g_P[20].A[0] = 0.008;
g_P[20].A[1] =-0.029;
g_P[20].A[2] =0.046;
g_P[20].A[3] =-0.017;
g_P[20].A[4] =0.000;
}

void MOV_Spline_Control()
{
	int li_flagplan;
	double pos[5];
	double vel = MAX_AUTOVEL_JOINT * 0.2;
	double acc = MAX_AUTOACC_JOINT;
	double jerk = MAX_JERK_JOINT;
	int i;
	
	std::deque<SplineStruct> splineDeque; // Spline插补数据队列
	SplineStruct splineData; // Spline插补数据
	
	Set_SplineData();
	
	
	// MOVJ - 第一点
	for(i=0; i<JOINT_NUM; i++)
		pos[i] = 0;
	Robot::I_Task.Set_RunState(FLAG_RUNNING);
	li_flagplan = Robot::I_Servo.Servo_PTPLink(pos,vel,acc,jerk);
	if(Ok != li_flagplan)
		Robot::I_Task.Task_RunErrStop(li_flagplan);
	
	
	// MOVS
	splineDeque.clear();
	
	for(i=0; i<2; i++)
	{
		splineData = g_P[i];
		splineDeque.push_back(splineData);	
	}
	
	
	li_flagplan = Robot::I_Task.m_iInterp.Plan_Spline(&splineDeque);
	
	if(Ok == li_flagplan) // 规划成功
	{
		// 运动
		Robot::I_Task.Set_RunState(FLAG_RUNNING);
		Robot::I_Task.Set_CurrLine(1);
		li_flagplan = Robot::I_Servo.Servo_InterpLink(Robot::I_Task.m_iInterp.m_iPath);				
	}
	if(Ok != li_flagplan)
	{
		Robot::I_Task.Task_RunErrStop(li_flagplan);
	}
}