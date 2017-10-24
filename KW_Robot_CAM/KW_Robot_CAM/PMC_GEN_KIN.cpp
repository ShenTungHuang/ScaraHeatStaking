/*--------------------------------------------------------------------------------
 * PMC_GEN_KIN.cpp            
 * Copyright					Precision Machinery Research Development Center, Taiching, Taiwan
 * Editor						Chien-Pin Chen,
 * Description				General robot arm Kinematic application
 *--------------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/
#include "StdAfx.h"
#include "PMC_GEN_KIN.h"
#include <math.h>
#include <windows.h>
#include <stdio.h>
/*-DEFINES---------------------------------------------------------------------*/
#define M_PI	3.1415926535897932384626433832795				//!< constant to present value of pi
#define DEG2RAD 0.01745329251994329576923690768489	//!< constant to present converting from radius to degree
#define RAD2DEG 57.295779513082320876798154814105		//!< constant to present  converting from degree to radius

/*-FUNCTION DEFINITIONS------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------
矩陣相乘函數 MatrixMuliply:  c = a * b
Input: 矩陣a,  矩陣b, 矩陣a的行數 a_row, 矩陣a的列數 a_col, 矩陣b的列數 b_col
Output: 矩陣c
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::MatrixMuliply(double *a, double *b, int a_row, int a_col, int b_col, double *c)
{
	for(int i = 0; i < a_row; i++)
	{
		for(int j = 0; j < b_col; j++)
		{
			c[i * b_col + j] = 0.0;
			for(int k = 0; k < a_col; k++)
			{ c[i * b_col + j] += a[i * a_col + k] * b[k * b_col + j]; }
		}
	}
}

/*----------------------------------------------------------------------------------------------
brief Matrix Transpose c=a
矩陣轉置函數
return N/A
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::TransposeMatrix(double *a, int a_row, int a_col, double *c)
{
	for(int i = 0; i < a_row; i++)
	{
		for(int j = 0; j < a_col; j++)
		{ c[j * a_row + i] = a[i * a_col + j]; }
	}
}

/*----------------------------------------------------------------------------------------------
brief Rotate along X-axis
對X軸旋轉矩陣函數
return N/A
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::RotX(double t, double *rot)
{
	/*~~~~~~~~~~~*/
	double	ct, st;
	ct = cos(t);
	st = sin(t);
	/*~~~~~~~~~~~*/

	rot[0] = 1;		rot[1] = 0;		rot[2] = 0;		rot[3] = 0;
	rot[4] = 0;		rot[5] = ct;		rot[6] = -st; 	rot[7] = 0;
	rot[8] = 0;		rot[9] = st;		rot[10] = ct;	rot[11] = 0;
	rot[12] = 0;		rot[13] = 0;		rot[14] = 0;		rot[15] = 1;
}

/*----------------------------------------------------------------------------------------------
brief Rotate along Y-axis
對Y軸旋轉矩陣函數
return N/A
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::RotY(double t, double *rot)
{
	/*~~~~~~~~~~~*/
	double	ct, st;
	ct = cos(t);
	st = sin(t);
	/*~~~~~~~~~~~*/

	rot[0] = ct;		rot[1] = 0;		rot[2] = st;		rot[3] = 0;
	rot[4] = 0;		rot[5] = 1;		rot[6] = 0;		rot[7] = 0;
	rot[8] = -st;		rot[9] = 0;		rot[10] = ct;	rot[11] = 0;
	rot[12] = 0;		rot[13] = 0;		rot[14] = 0;		rot[15] = 1;
}

/*----------------------------------------------------------------------------------------------
brief Rotate along Z-axis
對Z軸旋轉矩陣函數
return N/A
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::RotZ(double t, double *rot)
{
	/*~~~~~~~~~~~*/
	double	ct, st;
	ct = cos(t);
	st = sin(t);
	/*~~~~~~~~~~~*/

	rot[0] = ct;		rot[1] = -st;		rot[2] = 0;		rot[3] = 0;
	rot[4] = st;		rot[5] = ct;		rot[6] = 0;		rot[7] = 0;
	rot[8] = 0;		rot[9] = 0;		rot[10] = 1;		rot[11] = 0;
	rot[12] = 0;		rot[13] = 0;		rot[14] = 0;		rot[15] = 1;
}

/*----------------------------------------------------------------------------------------------
brief Roll Pitch Yaw to Rotatation Matrix
由roll,pitch,yaw求出旋轉矩陣
return N/A
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::rpy2tr(double roll_z, double pitch_y, double yaw_x, double *rot)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double	rotz[16], roty[16], rotx[16], temp[16], temp1[16];
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	RotZ(roll_z, rotz);
	RotY(pitch_y, roty);
	RotX(yaw_x, rotx);

	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			temp[i * 4 + j] = 0;
			for(int k = 0; k < 4; k++)
			{ temp[i * 4 + j] += rotz[i * 4 + k] * roty[k * 4 + j]; }
		}
	}

	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			temp1[i * 4 + j] = 0;
			for(int k = 0; k < 4; k++)
			{ temp1[i * 4 + j] += temp[i * 4 + k] * rotx[k * 4 + j]; }
		}
	}

	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{ rot[i * 4 + j] = temp1[i * 4 + j]; }
	}

	rot[3] = 0;	rot[7] = 0;	rot[11] = 0;
	rot[12] = 0;		rot[13] = 0;		rot[14] = 0;		rot[15] = 1;
}

/*----------------------------------------------------------------------------------------------
brief Rotatation Matrix to Roll Pitch Yaw
由旋轉矩陣求出roll,pitch,yaw的函數
return N/A
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::tr2rpy(double *m, double *roll_z, double *pitch_y, double *yaw_x)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double	eps = 2.22044604925031e-5, sp, cp;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if(fabs(m[0]) < eps && fabs(m[4]) < eps)
	{
		*roll_z = 0;
		*pitch_y = atan2(-m[8], m[0]);
		*yaw_x = atan2(-m[6], m[5]);
	}
	else
	{
		*roll_z = atan2(m[4], m[0]);
		sp = sin(*roll_z);
		cp = cos(*roll_z);
		*pitch_y = atan2(-m[8], cp * m[0] + sp * m[4]);;
		*yaw_x = atan2(sp * m[2] - cp * m[6], cp * m[5] - sp * m[1]);
	}
}

/*----------------------------------------------------------------------------------------------
反矩陣計算函數 inverse 
Input: 矩陣a,  矩陣a的行數a_row
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::inverse(double *a, int a_row)
{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	int i, icol, irow, j, k, l, ll;
	double big, dum, pivinv;
	int n = a_row;
	int m = 1;
	double b[4];
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	for(i = 0; i < a_row; i++)
	{ b[i] = 0.0; }

	/*~~~~~~~~~*/
	int indxc[4];
	int indxr[4];
	int ipiv[4];
	/*~~~~~~~~~*/

	for(j = 0; j < n; j++)
	{ ipiv[j] = 0; }

	for(i = 0; i < n; i++)
	{
		big = 0.0;
		for(j = 0; j < n; j++)
		{
			if(ipiv[j] != 1)
			{
				for(k = 0; k < n; k++)
				{
					if(ipiv[k] == 0)
					{
						if(fabs(a[j * n + k]) >= big)
						{
							big = fabs(a[j * n + k]);
							irow = j;
							icol = k;
						}
					}
				}
			}
		}

		++(ipiv[icol]);
		if(irow != icol)
		{
			for(l = 0; l < n; l++)
			{ SWAP(a[irow * n + l], a[icol * n + l]); }
			for(l = 0; l < m; l++)
			{ SWAP(b[irow * m + l], b[icol * m + l]); }
		}

		indxr[i] = irow;
		indxc[i] = icol;
		if(a[icol * n + icol] == 0.0) { return; }

		pivinv = 1.0 / a[icol * n + icol];
		a[icol * n + icol] = 1.0;
		for(l = 0; l < n; l++)
		{ a[icol * n + l] *= pivinv; }

		for(l = 0; l < m; l++)
		{ b[icol * m + l] *= pivinv; }

		for(ll = 0; ll < n; ll++)
		{
			if(ll != icol)
			{
				dum = a[ll * n + icol];
				a[ll * n + icol] = 0.0;
				for(l = 0; l < n; l++)
				{ a[ll * n + l] -= a[icol * n + l] * dum; }

				for(l = 0; l < m; l++)
				{ b[ll * m + l] -= b[icol * m + l] * dum; }
			}
		}
	}

	for(l = n - 1; l >= 0; l--)
	{
		if(indxr[l] != indxc[l])
		{
			for(k = 0; k < n; k++)
			{ SWAP(a[k * n + indxr[l]], a[k * n + indxc[l]]); }
		}
	}
}

/***************************************************************************************/

/* --------------------PMC_GEN_KIN 函式庫------------------- */
PMC_GEN_KIN::PMC_GEN_KIN()		// 默認建構式
{
	/* Initialize */
	for(int i = 0; i < 6; i++)
	{
		m_ini_theta[i] = 0;		// 儲存初始化的6軸角度
		m_pre_theta[i] = 0;	// 儲存上一組6軸角度,與求出來的8組解比較
	}

	for(int i = 0; i < 16; i++)
	{
		m_T_act[i] = 0;			// 刀具中心點(TCP) 相對於 base的 HT matrix
		m_tool_T[i] = 0;			// 刀具中心點(TCP) 相對於 joint 6 的 HT matrix
	}

	/*初始化默認的(KR5) Modified DH-Table*/
	/* mm */
	a[0] =   0.0;
	a[1] = 105.0;
	a[2] = 520.0;
	a[3] =  79.0;
	a[4] =   0.0;
	a[5] =   0.0;

	/* degree */
	alpha[0] = 180.0; 
	alpha[1] =  90.0;
	alpha[2] =   0.0;
	alpha[3] =  90.0;
	alpha[4] = -90.0;
	alpha[5] = -90.0;

	/* mm */
	d[0] = -581.5;
	d[1] =    0.0;
	d[2] =    0.0;
	d[3] = -475.0;
	d[4] =    0.0;
	d[5] =   93.2;

	/* degree */
	theta[0] =   0.0; 
	theta[1] = -90.0;
	theta[2] =   0.0;
	theta[3] =   0.0;
	theta[4] =   0.0;
	theta[5] =   0.0;

	/* degree */
	axis_limit[0] = 155.0 + theta[0];
	axis_limit[1] = 140.0 + theta[1];
	axis_limit[2] =  65.0 + theta[2];
	axis_limit[3] = 155.0 + theta[3];
	axis_limit[4] = 100.0 + theta[4];
	axis_limit[5] = 355.0 + theta[5];

	/* degree */
	axis_limit[6]  = -155.0 + theta[0];							
	axis_limit[7]  =  -85.0 + theta[1];
	axis_limit[8]  = -175.0 + theta[2];
	axis_limit[9]  = -155.0 + theta[3];
	axis_limit[10] = -100.0 + theta[4];
	axis_limit[11] = -355.0 + theta[5];

	/* initialize local varibles */
	m_tool_T[0]  = 1.0;		// tool center point Home Trans Matrix
	m_tool_T[5]  = 1.0;
	m_tool_T[10] = 1.0;
	m_tool_T[15] = 1.0;

	/* initialize HT matrix */
	for(int i = 0; i < 16; i++)
	{
		if(i % 5 == 0)
		{
			work_base_T[i]       = 1;
			_RobBase2RobRootT[i] = 1;
			_Flange2RobToolT[i]  = 1;
		}
		else
		{
			work_base_T[i]       = 0;
			_RobBase2RobRootT[i] = 0;
			_Flange2RobToolT[i]  = 0;
		}
	}

	m_ini_theta[0] = theta[0];		//	帶入初始theta角度
	m_ini_theta[1] = theta[1];
	m_ini_theta[2] = theta[2];
	m_ini_theta[3] = theta[3];
	m_ini_theta[4] = theta[4];
	m_ini_theta[5] = theta[5];

	/*~~~~~~~~~~~~~*/
	double	T1[16];					// delcare A01 D-H HT matrix
	double	T2[16];
	double	T3[16];
	double	T4[16];
	double	T5[16];
	double	T6[16];
	double	T12[16];
	double	T13[16];
	double	T14[16];
	double	T15[16];
	double	T16[16];
	/*~~~~~~~~~~~~~*/

	Homo_trans(m_ini_theta[0], 1, T1);		// A01
	Homo_trans(m_ini_theta[1], 2, T2);		// A12
	Homo_trans(m_ini_theta[2], 3, T3);
	Homo_trans(m_ini_theta[3], 4, T4);
	Homo_trans(m_ini_theta[4], 5, T5);
	Homo_trans(m_ini_theta[5], 6, T6);

	MatrixMuliply(T1, T2, 4, 4, 4, T12);		// A02 = A01*A12
	MatrixMuliply(T12, T3, 4, 4, 4, T13);
	MatrixMuliply(T13, T4, 4, 4, 4, T14);
	MatrixMuliply(T14, T5, 4, 4, 4, T15);
	MatrixMuliply(T15, T6, 4, 4, 4, T16);	// A06
	MatrixMuliply(T16, m_tool_T, 4, 4, 4, m_T_act);						// m_T_act is Tool center point HT Matrix
	MatrixMuliply(work_base_T, m_T_act, 4, 4, 4, work_base);	// get TCP in work base coordination

	/* calcualte xyzabc */
	m_pos_act.x = work_base[3];
	m_pos_act.y = work_base[7];
	m_pos_act.z = work_base[11];
	tr2rpy(work_base, &m_pos_act.a, &m_pos_act.b, &m_pos_act.c);
	m_pos_act.a = 180.0 * m_pos_act.a / M_PI;	// change rad to degree
	m_pos_act.b = 180.0 * m_pos_act.b / M_PI;
	m_pos_act.c = 180.0 * m_pos_act.c / M_PI;

	/* copy m_T_act */
	memcpy(m_pos_act.T[0], T1, sizeof(double) * 16);
	memcpy(m_pos_act.T[1], T12, sizeof(double) * 16);
	memcpy(m_pos_act.T[2], T13, sizeof(double) * 16);
	memcpy(m_pos_act.T[3], T14, sizeof(double) * 16);
	memcpy(m_pos_act.T[4], T15, sizeof(double) * 16);
	memcpy(m_pos_act.T[5], m_T_act, sizeof(double) * 16);
	memcpy(m_pos_act.T[6], work_base, sizeof(double) * 16);
}

/* 輸入DH table的建構式*/
PMC_GEN_KIN::PMC_GEN_KIN( double a0[6], double alpha0[6], 	double d0[6], double ini_theta[6], double axis_limit0[12] )
{/* degree */
	/* Initialize */
	for(int i = 0; i < 6; i++)
	{
		m_ini_theta[i] = 0;			// 儲存初始化的6軸角度
		m_pre_theta[i] = 0;		// 儲存上一組6軸角度,與求出來的8組解比較
	}

	for(int i = 0; i < 16; i++)
	{
		m_T_act[i] = 0;				// 刀具中心點(TCP) 相對於 joint 6 的 HT matrix
		m_tool_T[i] = 0;				// 刀具中心點(TCP) 相對於 base的 HT matrix
	}

	/* initialize Modified DH-Table */
	for(int i = 0; i < 6; i++)
	{
		a[i] = a0[i];
		alpha[i] = alpha0[i];
		d[i] = d0[i];
		theta[i] = ini_theta[i];
		axis_limit[i] = axis_limit0[i];
		axis_limit[i + 6] = axis_limit0[i + 6];
	}

	/* initialize local varibles */
	m_tool_T[0] = 1.0;					// tool center point Home Trans Matrix */
	m_tool_T[5] = 1.0;
	m_tool_T[10] = 1.0;
	m_tool_T[15] = 1.0;

	/* initialize work_base HT matrix */
	for(int i = 0; i < 16; i++)
	{
		if(i % 5 == 0)
		{ work_base_T[i] = 1; }
		else
		{ work_base_T[i] = 0; }
	}

	m_ini_theta[0] = ini_theta[0];	//	帶入初始theta角度
	m_ini_theta[1] = ini_theta[1];
	m_ini_theta[2] = ini_theta[2];
	m_ini_theta[3] = ini_theta[3];
	m_ini_theta[4] = ini_theta[4];
	m_ini_theta[5] = ini_theta[5];

	/*~~~~~~~~~~~~~*/
	double	T1[16];		// delcare A01 D-H HT matrix
	double	T2[16];
	double	T3[16];
	double	T4[16];
	double	T5[16];
	double	T6[16];
	double	T12[16];
	double	T13[16];
	double	T14[16];
	double	T15[16];
	double	T16[16];
	/*~~~~~~~~~~~~~*/

	Homo_trans(m_ini_theta[0], 1, T1);		// A01
	Homo_trans(m_ini_theta[1], 2, T2);		// A12
	Homo_trans(m_ini_theta[2], 3, T3);
	Homo_trans(m_ini_theta[3], 4, T4);
	Homo_trans(m_ini_theta[4], 5, T5);
	Homo_trans(m_ini_theta[5], 6, T6);

	MatrixMuliply(T1, T2, 4, 4, 4, T12);		// A02 = A01*A12
	MatrixMuliply(T12, T3, 4, 4, 4, T13);
	MatrixMuliply(T13, T4, 4, 4, 4, T14);
	MatrixMuliply(T14, T5, 4, 4, 4, T15);
	MatrixMuliply(T15, T6, 4, 4, 4, T16);	// A06
	MatrixMuliply(T16, m_tool_T, 4, 4, 4, m_T_act);						// m_T_act is Tool center point HT Matrix
	MatrixMuliply(work_base_T,m_T_act,  4, 4, 4, work_base);	// get TCP in work base coordination

	/* calcualte xyzabc */
	m_pos_act.x = work_base[3];
	m_pos_act.y = work_base[7];
	m_pos_act.z = work_base[11];
	tr2rpy(work_base, &m_pos_act.a, &m_pos_act.b, &m_pos_act.c);
	m_pos_act.a = 180.0 * m_pos_act.a / M_PI;				// change rad to degree
	m_pos_act.b = 180.0 * m_pos_act.b / M_PI;
	m_pos_act.c = 180.0 * m_pos_act.c / M_PI;

	/* copy m_T_act */
	memcpy(m_pos_act.T[0], T1, sizeof(double) * 16);
	memcpy(m_pos_act.T[1], T12, sizeof(double) * 16);
	memcpy(m_pos_act.T[2], T13, sizeof(double) * 16);
	memcpy(m_pos_act.T[3], T14, sizeof(double) * 16);
	memcpy(m_pos_act.T[4], T15, sizeof(double) * 16);
	memcpy(m_pos_act.T[5], m_T_act, sizeof(double) * 16);
	memcpy(m_pos_act.T[6], work_base, sizeof(double) * 16);
}

/* 解構式*/
PMC_GEN_KIN::~PMC_GEN_KIN(void)
{ }

/*----------------------------------------------------------------------------------------------
正向運動學函數 GEN_FK
Input: 6個軸角度, 
Output: 末端點的結構變數 ARM_POS
----------------------------------------------------------------------------------------------*/
ARM_POS PMC_GEN_KIN::GEN_FK( double	theta_1, double	theta_2, double	theta_3, double	theta_4, double	theta_5, double	theta_6 )
{/* degree */
	/* Initialize */
	double	T1[16];
	double	T2[16];
	double	T3[16];
	double	T4[16];
	double	T5[16];
	double	T6[16];
	double	T12[16];
	double	T13[16];
	double	T14[16];
	double	T15[16];
	double	T16[16];

	// storage input 6 angle as previous angles for finding best solution in IK
	m_pre_theta[0] = theta_1;
	m_pre_theta[1] = theta_2;
	m_pre_theta[2] = theta_3;
	m_pre_theta[3] = theta_4;
	m_pre_theta[4] = theta_5;
	m_pre_theta[5] = theta_6;

	// calculate homogenous matrix for each joint
	Homo_trans(theta_1, 1, T1);
	Homo_trans(theta_2, 2, T2);
	Homo_trans(theta_3, 3, T3);
	Homo_trans(theta_4, 4, T4);
	Homo_trans(theta_5, 5, T5);
	Homo_trans(theta_6, 6, T6);

	MatrixMuliply(T1, T2, 4, 4, 4, T12);
	MatrixMuliply(T12, T3, 4, 4, 4, T13);
	MatrixMuliply(T13, T4, 4, 4, 4, T14);
	MatrixMuliply(T14, T5, 4, 4, 4, T15);
	MatrixMuliply(T15, T6, 4, 4, 4, T16);
	MatrixMuliply(T16, m_tool_T, 4, 4, 4, m_T_act);
	MatrixMuliply(work_base_T, m_T_act, 4, 4, 4, work_base);	// get TCP in work base coordination

	// calcualte xyzabc
	m_pos_act.x = work_base[3];
	m_pos_act.y = work_base[7];
	m_pos_act.z = work_base[11];
	tr2rpy(work_base, &m_pos_act.a, &m_pos_act.b, &m_pos_act.c);
	m_pos_act.a = 180.0 * m_pos_act.a / M_PI;
	m_pos_act.b = 180.0 * m_pos_act.b / M_PI;
	m_pos_act.c = 180.0 * m_pos_act.c / M_PI;

	// copy m_T_act
	memcpy(m_pos_act.T[0], T1, sizeof(double) * 16);
	memcpy(m_pos_act.T[1], T12, sizeof(double) * 16);
	memcpy(m_pos_act.T[2], T13, sizeof(double) * 16);
	memcpy(m_pos_act.T[3], T14, sizeof(double) * 16);
	memcpy(m_pos_act.T[4], T15, sizeof(double) * 16);
	memcpy(m_pos_act.T[5], m_T_act, sizeof(double) * 16);
	memcpy(m_pos_act.T[6], work_base, sizeof(double) * 16);

	return m_pos_act;
}

/*----------------------------------------------------------------------------------------------
逆向運動學函數 GEN_IK , 
Input: 末端點的結構變數 ARM_POS,
Output: 軸角度解的結構變數, 計算結果代號
----------------------------------------------------------------------------------------------*/
int PMC_GEN_KIN::GEN_IK(ARM_POS tcp, ARM_AXIS_VALUE &temp_value)
{
	 // reset the 8 solution
	 // ARM_AXIS_VALUE temp_value
	for(int i = 0; i < 48; ++i)
	{ temp_value.axis_value[i] = 0.0; }

	for(int i = 0; i < 8; ++i)
	{
		temp_value.solution_check[i] = true;
		temp_value.singular_check[i] = true;
		temp_value.limit_check[i] = true;
	}

	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double	roll = M_PI * tcp.a / 180.0;
	double	pitch = M_PI * tcp.b / 180.0;
	double	yaw = M_PI * tcp.c / 180.0;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	/* calculate tcp T */
	rpy2tr(roll, pitch, yaw, work_base);
	work_base[3] = tcp.x;
	work_base[7] = tcp.y;
	work_base[11] = tcp.z;

	/*~~~~~~~~~~~~~~~~~~~~~~~~*/
	// translate work base coordination system to robot coordination
	double	work_base_T_inv[16];
	/*~~~~~~~~~~~~~~~~~~~~~~~~*/

	memcpy(work_base_T_inv, work_base_T, sizeof(double) * 16);
	inverse(work_base_T_inv, 4);

	/*~~~~~~~~~~~~~~~*/
	double	temp_T[16];
	/*~~~~~~~~~~~~~~~*/

	MatrixMuliply(work_base_T_inv,work_base,  4, 4, 4, temp_T);

	/*~~~~~~~~~~~~~~~~~~~~~~~~*/
	// copy tool offset T
	double	temp_tool_T_inv[16];
	/*~~~~~~~~~~~~~~~~~~~~~~~~*/

	memcpy(temp_tool_T_inv, m_tool_T, sizeof(double) * 16);

	/* inverse */
	inverse(temp_tool_T_inv, 4);

	/*~~~~~~~~~~~~~~~~~~~~~~*/
	// get flange T
	double	temp_flange_T[16];
	/*~~~~~~~~~~~~~~~~~~~~~~*/

	MatrixMuliply(temp_T, temp_tool_T_inv, 4, 4, 4, temp_flange_T);

	/*~~~~~~~~~~~~~~~~~~*/
	// start to solve IK, get wrist center position Oc(or P0 = A06* P6 eqn 2.77)
	double	Ocx, Ocy, Ocz;
	/*~~~~~~~~~~~~~~~~~~*/

	Ocx = temp_flange_T[3] - d[5] * temp_flange_T[2];
	Ocy = temp_flange_T[7] - d[5] * temp_flange_T[6];
	Ocz = temp_flange_T[11] - d[5] * temp_flange_T[10];

	// Solve theta 1 solution 1-4 (the same)
	temp_value.axis_value[0] = atan2(-Ocy, Ocx);
	for(int i = 1; i < 4; ++i)
	{ temp_value.axis_value[6 * i] = temp_value.axis_value[0]; }

	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	// solve theta 3 and 2 solution 1-2,3-4
	double	the1 = temp_value.axis_value[0];	// just make compute easily
	double	k1, k2, k3, ks;
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	k1 = 2 * a[2] * d[3];
	k2 = 2 * a[2] * a[3];
	k3 = pow(Ocx, 2) +
		pow(Ocy, 2) +
		pow((Ocz + d[0]), 2) -
		2 *
		a[1] *
		Ocx *
		cos(the1) +
		2 *
		a[1] *
		Ocy *
		sin(the1) +
		pow(a[1], 2) -
		pow(a[2], 2) -
		pow(a[3], 2) -
		pow(d[3], 2);
	ks = pow(k1, 2) + pow(k2, 2) - pow(k3, 2);

	if(ks < 0)
	{
		for(int i = 0; i < 4; ++i)
		{ temp_value.solution_check[i] = false; }
	}

	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	// 無實數解,跳過 theta 2,3,4,5,6 solution 1-4 的計算
	double	the3_12, the3_34, the2_12, the2_34;
	double	u1, v1, r1, u2, v2, r2, sin2, cos2; // for calculate theta 2
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	if(temp_value.solution_check[0] == true)
	{
		the3_12 = 2 * atan2(k1 - sqrt(ks), k3 + k2);
		the3_34 = 2 * atan2(k1 + sqrt(ks), k3 + k2);

		// check if theta3 over 180 or lower -180
		pre_check(3, the3_12, axis_limit, m_pre_theta);

		temp_value.axis_value[2] = temp_value.axis_value[8] = the3_12;

		pre_check(3, the3_34, axis_limit, m_pre_theta);

		temp_value.axis_value[14] = temp_value.axis_value[20] = the3_34;

		u1 = a[2] + a[3] * cos(the3_12) + d[3] * sin(the3_12);
		v1 = -a[3] * sin(the3_12) + d[3] * cos(the3_12);
		r1 = Ocx * cos(the1) - Ocy * sin(the1) - a[1];
		u2 = a[3] * sin(the3_12) - d[3] * cos(the3_12);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_12 = atan2(sin2, cos2);
		temp_value.axis_value[1] = temp_value.axis_value[7] = the2_12;

		u1 = a[2] + a[3] * cos(the3_34) + d[3] * sin(the3_34);
		v1 = -a[3] * sin(the3_34) + d[3] * cos(the3_34);
		r1 = Ocx * cos(the1) - Ocy * sin(the1) - a[1];
		u2 = a[3] * sin(the3_34) - d[3] * cos(the3_34);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_34 = atan2(sin2, cos2);
		temp_value.axis_value[13] = temp_value.axis_value[19] = the2_34;
	}

	 // solve theta 5,4,6 solution 1-2 ;
	 // calculate A01*A12*A23 to get R13( Homogenous Transformation matrix from base to joint 3)
	double	A01[16];
	double	A12[16];
	double	A23[16];
	double	A02[16];
	double	R13_1[16];
	double	R13_2[16];
	double	T_1[16];			// A36 matrix for solution 1-2
	double	T_2[16];			// A36 matrix for solution 3-4
	double	eps = 2.22044604925031e-5;		// to check if close to zero
	double	the5_1, the5_2, the4_1, the4_2, the6_1, the6_2;
	double	the5_3, the5_4, the4_3, the4_4, the6_3, the6_4;
	double	sum_theta;										// summary of theta 4 and 6

	// 跳過theta 4,5,6  if the 2,3 no solution (1-4)
	if(temp_value.solution_check[0] == true)
	{
		/* calulate R13_1 */
		Homo_trans((180.0 * the1 / M_PI), 1, A01);
		Homo_trans((180.0 * the2_12 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_12 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_1);
		inverse(R13_1, 4);		// get inverse of R13_1

		MatrixMuliply(R13_1, temp_flange_T, 4, 4, 4, T_1);

		if(fabs(T_1[2]) < eps && fabs(T_1[10]) < eps)
		{
			for(int i = 0; i < 2; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_1 = the5_2 = 0;
			temp_value.axis_value[4] = temp_value.axis_value[10] = the5_1;

			sum_theta = atan2(T_1[1], T_1[0]);

			the6_1 = the6_2 = 0;
			temp_value.axis_value[5] = temp_value.axis_value[11] = the6_1;
			the4_1 = the4_2 = sum_theta;
			temp_value.axis_value[3] = temp_value.axis_value[9] = the4_1;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 1 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_1 = acos(T_1[6]);
			temp_value.axis_value[4] = the5_1;
			c4 = -T_1[2] / sin(the5_1);
			s4 = -T_1[10] / sin(the5_1);
			the4_1 = atan2(s4, c4);
			pre_check(4, the4_1, axis_limit, m_pre_theta);
			temp_value.axis_value[3] = the4_1;

			c6 = T_1[4] / sin(the5_1);
			s6 = -T_1[5] / sin(the5_1);
			the6_1 = atan2(s6, c6);
			pre_check(6, the6_1, axis_limit, m_pre_theta);
			temp_value.axis_value[5] = the6_1;

			// solution 2 of theta 5,4,6
			the5_2 = -the5_1;
			temp_value.axis_value[10] = the5_2;
			c4 = -T_1[2] / sin(the5_2);
			s4 = -T_1[10] / sin(the5_2);
			the4_2 = atan2(s4, c4);
			pre_check(4, the4_2, axis_limit, m_pre_theta);
			temp_value.axis_value[9] = the4_2;

			c6 = T_1[4] / sin(the5_2);
			s6 = -T_1[5] / sin(the5_2);
			the6_2 = atan2(s6, c6);
			pre_check(6, the6_2, axis_limit, m_pre_theta);
			temp_value.axis_value[11] = the6_2;
		}

		 // solve theta 5,4,6 solution 3-4 ;
		 // calculate R13_2
		Homo_trans((180.0 * the1 / M_PI), 1, A01);
		Homo_trans((180.0 * the2_34 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_34 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_2);
		inverse(R13_2, 4);		// get inverse of R13_2

		MatrixMuliply(R13_2, temp_flange_T, 4, 4, 4, T_2);

		if(fabs(T_2[2]) < eps && fabs(T_2[10]) < eps)
		{
			for(int i = 2; i < 4; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_3 = the5_4 = 0;
			temp_value.axis_value[16] = temp_value.axis_value[22] = the5_3;

			sum_theta = atan2(T_2[1], T_2[0]);

			the6_3 = the6_4 = 0;
			temp_value.axis_value[17] = temp_value.axis_value[23] = the6_3;
			the4_3 = the4_4 = sum_theta;
			temp_value.axis_value[15] = temp_value.axis_value[21] = the4_3;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 3 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_3 = acos(T_2[6]);
			temp_value.axis_value[16] = the5_3;
			c4 = -T_2[2] / sin(the5_3);
			s4 = -T_2[10] / sin(the5_3);
			the4_3 = atan2(s4, c4);
			pre_check(4, the4_3, axis_limit, m_pre_theta);
			temp_value.axis_value[15] = the4_3;

			c6 = T_2[4] / sin(the5_3);
			s6 = -T_2[5] / sin(the5_3);
			the6_3 = atan2(s6, c6);
			pre_check(6, the6_3, axis_limit, m_pre_theta);
			temp_value.axis_value[17] = the6_3;

			// solution 4 of theta 5,4,6
			the5_4 = -the5_3;
			temp_value.axis_value[22] = the5_4;
			c4 = -T_2[2] / sin(the5_4);
			s4 = -T_2[10] / sin(the5_4);
			the4_4 = atan2(s4, c4);
			pre_check(4, the4_4, axis_limit, m_pre_theta);
			temp_value.axis_value[21] = the4_4;

			c6 = T_2[4] / sin(the5_4);
			s6 = -T_2[5] / sin(the5_4);
			the6_4 = atan2(s6, c6);
			pre_check(6, the6_4, axis_limit, m_pre_theta);
			temp_value.axis_value[23] = the6_4;
		}
	}

	/*-------------------------------------------- 
	need to check solution by comparing HT matrix
	with each self and temp_flange_T
	----------------------------------------------*/

	/*solve theta 1 backward solution 5-8*/
	double	the1_b = the1 + M_PI;
	if(the1_b >= 2 * M_PI)
	{ the1_b = the1_b - 2 * M_PI; }

	for(int i = 4; i < 8; ++i)
	{ temp_value.axis_value[6 * i] = the1_b; }

	 // solve theta 3 and 2 solution 5-6, 7-8 ;
	 // k1,k2,k3,ks declaim when solve forward
	k1 = 2 * a[2] * d[3];
	k2 = 2 * a[2] * a[3];
	k3 = pow(Ocx, 2) +
		pow(Ocy, 2) +
		pow((Ocz + d[0]), 2) -
		2 *
		a[1] *
		Ocx *
		cos(the1_b) +
		2 *
		a[1] *
		Ocy *
		sin(the1_b) +
		pow(a[1], 2) -
		pow(a[2], 2) -
		pow(a[3], 2) -
		pow(d[3], 2);
	ks = pow(k1, 2) + pow(k2, 2) - pow(k3, 2);

	if(ks < 0)
	{
		for(int i = 4; i < 8; ++i)
		{ temp_value.solution_check[i] = false; }
	}

	// 跳過 theta2,3 solution 5-8 if no solution
	if(temp_value.solution_check[4] == true)
	{
		double	the3_56, the3_78, the2_56, the2_78;

		the3_56 = 2 * atan2(k1 - sqrt(ks), k3 + k2);
		the3_78 = 2 * atan2(k1 + sqrt(ks), k3 + k2);

		// check if theta3 over 180 or lower -180
		if(the3_56 > M_PI)
		{ the3_56 = the3_56 - 2 * M_PI; }
		else if(the3_56 < -M_PI)
		{ the3_56 = the3_56 + 2 * M_PI; }

		temp_value.axis_value[26] = temp_value.axis_value[32] = the3_56;

		if(the3_78 > M_PI)
		{ the3_78 = the3_78 - 2 * M_PI; }
		else if(the3_78 < -M_PI)
		{ the3_78 = the3_78 + 2 * M_PI; }

		temp_value.axis_value[38] = temp_value.axis_value[44] = the3_78;

		 // u1,v1,r1,u2,v2,r2, sin2, cos2 declaim before (for calculate theta 2) ;
		 // solve theta2 solution 5-6
		u1 = a[2] + a[3] * cos(the3_56) + d[3] * sin(the3_56);
		v1 = -a[3] * sin(the3_56) + d[3] * cos(the3_56);
		r1 = Ocx * cos(the1_b) - Ocy * sin(the1_b) - a[1];
		u2 = a[3] * sin(the3_56) - d[3] * cos(the3_56);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_56 = atan2(sin2, cos2);
		temp_value.axis_value[25] = temp_value.axis_value[31] = the2_56;

		// solve theta2 solution 7-8
		u1 = a[2] + a[3] * cos(the3_78) + d[3] * sin(the3_78);
		v1 = -a[3] * sin(the3_78) + d[3] * cos(the3_78);
		r1 = Ocx * cos(the1_b) - Ocy * sin(the1_b) - a[1];
		u2 = a[3] * sin(the3_78) - d[3] * cos(the3_78);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_78 = atan2(sin2, cos2);
		temp_value.axis_value[37] = temp_value.axis_value[43] = the2_78;

		 // solve theta 5,4,6 solution 5-6 ;
		 // calculate R13 ;
		 // A01, A12, A23, A02, eps, sum_theta discalim already
		double	R13_3[16], R13_4[16], T_3[16], T_4[16];
		double	the5_5, the5_6, the4_5, the4_6, the6_5, the6_6;
		double	the5_7, the5_8, the4_7, the4_8, the6_7, the6_8;

		// calulate R13_3
		Homo_trans((180.0 * the1_b / M_PI), 1, A01);
		Homo_trans((180.0 * the2_56 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_56 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_3);
		inverse(R13_3, 4);		// get inverse of R13_3

		MatrixMuliply(R13_3, temp_flange_T, 4, 4, 4, T_3);

		if(fabs(T_3[2]) < eps && fabs(T_3[10]) < eps)
		{
			printf("Solution 5-6 Reach sigular position\n");
			for(int i = 4; i < 6; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_5 = the5_6 = 0;
			temp_value.axis_value[28] = temp_value.axis_value[34] = the5_5;

			sum_theta = atan2(T_3[1], T_3[0]);

			the6_5 = the6_6 = 0;
			temp_value.axis_value[29] = temp_value.axis_value[35] = the6_5;
			the4_5 = the4_6 = sum_theta;
			temp_value.axis_value[27] = temp_value.axis_value[33] = the4_5;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 5 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_5 = acos(T_3[6]);
			temp_value.axis_value[28] = the5_5;
			c4 = -T_3[2] / sin(the5_5);
			s4 = -T_3[10] / sin(the5_5);
			the4_5 = atan2(s4, c4);
			pre_check(4, the4_5, axis_limit, m_pre_theta);
			temp_value.axis_value[27] = the4_5;

			c6 = T_3[4] / sin(the5_5);
			s6 = -T_3[5] / sin(the5_5);
			the6_5 = atan2(s6, c6);
			pre_check(6, the6_5, axis_limit, m_pre_theta);
			temp_value.axis_value[29] = the6_5;

			// solution 6 of theta 5,4,6
			the5_6 = -the5_5;
			temp_value.axis_value[34] = the5_6;
			c4 = -T_3[2] / sin(the5_6);
			s4 = -T_3[10] / sin(the5_6);
			the4_6 = atan2(s4, c4);
			pre_check(4, the4_6, axis_limit, m_pre_theta);
			temp_value.axis_value[33] = the4_6;

			c6 = T_3[4] / sin(the5_6);
			s6 = -T_3[5] / sin(the5_6);
			the6_6 = atan2(s6, c6);
			pre_check(6, the6_6, axis_limit, m_pre_theta);
			temp_value.axis_value[35] = the6_6;
		}

		 // solve theta 5,4,6 solution 7-8
		 // calculate R13_4
		Homo_trans((180.0 * the1_b / M_PI), 1, A01);
		Homo_trans((180.0 * the2_78 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_78 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_4);
		inverse(R13_4, 4);		// get inverse of R13_4

		MatrixMuliply(R13_4, temp_flange_T, 4, 4, 4, T_4);

		if(fabs(T_4[2]) < eps && fabs(T_4[10]) < eps)
		{
			for(int i = 6; i < 8; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_7 = the5_8 = 0;
			temp_value.axis_value[40] = temp_value.axis_value[46] = the5_7;

			sum_theta = atan2(T_4[1], T_4[0]);

			the6_7 = the6_8 = 0;
			temp_value.axis_value[41] = temp_value.axis_value[47] = the6_7;
			the4_7 = the4_8 = sum_theta;
			temp_value.axis_value[39] = temp_value.axis_value[45] = the4_7;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 7 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_7 = acos(T_4[6]);
			temp_value.axis_value[40] = the5_7;
			c4 = -T_4[2] / sin(the5_7);
			s4 = -T_4[10] / sin(the5_7);
			the4_7 = atan2(s4, c4);
			pre_check(4, the4_7, axis_limit, m_pre_theta);
			temp_value.axis_value[39] = the4_7;

			c6 = T_4[4] / sin(the5_7);
			s6 = -T_4[5] / sin(the5_7);
			the6_7 = atan2(s6, c6);
			pre_check(6, the6_7, axis_limit, m_pre_theta);
			temp_value.axis_value[41] = the6_7;

			// solution 8 of theta 5,4,6
			the5_8 = -the5_7;
			temp_value.axis_value[46] = the5_8;
			c4 = -T_4[2] / sin(the5_8);
			s4 = -T_4[10] / sin(the5_8);
			the4_8 = atan2(s4, c4);
			pre_check(4, the4_8, axis_limit, m_pre_theta);
			temp_value.axis_value[45] = the4_8;

			c6 = T_4[4] / sin(the5_8);
			s6 = -T_4[5] / sin(the5_8);
			the6_8 = atan2(s6, c6);
			pre_check(6, the6_8, axis_limit, m_pre_theta);
			temp_value.axis_value[47] = the6_8;
		}
	}

	// Need to check solution by comparing HT matrix
	for(int i = 0; i < 48; ++i)
	{ temp_value.axis_value[i] = 180.0 * temp_value.axis_value[i] / M_PI; }

	// find the best fit solution
	int check = NO_SOLUTION;
	check = GEN_sol_check(temp_value, axis_limit, m_pre_theta);

	/* output result */
	if(check != 0)
	{ return check; }

	// get the best fit solution from temp_value, and upadate m_pre_theta and m_pos_act
	for(int i = 0; i < 6; ++i)
	{ m_pre_theta[i] = temp_value.axis_value[i + 6 * temp_value.fit]; }

	 // return 8 solution ;
	 // storage the input tool center point as m_pos_act 在安排位置
	m_pos_act.x = tcp.x;
	m_pos_act.y = tcp.y;
	m_pos_act.z = tcp.z;
	m_pos_act.a = tcp.a;
	m_pos_act.b = tcp.b;
	m_pos_act.c = tcp.c;
	for(int i = 0; i < 16; ++i)
	{
		m_pos_act.T[5][i] = temp_T[i];
		m_pos_act.T[6][i] = work_base[i];
	}

	return check;		// IK complete
}

/*----------------------------------------------------------------------------------------------
逆向運動學函數 GEN_IK, 
Input: 末端點 位置:x,y,z, 方向:roll,pitch,yaw
Output: 最佳解角度array, 軸角度解的結構變數, 計算結果代號
----------------------------------------------------------------------------------------------*/
int PMC_GEN_KIN::GEN_IK( double x,double y, double z, double roll, double pitch, double yaw, double *angle, ARM_AXIS_VALUE &temp_value )
{
	 // reset the 8 solution
	 // ARM_AXIS_VALUE temp_value
	for(int i = 0; i < 48; ++i)
	{ temp_value.axis_value[i] = 0.0; }

	for(int i = 0; i < 8; ++i)
	{
		temp_value.solution_check[i] = true;
		temp_value.singular_check[i] = true;
		temp_value.limit_check[i] = true;
	}

	roll = M_PI * roll / 180.0;
	pitch = M_PI * pitch / 180.0;
	yaw = M_PI * yaw / 180.0;

	// calculate tcp T
	rpy2tr(roll, pitch, yaw, work_base);
	work_base[3] = x;
	work_base[7] = y;
	work_base[11] = z;

	// translate work base coordination system to robot coordination
	double work_base_T_inv[16];

	memcpy(work_base_T_inv, work_base_T, sizeof(double) * 16);
	inverse(work_base_T_inv, 4);

	double	temp_T[16];
	MatrixMuliply(work_base_T_inv, work_base, 4, 4, 4, temp_T);

	// copy tool offset T
	double	temp_tool_T_inv[16];
	memcpy(temp_tool_T_inv, m_tool_T, sizeof(double) * 16);

	// inverse2
	inverse(temp_tool_T_inv, 4);

	// get flange T
	double	temp_flange_T[16];
	MatrixMuliply(temp_T, temp_tool_T_inv, 4, 4, 4, temp_flange_T);

	 // start to solve IK, get wrist center position Oc(or P0 = A06* P6 eqn 2.77)
	double	Ocx, Ocy, Ocz;
	Ocx = temp_flange_T[3] - d[5] * temp_flange_T[2];
	Ocy = temp_flange_T[7] - d[5] * temp_flange_T[6];
	Ocz = temp_flange_T[11] - d[5] * temp_flange_T[10];

	// Solve theta 1 solution 1-4 (the same
	temp_value.axis_value[0] = atan2(-Ocy, Ocx);
	for(int i = 1; i < 4; ++i)
	{ temp_value.axis_value[6 * i] = temp_value.axis_value[0]; }

	// solve theta 3 and 2 solution 1-2,3-4
	double	the1 = temp_value.axis_value[0];	// just make compute easily
	double	k1, k2, k3, ks;
	k1 = 2 * a[2] * d[3];
	k2 = 2 * a[2] * a[3];
	k3 = pow(Ocx, 2) +
		pow(Ocy, 2) +
		pow((Ocz + d[0]), 2) -
		2 *
		a[1] *
		Ocx *
		cos(the1) +
		2 *
		a[1] *
		Ocy *
		sin(the1) +
		pow(a[1], 2) -
		pow(a[2], 2) -
		pow(a[3], 2) -
		pow(d[3], 2);
	ks = pow(k1, 2) + pow(k2, 2) - pow(k3, 2);

	if(ks < 0)
	{
		for(int i = 0; i < 4; ++i)
		{ temp_value.solution_check[i] = false; }
	}

	// 無實數解,跳過 theta 2,3,4,5,6 solution 1-4 的計算
	double	the3_12, the3_34, the2_12, the2_34;
	double	u1, v1, r1, u2, v2, r2, sin2, cos2;	// for calculate theta 2

	if(temp_value.solution_check[0] == true)
	{
		the3_12 = 2 * atan2(k1 - sqrt(ks), k3 + k2);
		the3_34 = 2 * atan2(k1 + sqrt(ks), k3 + k2);

		pre_check(3, the3_12, axis_limit, m_pre_theta);

		temp_value.axis_value[2] = temp_value.axis_value[8] = the3_12;

		pre_check(3, the3_34, axis_limit, m_pre_theta);

		temp_value.axis_value[14] = temp_value.axis_value[20] = the3_34;

		u1 = a[2] + a[3] * cos(the3_12) + d[3] * sin(the3_12);
		v1 = -a[3] * sin(the3_12) + d[3] * cos(the3_12);
		r1 = Ocx * cos(the1) - Ocy * sin(the1) - a[1];
		u2 = a[3] * sin(the3_12) - d[3] * cos(the3_12);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_12 = atan2(sin2, cos2);
		temp_value.axis_value[1] = temp_value.axis_value[7] = the2_12;

		u1 = a[2] + a[3] * cos(the3_34) + d[3] * sin(the3_34);
		v1 = -a[3] * sin(the3_34) + d[3] * cos(the3_34);
		r1 = Ocx * cos(the1) - Ocy * sin(the1) - a[1];
		u2 = a[3] * sin(the3_34) - d[3] * cos(the3_34);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_34 = atan2(sin2, cos2);
		temp_value.axis_value[13] = temp_value.axis_value[19] = the2_34;
	}

	// solve theta 5,4,6 solution 1-2
	// calculate A01*A12*A23 to get R13( Homogenous Transformation matrix from base to joint 3)
	double A01[16];
	double A12[16];
	double A23[16];
	double A02[16];
	double R13_1[16];
	double R13_2[16];
	double T_1[16];				// A36 matrix for solution 1-2
	double T_2[16];				// A36 matrix for solution 3-4
	double eps = 2.22044604925031e-5;			// to check if close to zero
	double the5_1, the5_2, the4_1, the4_2, the6_1, the6_2;
	double the5_3, the5_4, the4_3, the4_4, the6_3, the6_4;
	double sum_theta;		// summary of theta 4 and 6

	// 跳過theta 4,5,6  if the 2,3 no solution (1-4)
	if(temp_value.solution_check[0] == true)
	{
		// calulate R13_1
		Homo_trans((180.0 * the1 / M_PI), 1, A01);
		Homo_trans((180.0 * the2_12 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_12 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_1);
		inverse(R13_1, 4);			// get inverse2 of R13_1

		MatrixMuliply(R13_1, temp_flange_T, 4, 4, 4, T_1);

		if(fabs(T_1[2]) < eps && fabs(T_1[10]) < eps)
		{
			for(int i = 0; i < 2; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_1 = the5_2 = 0;
			temp_value.axis_value[4] = temp_value.axis_value[10] = the5_1;

			sum_theta = atan2(T_1[1], T_1[0]);

			the6_1 = the6_2 = 0;
			temp_value.axis_value[5] = temp_value.axis_value[11] = the6_1;
			the4_1 = the4_2 = sum_theta;
			temp_value.axis_value[3] = temp_value.axis_value[9] = the4_1;
		}
		else	// will get two different solutions of theta 5
		{
			// solution 1 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_1 = acos(T_1[6]);
			temp_value.axis_value[4] = the5_1;
			c4 = -T_1[2] / sin(the5_1);
			s4 = -T_1[10] / sin(the5_1);
			the4_1 = atan2(s4, c4);
			pre_check(4, the4_1, axis_limit, m_pre_theta);
			temp_value.axis_value[3] = the4_1;

			c6 = T_1[4] / sin(the5_1);
			s6 = -T_1[5] / sin(the5_1);
			the6_1 = atan2(s6, c6);
			pre_check(6, the6_1, axis_limit, m_pre_theta);
			temp_value.axis_value[5] = the6_1;

			// solution 2 of theta 5,4,6
			the5_2 = -the5_1;
			temp_value.axis_value[10] = the5_2;
			c4 = -T_1[2] / sin(the5_2);
			s4 = -T_1[10] / sin(the5_2);
			the4_2 = atan2(s4, c4);
			pre_check(4, the4_2, axis_limit, m_pre_theta);
			temp_value.axis_value[9] = the4_2;

			c6 = T_1[4] / sin(the5_2);
			s6 = -T_1[5] / sin(the5_2);
			the6_2 = atan2(s6, c6);
			pre_check(6, the6_2, axis_limit, m_pre_theta);
			temp_value.axis_value[11] = the6_2;
		}

		 // solve theta 5,4,6 solution 3-4
		 // calculate R13_2
		Homo_trans((180.0 * the1 / M_PI), 1, A01);
		Homo_trans((180.0 * the2_34 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_34 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_2);
		inverse(R13_2, 4);		// get inverse2 of R13_2

		MatrixMuliply(R13_2, temp_flange_T, 4, 4, 4, T_2);

		if(fabs(T_2[2]) < eps && fabs(T_2[10]) < eps)
		{
			for(int i = 2; i < 4; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_3 = the5_4 = 0;
			temp_value.axis_value[16] = temp_value.axis_value[22] = the5_3;

			sum_theta = atan2(T_2[1], T_2[0]);

			the6_3 = the6_4 = 0;
			temp_value.axis_value[17] = temp_value.axis_value[23] = the6_3;
			the4_3 = the4_4 = sum_theta;
			temp_value.axis_value[15] = temp_value.axis_value[21] = the4_3;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 3 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_3 = acos(T_2[6]);
			temp_value.axis_value[16] = the5_3;
			c4 = -T_2[2] / sin(the5_3);
			s4 = -T_2[10] / sin(the5_3);
			the4_3 = atan2(s4, c4);
			pre_check(4, the4_3, axis_limit, m_pre_theta);
			temp_value.axis_value[15] = the4_3;

			c6 = T_2[4] / sin(the5_3);
			s6 = -T_2[5] / sin(the5_3);
			the6_3 = atan2(s6, c6);
			pre_check(6, the6_3, axis_limit, m_pre_theta);
			temp_value.axis_value[17] = the6_3;

			// solution 4 of theta 5,4,6
			the5_4 = -the5_3;
			temp_value.axis_value[22] = the5_4;
			c4 = -T_2[2] / sin(the5_4);
			s4 = -T_2[10] / sin(the5_4);
			the4_4 = atan2(s4, c4);
			pre_check(4, the4_4, axis_limit, m_pre_theta);
			temp_value.axis_value[21] = the4_4;

			c6 = T_2[4] / sin(the5_4);
			s6 = -T_2[5] / sin(the5_4);
			the6_4 = atan2(s6, c6);
			pre_check(6, the6_4, axis_limit, m_pre_theta);
			temp_value.axis_value[23] = the6_4;
		}
	}

	 // need to check solution by comparing HT matrix with each self and temp_flange_T
	 // solve theta 1 backward solution 5-8
	double	the1_b = the1 + M_PI;
	if(the1_b >= 2 * M_PI)
	{ the1_b = the1_b - 2 * M_PI; }

	for(int i = 4; i < 8; ++i)
	{ temp_value.axis_value[6 * i] = the1_b; }

	 // solve theta 3 and 2 solution 5-6, 7-8
	 // k1,k2,k3,ks declaim when solve forward
	k1 = 2 * a[2] * d[3];
	k2 = 2 * a[2] * a[3];
	k3 = pow(Ocx, 2) +
		pow(Ocy, 2) +
		pow((Ocz + d[0]), 2) -
		2 *
		a[1] *
		Ocx *
		cos(the1_b) +
		2 *
		a[1] *
		Ocy *
		sin(the1_b) +
		pow(a[1], 2) -
		pow(a[2], 2) -
		pow(a[3], 2) -
		pow(d[3], 2);
	ks = pow(k1, 2) + pow(k2, 2) - pow(k3, 2);

	if(ks < 0)
	{
		for(int i = 4; i < 8; ++i)
		{ temp_value.solution_check[i] = false; }
	}

	// 跳過 theta2,3 solution 5-8 if no solution
	if(temp_value.solution_check[4] == true)
	{
		double	the3_56, the3_78, the2_56, the2_78;

		the3_56 = 2 * atan2(k1 - sqrt(ks), k3 + k2);
		the3_78 = 2 * atan2(k1 + sqrt(ks), k3 + k2);

		// check if theta3 over 180 or lower -180
		if(the3_56 > M_PI)
		{ the3_56 = the3_56 - 2 * M_PI; }
		else if(the3_56 < -M_PI)
		{ the3_56 = the3_56 + 2 * M_PI; }

		temp_value.axis_value[26] = temp_value.axis_value[32] = the3_56;

		if(the3_78 > M_PI)
		{ the3_78 = the3_78 - 2 * M_PI; }
		else if(the3_78 < -M_PI)
		{ the3_78 = the3_78 + 2 * M_PI; }

		temp_value.axis_value[38] = temp_value.axis_value[44] = the3_78;

		 // u1,v1,r1,u2,v2,r2, sin2, cos2 declaim before (for calculate theta 2)
		 // solve theta2 solution 5-6
		u1 = a[2] + a[3] * cos(the3_56) + d[3] * sin(the3_56);
		v1 = -a[3] * sin(the3_56) + d[3] * cos(the3_56);
		r1 = Ocx * cos(the1_b) - Ocy * sin(the1_b) - a[1];
		u2 = a[3] * sin(the3_56) - d[3] * cos(the3_56);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_56 = atan2(sin2, cos2);
		temp_value.axis_value[25] = temp_value.axis_value[31] = the2_56;

		// solve theta2 solution 7-8
		u1 = a[2] + a[3] * cos(the3_78) + d[3] * sin(the3_78);
		v1 = -a[3] * sin(the3_78) + d[3] * cos(the3_78);
		r1 = Ocx * cos(the1_b) - Ocy * sin(the1_b) - a[1];
		u2 = a[3] * sin(the3_78) - d[3] * cos(the3_78);
		v2 = u1;
		r2 = -d[0] - Ocz;

		sin2 = (r1 / u1 - r2 / u2) / (v1 / u1 - v2 / u2);
		cos2 = (r1 / v1 - r2 / v2) / (u1 / v1 - u2 / v2);

		the2_78 = atan2(sin2, cos2);
		temp_value.axis_value[37] = temp_value.axis_value[43] = the2_78;

		 // solve theta 5,4,6 solution 5-6 ;
		 // calculate R13 ;
		 // A01, A12, A23, A02, eps, sum_theta discalim already
		double	R13_3[16], R13_4[16], T_3[16], T_4[16];
		double	the5_5, the5_6, the4_5, the4_6, the6_5, the6_6;
		double	the5_7, the5_8, the4_7, the4_8, the6_7, the6_8;

		// calulate R13_3
		Homo_trans((180.0 * the1_b / M_PI), 1, A01);
		Homo_trans((180.0 * the2_56 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_56 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_3);
		inverse(R13_3, 4);		// get inverse2 of R13_3

		MatrixMuliply(R13_3, temp_flange_T, 4, 4, 4, T_3);

		if(fabs(T_3[2]) < eps && fabs(T_3[10]) < eps)
		{
			for(int i = 4; i < 6; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_5 = the5_6 = 0;
			temp_value.axis_value[28] = temp_value.axis_value[34] = the5_5;

			sum_theta = atan2(T_3[1], T_3[0]);

			the6_5 = the6_6 = 0;
			temp_value.axis_value[29] = temp_value.axis_value[35] = the6_5;
			the4_5 = the4_6 = sum_theta;
			temp_value.axis_value[27] = temp_value.axis_value[33] = the4_5;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 5 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_5 = acos(T_3[6]);
			temp_value.axis_value[28] = the5_5;
			c4 = -T_3[2] / sin(the5_5);
			s4 = -T_3[10] / sin(the5_5);
			the4_5 = atan2(s4, c4);
			pre_check(4, the4_5, axis_limit, m_pre_theta);
			temp_value.axis_value[27] = the4_5;

			c6 = T_3[4] / sin(the5_5);
			s6 = -T_3[5] / sin(the5_5);
			the6_5 = atan2(s6, c6);
			pre_check(6, the6_5, axis_limit, m_pre_theta);
			temp_value.axis_value[29] = the6_5;

			// solution 6 of theta 5,4,6
			the5_6 = -the5_5;
			temp_value.axis_value[34] = the5_6;
			c4 = -T_3[2] / sin(the5_6);
			s4 = -T_3[10] / sin(the5_6);
			the4_6 = atan2(s4, c4);
			pre_check(4, the4_6, axis_limit, m_pre_theta);
			temp_value.axis_value[33] = the4_6;

			c6 = T_3[4] / sin(the5_6);
			s6 = -T_3[5] / sin(the5_6);
			the6_6 = atan2(s6, c6);
			pre_check(6, the6_6, axis_limit, m_pre_theta);
			temp_value.axis_value[35] = the6_6;
		}

		 // solve theta 5,4,6 solution 7-8
		 // calculate R13_4 
		Homo_trans((180.0 * the1_b / M_PI), 1, A01);
		Homo_trans((180.0 * the2_78 / M_PI), 2, A12);
		Homo_trans((180.0 * the3_78 / M_PI), 3, A23);

		MatrixMuliply(A01, A12, 4, 4, 4, A02);
		MatrixMuliply(A02, A23, 4, 4, 4, R13_4);
		inverse(R13_4, 4);		// get inverse2 of R13_4

		MatrixMuliply(R13_4, temp_flange_T, 4, 4, 4, T_4);

		if(fabs(T_4[2]) < eps && fabs(T_4[10]) < eps)
		{
			for(int i = 6; i < 8; ++i)
			{ temp_value.singular_check[i] = false; }

			the5_7 = the5_8 = 0;
			temp_value.axis_value[40] = temp_value.axis_value[46] = the5_7;

			sum_theta = atan2(T_4[1], T_4[0]);

			the6_7 = the6_8 = 0;
			temp_value.axis_value[41] = temp_value.axis_value[47] = the6_7;
			the4_7 = the4_8 = sum_theta;
			temp_value.axis_value[39] = temp_value.axis_value[45] = the4_7;
		}
		else		// will get two different solutions of theta 5
		{
			// solution 7 of theta 5,4,6
			double	c4, s4, c6, s6;

			the5_7 = acos(T_4[6]);
			temp_value.axis_value[40] = the5_7;
			c4 = -T_4[2] / sin(the5_7);
			s4 = -T_4[10] / sin(the5_7);
			the4_7 = atan2(s4, c4);
			pre_check(4, the4_7, axis_limit, m_pre_theta);
			temp_value.axis_value[39] = the4_7;

			c6 = T_4[4] / sin(the5_7);
			s6 = -T_4[5] / sin(the5_7);
			the6_7 = atan2(s6, c6);
			pre_check(6, the6_7, axis_limit, m_pre_theta);
			temp_value.axis_value[41] = the6_7;

			// solution 8 of theta 5,4,6
			the5_8 = -the5_7;
			temp_value.axis_value[46] = the5_8;
			c4 = -T_4[2] / sin(the5_8);
			s4 = -T_4[10] / sin(the5_8);
			the4_8 = atan2(s4, c4);
			pre_check(4, the4_8, axis_limit, m_pre_theta);
			temp_value.axis_value[45] = the4_8;

			c6 = T_4[4] / sin(the5_8);
			s6 = -T_4[5] / sin(the5_8);
			the6_8 = atan2(s6, c6);
			pre_check(6, the6_8, axis_limit, m_pre_theta);
			temp_value.axis_value[47] = the6_8;
		}
	}

	// Need to check solution by comparing HT matrix
	for(int i = 0; i < 48; ++i)
	{ temp_value.axis_value[i] = 180.0 * temp_value.axis_value[i] / M_PI; }

	// find the best fit solution
	int check = NO_SOLUTION;
	check = GEN_sol_check(temp_value, axis_limit, m_pre_theta);

	/* output result */
	if(check != 0)
	{
		for(int i = 0; i < 6; ++i)
		{ angle[i] = m_pre_theta[i]; }
		return check;
	}

	// get the best fit solution from temp_value, and upadate m_pre_theta and m_pos_act
	for(int i = 0; i < 6; ++i)
	{ angle[i] = m_pre_theta[i] = temp_value.axis_value[i + 6 * temp_value.fit]; }

	// storage the input tool center point as m_pos_act 在安排位置
	m_pos_act.x = x;
	m_pos_act.y = y;
	m_pos_act.z = z;
	m_pos_act.a = roll;
	m_pos_act.b = pitch;
	m_pos_act.c = yaw;
	for(int i = 0; i < 16; ++i)
	{
		m_pos_act.T[5][i] = temp_T[i];
		m_pos_act.T[6][i] = work_base[i];
	}

	return check;		// IK complete
}

/*----------------------------------------------------------------------------------------------
return true means reach joint limit
----------------------------------------------------------------------------------------------*/
bool PMC_GEN_KIN::GEN_check_limit(double *angle)
{
	for(int j = 0; j < 6; ++j)
	{
		if(angle[j] > axis_limit[j] || angle[j] < axis_limit[6 + j])
		{ return true; }
	}

	return false;
}

/*----------------------------------------------------------------------------------------------
return true means reach joint limit
----------------------------------------------------------------------------------------------*/
bool PMC_GEN_KIN::GEN_check_single_limit(double angle, int Axis_Number)
{
	if((Axis_Number < 0) || (Axis_Number >= 6))
	{ return true; }

	if(angle > axis_limit[Axis_Number] || angle < axis_limit[6 + Axis_Number])
	{ return true; }

	return false;
}

/*----------------------------------------------------------------------------------------------
選取最佳解函數 GEN_sol_check, Input: 軸角度解的結構變數, 角度極限array,
前一步六軸角度array; output: 計算結果代號
----------------------------------------------------------------------------------------------*/
int PMC_GEN_KIN::GEN_sol_check(ARM_AXIS_VALUE &sol, const double *angle_limit, const double *m_pre_theta)
{
	 // This is used to check solution. 0: COMPLETE (OK);
	 // 1: NO_SOLUTION;
	 // 2: ANGLE_LIMIT;
	 // 3: IS_SINGULAR
	int check = COMPLETE;

	// Compare with previous joint value
	sol.fit = 0;

	double	sqsum[8];		// storage square of summary of (solution angle - previous angle ) of every joint
	sqsum[sol.fit] = 1.0e+9;		// Give a extremely large number
	for(int i = 0; i < 8; ++i)
	{
		if(sol.solution_check[i] == true && sol.limit_check[i] == true)
		{
			sqsum[i] = 0;
			for(int j = 0; j < 6; ++j)
			{ sqsum[i] = sqsum[i] + pow((sol.axis_value[i * 6 + j] - m_pre_theta[j]), 2); }

			if(i > 0)
			{
				if(abs(sqsum[sol.fit]) > sqsum[i]) { sol.fit = i; }
			}
		}
	}

	if(sol.solution_check[0] == false && sol.solution_check[4] == false)
	{
		check = NO_SOLUTION;
		return check;
	}

	// check joint limit first
	int n_limit = 0;
	if(sol.solution_check[sol.fit] == true)
	{
		for(int j = 0; j < 6; ++j)
		{
			if(sol.axis_value[sol.fit * 6 + j] > angle_limit[j] || sol.axis_value[sol.fit * 6 + j] < angle_limit[6 + j])
			{
				check = ANGLE_LIMIT;
				return check;
			}
		}
	}

	if(sol.singular_check[sol.fit] == false)
	{
		check = IS_SINGULAR;
		sol.fit = 0;
		return check;
	}

	return check;
}

/*----------------------------------------------------------------------------------------------
各軸角度選取函數 pre_check, 
Input: 軸代號, 軸角度, 角度極限array,前一步六軸角度array,
Output: 軸角度
----------------------------------------------------------------------------------------------*/
void PMC_GEN_KIN::pre_check(int njoint, double &angle, const double *angle_limit, const double *m_pre_theta)
{
	// theta is "radian" in this fiunction
	njoint = njoint - 1;
	double	temp[3] = { 180 * angle / M_PI, (180 * angle / M_PI) + 360.0, (180 * angle / M_PI) - 360.0 };
	double	temp0[3] = { angle, angle + 2 * M_PI, angle - 2 * M_PI };

	for(int i = 1; i < 3; i++)
	{
		if(pow(temp[i] - m_pre_theta[njoint], 2) < pow(temp[0] - m_pre_theta[njoint], 2))
		{
			angle = temp0[i];
		}
	}
}

/*回傳末端點結構變數 GEN_get_pos_act*/
ARM_POS PMC_GEN_KIN::GEN_get_pos_act(void)
{ return m_pos_act; }

/*回傳6個軸的a 值*/
void PMC_GEN_KIN::GEN_get_a(double *a0)
{
	for(int i = 0; i < 6; ++i)
	{ a0[i] = a[i]; }
}

/*回傳6個軸的alpha 值*/
void PMC_GEN_KIN::GEN_get_alpha(double *alpha0)
{
	for(int i = 0; i < 6; ++i)
	{ alpha0[i] = alpha[i]; }
}

/*回傳6個軸的d 值*/
void PMC_GEN_KIN::GEN_get_d(double *d0)
{
	for(int i = 0; i < 6; ++i)
	{ d0[i] = d[i]; }
}

/*回傳6個軸的角度值*/
void PMC_GEN_KIN::GEN_get_theta(double *theta0)
{
	for(int i = 0; i < 6; ++i)
	{ theta0[i] = m_pre_theta[i]; }
}

/*設定6個軸的角度極限值*/
void PMC_GEN_KIN::GEN_set_limit(double *limit0)
{
	for(int i = 0; i < 12; ++i)
	{ axis_limit[i] = limit0[i]; }
}

/*回傳6個軸的角度極限值*/
void PMC_GEN_KIN::GEN_get_limit(double *limit0)
{
	for(int i = 0; i < 12; ++i)
	{ limit0[i] = axis_limit[i]; }
}

/*設定工作區座標系的轉換矩陣*/
void PMC_GEN_KIN::set_bsae(double *base0)
{
	for(int i = 0; i < 16; i++)
	{ work_base_T[i] = base0[i]; }
}

/*回傳工作區座標系的轉換矩陣*/
void PMC_GEN_KIN::get_base(double *base0)
{
	for(int i = 0; i < 16; i++)
	{ base0[i] = work_base_T[i]; }
}

/*設定末端點夾持工具的轉移矩陣*/
bool PMC_GEN_KIN::GEN_set_tool_offset(ARM_POS tool_offset)
{
	double	roll = M_PI * tool_offset.a / 180.0;
	double	pitch = M_PI * tool_offset.b / 180.0;
	double	yaw = M_PI * tool_offset.c / 180.0;

	// calculate T
	rpy2tr(roll, pitch, yaw, m_tool_T);
	m_tool_T[3] = tool_offset.x;
	m_tool_T[7] = tool_offset.y;
	m_tool_T[11] = tool_offset.z;

	return true;
}

/*取得末端點夾持工具的轉移矩陣*/
void PMC_GEN_KIN::Homo_trans(double theta, int n, double *T)
{
	theta = DEG2RAD * theta;

	double	cs = cos(theta);
	double	ss = sin(theta);
	double	ca = 0.0;
	double	sa = 0.0;
	double	A, D;
	double	temp_rad = 0.0;

	switch(n)
	{
	case 1:
		{
			temp_rad = DEG2RAD * alpha[0];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[0];
			D = d[0];
		}
		break;
	case 2:
		{
			temp_rad = DEG2RAD * alpha[1];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[1];
			D = d[1];
		}
		break;
	case 3:
		{
			temp_rad = DEG2RAD * alpha[2];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[2];
			D = d[2];
		}
		break;
	case 4:
		{
			temp_rad = DEG2RAD * alpha[3];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[3];
			D = d[3];
		}
		break;
	case 5:
		{
			temp_rad = DEG2RAD * alpha[4];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[4];
			D = d[4];
		}
		break;
	case 6:
		{
			temp_rad = DEG2RAD * alpha[5];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[5];
			D = d[5];
		}
		break;
	}

	/* Modified */
	T[0] = cs;				T[1] = -ss;			T[2] = 0;			T[3] = A;
	T[4] = ss * ca;		T[5] = cs * ca;		T[6] = -sa;		T[7] = -sa * D;
	T[8] = ss * sa;		T[9] = cs * sa;		T[10] = ca;		T[11] = ca * D;
	T[12] = 0;			T[13] = 0;			T[14] = 0;		T[15] = 1;
}


void PMC_GEN_KIN::changeRobotBaseTCP(const double baseChangeTCP[6])
{
	// rpy2tr( A, B, C, _RobBase2RobRootT );
	rpy2tr( baseChangeTCP[3]*DEG2RAD, baseChangeTCP[4]*DEG2RAD, baseChangeTCP[5]*DEG2RAD, _RobBase2RobRootT );
	_RobBase2RobRootT[3]  = baseChangeTCP[0];	// X
	_RobBase2RobRootT[7]  = baseChangeTCP[1];	// Y
	_RobBase2RobRootT[11] = baseChangeTCP[2];	// Z
}

void PMC_GEN_KIN::changeToolTCP(const double toolChangeTCP[6])
{
	// rpy2tr( A, B, C, _Flange2RobToolT );
	rpy2tr( toolChangeTCP[3]*DEG2RAD, toolChangeTCP[4]*DEG2RAD, toolChangeTCP[5]*DEG2RAD, _Flange2RobToolT );
	_Flange2RobToolT[3]  = toolChangeTCP[0];		// X
	_Flange2RobToolT[7]  = toolChangeTCP[1];		// Y
	_Flange2RobToolT[11] = toolChangeTCP[2];	// Z
}

void PMC_GEN_KIN::changeRobotBaseT(const double baseChangeT[16])
{
	for(int i = 0; i < 16; i++)
	{ _RobBase2RobRootT[i] = baseChangeT[i]; }
}

void PMC_GEN_KIN::changeToolT(const double toolChangeT[16])
{
	for(int i = 0; i < 16; i++)
	{ _Flange2RobToolT[i] = toolChangeT[i]; }
}

void PMC_GEN_KIN::attachBase(double output_RobBase2RobToolTCP[6] , const double input_RobRoot2RobToolTCP[6] , const double BASE_DATA[16])
{
	int i;
	double RobRoot2RobToolT[16] = {0};
	double RobBase2RobToolT[16] = {0};

	// change base
	changeRobotBaseT(BASE_DATA);

	// convert XYZABC to HT ( robot_root T rob_tool )
	// rpy2tr( A, B, C, RobRoot2RobToolT );
	rpy2tr( input_RobRoot2RobToolTCP[3]*DEG2RAD, input_RobRoot2RobToolTCP[4]*DEG2RAD, input_RobRoot2RobToolTCP[5]*DEG2RAD, RobRoot2RobToolT );
	RobRoot2RobToolT[3]  = input_RobRoot2RobToolTCP[0];		// X
	RobRoot2RobToolT[7]  = input_RobRoot2RobToolTCP[1];		// Y
	RobRoot2RobToolT[11] = input_RobRoot2RobToolTCP[2];	// Z

	// cal ( rob_base T rob_tool )
	// rob_base T rob_tool = rob_base T rob_root * rob_root T rob_tool
	MatrixMuliply(_RobBase2RobRootT , RobRoot2RobToolT, 4, 4, 4, RobBase2RobToolT);

	// convert HT to XYZABC 
	double A_RAD , B_RAD , C_RAD;
	tr2rpy( RobBase2RobToolT, &A_RAD, &B_RAD, &C_RAD );
	output_RobBase2RobToolTCP[3] = A_RAD*RAD2DEG;				// A
	output_RobBase2RobToolTCP[4] = B_RAD*RAD2DEG;				// B
	output_RobBase2RobToolTCP[5] = C_RAD*RAD2DEG;				// C 
	output_RobBase2RobToolTCP[0] = RobBase2RobToolT[3];		// X
	output_RobBase2RobToolTCP[1] = RobBase2RobToolT[7];		// Y
	output_RobBase2RobToolTCP[2] = RobBase2RobToolT[11];		// Z
}

void PMC_GEN_KIN::removeBase(double output_RobRoot2RobToolTCP[6] , const double input_RobBase2RobToolTCP[6] , const double BASE_DATA[16])
{
	double RobRoot2RobToolT[16] = {0};	// rob_root T rob_tool
	double RobBase2RobToolT[16] = {0};	// rob_base T rob_tool
	double RobRoot2RobBaseT[16] = {0};	// rob_root T rob_base

	double tempT[16] = {0};

	// change base
	changeRobotBaseT(BASE_DATA);

	// convert XYZABC to HT ( rob_base T rob_tool )
	rpy2tr( input_RobBase2RobToolTCP[3]*DEG2RAD, input_RobBase2RobToolTCP[4]*DEG2RAD, input_RobBase2RobToolTCP[5]*DEG2RAD, RobBase2RobToolT );
	RobBase2RobToolT[3]  = input_RobBase2RobToolTCP[0];		// X
	RobBase2RobToolT[7]  = input_RobBase2RobToolTCP[1];		// Y
	RobBase2RobToolT[11] = input_RobBase2RobToolTCP[2];	// Z

	// cal ( rob_root T rob_tool )
		
	// get -> rob_root T rob_base = inv( rob_base T rob_root )
	memcpy(RobRoot2RobBaseT , _RobBase2RobRootT, sizeof(double) * 16);
	inverse(RobRoot2RobBaseT, 4);
	
	// RobRoot2RobToolT = rob_root T rob_base * rob_base T rob_tool
	MatrixMuliply(RobRoot2RobBaseT, RobBase2RobToolT, 4, 4, 4, RobRoot2RobToolT);

	// convert HT to XYZABC 
	double A_RAD , B_RAD , C_RAD;
	tr2rpy( RobRoot2RobToolT, &A_RAD, &B_RAD, &C_RAD );
	output_RobRoot2RobToolTCP[3] = A_RAD*RAD2DEG;				// A
	output_RobRoot2RobToolTCP[4] = B_RAD*RAD2DEG;				// B
	output_RobRoot2RobToolTCP[5] = C_RAD*RAD2DEG;				// C 
	output_RobRoot2RobToolTCP[0] = RobRoot2RobToolT[3];		// X
	output_RobRoot2RobToolTCP[1] = RobRoot2RobToolT[7];		// Y
	output_RobRoot2RobToolTCP[2] = RobRoot2RobToolT[11];		// Z
}