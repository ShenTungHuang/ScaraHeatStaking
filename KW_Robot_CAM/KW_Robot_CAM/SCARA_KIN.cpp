/*-INCLUDES------------------------------------------------------------------*/
#include "StdAfx.h"

#include "SCARA_KIN.h"
#include <math.h>

/*-DEFINES---------------------------------------------------------------------*/
#define M_PI 3.1415926
#define DEG2RAD 0.0174532922222222 //M_PI/180

static ARM_POS_SCARA SCARA_pos_act;						// 刀具中心點(TCP) 的末端點 x,y,z a,b,c T
static double m_T_act[16] = {0};				// 刀具中心點(TCP) 相對於 base的 HT matrix
static double m_tool_T[16] = {0};
static double m_ini_theta[4] = {0};				//儲存初始化的6軸角度
static double m_pre_theta[4] = {0};
static double m_ini_di[4] = {0};				//儲存初始化的6軸角度

static ARM_POS_SCARA ISO_SCARA_pos_act;		

void S_MatrixMuliply(double *a,double *b,int a_row, int a_col, int b_col,double *c);
void S_TransposeMatrix(double *a,int a_row, int a_col,double *c);
void S_RotX(double t, double *rot);
void S_RotY(double t, double *rot);
void S_RotZ(double t, double *rot);
void S_rpy2tr(double roll_z,double pitch_y,double yaw_x,double *rot);
void S_tr2rpy(double *m,double *roll_z,double *pitch_y,double *yaw_x);
void S_inverse(double *a, int a_row);
void S_SWAP(double &a, double &b) {double dum=a; a=b; b=dum;}

/*-FUNCTION DEFINITIONS------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------
矩陣相乘函數 MatrixMuliply:  c = a * b
Input: 矩陣a,  矩陣b, 矩陣a的行數 a_row, 矩陣a的列數 a_col, 
          矩陣b的列數 b_col
Output: 矩陣c
----------------------------------------------------------------------------------------------*/
void S_MatrixMuliply(double *a,double *b,int a_row, int a_col, int b_col,double *c)
{
	for(int i=0;i<a_row;i++)
		for(int j=0;j<b_col;j++)
		{
			c[i*b_col+j]=0.0;
			for(int k=0;k<a_col;k++)
				c[i*b_col+j]+=a[i*a_col+k]*b[k*b_col+j];
		}
}

/********************************************************************************/
/** \brief Matrix Transpose c=a' 矩陣轉置函數
*
* \return N/A
*/
void S_TransposeMatrix(double *a,int a_row, int a_col,double *c)
{
	for(int i=0;i<a_row;i++)
		for(int j=0;j<a_col;j++)
			c[j*a_row+i]=a[i*a_col+j];
}

/********************************************************************************/
/** \brief Rotate along X-axis 
* 對X軸旋轉矩陣函數
* \return N/A
*/
void S_RotX(double t, double *rot)   
{ 
	double ct,st;
	ct = cos(t);
	st = sin(t);
	rot[0]=1;  rot[1]=0;   rot[2]=0;     rot[3]=0;
	rot[4]=0;  rot[5]=ct;  rot[6]=-st;   rot[7]=0;
	rot[8]=0;  rot[9]=st;  rot[10]=ct;   rot[11]=0;
	rot[12]=0; rot[13]=0;   rot[14]=0;    rot[15]=1;
}

/********************************************************************************/
/** \brief Rotate along Y-axis
*  對Y軸旋轉矩陣函數
* \return N/A
*/
void S_RotY(double t, double *rot)   
{ 
	double ct,st;
	ct = cos(t);
	st = sin(t);
	rot[0]=ct;   rot[1]=0;   rot[2]=st;    rot[3]=0;
	rot[4]=0;    rot[5]=1;   rot[6]=0;     rot[7]=0;
	rot[8]=-st;  rot[9]=0;   rot[10]=ct;   rot[11]=0;
	rot[12]=0;   rot[13]=0;  rot[14]=0;    rot[15]=1;
}

/********************************************************************************/
/** \brief Rotate along Z-axis
*  對Z軸旋轉矩陣函數
* \return N/A
*/
void S_RotZ(double t, double *rot)   
{ 
	double ct,st;
	ct = cos(t);
	st = sin(t);
	rot[0]=ct;   rot[1]=-st;   rot[2]=0;    rot[3]=0;
	rot[4]=st;   rot[5]=ct;    rot[6]=0;    rot[7]=0;
	rot[8]=0;    rot[9]=0;     rot[10]=1;   rot[11]=0;
	rot[12]=0;   rot[13]=0;    rot[14]=0;   rot[15]=1;
}

/********************************************************************************/
/** \brief Roll Pitch Yaw to Rotatation Matrix
* 由roll,pitch,yaw求出旋轉矩陣
* \return N/A
*/
void S_rpy2tr(double roll_z,double pitch_y,double yaw_x,double *rot)   
{
	double rotz[16],roty[16],rotx[16],temp[16],temp1[16];
	S_RotZ(roll_z,rotz);
	S_RotY(pitch_y,roty);
	S_RotX(yaw_x,rotx);

	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
		{
			temp[i*4+j]=0;
			for(int k=0;k<4;k++)
				temp[i*4+j]+=rotz[i*4+k]*roty[k*4+j];
		}

	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
		{
			temp1[i*4+j]=0;
			for(int k=0;k<4;k++)
				temp1[i*4+j]+=temp[i*4+k]*rotx[k*4+j];
		}
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			rot[i*4+j]=temp1[i*4+j];
	rot[3]=0;
	rot[7]=0;
	rot[11]=0;
	rot[12]=0;
	rot[13]=0;
	rot[14]=0;
	rot[15]=1;
}

/********************************************************************************/
/** \brief Rotatation Matrix to Roll Pitch Yaw
* 由旋轉矩陣求出roll,pitch,yaw的函數
* \return N/A
*/
void S_tr2rpy(double *m,double *roll_z,double *pitch_y,double *yaw_x)
{
	double eps=2.22044604925031e-5,sp,cp;
	if(fabs(m[0]) < eps && fabs(m[4]) < eps)
	{
		*roll_z  = 0;
		*pitch_y = atan2(-m[8], m[0]);
		*yaw_x   = atan2(-m[6], m[5]);
	}
	else
	{
		*roll_z  = atan2(m[4], m[0]);
		sp = sin(*roll_z);
		cp = cos(*roll_z);
		*pitch_y = atan2(-m[8], cp * m[0] + sp * m[4]);;
		*yaw_x   = atan2(sp * m[2] - cp * m[6], cp*m[5] - sp*m[1]);
	}
}

/*----------------------------------------------------------------------
反矩陣計算函數 inverse 
Input: 矩陣a,  矩陣a的行數a_row
----------------------------------------------------------------------*/
void S_inverse(double *a, int a_row)
{
	int i,icol,irow,j,k,l,ll;
	double big,dum,pivinv;
	double *b;
	int n=a_row;
	int m=1;
	int *indxc,*indxr,*ipiv;

	b=new double [a_row];
	for(i=0;i<a_row;i++)
		b[i]=0.0;
	
	indxc=new int [n];
	indxr=new int [n];
	ipiv=new int [n];
	for (j=0;j<n;j++) 
		ipiv[j]=0;
	for (i=0;i<n;i++)
	{
		big=0.0;
		for (j=0;j<n;j++)
			if (ipiv[j] != 1)
				for (k=0;k<n;k++) 
				{
					if (ipiv[k] == 0) 
					{
						if (fabs(a[j*n+k]) >= big) 
						{
							big=fabs(a[j*n+k]);
							irow=j;
							icol=k;
						}
					}
				}
		++(ipiv[icol]);
		if (irow != icol)
		{
			for (l=0;l<n;l++) 
				S_SWAP(a[irow*n+l],a[icol*n+l]);
			for (l=0;l<m;l++) 
				S_SWAP(b[irow*m+l],b[icol*m+l]);
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (a[icol*n+icol] == 0.0) 
			return;
		pivinv=1.0/a[icol*n+icol];
		a[icol*n+icol]=1.0;
		for (l=0;l<n;l++) 
			a[icol*n+l] *= pivinv;
		for (l=0;l<m;l++) 
			b[icol*m+l] *= pivinv;
		for (ll=0;ll<n;ll++)
			if (ll != icol) 
			{
				dum=a[ll*n+icol];
				a[ll*n+icol]=0.0;
				for (l=0;l<n;l++) 
					a[ll*n+l] -= a[icol*n+l]*dum;
				for (l=0;l<m;l++) 
					b[ll*m+l] -= b[icol*m+l]*dum;
			}
	}
	for (l=n-1;l>=0;l--) 
	{
		if (indxr[l] != indxc[l])
			for (k=0;k<n;k++)
				S_SWAP(a[k*n+indxr[l]],a[k*n+indxc[l]]);
	}
	delete [] b;
	delete [] indxc;
	delete [] indxr;
	delete [] ipiv;
}

/***************************************************************************************/

SCARA_KIN::SCARA_KIN()					// 默認建構式
{
	/*初始化默認的( 6Axis Robot) Modified DH-Table*/
   a[0]= 300.0;					/* mm */
   a[1]= 300.0;
   a[2]= 0;
   a[3]=	0;

   alpha[0]=  0;		/* degree */
   alpha[1]=  0;
   alpha[2]=  0.0;
   alpha[3]=  0;

   d[0]= 0.0;			/* mm */
   d[1]= 0.0;
   d[2]= 0;
   d[3]= 331.98;

   theta[0]=   0.0;			/* degree */
   theta[1]=   0.0;
   theta[2]=   0.0;
   theta[3]=   0.0;

   SCARA_axis_limit[0]= 150;
   SCARA_axis_limit[1]= 145;
   SCARA_axis_limit[2]= 180;
   SCARA_axis_limit[3]= 0;

   SCARA_axis_limit[4]= -150;	
   SCARA_axis_limit[5]= -145;
   SCARA_axis_limit[6]= -180;
   SCARA_axis_limit[7]= -200;

	m_ini_theta[0] = theta[0];	//	帶入初始theta角度
	m_ini_theta[1] = theta[1];
	m_ini_theta[2] = theta[2];
	m_ini_theta[3] = theta[3];

	m_ini_di[0]=d[0];			/* mm */
    m_ini_di[1]=d[1];
    m_ini_di[2]=d[2];
    m_ini_di[3]=d[3];

	/* initialize local varibles */
	m_tool_T[0] = 1.0;				// tool center point Home Trans Matrix
	m_tool_T[5] = 1.0;
	m_tool_T[10] = 1.0;
	m_tool_T[15] = 1.0;

	/* initialize work_base HT matrix*/
	for (int i = 0; i < 16; i++)
	{
		if (i%5 == 0)
			work_base_T [i] = 1;
		else
			work_base_T [i] = 0;
	}

	m_ini_theta[0] = theta[0];	//	帶入初始theta角度
	m_ini_theta[1] = theta[1];
	m_ini_theta[2] = theta[2];
	m_ini_theta[3] = theta[3];
	m_ini_theta[4] = theta[4];
	m_ini_theta[5] = theta[5];

	double T1[16];					// delcare A01 D-H HT matrix
	double T2[16];
	double T3[16];
	double T4[16];
	
	double T12[16];
	double T13[16];
	double T14[16];

	S_Homo_trans(m_ini_theta[0],m_ini_di[0],1,T1);	//A01
	S_Homo_trans(m_ini_theta[1],m_ini_di[1],2,T2);	//A12
	S_Homo_trans(m_ini_theta[2],m_ini_di[2],3,T3);
	S_Homo_trans(m_ini_theta[3],m_ini_di[3],4,T4);
    
	S_MatrixMuliply(T1,T2,4,4,4,T12);		// A02 = A01*A12
	S_MatrixMuliply(T12,T3,4,4,4,T13);
	S_MatrixMuliply(T13,T4,4,4,4,T14);

	S_MatrixMuliply(T14,m_tool_T,4,4,4,m_T_act);						// m_T_act is Tool center point HT Matrix
	S_MatrixMuliply(m_T_act,work_base_T,4,4,4,work_base);	
		
	/* calcualte xyzabc */
    SCARA_pos_act.x = work_base[3];
	SCARA_pos_act.y = work_base[7];
	SCARA_pos_act.z = work_base[11];
	S_tr2rpy(work_base,&SCARA_pos_act.a,&SCARA_pos_act.b,&SCARA_pos_act.c);
	SCARA_pos_act.a = 180.0*SCARA_pos_act.a/M_PI;		// change rad to degree
	SCARA_pos_act.b = 0;
	SCARA_pos_act.c = 0;

	/* copy m_T_act */
	memcpy(SCARA_pos_act.T[0],T1,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[1],T12,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[2],T13,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[3],T14,sizeof(double)*16);

	memcpy(SCARA_pos_act.T[4],m_T_act,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[5],work_base,sizeof(double)*16);
}

SCARA_KIN::SCARA_KIN(
      double a0[4]					/* degree */
	 ,double alpha0[4]
	 ,double d0[4]
	 ,double ini_theta[4]
	 ,double axis_limit0[8]
	 )
{
	/*initialize Modified DH-Table*/
	for (int i = 0; i < 4; i++)
	{
		a[i] = a0[i];
		alpha[i] = alpha0[i];
		d[i] = d0[i];
		theta[i] = ini_theta[i];
		SCARA_axis_limit[i] = axis_limit0[i];
	}

	/* initialize local varibles */
	m_tool_T[0] = 1.0;					// tool center point Home Trans Matrix
	m_tool_T[5] = 1.0;
	m_tool_T[10] = 1.0;
	m_tool_T[15] = 1.0;

	/* initialize work_base HT matrix*/
	for (int i = 0; i < 16; i++)
	{
		if (i%5 == 0)
			work_base_T [i] = 1;
		else
			work_base_T [i] = 0;
	}

	m_ini_theta[0] = ini_theta[0];	//	帶入初始theta角度
	m_ini_theta[1] = ini_theta[1];
	m_ini_theta[2] = ini_theta[2];
	m_ini_theta[3] = ini_theta[3];

	double T1[16];							// delcare A01 D-H HT matrix
	double T2[16];
	double T3[16];
	double T4[16];

	double T12[16];
	double T13[16];
	double T14[16];

	S_Homo_trans(m_ini_theta[0],m_ini_di[0],1,T1);	//A01
	S_Homo_trans(m_ini_theta[1],m_ini_di[0],2,T2);	//A12
	S_Homo_trans(m_ini_theta[2],m_ini_di[0],3,T3);
	S_Homo_trans(m_ini_theta[3],m_ini_di[0],4,T4);
    
	S_MatrixMuliply(T1,T2,4,4,4,T12);		// A02 = A01*A12
	S_MatrixMuliply(T12,T3,4,4,4,T13);
	S_MatrixMuliply(T13,T4,4,4,4,T14);

	S_MatrixMuliply(T14,m_tool_T,4,4,4,m_T_act);	// m_T_act is Tool center point HT Matrix
	S_MatrixMuliply(m_T_act, work_base_T,4,4,4, work_base);		// get TCP in work base coordination

	/* calcualte xyzabc */
    SCARA_pos_act.x = work_base[3];
	SCARA_pos_act.y = work_base[7];
	SCARA_pos_act.z = work_base[11];
	S_tr2rpy(work_base,&SCARA_pos_act.a,&SCARA_pos_act.b,&SCARA_pos_act.c);
	SCARA_pos_act.a = 180.0*SCARA_pos_act.a/M_PI;		// change rad to degree
	SCARA_pos_act.b = 0;
	SCARA_pos_act.c = 0;

	/* copy m_T_act */
	memcpy(SCARA_pos_act.T[0],T1,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[1],T12,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[2],T13,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[3],T14,sizeof(double)*16);

	memcpy(SCARA_pos_act.T[4],m_T_act,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[5],work_base,sizeof(double)*16);
}


/* 解構式*/
SCARA_KIN::~SCARA_KIN(void)
{
}

ARM_POS_SCARA SCARA_KIN::SCARA_FK(double theta_1,double theta_2,double theta_3,double theta_4)
{
// Initialize
	static double T1[16];
	static double T2[16];
	static double T3[16];
	static double T4[16];

	static double T12[16];
	static double T13[16];
	static double T14[16];
	static double T16[16];
	
	// storage input 6 angle as previous angles for finding best solution in IK
	m_pre_theta[0]  = theta_1;
	m_pre_theta[1]  = theta_2;
	m_pre_theta[2]  = theta_3;
	m_pre_theta[3]  = theta_4;

	// calculate homogenous matrix for each joint
	S_Homo_trans(theta_1,0,1,T1);
	S_Homo_trans(theta_2,0,2,T2);
	S_Homo_trans(0,theta_3,3,T3);//////////fix
	S_Homo_trans(theta_4,0,4,T4);

	S_MatrixMuliply(T1,T2,4,4,4,T12);
	S_MatrixMuliply(T12,T3,4,4,4,T13);
	S_MatrixMuliply(T13,T4,4,4,4,T14);
	S_MatrixMuliply(T14,m_tool_T,4,4,4,m_T_act);
	S_MatrixMuliply(m_T_act,work_base_T,4,4,4,work_base);		// get TCP in work base coordination

	/* calcualte xyzabc */
    SCARA_pos_act.x = work_base[3];
	SCARA_pos_act.y = work_base[7];
	SCARA_pos_act.z = work_base[11];
	S_tr2rpy(work_base,&SCARA_pos_act.a,&SCARA_pos_act.b,&SCARA_pos_act.c);
	SCARA_pos_act.a = 180.0*SCARA_pos_act.a/M_PI;
	SCARA_pos_act.b = 0;
	SCARA_pos_act.c = 0;

	/* copy m_T_act */
	memcpy(SCARA_pos_act.T[0],T1,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[1],T12,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[2],T13,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[3],T14,sizeof(double)*16);

	memcpy(SCARA_pos_act.T[5],m_T_act,sizeof(double)*16);
	memcpy(SCARA_pos_act.T[6],work_base,sizeof(double)*16);

	return SCARA_pos_act;
}

 int SCARA_KIN::SCARA_IK(ARM_POS_SCARA tcp, ARM_AXIS_VALUE_SCARA & temp_value)
 {
	//ARM_AXIS_VALUE temp_value;
	for (int i = 0; i < 8; ++i)
		temp_value.axis_value[i] = 0.0;
	   
	double roll = M_PI*tcp.a/180.0;
	double pitch = 0;
	double yaw = 0;

	/* calculate tcp T */
	S_rpy2tr(roll, pitch, yaw,work_base);
	work_base[3] = tcp.x;
	work_base[7] = tcp.y;
	work_base[11] = tcp.z;

	/* translate work base coordination system to robot coordination */
	double work_base_T_inv[16];
	memcpy(work_base_T_inv, work_base_T, sizeof(double)*16);
	S_inverse(work_base_T_inv,4);

	double temp_T[16];
	S_MatrixMuliply(work_base_T_inv,work_base ,4,4,4,temp_T);

	/* copy tool offset T */
	
	double temp_tool_T_inv[16];
	memcpy(temp_tool_T_inv,m_tool_T,sizeof(double)*16);

	/* inverse */
	S_inverse(temp_tool_T_inv,4);

	/* get flange T */
	double temp_flange_T[16];
    S_MatrixMuliply(temp_T,temp_tool_T_inv,4,4,4,temp_flange_T);

	double x0,y0,z0,a0,b0,c0;
	x0=temp_flange_T[3];
	y0=temp_flange_T[7];
	z0=temp_flange_T[11];
	
	S_tr2rpy(temp_flange_T,&a0,&b0,&c0);

	double c2temp;
	double the1,the2,the3,the4,the12,the22,the32,the42;
	c2temp=(x0*x0+y0*y0-a[0]*a[0]-a[1]*a[1])/(2*a[0]*a[1]);

	the2=acos(c2temp);
	the1=atan2(y0,x0)-atan2((a[1]*sin(the2)),(a[0]+a[1]*cos(the2)));
	the4=-the2-the1+a0;
	the3=z0-d[3];

	the22=-acos(c2temp);
	the12=atan2(y0,x0)-atan2((a[1]*sin(the22)),(a[0]+a[1]*cos(the22)));
	the42=-the22-the12+a0;
	the32=z0-d[3];
	
	temp_value.axis_value[0]=the1/M_PI*180;
	temp_value.axis_value[1]=the2/M_PI*180;
	temp_value.axis_value[2]=the3;
    temp_value.axis_value[3]=the4/M_PI*180;    

	temp_value.axis_value[4]=the12/M_PI*180;
	temp_value.axis_value[5]=the22/M_PI*180;
	temp_value.axis_value[6]=the32;
    temp_value.axis_value[7]=the42/M_PI*180;

	 return 0;
 }

 int SCARA_KIN::SCARA_IK(double x, double y, double z,
		double roll,
		double* angle, ARM_AXIS_VALUE_SCARA& temp_value)
 {
	double x0,y0,z0,a0,b0,c0;
	x0=x;
	y0=y;
	z0=z;
	a0=roll/180*M_PI;
	b0=0;
	c0=0;
	 
	 double c2temp;
	double the1,the2,the3,the4,the12,the22,the32,the42;
	c2temp=(x0*x0+y0*y0-a[0]*a[0]-a[1]*a[1])/(2*a[0]*a[1]);

	the2=acos(c2temp);
	the1=atan2(y0,x0)-atan2((a[1]*sin(the2)),(a[0]+a[1]*cos(the2)));
	the4=-the2-the1+a0;
	the3=z0-d[3];

	the22=-acos(c2temp);
	the12=atan2(y0,x0)-atan2((a[1]*sin(the22)),(a[0]+a[1]*cos(the22)));
	the42=-the22-the12+a0;
	the32=z0-d[3];

	temp_value.axis_value[0]=the1/M_PI*180;
	temp_value.axis_value[1]=the2/M_PI*180;
	temp_value.axis_value[2]=the3;
    temp_value.axis_value[3]=the4/M_PI*180;    

	temp_value.axis_value[4]=the12/M_PI*180;
	temp_value.axis_value[5]=the22/M_PI*180;
	temp_value.axis_value[6]=the32;
    temp_value.axis_value[7]=the42/M_PI*180;

	if (m_pre_theta[1]>=0)
	{
		angle[0]=the1/M_PI*180;
		angle[1]=the2/M_PI*180;
		angle[2]=the3;
		angle[3]=the4/M_PI*180;
	}
	else
	{       
		angle[0]=the12/M_PI*180;
		angle[1]=the22/M_PI*180;
		angle[2]=the32;
		angle[3]=the42/M_PI*180;
	}

	 return 0;
 }

 ARM_POS_SCARA SCARA_KIN::SCARA_get_pos_act(void)
{
	return SCARA_pos_act;
}

bool SCARA_KIN::SCARA_check_limit(double *angle)
{
	for(int j = 0; j < 4; ++j)
	{								
		if (angle[j]  > SCARA_axis_limit[j] || angle[j] < SCARA_axis_limit[4+j])
		{
			return true;			
		}
	}

	return false;
}


bool SCARA_KIN::SCARA_check_single_limit(double angle,int Axis_Number)
{
	return false;
}






int SCARA_KIN::SCARA_sol_check(ARM_AXIS_VALUE_SCARA& SCARA_sol, const double* SCARA_angle_limit)
{
	/* 
	This is used to check SCARA_solution. 
	0: COMPLETE (OK); 
	1: NO_SCARA_solUTION; 
	2: ANGLE_LIMIT; 
	3: IS_SINGULAR 
	*/
	int check = COMPLETE;				
	/* check joint limit first*/
	
		int n_limit = 0;
			for(int j = 0; j < 8; ++j)
			{								
				if (SCARA_sol.axis_value[j]  > SCARA_angle_limit[j] || SCARA_sol.axis_value[j] < SCARA_angle_limit[4+j])
				{
					SCARA_sol.limit_check[j] = false;
				}
				if (SCARA_sol.limit_check[j] == false)
				n_limit = n_limit + 1;
			}
		if (n_limit >0)
		{
			check = ANGLE_LIMIT;
			return check;
		}
	
	return check;
}



bool SCARA_KIN::SCARA_set_tool_offset(ARM_POS_SCARA d_tool_offset)
{
	double roll = M_PI*d_tool_offset.a/180.0;
	double pitch = M_PI*d_tool_offset.b/180.0;
	double yaw = M_PI*d_tool_offset.c/180.0;

	/* calculate T */
	double tool_xyzabc[6]={0};
	tool_xyzabc[0]=d_tool_offset.x;
	tool_xyzabc[1]=d_tool_offset.y;
	tool_xyzabc[2]=d_tool_offset.z;
	tool_xyzabc[3]=roll;
	tool_xyzabc[4]=0;
	tool_xyzabc[5]=0;
	
	return true;
}

void SCARA_KIN::S_Homo_trans(double theta,double di, int n, double *T)
{
	theta = DEG2RAD*theta;
	 
	double cs = cos(theta);
	double ss = sin(theta);
	
	double ca = 0.0;
	double sa = 0.0;
	double A,D;

	double temp_rad = 0.0;
	
	switch(n){
		case 1:
			temp_rad = DEG2RAD*alpha[0];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[0];
			D = d[0];
			break;
		case 2:
			temp_rad = DEG2RAD*alpha[1];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[1];
			D = d[1];
			break;
		case 3:
			temp_rad = DEG2RAD*alpha[2];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[2];
			D = di+d[2];
			break;
		case 4:
			temp_rad = DEG2RAD*alpha[3];
			ca = cos(temp_rad);
			sa = sin(temp_rad);
			A = a[3];
			D = d[3];
			break;
	}
	
	T[0] =     cs;
	T[1] = -ss*ca;
	T[2] =  ss*sa;
	T[3] =   A*cs;
	T[4] =     ss;
	T[5] =  cs*ca;
	T[6] = -cs*sa;
	T[7] =   A*ss;
	T[8] =      0;
	T[9] =     sa;
	T[10] =    ca;
	T[11] =     D;
	T[12] =     0;
	T[13] =     0;
	T[14] =     0;
	T[15] =     1;
}