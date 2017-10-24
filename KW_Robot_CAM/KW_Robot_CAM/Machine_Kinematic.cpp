#include "StdAfx.h"

#include "Machine_Kinematic.h"
#include "MainFrm.h"
#include <BRepBuilderAPI_Transform.hxx>
#include "Resource.h"
#include "KW_Robot_CAMDoc.h"

#define PI 3.14159265359
/*Local function*/
void MatrixMuliply2(double *a,double *b,int a_row, int a_col, int b_col,double *c);
//void trsf2rpy2(double *m,double *roll_z,double *pitch_y,double *yaw_x);
//extern ARM_POS tool_test;
//extern ARM_POS tool;
//ARM_POS tool_param;


Machine_Kinematic::Machine_Kinematic(void): m_ini_A2(-90.0),m_ini_A3(90.0)
{}

Machine_Kinematic::~Machine_Kinematic(void)
{}

std::vector<Handle(AIS_Shape)> *Machine_Kinematic::MK_Get_Machine_Part_Shape(MACHINE_PART part)
{
	switch(part)
	{
		case PART_BASE:
			return &m_BASE;
			break;
		case PART_AX1:
			return &m_AX1;
			break;
		case PART_AX2:
			return &m_AX2;
			break;
		case PART_AX3:
			return &m_AX3;
			break;
		case PART_TOOL:
			return &m_Tool;
			break;
	}

	return NULL;
}

void Machine_Kinematic::SetKinematic_Init(/*double m_a[6] , double m_d[6]*/)
{
	m_A1_Reference = gp_Ax1(gp_Pnt(0., 0., 265.), gp_Dir(0., 0., 1.));
	m_A2_Reference = gp_Ax1(gp_Pnt(300., 0., 364.), gp_Dir(0., 0., 1.));
	m_A3_Reference = gp_Ax1(gp_Pnt(600., 0., 331.98), gp_Dir(0., 0., 1.));
	m_Tool_Reference.SetLocation( gp_Pnt(600., 0., 331.98) );
	m_Tool_Reference.SetDirection( gp_Dir(0., 0., -1.) );	
	m_Tool_Reference.SetXDirection( gp_Dir(1., 0., 0.) );	

	m_A1 = 0.0;
	m_A2 = 0.0;
	m_A3 = 0.0;
	m_A4 = 0.0;
}

/*----------------------------------------------------------------------------------------------
brief Get Homogeneous Transfoamtion matric of each robot axis (in initialized position)
求手臂各軸的轉置矩陣的函數
return none
----------------------------------------------------------------------------------------------*/
void Machine_Kinematic::get_HTmat(double a1, double a2, double a3, double d0, double d3, double d5, double (*m)[16] /*double** m*/)
{
	// assgin new DH table parameter, user can only change certain parameter
	a[1] = a1;
	a[2] = a2;
	a[3] = a3;
	d[0] = d0;
	d[3] = d3;
	d[5] = d5;

	double T1[16];					// delcare A01 D-H HT matrix
	double T2[16];
	double T3[16];
	double T4[16];
	double T5[16];
	double T6[16];

	Homo_trans(theta[0],1,T1);	//A01
	Homo_trans(theta[1],2,T2);	//A12
	Homo_trans(theta[2],3,T3);	//A23
	Homo_trans(theta[3],4,T4);	//A34
	Homo_trans(theta[4],5,T5);	//A45
	Homo_trans(theta[5],6,T6);	//A56

	memcpy(m[0],T1,sizeof(double)*16);	// put A01 into output 2d array
	MatrixMuliply2(T1,T2,4,4,4,m[1]);		// A02 = A01*A12
	MatrixMuliply2(m[1],T3,4,4,4,m[2]);
	MatrixMuliply2(m[2],T4,4,4,4,m[3]);
	MatrixMuliply2(m[3],T5,4,4,4,m[4]);
	MatrixMuliply2(m[4],T6,4,4,4,m[5]);	// A06
}

/*-----------------------------------------------------------------------
轉移矩陣建立函數 Homo_trans, 
Input:軸角度theta, 軸代號 n; 
Output: 軸的轉移矩陣 T
-----------------------------------------------------------------------*/
void Machine_Kinematic::Homo_trans(double theta, int n, double *T)
{
	theta = PI*theta/180.0;

	double cs = cos(theta);
	double ss = sin(theta);
	
	double ca = 0.0;
	double sa = 0.0;
	double A,D;
	
	switch(n){
		case 1:
			ca = cos(PI*alpha[0]/180.0);
			sa = sin(PI*alpha[0]/180.0);
			A = a[0];
			D = d[0];
			break;
		case 2:
			ca = cos(PI*alpha[1]/180.0);
			sa = sin(PI*alpha[1]/180.0);
			A = a[1];
			D = d[1];
			break;
		case 3:
			ca = cos(PI*alpha[2]/180.0);
			sa = sin(PI*alpha[2]/180.0);
			A = a[2];
			D = d[2];
			break;
		case 4:
			ca = cos(PI*alpha[3]/180.0);
			sa = sin(PI*alpha[3]/180.0);
			A = a[3];
			D = d[3];
			break;
		case 5:
			ca = cos(PI*alpha[4]/180.0);
			sa = sin(PI*alpha[4]/180.0);
			A = a[4];
			D = d[4];
			break;
		case 6:
			ca = cos(PI*alpha[5]/180.0);
			sa = sin(PI*alpha[5]/180.0);
			A = a[5];
			D = d[5];
			break;
	}

	/* Modified */
	T[0] = cs;
	T[1] = -ss;
	T[2] = 0;
	T[3] = A;
	T[4] = ss*ca;
	T[5] = cs*ca;
	T[6] = -sa;
	T[7] = -sa*D;
	T[8] = ss*sa;
	T[9] = cs*sa;
	T[10] = ca;
	T[11] = ca*D;
	T[12] = 0;
	T[13] = 0;
	T[14] = 0;
	T[15] = 1;
}

/*----------------------------------------------------------------------------------------------
矩陣相乘函數 MatrixMuliply2:  c = a * b
Input: 矩陣a,  矩陣b, 矩陣a的行數 a_row, 矩陣a的列數 a_col, 
          矩陣b的列數 b_col
Output: 矩陣c
----------------------------------------------------------------------------------------------*/
void MatrixMuliply2(double *a,double *b,int a_row, int a_col, int b_col,double *c)
{
	for(int i=0;i<a_row;i++)
		for(int j=0;j<b_col;j++)
		{
			c[i*b_col+j]=0.0;
			for(int k=0;k<a_col;k++)
				c[i*b_col+j]+=a[i*a_col+k]*b[k*b_col+j];
		}
}

/*----------------------------------------------------------------------------------------------
brief Rotatation Matrix to Roll Pitch Yaw
由旋轉矩陣求出roll,pitch,yaw的函數
return N/A
----------------------------------------------------------------------------------------------*/
//void trsf2rpy2(double *m,double *roll_z,double *pitch_y,double *yaw_x)
//{
//	double eps=2.22044604925031e-5,sp,cp;
//	if(fabs(m[0]) < eps && fabs(m[4]) < eps)
//	{
//		*roll_z  = 0;
//		*pitch_y = atan2(-m[8], m[0]);
//		*yaw_x   = atan2(-m[6], m[5]);
//	}
//	else
//	{
//		*roll_z  = atan2(m[4], m[0]);
//		sp = sin(*roll_z);
//		cp = cos(*roll_z);
//		*pitch_y = atan2(-m[8], cp * m[0] + sp * m[4]);;
//		*yaw_x   = atan2(sp * m[2] - cp * m[6], cp*m[5] - sp*m[1]);
//	}
//}

/* ----- Move function for Robot ----- */
void Machine_Kinematic::MK_Move_A1(double deg,MOVE_TYPE type)
{
	gp_Ax3 ax3C_1(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Vec(1,0,0));
	//Get Degree of A3
	switch(type)
	{
	case MOVE_ABS:
        m_A1 = -deg;
		break;
	case MOVE_REL:
		m_A1 = m_A1 - deg;
		break;
	}
	//Get Transformation 
	ax3C_1 =  ax3C_1.Rotated(m_A1_Reference, PI*m_A1/180.0);//Rotate A1
	m_A1_Trsf.SetTransformation(ax3C_1);
	ChangeLocation(&m_AX1, m_A1_Trsf);

	//A2被A1帶動+自己的軸轉動
	gp_Ax3 ax3C_A2 = rotatedCoupleArm(ax3C_1, m_A1_Trsf, PART_AX2, m_A2_Reference, m_A2);
	//A3被A1,2帶動+自己的軸轉動
	gp_Ax3 ax3C_A3 = translatedCoupleArm(ax3C_A2, m_A1_Trsf, PART_AX3, m_A3_Reference, m_A3);
	////A4被A1,2,3帶動+自己的軸轉動
	gp_Ax3 ax3C_A4 = rotatedCoupleArm(ax3C_A3, m_A1_Trsf, PART_AX3, m_A3_Reference, m_A4);
}

void Machine_Kinematic::MK_Move_A2(double deg,MOVE_TYPE type)
{
	gp_Ax3 ax3C_1(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Vec(1,0,0));
	//Get Degree of A3
	switch(type)
	{
	case MOVE_ABS:
        m_A2 = -deg;
		break;
	case MOVE_REL:
		m_A2 = m_A2 - deg;
		break;
	}
	//Get Transformation (A2 被A1 帶動)
	ax3C_1 =  ax3C_1.Rotated(m_A1_Reference, PI*m_A1/180.0);//Rotate A1
	ax3C_1 =  ax3C_1.Rotated(m_A2_Reference, PI*m_A2/180.0);//Rotate A2  (一開始設-90會出錯 #define ini_m_A2 -90)
	m_A2_Trsf.SetTransformation(ax3C_1);
	ChangeLocation(&m_AX2, m_A2_Trsf);

	//A3被A2帶動+自己的軸轉動
	gp_Ax3 ax3C_A3 = translatedCoupleArm(ax3C_1, m_A2_Trsf, PART_AX3, m_A3_Reference, m_A3);
	////A4被A2帶動+自己的軸轉動
	gp_Ax3 ax3C_A4 = rotatedCoupleArm(ax3C_A3, m_A2_Trsf, PART_AX3, m_A3_Reference, m_A4);
}

void Machine_Kinematic::MK_Move_A3(double deg,MOVE_TYPE type)
{
	gp_Ax3 ax3C_1(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Vec(1,0,0));
	//Get Degree of A3
	switch(type)
	{
	case MOVE_ABS:
        m_A3 = -deg;
		break;
	case MOVE_REL:
		m_A3 = m_A3 - deg;
		break;
	}
	//Get Transformation (A3 被A1,2 帶動)
	ax3C_1 =  ax3C_1.Rotated(m_A1_Reference, PI*m_A1/180.0);//Rotate A1
	ax3C_1 =  ax3C_1.Rotated(m_A2_Reference, PI*m_A2/180.0);//Rotate A2  (一開始設-90會出錯 #define ini_m_A2 -90)
	ax3C_1 =  ax3C_1.Translated(gp_Vec(0., 0., -m_A3));//Translate A3
	m_A3_Trsf.SetTransformation(ax3C_1);
	ChangeLocation(&m_AX3, m_A3_Trsf);

	////A4被A3帶動+自己的軸轉動
	gp_Ax3 ax3C_A4 = rotatedCoupleArm(ax3C_1, m_A3_Trsf, PART_AX3, m_A3_Reference, m_A4);
}

void Machine_Kinematic::MK_Move_A4(double deg,MOVE_TYPE type)
{
	gp_Ax3 ax3C_1(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Vec(1,0,0));
	//Get Degree of A4
	switch(type)
	{
	case MOVE_ABS:
        m_A4 = -deg;
		break;
	case MOVE_REL:
		m_A4 = m_A4 - deg;
		break;
	}
	//Get Transformation (A4 被A1,2,3 帶動)
	ax3C_1 =  ax3C_1.Rotated(m_A1_Reference, PI*m_A1/180.0);//Rotate A1
	ax3C_1 =  ax3C_1.Rotated(m_A2_Reference, PI*m_A2/180.0);//Rotate A2  (一開始設-90會出錯 #define ini_m_A2 -90)
	ax3C_1 =  ax3C_1.Translated(gp_Vec(0., 0., -m_A3));//Rotate A3
	ax3C_1 =  ax3C_1.Rotated(m_A3_Reference, PI*m_A4/180.0);//Rotate A4
	m_A4_Trsf.SetTransformation(ax3C_1);
	ChangeLocation(&m_AX3, m_A4_Trsf);
}

void Machine_Kinematic::ChangeLocation(std::vector<Handle(AIS_Shape)> *part_shape,const gp_Trsf &trsf)
{
	CMDIFrameWnd *pFrame =  (CMDIFrameWnd*)AfxGetApp()->m_pMainWnd;
	CMDIChildWnd *pChild =  (CMDIChildWnd *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	pDoc->GetAISContext()->SetLocation((*part_shape)[0],trsf);
}

gp_Ax3 Machine_Kinematic::rotatedCoupleArm(const gp_Ax3 drive_Ax3, gp_Trsf drive_Trsf, int drived_part, gp_Ax1 drived_reference, double drived_degree)
{
	gp_Ax3 ax3C_temp = drive_Ax3;
	ax3C_temp = ax3C_temp.Rotated(drived_reference,PI*drived_degree/180.0);
	drive_Trsf.SetTransformation(ax3C_temp);
	ChangeLocation(MK_Get_Machine_Part_Shape((MACHINE_PART) drived_part), drive_Trsf);
	return ax3C_temp;
}

gp_Ax3 Machine_Kinematic::translatedCoupleArm(const gp_Ax3 drive_Ax3, gp_Trsf drive_Trsf, int drived_part, gp_Ax1 drived_reference, double drived_degree)
{
	gp_Ax3 ax3C_temp = drive_Ax3;
	ax3C_temp = ax3C_temp.Translated(gp_Vec(0., 0., -drived_degree));
	drive_Trsf.SetTransformation(ax3C_temp);
	ChangeLocation(MK_Get_Machine_Part_Shape((MACHINE_PART) drived_part), drive_Trsf);
	return ax3C_temp;
}

gp_Ax3 Machine_Kinematic::MK_Get_Tool_Original(void)
{ return m_Tool_Reference; }

std::vector<Handle(AIS_Shape)> *Machine_Kinematic::MK_Get_Tool_Part_Shape(void)
{ return &m_Tool; }

void Machine_Kinematic::MK_Move_Tool(double x, double y, double z)
{
	FK_Kinemetic(x, y, z+80);
	ChangeToolLocation();
}

void Machine_Kinematic::FK_Kinemetic(double x, double y, double z)
{
	m_Tool_Reference.SetLocation( gp_Pnt(x, y, z) );
	m_Tool_Reference.SetDirection( gp_Dir(0., 0., -1.) );	
	m_Tool_Reference.SetXDirection( gp_Dir(1., 0., 0.) );
}

void Machine_Kinematic::ChangeToolLocation(void)
{
	CMDIFrameWnd *pFrame =  (CMDIFrameWnd*)AfxGetApp()->m_pMainWnd;
	CMDIChildWnd *pChild =  (CMDIChildWnd *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();

	gp_Trsf m_Trsf;	
	m_Trsf.SetTransformation(m_Tool_Reference);

	pDoc->GetAISContext()->SetLocation(m_Tool[0],m_Trsf.Inverted());
	pDoc->GetAISContext()->UpdateCurrentViewer();
}
