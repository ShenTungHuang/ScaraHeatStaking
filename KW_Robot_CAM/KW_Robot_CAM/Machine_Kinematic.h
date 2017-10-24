#pragma once
#include <vector>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>
#pragma comment (lib , "TKMath.lib")



enum MOVE_TYPE	// Add by STH 2014-12/17
{
	MOVE_ABS,
	MOVE_REL
};

enum MACHINE_PART
{
	PART_BASE,
	PART_AX1,
	PART_AX2,
	PART_AX3,
	PART_TOOL,
};

class Machine_Kinematic
{
public:
	Machine_Kinematic(void);
	~Machine_Kinematic(void);

private:
	gp_Ax1 m_A1_Reference;
	gp_Ax1 m_A2_Reference;
	gp_Ax1 m_A3_Reference;
	gp_Trsf m_A1_Trsf;
	gp_Trsf m_A2_Trsf;
	gp_Trsf m_A3_Trsf;
	gp_Trsf m_A4_Trsf;
	gp_Ax3 m_Tool_Reference;

public:
	std::vector<Handle(AIS_Shape)> m_BASE;
	std::vector<Handle(AIS_Shape)> m_AX1;
	std::vector<Handle(AIS_Shape)> m_AX2;
	std::vector<Handle(AIS_Shape)> m_AX3;
	std::vector<Handle(AIS_Shape)> m_Tool;
	
private:
	gp_Ax3 rotatedCoupleArm(const gp_Ax3 drive_Ax3, gp_Trsf drive_Trsf, int drived_part, gp_Ax1 drived_reference,double drived_degree);
	gp_Ax3 translatedCoupleArm(const gp_Ax3 drive_Ax3, gp_Trsf drive_Trsf, int drived_part, gp_Ax1 drived_reference,double drived_degree);
	void ChangeToolLocation(void);

public:
	void SetKinematic_Init(/*double m_a[6] , double m_d[6]*/);
	void get_HTmat(double a1, double a2, double a3, double d0, double d3, double d5, double (*m)[16]);
	std::vector<Handle(AIS_Shape)> *MK_Get_Machine_Part_Shape(MACHINE_PART part);
	void Homo_trans(double theta, int n, double *T);
	void ChangeLocation(std::vector<Handle(AIS_Shape)> *part_shape,const gp_Trsf &trsf);
	gp_Ax3 MK_Get_Tool_Original(void);
	std::vector<Handle(AIS_Shape)> *MK_Get_Tool_Part_Shape(void);

private:
	/*機械手臂Modified DH-Table 的輸入資料*/
	double a[6];																					// mm
	double alpha[6]/*= {180.0, 90.0, 0.0, 90.0, -90.0, 90.0}*/;		// degree
	double d[6];																					// mm
	double theta[6] /*= {0.0, -90.0, 0.0, 0.0, 0.0, 0.0}*/;					// degree

public:		//移動各軸的function
	void MK_Move_A1(double deg,MOVE_TYPE type);
	void MK_Move_A2(double deg,MOVE_TYPE type);
	void MK_Move_A3(double deg,MOVE_TYPE type);
	void MK_Move_A4(double deg,MOVE_TYPE type);
	void MK_Move_Tool(double x, double y, double z);
	void FK_Kinemetic(double x, double y, double z);

public:		// 儲存各軸現在角度的變數
	double m_A1;
	double m_A2;
	double m_A3;
	double m_A4;
public:		// 儲存空間位置
	double m_X;
	double m_Y;
	double m_Z;
	double m_A;
	double m_B;
	double m_C;
public:
	const double m_ini_A2;		//Compensate offset
	const double m_ini_A3;
};
