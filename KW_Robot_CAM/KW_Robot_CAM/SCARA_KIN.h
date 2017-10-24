#pragma once

/*-DEFINES---------�f�V�B�ʾǭp�⵲�G�N��-------------------*/
#define COMPLETE			0
#define NO_SOLUTION		1
#define ANGLE_LIMIT		2
#define IS_SINGULAR		3
#define INPUT_INVALID	4

/*------------------------------------------------
�x�s���u�����I�����c�ܼ�
 ���V�B�ʾǪ���X
 ------------------------------------------------*/
typedef struct
{
	/* �����I����m (mm) */
	double x;      
	double y;
	double z;
	/* �����I����V (degree) */ 
	double a;	// roll     
	double b;
	double c;

	double T[6][16];



}ARM_POS_SCARA;

/*-------------------------------------------------------
�x�s8�նb���׸Ѫ����c�ܼ�
�f�V�B�ʾǪ���X
-------------------------------------------------------*/
typedef struct
{
	/* 6�Ӷb8�ո�48������*/
	double axis_value[8];		// degree // 1-4�k�� 5-8����
	bool limit_check[8];			// �ˬd���׷���
}ARM_AXIS_VALUE_SCARA;
/*----------------------------*/




/* --------------------SCARA_KIN �禡�w------------------- */
class SCARA_KIN
{
public:
	SCARA_KIN();

	SCARA_KIN::SCARA_KIN(							// �غc��,��JDH table 
     double a0[4]					/* degree */
	 ,double alpha0[4]
	 ,double d0[4]
	 ,double ini_theta[4]
	 ,double axis_limit0[8]			// degree
    );
	
	~SCARA_KIN(void);// �Ѻc��

	/* ���V�B�ʾǨ�� SCARA_FK , Input: 6�Ӷb����, 
	Output: �����I�����c�ܼ� ARM_POS_SCARA*/
	ARM_POS_SCARA SCARA_FK(
      double theta_1      //  all input in degree 
	 ,double theta_2
	 ,double theta_3
	 ,double theta_4
	);
	ARM_POS_SCARA SCARA_ISO_FK(
      double theta_1      //  all input in degree 
	 ,double theta_2
	 ,double theta_3
	 ,double theta_4
	);

	/*---------------------------------------------------------------------------------- 
	�f�V�B�ʾǨ�� SCARA_IK , 
	Input: �����I�����c�ܼ� ARM_POS_SCARA,
	Output: �b���׸Ѫ����c�ܼ�, �p�⵲�G�N��
	----------------------------------------------------------------------------------*/
	int SCARA_IK(ARM_POS_SCARA tcp, ARM_AXIS_VALUE_SCARA& temp_value); // ��J���c�x�},��X���c�x�}
	//int SCARA_ISO_IK(ARM_POS_SCARA tcp, ARM_AXIS_VALUE_SCARA& temp_value);
	
	/*----------------------------------------------------------------------------------
	�f�V�B�ʾǨ�� SCARA_IK, 
	Input: �����I x,y,z,a,b,c
	Output: �̨θѨ���array, �b���׸Ѫ����c�ܼ�, �p�⵲�G�N��
	----------------------------------------------------------------------------------*/
	int SCARA_IK(double x, double y, double z, 
		double roll, 
		double* angle/*1X4�x�}*/, ARM_AXIS_VALUE_SCARA& temp_value); // ��J��m�A��X����(�׫׶q)
	
	/*---------------------------------------------------------------------------------- 
	����̨θѨ�� SCARA_sol_check, 
	Input: �b���׸Ѫ����c�ܼ�, ���׷���array,
	�e�@�B���b����array; output: �p�⵲�G�N��
	----------------------------------------------------------------------------------*/
	int SCARA_sol_check(ARM_AXIS_VALUE_SCARA& SCARA_sol, const double* angle_limit);

	/*----------------------------------------------------------------------------------
	�U�b���׿����� pre_check, 
	Input: �b�N��n, �b����theta, ���׷���array,�e�@�B���b����array, 
	Output: �b����theta
	----------------------------------------------------------------------------------*/
	void pre_check(int njoint, double& angle, const double* angle_limit, const double* m_pre_theta);

	/*----------------------------------------------------------------------------------
	�ಾ�x�}�إߨ�� Homo_trans, 
	Input:�b����, �b�N��; 
	Output: �b���ಾ�x�}
	----------------------------------------------------------------------------------*/
	void S_Homo_trans(double theta, double di,int n, double *T);
	void S_ISO_Homo_trans(double theta,double di, int n, double *T);

	/*�^�ǥ����I���c�ܼ�*/
	ARM_POS_SCARA SCARA_get_pos_act(void);										
	
	ARM_POS_SCARA SCARA_get_ISO_pos_act(void);										

	/*�]�w�����I�����u�㪺�ಾ�x�}*/
	bool SCARA_set_tool_offset(ARM_POS_SCARA d_tool_offset);			// m.f. input data type ARM_POS_SCARA; output bool
	/*�^��6�Ӷb��a ��*/
	void SCARA_get_a(double* a0);							// return data member a[6]  (offset of robot arm)

	/*�^��6�Ӷb��alpha ��*/
	void SCARA_get_alpha(double* alpha0);				// return data member alpha[6]  (alpha of robot arm)

	/*�^��6�Ӷb��d ��*/
	void SCARA_get_d(double* d0);							// return data member d[6]  (dsitance of robot arm)

	/*�^��6�Ӷb�����׭�*/
	void SCARA_get_theta(double* theta0);				// return data member theta[6]  (intial joint angle of robot arm)

	/*�]�w6�Ӷb�����׷�����*/
	void SCARA_set_limit(double* limit0);					// set data member axix_limit[12] (angle limti of each joint)
	
	/*�^��6�Ӷb�����׷�����*/
	void SCARA_get_limit(double* limit0);					// return data member axix_limit[12] (angle limti of each joint)

	/*�ˬd6�Ӷb�O�_����F���׷�����*/
	bool SCARA_check_limit(double *angle);        // return true means reach joint limit

	/*�ˬd��Ӷb�O�_����F���׷�����*/
	bool SCARA_check_single_limit(double angle,int Axis_Number);   // return true means reach joint limit

	/*�]�w�u�@�Ϯy�Шt���ഫ�x�}*/
	void set_bsae(double* base0);				// set new work_base HT matrix

	/*�^�Ǥu�@�Ϯy�Шt���ഫ�x�}*/
	void get_base(double* base0);				// get current work_base


	int SCARA_calcAngleYZ(double x0, double y0, double z0, double &theta);
	
	int SCARA_calcForward(double theta1, double theta2, double theta3, double &x0, double &y0, double &z0);

protected:
	
	protected:
	//NON_Modified DH-Table ///
	double a[4];								// mm
	double alpha[4];						// degree
	double d[4];								// mm
	double theta[4];	
	

			// degree

	double SCARA_axis_limit[4];				// degree
	/*�u�@�Ϯy�Шt���ಾ�x�}*/
	double work_base_T[16];		// HomoSCARAous transformation matrix 
	/*�b�u�@�Ϯy�Шt, �����I��m���x�}*/
	double work_base[16];				// TCP in work_ base coordination
	double ISO_work_base[16];				// TCP in work_ base coordination
};

