#pragma once

/*-DEFINES---------逆向運動學計算結果代號-------------------*/
#define COMPLETE			0
#define NO_SOLUTION		1
#define ANGLE_LIMIT		2
#define IS_SINGULAR		3
#define INPUT_INVALID	4

/*------------------------------------------------
儲存手臂末端點的結構變數
 正向運動學的輸出
 ------------------------------------------------*/
typedef struct
{
	/* 末端點的位置 (mm) */
	double x;      
	double y;
	double z;
	/* 末端點的方向 (degree) */ 
	double a;	// roll     
	double b;
	double c;

	double T[6][16];



}ARM_POS_SCARA;

/*-------------------------------------------------------
儲存8組軸角度解的結構變數
逆向運動學的輸出
-------------------------------------------------------*/
typedef struct
{
	/* 6個軸8組解48的角度*/
	double axis_value[8];		// degree // 1-4右手 5-8左手
	bool limit_check[8];			// 檢查角度極限
}ARM_AXIS_VALUE_SCARA;
/*----------------------------*/




/* --------------------SCARA_KIN 函式庫------------------- */
class SCARA_KIN
{
public:
	SCARA_KIN();

	SCARA_KIN::SCARA_KIN(							// 建構式,輸入DH table 
     double a0[4]					/* degree */
	 ,double alpha0[4]
	 ,double d0[4]
	 ,double ini_theta[4]
	 ,double axis_limit0[8]			// degree
    );
	
	~SCARA_KIN(void);// 解構式

	/* 正向運動學函數 SCARA_FK , Input: 6個軸角度, 
	Output: 末端點的結構變數 ARM_POS_SCARA*/
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
	逆向運動學函數 SCARA_IK , 
	Input: 末端點的結構變數 ARM_POS_SCARA,
	Output: 軸角度解的結構變數, 計算結果代號
	----------------------------------------------------------------------------------*/
	int SCARA_IK(ARM_POS_SCARA tcp, ARM_AXIS_VALUE_SCARA& temp_value); // 輸入結構矩陣,輸出結構矩陣
	//int SCARA_ISO_IK(ARM_POS_SCARA tcp, ARM_AXIS_VALUE_SCARA& temp_value);
	
	/*----------------------------------------------------------------------------------
	逆向運動學函數 SCARA_IK, 
	Input: 末端點 x,y,z,a,b,c
	Output: 最佳解角度array, 軸角度解的結構變數, 計算結果代號
	----------------------------------------------------------------------------------*/
	int SCARA_IK(double x, double y, double z, 
		double roll, 
		double* angle/*1X4矩陣*/, ARM_AXIS_VALUE_SCARA& temp_value); // 輸入位置，輸出角度(度度量)
	
	/*---------------------------------------------------------------------------------- 
	選取最佳解函數 SCARA_sol_check, 
	Input: 軸角度解的結構變數, 角度極限array,
	前一步六軸角度array; output: 計算結果代號
	----------------------------------------------------------------------------------*/
	int SCARA_sol_check(ARM_AXIS_VALUE_SCARA& SCARA_sol, const double* angle_limit);

	/*----------------------------------------------------------------------------------
	各軸角度選取函數 pre_check, 
	Input: 軸代號n, 軸角度theta, 角度極限array,前一步六軸角度array, 
	Output: 軸角度theta
	----------------------------------------------------------------------------------*/
	void pre_check(int njoint, double& angle, const double* angle_limit, const double* m_pre_theta);

	/*----------------------------------------------------------------------------------
	轉移矩陣建立函數 Homo_trans, 
	Input:軸角度, 軸代號; 
	Output: 軸的轉移矩陣
	----------------------------------------------------------------------------------*/
	void S_Homo_trans(double theta, double di,int n, double *T);
	void S_ISO_Homo_trans(double theta,double di, int n, double *T);

	/*回傳末端點結構變數*/
	ARM_POS_SCARA SCARA_get_pos_act(void);										
	
	ARM_POS_SCARA SCARA_get_ISO_pos_act(void);										

	/*設定末端點夾持工具的轉移矩陣*/
	bool SCARA_set_tool_offset(ARM_POS_SCARA d_tool_offset);			// m.f. input data type ARM_POS_SCARA; output bool
	/*回傳6個軸的a 值*/
	void SCARA_get_a(double* a0);							// return data member a[6]  (offset of robot arm)

	/*回傳6個軸的alpha 值*/
	void SCARA_get_alpha(double* alpha0);				// return data member alpha[6]  (alpha of robot arm)

	/*回傳6個軸的d 值*/
	void SCARA_get_d(double* d0);							// return data member d[6]  (dsitance of robot arm)

	/*回傳6個軸的角度值*/
	void SCARA_get_theta(double* theta0);				// return data member theta[6]  (intial joint angle of robot arm)

	/*設定6個軸的角度極限值*/
	void SCARA_set_limit(double* limit0);					// set data member axix_limit[12] (angle limti of each joint)
	
	/*回傳6個軸的角度極限值*/
	void SCARA_get_limit(double* limit0);					// return data member axix_limit[12] (angle limti of each joint)

	/*檢查6個軸是否有到達角度極限值*/
	bool SCARA_check_limit(double *angle);        // return true means reach joint limit

	/*檢查單個軸是否有到達角度極限值*/
	bool SCARA_check_single_limit(double angle,int Axis_Number);   // return true means reach joint limit

	/*設定工作區座標系的轉換矩陣*/
	void set_bsae(double* base0);				// set new work_base HT matrix

	/*回傳工作區座標系的轉換矩陣*/
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
	/*工作區座標系的轉移矩陣*/
	double work_base_T[16];		// HomoSCARAous transformation matrix 
	/*在工作區座標系, 末端點位置方位矩陣*/
	double work_base[16];				// TCP in work_ base coordination
	double ISO_work_base[16];				// TCP in work_ base coordination
};

