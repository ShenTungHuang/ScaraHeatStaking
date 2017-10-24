#pragma once

/* DEFINES 逆向運動學計算結果代號 */
#define COMPLETE		0
#define NO_SOLUTION		1
#define ANGLE_LIMIT		2
#define IS_SINGULAR		3
#define INPUT_INVALID	4

/*
 -----------------------------------------------------------------------------------------------------------------------
    儲存手臂末端點的結構變數 正向運動學的輸出
 -----------------------------------------------------------------------------------------------------------------------
 */
typedef struct
{

	/* 末端點的位置 (mm) */
	double	x;
	double	y;
	double	z;

	/* 末端點的方向 (degree) */
	double	a;	/* roll */
	double	b;	/* pitch */
	double	c;	/* yaw */

	/* 6個軸與work_base座標系的轉換矩陣 */
	double	T[7][16];
} ARM_POS;

/*
 -----------------------------------------------------------------------------------------------------------------------
    儲存8組軸角度解的結構變數 逆向運動學的輸出
 -----------------------------------------------------------------------------------------------------------------------
 */
typedef struct ARM_AXIS_VALUE
{

	/* 6個軸8組解48的角度 */
	double	axis_value[48];		/* degree */

	/* 以下用來用選取最佳解 */
	int		fit;				/* 最佳解的代號 */
	bool	solution_check[8];	/* 檢查是否有解 */
	bool	singular_check[8];	/* 檢查Singular point */
	bool	limit_check[8];		/* 檢查角度極限 */

	ARM_AXIS_VALUE()
	{
		for(int i = 0; i < 48; i++)
			axis_value[i] = 0;
		fit = 0;
		for(int i = 0; i < 8; i++)
			solution_check[i] = singular_check[i] = limit_check[i] = 0;
	}
};

/*
 =======================================================================================================================
    PMC_GEN_KIN 函式庫
 =======================================================================================================================
 */
class	PMC_GEN_KIN
{
/*
 -----------------------------------------------------------------------------------------------------------------------
 -----------------------------------------------------------------------------------------------------------------------
 */
public:
	PMC_GEN_KIN();						/* 默認建構式 */
	PMC_GEN_KIN::PMC_GEN_KIN
		(								/* 建構式,輸入DH table */
			double a0[6] /* mm */, double alpha0[6] /* degree */, double d0[6] /* mm */, double ini_theta[6] /* degree */,
				double axis_limit0[12]	/* degree */
		);
	~		PMC_GEN_KIN(void);			/* 解構式 */

	/*
	 * 正向運動學函數 GEN_FK , Input: 6個軸角度, Output:
	 * 末端點的結構變數 ARM_POS
	 */
	ARM_POS GEN_FK
			(
				double	theta_1 /* all input in degree */,
				double	theta_2,
				double	theta_3,
				double	theta_4,
				double	theta_5,
				double	theta_6
			);

	/*
	 * 逆向運動學函數 GEN_IK , Input: 末端點的結構變數 ARM_POS, Output:
	 * 軸角度解的結構變數, 計算結果代號
	 */
	int		GEN_IK(ARM_POS tcp, ARM_AXIS_VALUE &temp_value);

	/*
	 * 逆向運動學函數 GEN_IK, Input: 末端點 x,y,z,a,b,c Output:
	 * 最佳解角度array, 軸角度解的結構變數, 計算結果代號
	 */
	int		GEN_IK
			(
				double			x,
				double			y,
				double			z,
				double			roll,
				double			pitch,
				double			yaw,
				double			*angle,
				ARM_AXIS_VALUE	&temp_value
			);

	/*
	 * 選取最佳解函數 GEN_sol_check, Input: 軸角度解的結構變數,
	 * 角度極限array, 前一步六軸角度array;
	 * output: 計算結果代號
	 */
	int		GEN_sol_check(ARM_AXIS_VALUE &sol, const double *angle_limit, const double *m_pre_theta);

	/*
	 * 各軸角度選取函數 pre_check, Input: 軸代號n, 軸角度theta,
	 * 角度極限array,前一步六軸角度array, Output: 軸角度theta
	 */
	void	pre_check(int njoint, double &angle, const double *angle_limit, const double *m_pre_theta);

	/*
	 * 轉移矩陣建立函數 Homo_trans, Input:軸角度, 軸代號;
	 * Output: 軸的轉移矩陣
	 */
	void	Homo_trans(double theta, int n, double *T);

	/* 回傳末端點結構變數 */
	ARM_POS GEN_get_pos_act(void);

	/* 設定末端點夾持工具的轉移矩陣 */
	bool	GEN_set_tool_offset(ARM_POS tool_offset);				/* m.f. input data type
																	 * ARM_POS;
																	 * output bool */

	/* 回傳6個軸的a 值 */
	void	GEN_get_a(double *a0);									/* return data member a[6] (offset of robot arm) */

	/* 回傳6個軸的alpha 值 */
	void	GEN_get_alpha(double *alpha0);							/* return data member alpha[6] (alpha of robot arm) */

	/* 回傳6個軸的d 值 */
	void	GEN_get_d(double *d0);									/* return data member d[6] (dsitance of robot arm) */

	/* 回傳6個軸的角度值 */
	void	GEN_get_theta(double *theta0);							/* return data member theta[6] (intial joint angle
																	 * of robot arm) */

	/* 設定6個軸的角度極限值 */
	void	GEN_set_limit(double *limit0);							/* set data member axix_limit[12] (angle limti of
																	 * each joint) */

	/* 回傳6個軸的角度極限值 */
	void	GEN_get_limit(double *limit0);							/* return data member axix_limit[12] (angle limti
																	 * of each joint) */

	/* 檢查6個軸是否有到達角度極限值 */
	bool	GEN_check_limit(double *angle);							/* return true means reach joint limit */

	/* 檢查單個軸是否有到達角度極限值 */
	bool	GEN_check_single_limit(double angle, int Axis_Number);	/* return true means reach joint limit */

	/* 設定工作區座標系的轉換矩陣 */
	void	set_bsae(double *base0);								/* set new work_base HT matrix */

	/* 回傳工作區座標系的轉換矩陣 */
	void	get_base(double *base0);								/* get current work_base */

/*
 -----------------------------------------------------------------------------------------------------------------------
 -----------------------------------------------------------------------------------------------------------------------
 */
protected:

	/* 機械手臂Modified DH-Table 的輸入資料 */
	double	a[6];				/* mm */
	double	alpha[6];			/* degree */
	double	d[6];				/* mm */
	double	theta[6];			/* degree */

	/* 六軸的角度極限 */
	double	axis_limit[12];		/* degree */

	/* 工作區座標系的轉移矩陣 */
	double	work_base_T[16];	/* Homogenous transformation matrix */

	/* 在工作區座標系, 末端點位置方位矩陣 */
	double	work_base[16];		/* TCP in work_ base coordination */

/*
 -----------------------------------------------------------------------------------------------------------------------
 -----------------------------------------------------------------------------------------------------------------------
 */
private:
	void	MatrixMuliply(double *a, double *b, int a_row, int a_col, int b_col, double *c);
	void	TransposeMatrix(double *a, int a_row, int a_col, double *c);
	void	RotX(double t, double *rot);
	void	RotY(double t, double *rot);
	void	RotZ(double t, double *rot);
	void	rpy2tr(double roll_z, double pitch_y, double yaw_x, double *rot);
	void	tr2rpy(double *m, double *roll_z, double *pitch_y, double *yaw_x);
	void	inverse(double *a, int a_row);

	/*
	 ===================================================================================================================
	 ===================================================================================================================
	 */
	void	SWAP(double &a, double &b)	{
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	double dum = a; a = b; b = dum; }
	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*
 -----------------------------------------------------------------------------------------------------------------------
 -----------------------------------------------------------------------------------------------------------------------
 */
private:
	double	m_ini_theta[6]; /* 儲存初始化的6軸角度 */
	double	m_pre_theta[6]; /* 儲存上一組6軸角度,與求出來的8組解比較 */
	ARM_POS m_pos_act;		/* 刀具中心點(TCP) 的末端點 x,y,z a,b,c T */
	double	m_T_act[16];	/* 刀具中心點(TCP) 相對於 base的 HT matrix */
	double	m_tool_T[16];	/* 刀具中心點(TCP) 相對於 joint 6 的 HT matrix */



public:
	// setting tool & base funciton
	//-------------------------------------------------------//
		// input TCP[6]
		void changeRobotBaseTCP(const double baseChangeTCP[6]);
		void changeToolTCP(const double toolChangeTCP[6]);

		// input T[16]
		void changeRobotBaseT(const double baseChangeT[16]);
		void changeToolT(const double toolChangeT[16]);
	//-------------------------------------------------------//


	// change tool & base function
	//-------------------------------------------------------------------------------------------------------------------------//
	void attachBase(double output_RobBase2RobToolTCP[6] , const double input_RobRoot2RobToolTCP[6] , const double BASE_DATA[16]);
	void removeBase(double output_RobRoot2RobToolTCP[6] , const double input_RobBase2RobToolTCP[6] , const double BASE_DATA[16]);
	//-------------------------------------------------------------------------------------------------------------------------//


private:
	double _RobBase2RobRootT[16]; // rob_base T rob_root
	double _Flange2RobToolT[16];  //   flange T rob_tool
};
