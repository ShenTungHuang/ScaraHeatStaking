#pragma once
#include "afxwin.h"
#include "SCARA_KIN.h"
#include "afxcmn.h"
#include "ISession_Point.h"
#include "ISession_Text.h"

struct PntIndex
{
	gp_Pnt H_Pnt;
	gp_Pnt L_Pnt;
	int Index;
	double dist;
	Handle(AIS_Shape) AIS_Path;
};

struct Robot_Deg
{
	double x, y, z;
	double angle[4];
};


// Pattern_Select dialog

class Pattern_Select : public CDialog
{
	DECLARE_DYNCREATE(Pattern_Select)

public:
	Pattern_Select(CWnd* pParent = NULL);   // standard constructor
	virtual ~Pattern_Select();

// Dialog Data
	enum { IDD = IDD_PATTERN_SELECT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	CListBox m_PatternList;
	afx_msg void OnBnClickedsedown();
	afx_msg void OnLbnSelchangePatternlist();
	int LastSelect;
	afx_msg void OnBnClickedSimulation();
	SCARA_KIN MK;
	Handle(AIS_InteractiveContext) myAISContext;
	void MoveScara(Handle(AIS_InteractiveContext) myAISContext,double *MoveAngle, double x, double y, double z);
	afx_msg void OnBnClickedsedelete();
	gp_Pnt OriPnt;
	std::vector<PntIndex> AllPnt;
	std::vector<PntIndex> SortPnt;
	std::vector<Robot_Deg> RoboDeg;
	void NN_Path();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	void Interpolation(int index, int type);
	CListCtrl m_Code;
	std::vector<Handle(ISession_Text)> DistStr;
};
