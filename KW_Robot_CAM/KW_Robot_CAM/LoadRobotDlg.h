#pragma once
//#include "Scara_Part.h"
#include "afxwin.h"
#include "TestSimulationDlg.h"
#include "Machine_Kinematic.h"


// LoadRobotDlg dialog

class LoadRobotDlg : public CDialog
{
	DECLARE_DYNCREATE(LoadRobotDlg)

public:
	LoadRobotDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~LoadRobotDlg();

// Dialog Data
	enum { IDD = IDD_LOADROBOTDLG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();

public:
	bool b_COLOR_Check;
	//Scara_Part SP;

public:
	Handle(AIS_InteractiveContext) myAISContext;
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedLoadrobot();
	void LoadPart(MACHINE_PART part,CString PartShowsName,CString PartFileName,void *dlg);
	void LoadTool(CString PartFileName);
	afx_msg void OnBnClickedLoadworkpiece();
	afx_msg void OnBnClickedWorkspace();
	CButton m_workspace;
	Handle(AIS_Shape) WorkSpace;
	void BuildWorkSpace();
	CEdit m_X;
	CEdit m_Y;
	CEdit m_Z;
	CEdit m_A;
	CEdit m_B;
	CEdit m_C;
	afx_msg void OnEnChangeEditX();
	afx_msg void OnEnChangeEditY();
	afx_msg void OnEnChangeEditZ();
	afx_msg void OnEnChangeEditA();
	afx_msg void OnEnChangeEditB();
	afx_msg void OnEnChangeEditC();
	void SetWPLocation();
	CScrollBar m_SetX;
	CScrollBar m_SetY;
	CScrollBar m_SetZ;
	CScrollBar m_SetA;
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnBnClickedSelectpattern();
	bool HasLoadPart;
	CTestSimulationDlg * aTSDlg;
};
