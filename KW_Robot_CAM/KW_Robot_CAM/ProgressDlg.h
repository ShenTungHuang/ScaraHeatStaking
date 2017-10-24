#pragma once
#include "afxcmn.h"


// ProgressDlg dialog

class ProgressDlg : public CDialog
{
	DECLARE_DYNAMIC(ProgressDlg)

public:
	ProgressDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~ProgressDlg();

// Dialog Data
	enum { IDD = IDD_PROGRESSDLG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

public:
	//Set Load items Number and Progress bar pos
	int Total_Number;
	int Progress_Bar_Pos;
	double Each_Item_Total_Pos;
	int size_AIS_Shape;
	double incremental_cnt;
	CProgressCtrl m_MyProgress;
	CString m_Info;
	

	DECLARE_MESSAGE_MAP()
	virtual BOOL OnInitDialog();
	
public:
	virtual void OnOK();
};
