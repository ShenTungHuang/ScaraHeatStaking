#pragma once
#include "afxcmn.h"
#include "afxwin.h"


// CTestSimulationDlg dialog

class CTestSimulationDlg : public CDialog
{
	DECLARE_DYNAMIC(CTestSimulationDlg)

public:
	CTestSimulationDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CTestSimulationDlg();

// Dialog Data
	enum { IDD = IDD_TESTSIMULATIONDLG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	CSliderCtrl m_SliderJ1;
	CSliderCtrl m_SliderJ2;
	CSliderCtrl m_SliderJ3;
	CStatic m_StaticJ1;
	CStatic m_StaticJ2;
	CStatic m_StaticJ3;
	virtual BOOL OnInitDialog();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	CSliderCtrl m_SliderJ4;
	CStatic m_StaticJ4;
};
