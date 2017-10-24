// ChildFrm.h : interface of the CChildFrame class
//


#pragma once
#include "LoadRobotDlg.h"


class CChildFrame : public CMDIChildWnd
{
	DECLARE_DYNCREATE(CChildFrame)
public:
	CChildFrame();

public:
	CSizingControlBar m_LeftBar;
	CCoolTabCtrl m_TabCtrl;
// Attributes
public:
	CDialog* m_AttributeDlg;
	CDialog* m_pattern_select_Dlg;
	bool ChangeAttributeTab(CRuntimeClass* pClass,UINT nIDTemplate,CString Name);
	bool ChangeAttributeTab(CRuntimeClass* pClass,UINT nIDTemplate);

// Operations
public:

// Overrides
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

// Implementation
public:
	virtual ~CChildFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
};
