// ChildFrm.cpp : implementation of the CChildFrame class
//
#include "stdafx.h"
#include "KW_Robot_CAM.h"

#include "ChildFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CChildFrame

IMPLEMENT_DYNCREATE(CChildFrame, CMDIChildWnd)

BEGIN_MESSAGE_MAP(CChildFrame, CMDIChildWnd)
	ON_WM_CREATE()
END_MESSAGE_MAP()


// CChildFrame construction/destruction

CChildFrame::CChildFrame()
{
	// TODO: add member initialization code here
}

CChildFrame::~CChildFrame()
{
}


BOOL CChildFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying the CREATESTRUCT cs
	if( !CMDIChildWnd::PreCreateWindow(cs) )
		return FALSE;

	cs.style = WS_CHILD | WS_VISIBLE | WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU
		| FWS_ADDTOTITLE | WS_THICKFRAME | WS_MAXIMIZE;

	return TRUE;
}


// CChildFrame diagnostics

#ifdef _DEBUG
void CChildFrame::AssertValid() const
{
	CMDIChildWnd::AssertValid();
}

void CChildFrame::Dump(CDumpContext& dc) const
{
	CMDIChildWnd::Dump(dc);
}

#endif //_DEBUG


// CChildFrame message handlers

int CChildFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CMDIChildWnd::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here
	EnableDocking(CBRS_ALIGN_ANY);
	if(!m_LeftBar.Create(_T("Test Dock"),this,CSize(240,260),TRUE,124))
	{
		TRACE0("Failed to Test Dock\n");
		return -1;	
	}

	m_LeftBar.SetSCBStyle(m_LeftBar.GetSCBStyle() |
		SCBS_SIZECHILD |	CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC);

	m_LeftBar.EnableDocking(CBRS_ALIGN_ANY);

	DockControlBar(&m_LeftBar, AFX_IDW_DOCKBAR_LEFT);
	m_TabCtrl.Create(TCS_UP | WS_CHILD | WS_VISIBLE,CRect(0,0,200,200),&m_LeftBar,125);
	ChangeAttributeTab(RUNTIME_CLASS(LoadRobotDlg), IDD_LOADROBOTDLG,"STH");


	return 0;
}

bool CChildFrame::ChangeAttributeTab(CRuntimeClass* pClass,UINT nIDTemplate,CString Name)
{
	m_AttributeDlg = (CDialog*)pClass->CreateObject();
	if(m_AttributeDlg != NULL)
	{
		m_AttributeDlg->Create(nIDTemplate,&m_TabCtrl);
	}
	m_TabCtrl.AddPage(m_AttributeDlg,Name);
	m_TabCtrl.UpdateWindow();
	return true;
}

bool CChildFrame::ChangeAttributeTab(CRuntimeClass* pClass,UINT nIDTemplate)
{
	if ( nIDTemplate == IDD_PATTERN_SELECT )
	{ m_pattern_select_Dlg = (CDialog*)pClass->CreateObject(); }


	if(m_AttributeDlg != NULL)
	{
		if ( nIDTemplate == IDD_PATTERN_SELECT )
		{
			m_pattern_select_Dlg = (CDialog*)pClass->CreateObject();
			m_pattern_select_Dlg->Create(nIDTemplate,&m_TabCtrl);
		}
		else { m_AttributeDlg->Create(nIDTemplate,&m_TabCtrl); }
	}

	if ( nIDTemplate == IDD_PATTERN_SELECT )
	{
		CString stable;
		stable = "Pattern";
		m_TabCtrl.AddPage(m_pattern_select_Dlg,(stable));
	}
	
	m_TabCtrl.UpdateWindow();
	return true;
}
