// TestSimulation.cpp : implementation file
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "TestSimulationDlg.h"
#include "MainFrm.h"
#include "ChildFrm.h"


// CTestSimulation dialog

IMPLEMENT_DYNAMIC(CTestSimulationDlg, CDialog)

CTestSimulationDlg::CTestSimulationDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CTestSimulationDlg::IDD, pParent)
{

}

CTestSimulationDlg::~CTestSimulationDlg()
{
}

void CTestSimulationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SJ1, m_SliderJ1);
	DDX_Control(pDX, IDC_SJ2, m_SliderJ2);
	DDX_Control(pDX, IDC_SJ3, m_SliderJ3);
	DDX_Control(pDX, IDC_SA1, m_StaticJ1);
	DDX_Control(pDX, IDC_SA2, m_StaticJ2);
	DDX_Control(pDX, IDC_SA3, m_StaticJ3);
	DDX_Control(pDX, IDC_SJ4, m_SliderJ4);
	DDX_Control(pDX, IDC_SA4, m_StaticJ4);
}


BEGIN_MESSAGE_MAP(CTestSimulationDlg, CDialog)
	ON_WM_HSCROLL()
END_MESSAGE_MAP()


// CTestSimulation message handlers

BOOL CTestSimulationDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Slider initial setting
	m_SliderJ1.SetRange(-150., 150.);
	m_SliderJ1.SetPos(-180.);
	m_SliderJ1.SetPos(0.);
	m_SliderJ2.SetRange(-150., 150.);
	m_SliderJ2.SetPos(-180.);
	m_SliderJ2.SetPos(0.);
	m_SliderJ3.SetRange(0., 200.);
	m_SliderJ3.SetPos(200.);
	m_SliderJ3.SetPos(0.);
	m_SliderJ4.SetRange(-360., 360.);
	m_SliderJ4.SetPos(-360.);
	m_SliderJ4.SetPos(0.);

	// Text initial setting
	CString m_a1, m_a2, m_a3, m_a4;
	m_a1.Format("%d", m_SliderJ1.GetPos());
	m_StaticJ1.SetWindowTextA(m_a1);
	m_a2.Format("%d", m_SliderJ2.GetPos());
	m_StaticJ2.SetWindowTextA(m_a2);
	m_a3.Format("%d", m_SliderJ3.GetPos());
	m_StaticJ3.SetWindowTextA(m_a3);;
	m_a4.Format("%d", m_SliderJ4.GetPos());
	m_StaticJ4.SetWindowTextA(m_a4);

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void CTestSimulationDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	int SliderID = pScrollBar->GetDlgCtrlID();

	switch ( SliderID )
	{
	case IDC_SJ1:
		{
			CString str;
			str.Format("%d", m_SliderJ1.GetPos());
			m_StaticJ1.SetWindowTextA(str);
			UpdateData(false);
			pDoc->MK.MK_Move_A1(m_SliderJ1.GetPos(), MOVE_ABS);
		}
		break;
	case IDC_SJ2:
		{
			CString str;
			str.Format("%d", m_SliderJ2.GetPos());
			m_StaticJ2.SetWindowTextA(str);
			UpdateData(false);
			pDoc->MK.MK_Move_A2(m_SliderJ2.GetPos(), MOVE_ABS);
		}
		break;
	case IDC_SJ3:
		{
			CString str;
			str.Format("%d", m_SliderJ3.GetPos());
			m_StaticJ3.SetWindowTextA(str);
			UpdateData(false);
			pDoc->MK.MK_Move_A3(m_SliderJ3.GetPos(), MOVE_ABS);
		}
		break;
	case IDC_SJ4:
		{
			CString str;
			str.Format("%d", m_SliderJ4.GetPos());
			m_StaticJ4.SetWindowTextA(str);
			UpdateData(false);
			pDoc->MK.MK_Move_A4(m_SliderJ4.GetPos(), MOVE_ABS);
		}
		break;
	}

	double a1 = m_SliderJ1.GetPos(), a2 = m_SliderJ2.GetPos(), a3 = m_SliderJ3.GetPos();

	gp_Pnt temp_a1(300., 0., 0.);
	gp_Pnt temp_a2(600., 0., 0.);
	temp_a1.Rotate( gp_Ax1(gp_Pnt(0., 0., 0.), gp_Dir(0., 0., 1.)), a1*PI/180.);
	temp_a2.Rotate( gp_Ax1(gp_Pnt(0., 0., 0.), gp_Dir(0., 0., 1.)), a1*PI/180.);
	temp_a2.Rotate( gp_Ax1(temp_a1, gp_Dir(0., 0., 1.)), a2*PI/180.);
	pDoc->MK.MK_Move_Tool( temp_a2.X(), temp_a2.Y(), (331.98-a3-80) );
	
	pDoc->GetAISContext()->UpdateCurrentViewer();
	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}
