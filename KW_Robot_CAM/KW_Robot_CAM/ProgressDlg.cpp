// ProgressDlg.cpp : implementation file
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "ProgressDlg.h"


// ProgressDlg dialog

IMPLEMENT_DYNAMIC(ProgressDlg, CDialog)

ProgressDlg::ProgressDlg(CWnd* pParent /*=NULL*/)
	: CDialog(ProgressDlg::IDD, pParent)
	, m_Info(_T(""))	
{

}

ProgressDlg::~ProgressDlg()
{
}

void ProgressDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PROGRESS1, m_MyProgress);
	DDX_Text(pDX, IDC_Load_Progress, m_Info);
}


BEGIN_MESSAGE_MAP(ProgressDlg, CDialog)
END_MESSAGE_MAP()


// ProgressDlg message handlers

BOOL ProgressDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here
	ModifyStyle(WS_CAPTION,NULL,SWP_DRAWFRAME);
	m_MyProgress.SetRange(0,2000);
	m_MyProgress.SetPos(0);

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void ProgressDlg::OnOK()
{
	// TODO: Add your specialized code here and/or call the base class

	CDialog::OnOK();
}
