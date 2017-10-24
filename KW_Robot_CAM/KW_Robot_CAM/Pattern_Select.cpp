// Pattern_Select.cpp : implementation file
//
#pragma once
#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "Pattern_Select.h"
#include "MainFrm.h"
#include "ChildFrm.h"
#include "PatternSelect.h"
#include <math.h>
#include "KW_Robot_CAMView.h"

extern bool PatternSelect_Mode;

// Pattern_Select dialog

IMPLEMENT_DYNCREATE(Pattern_Select, CDialog)

Pattern_Select::Pattern_Select(CWnd* pParent /*=NULL*/)
	: CDialog(Pattern_Select::IDD, pParent)
{

}

Pattern_Select::~Pattern_Select()
{
}

void Pattern_Select::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PatternList, m_PatternList);
}


BEGIN_MESSAGE_MAP(Pattern_Select, CDialog)
	ON_BN_CLICKED(IDC_sedown, &Pattern_Select::OnBnClickedsedown)
	ON_LBN_SELCHANGE(IDC_PatternList, &Pattern_Select::OnLbnSelchangePatternlist)
	ON_BN_CLICKED(IDC_Simulation, &Pattern_Select::OnBnClickedSimulation)
	ON_BN_CLICKED(IDC_sedelete, &Pattern_Select::OnBnClickedsedelete)
	ON_WM_TIMER()
END_MESSAGE_MAP()


// Pattern_Select message handlers

BOOL Pattern_Select::OnInitDialog()
{
	CDialog::OnInitDialog();

	GetDlgItem(IDC_sedown)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_sedelete)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_Simulation)->ShowWindow(SW_HIDE);
	
	LastSelect = -1;

	// TODO:  Add extra initialization here
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
	myAISContext->CloseLocalContext();
	myAISContext->DeactivateStandardMode(TopAbs_COMPOUND);
	myAISContext->DeactivateStandardMode(TopAbs_SOLID);
	myAISContext->DeactivateStandardMode(TopAbs_SHELL);
	myAISContext->DeactivateStandardMode(TopAbs_FACE);
	myAISContext->DeactivateStandardMode(TopAbs_WIRE);
	myAISContext->DeactivateStandardMode(TopAbs_EDGE);
	myAISContext->DeactivateStandardMode(TopAbs_VERTEX);
	myAISContext->DeactivateStandardMode(TopAbs_SHAPE);
	myAISContext->DeactivateStandardMode(TopAbs_COMPOUND);
	myAISContext->OpenLocalContext();
	myAISContext->ActivateStandardMode(TopAbs_FACE);
	myAISContext->SelectionColor(Quantity_NOC_SLATEBLUE);

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void Pattern_Select::OnBnClickedsedown() // generate path for sumulation
{	
	MK.SCARA_FK(0,0,0,0);
	for ( int ii = 0 ; ii < SortPnt.size() ; ii++ )
	{
		double angle[4];
		ARM_AXIS_VALUE_SCARA temp_struct;
		Robot_Deg temp_deg, temp_deg2;

		if ( ii == 0 )
		{
			MK.SCARA_IK(600., 0., (331.98 - 80), 0., angle, temp_struct);
			temp_deg.angle[0] = angle[0]; temp_deg.angle[1] = angle[1]; temp_deg.angle[2] = angle[2]; temp_deg.angle[3] = angle[3];
			temp_deg.x = 600.; temp_deg.y = 0.; temp_deg.z = (331.98 - 80);
			RoboDeg.push_back(temp_deg);

			Interpolation(ii, 0);
		}
		else if ( ii != 0 ) { Interpolation(ii, 0); }

		if ( ii < SortPnt.size()-1 )
		{
			MK.SCARA_IK(SortPnt[ii].L_Pnt.X(), SortPnt[ii].L_Pnt.Y(), SortPnt[ii].L_Pnt.Z(), 0., angle, temp_struct);
			temp_deg2.angle[0] = angle[0]; temp_deg2.angle[1] = angle[1]; temp_deg2.angle[2] = angle[2]; temp_deg2.angle[3] = angle[3];
			temp_deg2.x = SortPnt[ii].L_Pnt.X(); temp_deg2.y = SortPnt[ii].L_Pnt.Y(); temp_deg2.z = SortPnt[ii].L_Pnt.Z();
			RoboDeg.push_back(temp_deg2);
		}
	}

	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	pDoc->OnRobotShowrobot();
	SetTimer(1, 1, NULL);

}

void Pattern_Select::Interpolation(int index, int type)
{
	switch ( type )
	{
	case 0:
		{
			double ori_angle[4];
			ori_angle[0] = RoboDeg[RoboDeg.size()-1].angle[0];
			ori_angle[1] = RoboDeg[RoboDeg.size()-1].angle[1];
			ori_angle[2] = RoboDeg[RoboDeg.size()-1].angle[2];
			ori_angle[3] = RoboDeg[RoboDeg.size()-1].angle[3];

			double angle[4];
			ARM_AXIS_VALUE_SCARA temp_struct;
			Robot_Deg temp_deg;
			MK.SCARA_IK(SortPnt[index].H_Pnt.X(), SortPnt[index].H_Pnt.Y(), SortPnt[index].H_Pnt.Z(), 0., angle, temp_struct);

			int steps = 5;
			for ( int ii = 0 ; ii < steps ; ii++ )
			{
				double a1 = ori_angle[0]+(angle[0]-ori_angle[0])/steps*ii;
				double a2 = ori_angle[1]+(angle[1]-ori_angle[1])/steps*ii;
				double a3 = ori_angle[2]+(angle[2]-ori_angle[2])/steps*ii;
				double a4 = ori_angle[3]+(angle[3]-ori_angle[3])/steps*ii;
				ARM_POS_SCARA temp = MK.SCARA_FK(a1, a2, a3, a4);
				Robot_Deg temp_deg1;
				temp_deg1.angle[0] = ori_angle[0]+(angle[0]-ori_angle[0])/steps*ii;
				temp_deg1.angle[1] = ori_angle[1]+(angle[1]-ori_angle[1])/steps*ii;
				temp_deg1.angle[2] = ori_angle[2]+(angle[2]-ori_angle[2])/steps*ii;
				temp_deg1.angle[3] = ori_angle[3]+(angle[3]-ori_angle[3])/steps*ii;
				temp_deg1.x = temp.x; temp_deg1.y = temp.y; temp_deg1.z = temp.z;
				RoboDeg.push_back(temp_deg1);
			}
			Robot_Deg temp_deg2;
			MK.SCARA_IK(SortPnt[index].H_Pnt.X(), SortPnt[index].H_Pnt.Y(), SortPnt[index].H_Pnt.Z(), 0., angle, temp_struct);
			temp_deg2.x = SortPnt[index].H_Pnt.X(); temp_deg2.y = SortPnt[index].H_Pnt.Y(); temp_deg2.z = SortPnt[index].H_Pnt.Z();
			temp_deg2.angle[0] = angle[0]; temp_deg2.angle[1] = angle[1]; temp_deg2.angle[2] = angle[2]; temp_deg2.angle[3] = angle[3];
			RoboDeg.push_back(temp_deg2);
		}
		break;
	case 1:
		{}
		break;
	}
}

void Pattern_Select::OnLbnSelchangePatternlist()
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();

	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
	int SelectList =  m_PatternList.GetCurSel();

	if ( LastSelect == -1 ) {}
	if ( LastSelect != -1 )
	{
		myAISContext->SetColor(pDoc->PS.PatternInfo[LastSelect].Path,Quantity_NOC_SEAGREEN);
		myAISContext->Redisplay(pDoc->PS.PatternInfo[LastSelect].Path,1);
		myAISContext->SetColor(pDoc->PS.PatternInfo[LastSelect].L_Face,Quantity_NOC_SEAGREEN);
		myAISContext->Redisplay(pDoc->PS.PatternInfo[LastSelect].L_Face,1);
	}
	myAISContext->SetColor(pDoc->PS.PatternInfo[SelectList].Path,Quantity_NOC_RED);
	myAISContext->Redisplay(pDoc->PS.PatternInfo[SelectList].Path,1);
	myAISContext->SetColor(pDoc->PS.PatternInfo[SelectList].L_Face,Quantity_NOC_RED);
	myAISContext->Redisplay(pDoc->PS.PatternInfo[SelectList].L_Face,1);
	myAISContext->UpdateCurrentViewer();

	if ( SelectList >= 0 ) { GetDlgItem(IDC_sedelete)->ShowWindow(SW_SHOW); }

	LastSelect = SelectList;
}

void Pattern_Select::OnBnClickedSimulation() // Generate NN Path
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	pDoc->OnRobotShowrobot();

	OriPnt.SetX(600.); OriPnt.SetY(0.); OriPnt.SetZ(331.98-80); AllPnt.clear();
	for ( int ii = 0 ; ii < pDoc->PS.PatternInfo.size() ; ii++ )
	{
		PntIndex temp_PI;
		temp_PI.Index =ii; temp_PI.H_Pnt = pDoc->PS.PatternInfo[ii].HPnt; temp_PI.L_Pnt = pDoc->PS.PatternInfo[ii].LPnt;
		AllPnt.push_back(temp_PI);
	}

	for ( int ii = 0 ; ii < SortPnt.size() ; ii++ )
	{ pDoc->GetAISContext()->Remove( SortPnt[ii].AIS_Path, Standard_False ); }
	for ( int ii = 0 ; ii < DistStr.size() ; ii++ )
	{ pDoc->GetAISContext()->Remove( DistStr[ii], Standard_False ); }
	pDoc->GetAISContext()->UpdateCurrentViewer();
	SortPnt.clear(); DistStr.clear();

	while ( AllPnt.size() > 0 )
	{ NN_Path(); }

	// Calaulate Path Length
	for ( int ii = 0 ; ii < SortPnt.size() ; ii++ )
	{
		TopoDS_Shape temp_Shape = SortPnt[ii].AIS_Path->Shape();

		// show distance
		BRepAdaptor_Curve curve;
		Standard_Real U;
		gp_Pnt ptn;
		curve.Initialize(TopoDS::Edge(temp_Shape));
		curve.D0(curve.FirstParameter() + (curve.LastParameter() - curve.FirstParameter())/2, ptn);
		GeomAdaptor_Curve adapt =  curve .Curve();
		Standard_Real length = GCPnts_AbscissaPoint::Length(adapt);
		CString CS_Dist;
		CS_Dist.Format("%.2f",length);
		TCollection_AsciiString Ascii_Name((Standard_CString)(LPCTSTR)CS_Dist);
		Handle(ISession_Text) aGraphicText = new ISession_Text(Ascii_Name, ptn.X(), ptn.Y(), ptn.Z());
		aGraphicText->SetScale(5);
		aGraphicText->SetColor(Quantity_NOC_RED);
		DistStr.push_back(aGraphicText);
		pDoc->GetAISContext()->Display(DistStr[DistStr.size() - 1], Standard_False);
	}
	pDoc->GetAISContext()->UpdateCurrentViewer();

	GetDlgItem(IDC_sedown)->ShowWindow(SW_SHOW);
}

void Pattern_Select::NN_Path() // Calculate NN Path
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();

	// Calculate all path
	int min_index = 0; double distance;
	for ( int ii = 0 ; ii < AllPnt.size() ; ii++ )
	{
		TopoDS_Edge temp_E = BRepBuilderAPI_MakeEdge(OriPnt , AllPnt[ii].H_Pnt);
		AllPnt[ii].dist = OriPnt.Distance(AllPnt[ii].H_Pnt);
		AllPnt[ii].AIS_Path = new AIS_Shape(temp_E);
		pDoc->GetAISContext()->Display( AllPnt[ii].AIS_Path, Standard_True );

		if ( ii == 0 ) { distance = AllPnt[ii].dist; min_index = ii; }
		else if ( ii > 0 )
		{
			if ( distance > AllPnt[ii].dist ) { distance = AllPnt[ii].dist; min_index = ii; }
			else if ( distance < AllPnt[ii].dist ) {}
		}
	}

	// plot shortest path
	SortPnt.push_back( AllPnt[min_index] );
	SortPnt[SortPnt.size()-1].AIS_Path->SetColor(Quantity_NOC_SEAGREEN);
	OriPnt.SetX(AllPnt[min_index].H_Pnt.X()); OriPnt.SetY(AllPnt[min_index].H_Pnt.Y()); OriPnt.SetZ(AllPnt[min_index].H_Pnt.Z());

	// Remove previous plot and remove selected path
	for ( int ii = 0 ; ii < AllPnt.size() ; ii++ )
	{ pDoc->GetAISContext()->Remove( AllPnt[ii].AIS_Path, Standard_False ); }
	AllPnt.erase( AllPnt.begin() + min_index );
	pDoc->GetAISContext()->Display( SortPnt[SortPnt.size()-1].AIS_Path, Standard_False );

	if ( AllPnt.size() == 0 )
	{
		PntIndex temp;
		temp.H_Pnt = gp_Pnt(600., 0., (331.98-80));
		TopoDS_Edge temp_E = BRepBuilderAPI_MakeEdge(OriPnt , gp_Pnt(600., 0., (331.98-80)));
		temp.AIS_Path = new AIS_Shape(temp_E);
		SortPnt.push_back(temp);

		SortPnt[SortPnt.size()-1].AIS_Path->SetColor(Quantity_NOC_SEAGREEN);
		pDoc->GetAISContext()->Display( SortPnt[SortPnt.size()-1].AIS_Path, Standard_False );
	}

	pDoc->GetAISContext()->UpdateCurrentViewer();
}

void Pattern_Select::MoveScara(Handle(AIS_InteractiveContext) myAISContext,double *MoveAgnle, double x, double y, double z)
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();

	pDoc->MK.MK_Move_A1(MoveAgnle[0], MOVE_ABS);
	pDoc->MK.MK_Move_A2(MoveAgnle[1], MOVE_ABS);
	pDoc->MK.MK_Move_A3(-(MoveAgnle[2]+80), MOVE_ABS);
	pDoc->MK.MK_Move_Tool(x, y, z);

	pDoc->GetAISContext()->UpdateCurrentViewer();
}

void Pattern_Select::OnBnClickedsedelete()
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();

	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
	int SelectList =  m_PatternList.GetCurSel();

	myAISContext->Remove(pDoc->PS.PatternInfo[SelectList].AIS_Pattern, Standard_False);
	myAISContext->Remove(pDoc->PS.PatternInfo[SelectList].H_Pnt, Standard_False);
	myAISContext->Remove(pDoc->PS.PatternInfo[SelectList].Path, Standard_False);
	myAISContext->Remove(pDoc->PS.PatternInfo[SelectList].L_Face, Standard_False);
	myAISContext->Remove(pDoc->PS.PatternInfo[SelectList].IS_Pnt, Standard_False);
	myAISContext->Remove(pDoc->PS.PatternInfo[SelectList].IS_Text, Standard_False);

	pDoc->PS.PatternInfo.erase( pDoc->PS.PatternInfo.begin()+SelectList );
	m_PatternList.DeleteString(SelectList);

	LastSelect = -1;

	myAISContext->UpdateCurrentViewer();
}

void Pattern_Select::OnTimer(UINT_PTR nIDEvent)
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	myAISContext = pDoc->GetAISContext();

	switch ( nIDEvent )
	{
	case 1:
		{
			MoveScara(myAISContext, RoboDeg[0].angle, RoboDeg[0].x, RoboDeg[0].y, RoboDeg[0].z);
			RoboDeg.erase( RoboDeg.begin() );

			if ( RoboDeg.empty() == true ) { KillTimer(1); }
		}
		break;
	}

	CDialog::OnTimer(nIDEvent);
}
