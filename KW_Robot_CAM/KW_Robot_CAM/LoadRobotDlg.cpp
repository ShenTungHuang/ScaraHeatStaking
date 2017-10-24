// LoadRobotDlg.cpp : implementation file
//
#pragma once
#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "LoadRobotDlg.h"
#include "MainFrm.h"
#include "ChildFrm.h"
#include "KW_Robot_CAMDoc.h"
#include "KW_Robot_CAMView.h"
#include "ProgressDlg.h"
//#include "Scara_Part.h"
#include "ImportExport.h"
#include "Pattern_Select.h"

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <Standard_PrimitiveTypes.hxx>
#include <gp_Quaternion.hxx>



// LoadRobotDlg dialog
extern Handle(AIS_Shape) 	aisWorkpiece;
extern bool HasLoadRobot;
extern bool PatternSelect_Mode;

IMPLEMENT_DYNCREATE(LoadRobotDlg, CDialog)

LoadRobotDlg::LoadRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialog(LoadRobotDlg::IDD, pParent)
{

}

LoadRobotDlg::~LoadRobotDlg()
{
}

void LoadRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_WorkSpace, m_workspace);
	DDX_Control(pDX, IDC_EDIT_X, m_X);
	DDX_Control(pDX, IDC_EDIT_Y, m_Y);
	DDX_Control(pDX, IDC_EDIT_Z, m_Z);
	DDX_Control(pDX, IDC_EDIT_A, m_A);
	DDX_Control(pDX, IDC_EDIT_B, m_B);
	DDX_Control(pDX, IDC_EDIT_C, m_C);
	DDX_Control(pDX, IDC_SCROLLX, m_SetX);
	DDX_Control(pDX, IDC_SCROLLY, m_SetY);
	DDX_Control(pDX, IDC_SCROLLZ, m_SetZ);
	DDX_Control(pDX, IDC_SCROLLA, m_SetA);
}


BEGIN_MESSAGE_MAP(LoadRobotDlg, CDialog)
	ON_BN_CLICKED(IDOK, &LoadRobotDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_LoadRobot, &LoadRobotDlg::OnBnClickedLoadrobot)
	ON_BN_CLICKED(IDC_LoadWorkpiece, &LoadRobotDlg::OnBnClickedLoadworkpiece)
	ON_BN_CLICKED(IDC_WorkSpace, &LoadRobotDlg::OnBnClickedWorkspace)
	ON_EN_CHANGE(IDC_EDIT_X, &LoadRobotDlg::OnEnChangeEditX)
	ON_EN_CHANGE(IDC_EDIT_Y, &LoadRobotDlg::OnEnChangeEditY)
	ON_EN_CHANGE(IDC_EDIT_Z, &LoadRobotDlg::OnEnChangeEditZ)
	ON_EN_CHANGE(IDC_EDIT_A, &LoadRobotDlg::OnEnChangeEditA)
	ON_EN_CHANGE(IDC_EDIT_B, &LoadRobotDlg::OnEnChangeEditB)
	ON_EN_CHANGE(IDC_EDIT_C, &LoadRobotDlg::OnEnChangeEditC)
	ON_WM_HSCROLL()
	ON_BN_CLICKED(IDC_SelectPattern, &LoadRobotDlg::OnBnClickedSelectpattern)
END_MESSAGE_MAP()


// LoadRobotDlg message handlers

BOOL LoadRobotDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here
	b_COLOR_Check = false;
	HasLoadPart = false;
	PatternSelect_Mode = false;

	m_workspace.ShowWindow(SW_HIDE);

	m_X.ShowWindow(SW_HIDE);
	m_Y.ShowWindow(SW_HIDE);
	m_Z.ShowWindow(SW_HIDE);
	m_A.ShowWindow(SW_HIDE);
	m_B.ShowWindow(SW_HIDE);
	m_C.ShowWindow(SW_HIDE);
	m_SetX.ShowWindow(SW_HIDE);
	m_SetY.ShowWindow(SW_HIDE);
	m_SetZ.ShowWindow(SW_HIDE);
	m_SetA.ShowWindow(SW_HIDE);
	GetDlgItem(IDC_STATIC_X)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_STATIC_Y)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_STATIC_Z)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_STATIC_A)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_LoadWorkpiece)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_SelectPattern)->ShowWindow(SW_HIDE);

	m_SetX.SetScrollRange(-510,600);
	m_SetY.SetScrollRange(-600,600);
	m_SetZ.SetScrollRange(0,332);
	m_SetA.SetScrollRange(-180,180);

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void LoadRobotDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here

	OnOK();
}

void LoadRobotDlg::OnBnClickedLoadrobot()
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	// TODO: Add your control notification handler code here
	AfxGetMainWnd()->BeginWaitCursor();
	ProgressDlg *myProgressDlg = new ProgressDlg;
	myProgressDlg->Create(IDD_PROGRESSDLG);	
	myProgressDlg->CenterWindow();
	myProgressDlg->ShowWindow(SW_SHOWNORMAL);

	myProgressDlg->Total_Number = 4;
	myProgressDlg->Progress_Bar_Pos = 0;
	myProgressDlg->Each_Item_Total_Pos = 2000/(double(myProgressDlg->Total_Number));
	myProgressDlg->size_AIS_Shape = 0;
	myProgressDlg->incremental_cnt = 0.0;
	myProgressDlg->m_MyProgress.SetPos(5);	

	pDoc->MK.SetKinematic_Init();
	CString str;
	str = "Load Base";
	LoadPart(PART_BASE,str,"BASE.STEP",(void*)myProgressDlg);
	str = "Load J1";
	LoadPart(PART_AX1,str,"J1.STEP",(void*)myProgressDlg);
	str = "Load J2";
	LoadPart(PART_AX2,str,"J2.STEP",(void*)myProgressDlg);
	str = "Load J3";
	LoadPart(PART_AX3,str,"J3.STEP",(void*)myProgressDlg);
	LoadTool("Tool.STEP");

	
	CKW_Robot_CAMView* pView = (CKW_Robot_CAMView *) pChild->GetActiveView();
	pView->GetView()->SetProj(V3d_XposYnegZpos);
	pView->GetView()->FitAll();

	// Build Work Space
	HasLoadRobot = true;
	BuildWorkSpace();
	m_workspace.EnableWindow(true);

	m_workspace.ShowWindow(SW_HIDE);

	GetDlgItem(IDC_LoadRobot)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_LoadWorkpiece)->ShowWindow(SW_SHOW);

	myProgressDlg->OnOK();
	AfxGetMainWnd()->EndWaitCursor();
}

void LoadRobotDlg::LoadPart(MACHINE_PART part,CString PartShowsName,CString PartFileName,void *dlg)
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();

	TCHAR szFullPath[MAX_PATH];
	TCHAR szDir[_MAX_DIR];
	TCHAR szDrive[_MAX_DRIVE];
	GetModuleFileName(NULL,szFullPath,MAX_PATH);
	_splitpath_s(szFullPath,szDrive,_MAX_DRIVE,szDir,_MAX_DIR,NULL,NULL,NULL,NULL);
	
	std::vector<Handle(AIS_Shape)> aHSequenceOfAIS_Shape;
	
	CString strPath=("");
	strPath.Format("%s%s",szDrive,szDir);

	std::vector<Handle(AIS_Shape)> *part_shape_ptr;
	part_shape_ptr = pDoc->MK.MK_Get_Machine_Part_Shape(part);

	//Progress bar setting
	ProgressDlg *Dlg = (ProgressDlg*)dlg;
	Dlg->m_Info = PartShowsName;
	Dlg->UpdateData(FALSE);	

	CString temp_Path;
	temp_Path = strPath + "\Body\\" + PartFileName;

	if(b_COLOR_Check)
	{
		CImportExport::ReadSTEP(temp_Path,aHSequenceOfAIS_Shape);	
	}
	else
	{
		Handle(TopTools_HSequenceOfShape) aSequence  =new TopTools_HSequenceOfShape();
		CImportExport::ReadSTEP(temp_Path,aSequence);

		aHSequenceOfAIS_Shape.clear();
		for(int i=1;i<= aSequence->Length();i++)
		{
			Handle(AIS_Shape) temp_AIS_Shape =  new AIS_Shape(aSequence->ChangeValue(i));
			aHSequenceOfAIS_Shape.push_back(temp_AIS_Shape);
		}
	}
	
	//Progress bar setting
	Dlg->size_AIS_Shape = aHSequenceOfAIS_Shape.size();
	Dlg->incremental_cnt = Dlg->Each_Item_Total_Pos/double(Dlg->size_AIS_Shape);

	for(int i=0;i<aHSequenceOfAIS_Shape.size();i++)
	{				
		part_shape_ptr->push_back(aHSequenceOfAIS_Shape[i]);
		
		/* change location */ // 2013-0307
		gp_Ax3 ax3C_1(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Vec(1,0,0));
		gp_Trsf m_Trsf;	
		m_Trsf.SetTransformation(ax3C_1);
		myAISContext->SetLocation((*part_shape_ptr)[part_shape_ptr->size()-1],m_Trsf);
		myAISContext->SetMaterial((*part_shape_ptr)[part_shape_ptr->size()-1], Graphic3d_NOM_SHINY_PLASTIC, Standard_False);
		myAISContext->SetColor((*part_shape_ptr)[part_shape_ptr->size()-1],Quantity_NOC_ALICEBLUE);
		myAISContext->SetDisplayMode((*part_shape_ptr)[part_shape_ptr->size()-1],AIS_Shaded);		
		myAISContext->Display((*part_shape_ptr)[part_shape_ptr->size()-1],Standard_False);			
		
        //Progress bar setting
		Dlg->Progress_Bar_Pos = Dlg->Progress_Bar_Pos + Dlg->incremental_cnt;
		Dlg->m_MyProgress.SetPos(Dlg->Progress_Bar_Pos);		
	}
	myAISContext->UpdateCurrentViewer();
}

void LoadRobotDlg::OnBnClickedLoadworkpiece()
{
	AfxGetMainWnd()->BeginWaitCursor();
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
	bool HasImport = false;
	HasImport = CImportExport::ReadSTEP(myAISContext);
	// 判斷是否讀入工件
	if(!HasImport) { return; }

	// 將讀入之工件存入
	pDoc->Workpicec = aisWorkpiece;

	// 儲存工件座標
	gp_Ax3 new_WorkCoordinate;
	pDoc->WorkCoordinate = new_WorkCoordinate;

	HasLoadPart = true;

	m_X.ShowWindow(SW_SHOW);
	m_Y.ShowWindow(SW_SHOW);
	m_Z.ShowWindow(SW_SHOW);
	m_A.ShowWindow(SW_SHOW);
	m_B.ShowWindow(SW_SHOW);
	m_C.ShowWindow(SW_SHOW);
	m_SetX.ShowWindow(SW_SHOW);
	m_SetY.ShowWindow(SW_SHOW);
	m_SetZ.ShowWindow(SW_SHOW);
	m_SetA.ShowWindow(SW_SHOW);
	GetDlgItem(IDC_STATIC_X)->ShowWindow(SW_SHOW);
	GetDlgItem(IDC_STATIC_Y)->ShowWindow(SW_SHOW);
	GetDlgItem(IDC_STATIC_Z)->ShowWindow(SW_SHOW);
	GetDlgItem(IDC_STATIC_A)->ShowWindow(SW_SHOW);
	GetDlgItem(IDC_SelectPattern)->ShowWindow(SW_SHOW);
	m_workspace.ShowWindow(SW_SHOW);
	GetDlgItem(IDC_LoadWorkpiece)->ShowWindow(SW_HIDE);
	m_X.EnableWindow(true);
	m_Y.EnableWindow(true);
	m_Z.EnableWindow(true);
	m_A.EnableWindow(true);
	m_B.EnableWindow(true);
	m_C.EnableWindow(true);

	m_X.SetWindowTextA("0");
	m_Y.SetWindowTextA("0");
	m_Z.SetWindowTextA("0");
	m_A.SetWindowTextA("0");
	AfxGetMainWnd()->EndWaitCursor();

	for ( int ii = 0 ; ii < 5 ; ii++ )
	{
		std::vector<Handle(AIS_Shape)> *part_shape_temp;
		part_shape_temp = pDoc->MK.MK_Get_Machine_Part_Shape( (MACHINE_PART)(ii) );
		myAISContext->SetTransparency( (*part_shape_temp)[0] , 0.9 , Standard_False );
	}
	myAISContext->UpdateCurrentViewer();
}

void LoadRobotDlg::OnBnClickedWorkspace()
{
	if ( m_workspace.GetCheck() == 0 )
	{
		CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
		CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
		CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
		Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
		myAISContext->Erase(WorkSpace,Standard_True);
	}
	else if ( m_workspace.GetCheck() == 1 )
	{
		CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
		CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
		CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
		Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
		myAISContext->SetDisplayMode(WorkSpace,AIS_Shaded);
		myAISContext->SetTransparency(WorkSpace,0.9);
		myAISContext->Display(WorkSpace);
	}
}

void LoadRobotDlg::BuildWorkSpace()
{
	TopoDS_Shape C1 = BRepPrimAPI_MakeCylinder (gp_Ax2(gp_Pnt(0., 0., -80.),gp_Dir(0.,0.,1.)),600.,331.98,300.*PI/180.); 
	TopoDS_Shape C2 = BRepPrimAPI_MakeCylinder (gp_Ax2(gp_Pnt(0., 0., -80.),gp_Dir(0.,0.,1.)),200,331.98,360.*PI/180.); 
	TopoDS_Shape ShapeCut = BRepAlgoAPI_Cut(C1,C2);

	gp_Trsf theTransformation;
	gp_Ax1 axe = gp_Ax1(gp_Pnt(0.,0.,0.),gp_Dir(0.,0.,1.));
	theTransformation.SetRotation(axe,210*PI/180);
	BRepBuilderAPI_Transform myBRepTransformation(ShapeCut,theTransformation);
	TopoDS_Shape S2 = myBRepTransformation.Shape();
	WorkSpace = new AIS_Shape(S2);
}

void LoadRobotDlg::OnEnChangeEditX()
{
	double temp_dou;
	CString temp_str;
	m_X.GetWindowTextA(temp_str);
	temp_dou = atof(temp_str);
	m_SetX.SetScrollPos( (int)temp_dou );
	SetWPLocation();
}

void LoadRobotDlg::OnEnChangeEditY()
{
	double temp_dou;
	CString temp_str;
	m_Y.GetWindowTextA(temp_str);
	temp_dou = atof(temp_str);
	m_SetY.SetScrollPos( (int)temp_dou );
	SetWPLocation();
}

void LoadRobotDlg::OnEnChangeEditZ()
{
	double temp_dou;
	CString temp_str;
	m_Z.GetWindowTextA(temp_str);
	temp_dou = atof(temp_str);
	m_SetZ.SetScrollPos( (int)temp_dou );
	SetWPLocation();
}

void LoadRobotDlg::OnEnChangeEditA()
{
	double temp_dou;
	CString temp_str;
	m_A.GetWindowTextA(temp_str);
	temp_dou = atof(temp_str);
	m_SetA.SetScrollPos( (int)temp_dou );
	SetWPLocation();
}

void LoadRobotDlg::OnEnChangeEditB()
{ SetWPLocation(); }

void LoadRobotDlg::OnEnChangeEditC()
{ SetWPLocation(); }

void LoadRobotDlg::SetWPLocation()
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();

	// 選取欲改變位置之工件
	gp_Trsf now_trsf;
	now_trsf = pDoc->Workpicec->Shape().Location();

	// 獲得XYZ位置
	double x , y , z , Rx , Ry , Rz ; 
	CString str_x , str_y , str_z , str_Rx , str_Ry , str_Rz ;
	m_X.GetWindowTextA(str_x);
	x = atof(str_x);

	m_Y.GetWindowTextA(str_y);
	y = atof(str_y);

	m_Z.GetWindowTextA(str_z);
	z = atof(str_z);

	// 獲得Rx Ry Rz角度
	double now_rx , now_rz , now_ry;
	now_trsf.GetRotation().GetEulerAngles(gp_Intrinsic_ZYX , now_rx , now_ry , now_rz);
	m_C.GetWindowTextA(str_Rx);
	Rx = atof(str_Rx);

	m_B.GetWindowTextA(str_Ry);
	Ry = atof(str_Ry);

	m_A.GetWindowTextA(str_Rz);
	Rz = atof(str_Rz);

	gp_Trsf last_trsf;
	gp_Trsf now_trsf_Z;
	gp_Trsf now_trsf_Y;
	gp_Trsf now_trsf_X;
	gp_Ax3 ax3C_1(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Vec(1,0,0));
	gp_Ax3 last_workCoordinate = pDoc->WorkCoordinate;
	gp_Ax3 now_workCoordinate;

	// 設定旋轉矩陣
	now_workCoordinate =  now_workCoordinate.Rotated(gp_Ax1(now_workCoordinate.Location(),now_workCoordinate.Axis().Direction()), Rz*PI/180);
	now_workCoordinate =  now_workCoordinate.Rotated(gp_Ax1(now_workCoordinate.Location(),now_workCoordinate.YDirection()) , Ry*PI/180);
	now_workCoordinate =  now_workCoordinate.Rotated(gp_Ax1(now_workCoordinate.Location(),now_workCoordinate.XDirection()) , Rx*PI/180);
	
	// 設定平移矩陣
	now_workCoordinate =  now_workCoordinate.Translated(gp_Vec(1,0,0) * x);//translate X
	now_workCoordinate =  now_workCoordinate.Translated(gp_Vec(0,1,0) * y);//translate Y
	now_workCoordinate =  now_workCoordinate.Translated(gp_Vec(0,0,1) * z);//translate Z

	now_trsf_X.SetDisplacement(last_workCoordinate , now_workCoordinate);
	now_trsf_Y.SetDisplacement(ax3C_1 , now_workCoordinate);

	pDoc->WorkCoordinate = now_workCoordinate;
	
	// 移動工件
	BRepBuilderAPI_Transform myBRepTransformationX(now_trsf_X);
	myBRepTransformationX.Perform(pDoc->Workpicec->Shape());		
	pDoc->Workpicec->Set(myBRepTransformationX.Shape());
	pDoc->GetAISContext()->Redisplay(pDoc->Workpicec);
}

void LoadRobotDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: Add your message handler code here and/or call default
	int hScrollPos = pScrollBar->GetScrollPos();
	int ScrollLimit = pScrollBar->GetScrollLimit() - 1;

	// Get each Limite Range
	int Max_Limi,Min_Limi;
	pScrollBar->GetScrollRange(&Min_Limi , &Max_Limi);

	// Move ScrollBar By Limite Range
	switch(nSBCode)
	{
	case SB_THUMBTRACK:
		hScrollPos =nPos;
		break;
	case SB_PAGEDOWN:
		if(hScrollPos <= (Max_Limi - 10)) {  hScrollPos += 10; }
		break;
	case SB_PAGEUP:
		if(hScrollPos >= (Min_Limi + 10)) { hScrollPos -= 10; }
		break;
	case SB_LINEDOWN:
		if( hScrollPos < Max_Limi ) { ++hScrollPos; }
		break;
	case SB_LINEUP:
		if( hScrollPos > Min_Limi ) { --hScrollPos; }
		break;
	}
	pScrollBar->SetScrollPos(hScrollPos);

	CString temp_str;
	temp_str.Format("%d",hScrollPos);
	if ( Min_Limi == -510 ) { m_X.SetWindowTextA(temp_str); }
	else if ( Min_Limi == -600 ) { m_Y.SetWindowTextA(temp_str); }
	else if ( Min_Limi == 0 ) { m_Z.SetWindowTextA(temp_str); }
	else if ( Min_Limi == -180 ) { m_A.SetWindowTextA(temp_str); }
	
	UpdateData(FALSE);

	CDialog::OnHScroll(nSBCode, nPos, pScrollBar);
}

void LoadRobotDlg::OnBnClickedSelectpattern()
{
	// TODO: Add your control notification handler code here
	if ( HasLoadPart == false ) { AfxMessageBox("No Workpiece"); }
	else if ( HasLoadPart == true )
	{
		CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
		CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
		CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
		Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
		myAISContext->Erase(WorkSpace);
		for ( int ii = 0 ; ii < 5 ; ii++ )
		{
			std::vector<Handle(AIS_Shape)> *part_shape_temp;
			part_shape_temp = pDoc->MK.MK_Get_Machine_Part_Shape( (MACHINE_PART)(ii) );
			myAISContext->Erase( (*part_shape_temp)[0] , 0.9 , Standard_False );
		}
		myAISContext->UpdateCurrentViewer();

		pDoc->HideRobot = true;
		PatternSelect_Mode = true;
		pChild->ChangeAttributeTab(RUNTIME_CLASS(Pattern_Select),IDD_PATTERN_SELECT);
	}
}

void LoadRobotDlg::LoadTool(CString PartFileName)
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
	
	std::vector<Handle(AIS_Shape)> *part_shape_ptr;
	part_shape_ptr = pDoc->MK.MK_Get_Tool_Part_Shape();

	//declaration
	char szFullPath[MAX_PATH];
	char szDir[_MAX_DIR];
	char szDrive[_MAX_DRIVE];
	GetModuleFileName(NULL,szFullPath,MAX_PATH);
	_splitpath_s(szFullPath,szDrive,_MAX_DRIVE,szDir,_MAX_DIR,NULL,NULL,NULL,NULL);
	CString temp_Path;
	CString strPath=("");
	strPath.Format("%s%s",szDrive,szDir);    
	temp_Path = strPath + "\Body\\" + PartFileName;

	/////Load Tool
	std::vector<Handle(AIS_Shape)> aHSequenceOfAIS_Shape;
	Handle(TopTools_HSequenceOfShape) aSequence  =new TopTools_HSequenceOfShape();
	CImportExport::ReadSTEP(temp_Path, aSequence);	
	aHSequenceOfAIS_Shape.clear();
	for( int ii = 1 ; ii <= aSequence->Length() ; ii++ )
	{
		Handle(AIS_Shape) temp_AIS_Shape =  new AIS_Shape( aSequence->ChangeValue(ii) );
		aHSequenceOfAIS_Shape.push_back(temp_AIS_Shape);
	}

	for(int i=0;i<aHSequenceOfAIS_Shape.size();i++)
	{				
		part_shape_ptr->push_back(aHSequenceOfAIS_Shape[i]);

		/* change location */
		gp_Ax3 ax3C_1 = pDoc->MK.MK_Get_Tool_Original();
		gp_Trsf m_Trsf;	
		m_Trsf.SetTransformation(ax3C_1);
		myAISContext->SetLocation((*part_shape_ptr)[part_shape_ptr->size()-1],m_Trsf.Inverted());

		myAISContext->SetDisplayMode((*part_shape_ptr)[part_shape_ptr->size()-1],AIS_Shaded);
		myAISContext->SetMaterial((*part_shape_ptr)[part_shape_ptr->size()-1], Graphic3d_NOM_COPPER, Standard_False);
		myAISContext->Display((*part_shape_ptr)[part_shape_ptr->size()-1],Standard_False);	
	}
	myAISContext->UpdateCurrentViewer();
}
