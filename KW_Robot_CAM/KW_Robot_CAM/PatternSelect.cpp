#pragma once
#include "StdAfx.h"
#include "PatternSelect.h"
#include "Resource.h"
#include "MainFrm.h"
#include "ChildFrm.h"
#include "KW_Robot_CAM.h"
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAdaptor_CompCurve.hxx>
#include <gp_Circ.hxx>
#include <Geom_Circle.hxx>
#include <gp_Ax2.hxx>
#include "ISession_Point.h"
#include "ISession_Text.h"

#pragma comment (lib , "TKV2d.lib")


PatternSelect::PatternSelect(void)
{
	SelectIndex = 1;
}

PatternSelect::~PatternSelect(void)
{
}

void PatternSelect::Select_Pattern(Handle(AIS_InteractiveContext) myAISContext)
{
	// Get Select Shape
	TopoDS_Shape selectshape = myAISContext->SelectedShape();
	Handle(AIS_Shape) temp = new AIS_Shape(selectshape);

	// Get Location
	TopoDS_Vertex pnt1,pnt2;
	TopoDS_Edge PathEdge;
	TopoDS_Face EndFace;
	GetVertex(selectshape/*,&pnt1,&pnt2,&PathEdge,&EndFace*/);

	// Add String
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	CListBox* listBox=(CListBox*)pChild->m_pattern_select_Dlg->GetDlgItem(IDC_PatternList);
	//int temp_int = listBox->GetCount()+1;
	CString temp_str,temp_str2;
	temp_str2.Format("%d",SelectIndex);
	temp_str = "Pattern ";
	temp_str = temp_str + temp_str2;
	listBox->AddString(temp_str);

	if ( listBox->GetCount() > 0 )
	{
		CButton* But=(CButton*)pChild->m_pattern_select_Dlg->GetDlgItem(IDC_Simulation);
		But->ShowWindow(SW_SHOW);
	}

	myAISContext->EraseSelected(Standard_False,Standard_True);
	myAISContext->DeactivateStandardMode(TopAbs_COMPOUND);
	myAISContext->DeactivateStandardMode(TopAbs_SOLID);
	myAISContext->DeactivateStandardMode(TopAbs_SHELL);
	myAISContext->DeactivateStandardMode(TopAbs_FACE);
	myAISContext->DeactivateStandardMode(TopAbs_WIRE);
	myAISContext->DeactivateStandardMode(TopAbs_EDGE);
	myAISContext->DeactivateStandardMode(TopAbs_VERTEX);
	myAISContext->DeactivateStandardMode(TopAbs_SHAPE);
	myAISContext->DeactivateStandardMode(TopAbs_COMPOUND);
	myAISContext->ActivateStandardMode(TopAbs_FACE);

	SelectIndex++;
}

void PatternSelect::GetVertex( TopoDS_Shape inputshape)
{
	// explore shape to edges
	std::vector<TopoDS_Edge> temp_Edge;
	TopExp_Explorer Exp;
	for(Exp.Init(inputshape,TopAbs_EDGE);Exp.More();Exp.Next())
	{
		TopoDS_Edge temp_Solid = TopoDS::Edge(Exp.Current());
		temp_Edge.push_back(temp_Solid);
	}
	BRepBuilderAPI_MakeWire temp_Wire;
	temp_Wire.Add(temp_Edge[0]);
	temp_Wire.Add(temp_Edge[1]);

	// calculate 3 point on shape
	BRepAdaptor_CompCurve compCurve;
	compCurve.Initialize(temp_Wire.Wire() ,Standard_False );
	Standard_Real p1 = compCurve.FirstParameter();
	Standard_Real p2 = (compCurve.FirstParameter()+compCurve.LastParameter())/3;
	Standard_Real p3 = (compCurve.FirstParameter()+compCurve.LastParameter())*2/3;
	gp_Pnt Pnt1 = compCurve.Value(p1);
	gp_Pnt Pnt2 = compCurve.Value(p2);
	gp_Pnt Pnt3 = compCurve.Value(p3);

	// calculate center point & make end vertex and path edge
	gp_Pnt temp_L( (Pnt1.Coord(1)+Pnt2.Coord(1)+Pnt3.Coord(1))/3 , (Pnt1.Coord(2)+Pnt2.Coord(2)+Pnt3.Coord(2))/3 , (Pnt1.Coord(3)+Pnt2.Coord(3)+Pnt3.Coord(3))/3 );
	gp_Pnt temp_H = temp_L;
	temp_H.SetCoord(3,temp_L.Coord(3)+10);
	TopoDS_Vertex Ver1 = BRepBuilderAPI_MakeVertex(temp_L);
	TopoDS_Vertex Ver2 = BRepBuilderAPI_MakeVertex(temp_H);
	TopoDS_Edge temp_E = BRepBuilderAPI_MakeEdge(temp_L , temp_H);
	text_pnt = temp_L;

	// get low point face for plot
	TopoDS_Face temp_Shape;
	Standard_Real temp_dia = temp_L.Distance(Pnt1);
	GetLCircle( temp_L,temp_dia,&temp_Shape);

	PInfo temp_struct;
	temp_struct.AIS_Pattern = new AIS_Shape(inputshape);
	temp_struct.H_Pnt = new AIS_Shape(Ver2);
	temp_struct.HPnt = temp_H;
	temp_struct.L_Pnt = new AIS_Shape(Ver1);
	temp_struct.LPnt = temp_L;
	temp_struct.Path = new AIS_Shape(temp_E);
	temp_struct.L_Face = new AIS_Shape(temp_Shape);
	PatternInfo.push_back(temp_struct);

	DisplayPattern();
}

void PatternSelect::GetLCircle( gp_Pnt LPnt , Standard_Real  C_diameter , TopoDS_Face *Cir )
{
	gp_Circ circle(gp_Ax2(LPnt,gp_Dir(0,0,1)),(C_diameter/2)+1);
	TopoDS_Edge Edge1 = BRepBuilderAPI_MakeEdge(circle,0,2*PI);
	TopoDS_Wire temp_Wire = BRepBuilderAPI_MakeWire(Edge1);
	BRepBuilderAPI_MakeFace temp_Face(temp_Wire);
	*Cir = temp_Face.Face();
}

void PatternSelect::DisplayPattern()
{
	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();

	int temp_int = PatternInfo.size()-1;

	myAISContext->Display(PatternInfo[temp_int].H_Pnt);
	myAISContext->SetColor(PatternInfo[temp_int].Path,Quantity_NOC_SEAGREEN);
	myAISContext->Display(PatternInfo[temp_int].Path);
	myAISContext->SetColor(PatternInfo[temp_int].L_Face,Quantity_NOC_SEAGREEN);
	myAISContext->Display(PatternInfo[temp_int].L_Face);

	//int SelectedCount = PatternInfo.size();
	CString CS_Count;
	CS_Count.Format("%i",SelectIndex);
	CString CS_Name = "P ";
	CS_Name = CS_Name + CS_Count;
	TCollection_AsciiString Ascii_Name((Standard_CString)(LPCTSTR)CS_Name);
    Handle(ISession_Point) aGraphicPoint = new ISession_Point(text_pnt);
	PatternInfo[temp_int].IS_Pnt = aGraphicPoint;
	myAISContext->Display(PatternInfo[temp_int].IS_Pnt);
	Handle(ISession_Text) aGraphicText = new ISession_Text(Ascii_Name,text_pnt.X()+2,text_pnt.Y()+2,text_pnt.Z()+2);
    aGraphicText->SetScale(5);
	PatternInfo[temp_int].IS_Text = aGraphicText;
    myAISContext->Display(PatternInfo[temp_int].IS_Text);
}

void PatternSelect::DrawPath()
{
	TopoDS_Edge temp_E = BRepBuilderAPI_MakeEdge(PatternInfo[PatternInfo.size()-2].HPnt , PatternInfo[PatternInfo.size()-1].HPnt);
	AIS_Path.push_back(new AIS_Shape(temp_E));
	int temp_int = AIS_Path.size()-1;

	CMainFrame *pFrame =  (CMainFrame*)AfxGetApp()->m_pMainWnd;
	CChildFrame *pChild =  (CChildFrame *) pFrame->GetActiveFrame();
	CKW_Robot_CAMDoc *pDoc=(CKW_Robot_CAMDoc*)pChild->GetActiveDocument();
	Handle(AIS_InteractiveContext) myAISContext = pDoc->GetAISContext();
	 myAISContext->Display(AIS_Path[temp_int]);
}
