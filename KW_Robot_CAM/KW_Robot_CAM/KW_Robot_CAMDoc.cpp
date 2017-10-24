// KW_Robot_CAMDoc.cpp : implementation of the CKW_Robot_CAMDoc class
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include <vector>

#include "KW_Robot_CAMDoc.h"
#include <gp_Pln.hxx>
#include <Geom_Plane.hxx>
#include <GC_MakePlane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include "ISession_Surface.h"
#pragma comment (lib , "TKG3d.lib")
#pragma comment (lib , "TKGeomBase.lib")

extern bool HasLoadRobot;
extern bool PatternSelect_Mode;


//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// CKW_Robot_CAMDoc

IMPLEMENT_DYNCREATE(CKW_Robot_CAMDoc, CDocument)

BEGIN_MESSAGE_MAP(CKW_Robot_CAMDoc, CDocument)
	ON_COMMAND(ID_ROBOT_HIDEROBOT, &CKW_Robot_CAMDoc::OnRobotHiderobot)
	ON_UPDATE_COMMAND_UI(ID_ROBOT_HIDEROBOT, &CKW_Robot_CAMDoc::OnUpdateRobotHiderobot)
	ON_COMMAND(ID_ROBOT_SHOWROBOT, &CKW_Robot_CAMDoc::OnRobotShowrobot)
	ON_UPDATE_COMMAND_UI(ID_ROBOT_SHOWROBOT, &CKW_Robot_CAMDoc::OnUpdateRobotShowrobot)
END_MESSAGE_MAP()


// CKW_Robot_CAMDoc construction/destruction

CKW_Robot_CAMDoc::CKW_Robot_CAMDoc()
{
	// TODO: add one-time construction code here

}

CKW_Robot_CAMDoc::~CKW_Robot_CAMDoc()
{
}

BOOL CKW_Robot_CAMDoc::OnNewDocument()
{
	if (!OCCDoc::OnNewDocument())
		return FALSE;	

	myAISContext = GetAISContext();

	// axis
	Handle(Geom_Axis2Placement) anAxis = new Geom_Axis2Placement(gp_Pnt(0,0,0),gp_Vec(0,0,1),gp_Dir(1,0,0));
	Handle(AIS_Trihedron) aTrihedron = new AIS_Trihedron(anAxis);
	aTrihedron->SetSize(200);
	myAISContext->Display(aTrihedron);

	gp_Ax3 theAxe(gp::XOY());
	gp_Pln PL(theAxe);                            
	Handle(Geom_Plane) aPlane = GC_MakePlane(PL).Value();
	Handle(Geom_RectangularTrimmedSurface) aSurface= new Geom_RectangularTrimmedSurface(aPlane,-600.,600.,-600.,600.);
	Handle(ISession_Surface) aGraphicalSurface = new ISession_Surface(aSurface);
	myAISContext->Display(aGraphicalSurface);

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)
	HasLoadRobot = false;
	HideRobot = false;

	return TRUE;
}




// CKW_Robot_CAMDoc serialization

void CKW_Robot_CAMDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CKW_Robot_CAMDoc diagnostics

#ifdef _DEBUG
void CKW_Robot_CAMDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CKW_Robot_CAMDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CKW_Robot_CAMDoc commands

void CKW_Robot_CAMDoc::OnRobotHiderobot()
{
	// TODO: Add your command handler code here
	if ( HasLoadRobot == false ) {}
	else if ( HasLoadRobot == true )
	{		
		for ( int ii = 0 ; ii < 4 ; ii++ )
		{
			std::vector<Handle(AIS_Shape)> *part_shape_temp;
			part_shape_temp = MK.MK_Get_Machine_Part_Shape( (MACHINE_PART)(ii) );
			myAISContext->Erase( (*part_shape_temp)[0] , 0.9 , Standard_False );
		}
		myAISContext->UpdateCurrentViewer();
		HideRobot = true;
		PatternSelect_Mode = true;
	}
}

void CKW_Robot_CAMDoc::OnUpdateRobotHiderobot(CCmdUI *pCmdUI)
{
	// TODO: Add your command update UI handler code here
	if ( (HasLoadRobot == false) || (HideRobot == true) ) { pCmdUI->Enable(false); }
}

void CKW_Robot_CAMDoc::OnRobotShowrobot()
{
	// TODO: Add your command handler code here
	if ( HasLoadRobot == false ) {}
	else if ( HasLoadRobot == true )
	{
		for (int ii = 0 ; ii < 5 ; ii++)
		{		
			std::vector<Handle(AIS_Shape)> *part_shape_temp;
			part_shape_temp = MK.MK_Get_Machine_Part_Shape( (MACHINE_PART)(ii) );
			if ( ii < 4 )
			{
				myAISContext->SetMaterial((*part_shape_temp)[0], Graphic3d_NOM_SHINY_PLASTIC, Standard_False);
				myAISContext->SetColor((*part_shape_temp)[0],Quantity_NOC_ALICEBLUE);
			}
			else if ( ii == 4 )
			{ myAISContext->SetMaterial((*part_shape_temp)[0], Graphic3d_NOM_COPPER, Standard_False); }
			myAISContext->SetDisplayMode((*part_shape_temp)[0],AIS_Shaded);		
			myAISContext->Display((*part_shape_temp)[0],Standard_False);
		}
		myAISContext->UpdateCurrentViewer();
		HideRobot = false;
		PatternSelect_Mode = false;
	}
}

void CKW_Robot_CAMDoc::OnUpdateRobotShowrobot(CCmdUI *pCmdUI)
{
	// TODO: Add your command update UI handler code here
	if ( (HasLoadRobot == false) || (HideRobot == false) ) { pCmdUI->Enable(false); }
}
