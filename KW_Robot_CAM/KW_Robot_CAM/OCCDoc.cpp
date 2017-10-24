// OCCDoc.cpp : 實作檔
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "OCCDoc.h"
#include "OCCApp.h"

extern bool PatternSelect_Mode;


// OCCDoc

//IMPLEMENT_DYNCREATE(OCCDoc, CDocument)

OCCDoc::OCCDoc()
{
}

void OCCDoc::MoveEvent(const Standard_Integer  x , const Standard_Integer  y , const Handle(V3d_View)& aView   ) 
{ myAISContext->MoveTo(x,y,aView); }

BOOL OCCDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

    // TODO: 在此加入一次建構程式碼
	myCView = (CView *) -1;

	Handle(Graphic3d_WNTGraphicDevice) theGraphicDevice = 
		((OCCApp*)AfxGetApp())->GetGraphicDevice();

    TCollection_ExtendedString a3DName("Visu3D");
	myViewer = new V3d_Viewer(theGraphicDevice,a3DName.ToExtString(),"", 1000.0, 
                              V3d_XposYnegZpos, Quantity_NOC_GRAY30,
                              V3d_ZBUFFER,V3d_GOURAUD,V3d_WAIT, 
                              Standard_True, Standard_False);

	myViewer->SetDefaultLights();
	myViewer->SetLightOn();

	Handle(V3d_Light) myCurrent_AmbientLight=new V3d_AmbientLight(myViewer, Quantity_NOC_GRAY);
    myViewer->SetLightOn(myCurrent_AmbientLight) ;
	
 

	myCViewer = new V3d_Viewer(theGraphicDevice,a3DName.ToExtString(),"", 1000.0, 
                              V3d_XposYnegZpos, Quantity_NOC_GRAY30,
                              V3d_ZBUFFER,V3d_GOURAUD,V3d_WAIT, 
                              Standard_True, Standard_False);

	myCViewer->SetDefaultLights();
	myCViewer->SetLightOn();

	myAISContext =new AIS_InteractiveContext(myViewer,myCViewer);


	return TRUE;
}

OCCDoc::~OCCDoc()
{
}


BEGIN_MESSAGE_MAP(OCCDoc, CDocument)
END_MESSAGE_MAP()


// OCCDoc 診斷

#ifdef _DEBUG
void OCCDoc::AssertValid() const
{
	CDocument::AssertValid();
}

#ifndef _WIN32_WCE
void OCCDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif
#endif //_DEBUG

#ifndef _WIN32_WCE
// OCCDoc 序列化

void OCCDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: 在此加入儲存程式碼
	}
	else
	{
		// TODO: 在此加入載入程式碼
	}
}
#endif


// OCCDoc 命令
void OCCDoc::InputEvent(const Standard_Integer  x , const Standard_Integer  y , const Handle(V3d_View)& aView ) 
{
	if ( PatternSelect_Mode == true )
	{
		myAISContext->Select(); 
		myAISContext->InitSelected();
		if (myAISContext->MoreSelected())
		{
			TopoDS_Shape selectshape=myAISContext->SelectedShape();
			TopAbs_ShapeEnum shape_type = selectshape.ShapeType();

			// add by STH for select tube in view
			if ( shape_type == TopAbs_FACE ) { PS.Select_Pattern(myAISContext); }
		}
	}
}
