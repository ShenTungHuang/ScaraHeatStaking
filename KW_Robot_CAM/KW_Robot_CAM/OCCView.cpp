// OCCView.cpp : ��@��
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "OCCView.h"
#include "OCCApp.h"


#define CASCADESHORTCUTKEY MK_CONTROL
extern bool PatternSelect_Mode;


// OCCView

IMPLEMENT_DYNCREATE(OCCView, CView)

OCCView::OCCView()
{

}

OCCView::~OCCView()
{
}

BEGIN_MESSAGE_MAP(OCCView, CView)
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONUP()
	ON_WM_MBUTTONDOWN()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
	ON_WM_LBUTTONDOWN()
END_MESSAGE_MAP()


// OCCView �yø

void OCCView::OnDraw(CDC* pDC)
{
	//CDocument* pDoc = GetDocument();
	// TODO: �b���[�J�yø�{���X
	myView->Redraw();
}


// OCCView �E�_

#ifdef _DEBUG
void OCCView::AssertValid() const
{
	CView::AssertValid();
}

#ifndef _WIN32_WCE
void OCCView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}
#endif
#endif //_DEBUG

OCCDoc* OCCView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(OCCDoc)));
	return (OCCDoc*)m_pDocument;
}


// OCCView �T���B�z�`��


void OCCView::OnInitialUpdate()
{
	CView::OnInitialUpdate();

	// TODO: �b���[�J�S�w���{���X�M (��) �I�s�����O
	myView = GetDocument()->GetViewer()->CreateView();

    // store for restore state after rotation (witch is in Degenerated mode)
    //myDegenerateModeIsOn = myView->DegenerateModeIsOn();

	Handle(Graphic3d_WNTGraphicDevice) theGraphicDevice = 
		((OCCApp*)AfxGetApp())->GetGraphicDevice();
    
    Handle(WNT_Window) aWNTWindow = new WNT_Window(theGraphicDevice,GetSafeHwnd ());
    myView->SetWindow(aWNTWindow);
    if (!aWNTWindow->IsMapped()) aWNTWindow->Map();

    // store the mode ( nothing , dynamic zooming, dynamic ... )
    myCurrentMode = CurAction3d_Nothing;

	//�e�y�Шt
	myView->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_RED, 0.1,V3d_ZBUFFER);

	//����
	myView->SetAntialiasingOn();


	// MODIF for ususal bug in graphic card : first redraw is not done 
	Standard_Integer w=100, h=100;
	aWNTWindow->Size(w,h);
	::PostMessage ( GetSafeHwnd (), WM_SIZE, SIZE_RESTORED , w + h*65536);
}

void OCCView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��
	//   ============================  LEFT BUTTON =======================
	if ( nFlags & MK_LBUTTON)
    {
		if(nFlags & CASCADESHORTCUTKEY)
		{
			myView->Pan(point.x-myXmax,myYmax-point.y); // Realize the panning
		    myXmax = point.x; myYmax = point.y;	
		}		     
	}//   if ( nFlags & MK_LBUTTON) 		
	//   ============================  RIGHT BUTTON =======================
    if ( nFlags & MK_RBUTTON)
    {
     if ( nFlags & CASCADESHORTCUTKEY ) 
	  {
      	 myView->Rotation(point.x,point.y);
	  }
    }
	else
	//   ============================  NO BUTTON =======================
    {  // No buttons 
		myXmax = point.x; myYmax = point.y;
		//GetDocument()->MoveEvent(point.x,point.y,myView);
		if ( PatternSelect_Mode == true )
		{			
			GetDocument()->MoveEvent(point.x,point.y,myView);
			//GetDocument()->GetAISContext()->MoveTo(point.x , point.y , myView );
		}
    }

	CView::OnMouseMove(nFlags, point);
}

void OCCView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��
	if(nFlags & CASCADESHORTCUTKEY)
	{
			return;
	}
	switch (myCurrentMode)
	{
         case CurAction3d_Nothing :
         if (point.x == myXmin && point.y == myYmin)
         { // no offset between down and up --> selectEvent
            myXmax=point.x;  
            myYmax=point.y;
			GetDocument()->InputEvent(point.x,point.y,myView);            
         } 
		 else
         {            
            myXmax=point.x;  
            myYmax=point.y;		    
         }
         break;
	}

	CView::OnLButtonUp(nFlags, point);
}

void OCCView::OnMButtonDown(UINT nFlags, CPoint point)
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��

	CView::OnMButtonDown(nFlags, point);
}

BOOL OCCView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��
	Standard_Integer Xmax = pt.x+1*zDelta/20;
	Standard_Integer Ymax = pt.y+1*zDelta/20;
	myView->StartZoomAtPoint(pt.x,pt.y);
	myView->ZoomAtPoint(pt.x,pt.y,Xmax,Ymax);

	return CView::OnMouseWheel(nFlags, zDelta, pt);
}

void OCCView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��
	if(nFlags & CASCADESHORTCUTKEY)
	{
		myView->StartRotation(point.x,point.y);  
	}
	else
	{
	    //GetDocument()->Popup(point.x,point.y,myView);	
	}

	CView::OnRButtonDown(nFlags, point);
}

void OCCView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��
	myXmin=point.x;  myYmin=point.y;
    myXmax=point.x;  myYmax=point.y;

	CView::OnLButtonDown(nFlags, point);
}
