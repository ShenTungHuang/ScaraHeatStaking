// KW_Robot_CAMView.cpp : implementation of the CKW_Robot_CAMView class
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"

#include "KW_Robot_CAMDoc.h"
#include "KW_Robot_CAMView.h"

#include "MainFrm.h"
#include "ChildFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CKW_Robot_CAMView

IMPLEMENT_DYNCREATE(CKW_Robot_CAMView, OCCView)

BEGIN_MESSAGE_MAP(CKW_Robot_CAMView, OCCView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
	ON_WM_SIZE()
END_MESSAGE_MAP()

// CKW_Robot_CAMView construction/destruction

CKW_Robot_CAMView::CKW_Robot_CAMView()
{
	// TODO: add construction code here

}

CKW_Robot_CAMView::~CKW_Robot_CAMView()
{
}

BOOL CKW_Robot_CAMView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CKW_Robot_CAMView drawing

void CKW_Robot_CAMView::OnDraw(CDC* pDC)
{
	//CKW_Robot_CAMDoc* pDoc = GetDocument();
	//ASSERT_VALID(pDoc);
	//if (!pDoc)
	//	return;

	// TODO: add draw code for native data here
	OCCView::OnDraw(pDC);
}


// CKW_Robot_CAMView printing

BOOL CKW_Robot_CAMView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CKW_Robot_CAMView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CKW_Robot_CAMView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}


// CKW_Robot_CAMView diagnostics

#ifdef _DEBUG
void CKW_Robot_CAMView::AssertValid() const
{
	CView::AssertValid();
}

void CKW_Robot_CAMView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CKW_Robot_CAMDoc* CKW_Robot_CAMView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CKW_Robot_CAMDoc)));
	return (CKW_Robot_CAMDoc*)m_pDocument;
}
#endif //_DEBUG


// CKW_Robot_CAMView message handlers

void CKW_Robot_CAMView::OnSize(UINT nType, int cx, int cy)
{
	OCCView::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	if (!GetView().IsNull())
		  GetView()->MustBeResized();
}
