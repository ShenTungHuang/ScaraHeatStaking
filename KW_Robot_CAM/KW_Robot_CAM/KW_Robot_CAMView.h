// KW_Robot_CAMView.h : interface of the CKW_Robot_CAMView class
//


#pragma once
#include "OCCView.h"


class CKW_Robot_CAMView : public OCCView
{
protected: // create from serialization only
	CKW_Robot_CAMView();
	DECLARE_DYNCREATE(CKW_Robot_CAMView)

// Attributes
public:
	CKW_Robot_CAMDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// Implementation
public:
	virtual ~CKW_Robot_CAMView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnSize(UINT nType, int cx, int cy);
};

#ifndef _DEBUG  // debug version in KW_Robot_CAMView.cpp
inline CKW_Robot_CAMDoc* CKW_Robot_CAMView::GetDocument() const
   { return reinterpret_cast<CKW_Robot_CAMDoc*>(m_pDocument); }
#endif

