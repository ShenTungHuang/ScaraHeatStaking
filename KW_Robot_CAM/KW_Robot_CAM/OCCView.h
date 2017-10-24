#pragma once
#include "OCCDoc.h"

enum CurrentAction3d { 
  CurAction3d_Nothing,
  CurAction3d_DynamicZooming,
  CurAction3d_WindowZooming,
  CurAction3d_DynamicPanning,
  CurAction3d_GlobalPanning,
  CurAction3d_DynamicRotation
};


// OCCView 檢視

class OCCView : public CView
{
	DECLARE_DYNCREATE(OCCView)

protected:
	OCCView();           // 動態建立所使用的保護建構函式
	virtual ~OCCView();

public:
	virtual void OnDraw(CDC* pDC);      // 覆寫以描繪此檢視
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif

private:
	Handle_V3d_View		 myView;
	CurrentAction3d		 myCurrentMode;
public:
	Handle(V3d_View)&    GetView() { return myView;};
public:
	OCCDoc* GetDocument();
private:
	Standard_Integer     myXmin;
    Standard_Integer     myYmin;  
    Standard_Integer     myXmax;
    Standard_Integer     myYmax;

protected:
	DECLARE_MESSAGE_MAP()
public:
	virtual void OnInitialUpdate();
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
};


