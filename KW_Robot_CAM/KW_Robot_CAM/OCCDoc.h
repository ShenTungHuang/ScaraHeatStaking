#pragma once
#include <Standard_Macro.hxx>
#include "PatternSelect.h"

// OCCDoc 文件

class OCCDoc : public CDocument
{
	//DECLARE_DYNCREATE(OCCDoc)

public:
	OCCDoc();
	virtual ~OCCDoc();
#ifndef _WIN32_WCE
	virtual void Serialize(CArchive& ar);   // 文件 I/O 的覆寫
#endif
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif

private:
	Handle(AIS_InteractiveContext) myAISContext;    
	Handle(V3d_Viewer) myViewer;
	Handle(V3d_Viewer) myCViewer;	
public :	
	Handle(AIS_InteractiveContext)& GetAISContext(){ return myAISContext; };
	Handle_V3d_Viewer GetViewer()  { return myViewer; };
	Handle_V3d_Viewer GetCViewer()  { return myCViewer; };
	CView *myCView;
	virtual void MoveEvent(const Standard_Integer x,
                           const Standard_Integer y,
                           const Handle(V3d_View)& aView); 
	virtual void InputEvent(const Standard_Integer  x,
	    			        const Standard_Integer  y,
                            const Handle(V3d_View)& aView);  
protected:
	virtual BOOL OnNewDocument();

public:
	PatternSelect PS;

	DECLARE_MESSAGE_MAP()
};
