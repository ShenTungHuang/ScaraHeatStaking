// KW_Robot_CAMDoc.h : interface of the CKW_Robot_CAMDoc class
//


#pragma once
#include "OCCDoc.h"
#include "Machine_Kinematic.h"


class CKW_Robot_CAMDoc : public OCCDoc
{
protected: // create from serialization only
	CKW_Robot_CAMDoc();
	DECLARE_DYNCREATE(CKW_Robot_CAMDoc)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CKW_Robot_CAMDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()

private:
	Handle(AIS_InteractiveContext) myAISContext;
public:
	Handle(AIS_Shape) Workpicec;
	gp_Ax3 WorkCoordinate;

	bool HideRobot;
	afx_msg void OnRobotHiderobot();
	afx_msg void OnUpdateRobotHiderobot(CCmdUI *pCmdUI);
	afx_msg void OnRobotShowrobot();
	afx_msg void OnUpdateRobotShowrobot(CCmdUI *pCmdUI);

public:
	Machine_Kinematic MK;
};


