// KW_Robot_CAM.h : main header file for the KW_Robot_CAM application
//
#pragma once
#include "OCCApp.h"
#include "OCCView.h"
#include "KW_Robot_CAMDoc.h"

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols



// CKW_Robot_CAMApp:
// See KW_Robot_CAM.cpp for the implementation of this class
//

class CKW_Robot_CAMApp : public OCCApp
{
public:
	CKW_Robot_CAMApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()

public:
	CFrameWnd*  CreateView3D(CKW_Robot_CAMDoc* pDoc);
private:
	CMultiDocTemplate* pDocTemplateForView3d;

};

extern CKW_Robot_CAMApp theApp;