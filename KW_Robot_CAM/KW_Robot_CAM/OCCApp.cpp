// OCCApp.cpp : 實作檔
//

#include "stdafx.h"
#include "KW_Robot_CAM.h"
#include "OCCApp.h"



// OCCApp

IMPLEMENT_DYNCREATE(OCCApp, CWinApp)

OCCApp::OCCApp()
{
	try
	{
		myGraphicDevice = new Graphic3d_WNTGraphicDevice();
	}
	catch(Standard_Failure)
	{
		AfxMessageBox(_T("Fatal error during graphic initialization"),MB_ICONSTOP);
		ExitProcess(1);
	}
}

OCCApp::~OCCApp()
{
}

BOOL OCCApp::InitInstance()
{
	// TODO:  在此執行任何個別執行緒的初始設定
	return TRUE;
}

int OCCApp::ExitInstance()
{
	// TODO:  在此執行任何個別執行緒的初始設定
	return CWinApp::ExitInstance();
}

BEGIN_MESSAGE_MAP(OCCApp, CWinApp)
END_MESSAGE_MAP()


// OCCApp 訊息處理常式
