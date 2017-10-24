#pragma once

#include <Standard_Macro.hxx>
#include <Handle_Graphic3d_WNTGraphicDevice.hxx>

// OCCApp

class OCCApp : public CWinApp
{
	DECLARE_DYNCREATE(OCCApp)

protected:
	OCCApp();           // 動態建立所使用的保護建構函式
	virtual ~OCCApp();

public:
	Handle_Graphic3d_WNTGraphicDevice GetGraphicDevice() const { return myGraphicDevice; } ;    

protected :
     Handle_Graphic3d_WNTGraphicDevice myGraphicDevice;

public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

protected:
	DECLARE_MESSAGE_MAP()
};


