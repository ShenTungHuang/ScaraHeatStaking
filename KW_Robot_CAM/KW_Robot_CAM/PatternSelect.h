#pragma once
#include <vector>
#include "ISession_Point.h"
#include "ISession_Text.h"

struct PInfo
{
	Handle(AIS_Shape) AIS_Pattern;
	Handle(AIS_Shape) H_Pnt;
	gp_Pnt HPnt;
	Handle(AIS_Shape) L_Pnt;
	gp_Pnt LPnt;
	Handle(AIS_Shape) Path;
	Handle(AIS_Shape) L_Face;

	Handle(ISession_Point) IS_Pnt;
	Handle(ISession_Text) IS_Text;
};

class PatternSelect
{
public:
	PatternSelect(void);
	~PatternSelect(void);

public:
	void Select_Pattern( Handle(AIS_InteractiveContext) myAISContext );
	void GetVertex( TopoDS_Shape inputshape);
	void GetLCircle( gp_Pnt LPnt , Standard_Real  C_diameter , TopoDS_Face *Cir );
	void DrawPath();
	void DisplayPattern();
	std::vector<PInfo> PatternInfo;
	std::vector<Handle(AIS_Shape)> AIS_Path;
	gp_Pnt text_pnt;
	int SelectIndex;
};
