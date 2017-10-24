#pragma once
#include <vector>
#include "StdAfx.h"

enum MACHINE_PART
{
	PART_BASE,
	PART_J1,
	PART_J2,
	PART_J3,
};

class Scara_Part
{
public:
	Scara_Part(void);
	~Scara_Part(void);

private:
	std::vector<Handle(AIS_Shape)> m_BASE;
	std::vector<Handle(AIS_Shape)> m_J1;
	std::vector<Handle(AIS_Shape)> m_J2;
	std::vector<Handle(AIS_Shape)> m_J3;
		
private:
	Handle(AIS_InteractiveContext) myAISContext;

public:
	std::vector<Handle(AIS_Shape)> *MK_Get_Machine_Part_Shape(MACHINE_PART part);
};
