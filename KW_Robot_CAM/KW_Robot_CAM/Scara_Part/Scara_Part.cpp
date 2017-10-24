#include "StdAfx.h"
#include "Scara_Part.h"

Scara_Part::Scara_Part(void)
{
}

Scara_Part::~Scara_Part(void)
{
}

std::vector<Handle(AIS_Shape)> *Scara_Part::MK_Get_Machine_Part_Shape(MACHINE_PART part)
{
	// what this for?
	switch(part)
	{
	    case PART_BASE:
		    return &m_BASE;
		    break;
		case PART_J1:
			return &m_J1;
			break;
		case PART_J2:
			return &m_J2;
			break;
		case PART_J3:
			return &m_J3;
			break;
	}

	return NULL;
}
