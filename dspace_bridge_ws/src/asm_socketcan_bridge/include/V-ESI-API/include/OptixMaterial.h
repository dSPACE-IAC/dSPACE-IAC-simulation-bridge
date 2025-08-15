#pragma once

typedef int materialId_t; //using int for gpu performance reasons (even though mat db uses uint16_t)

enum SpecialMaterialIds : materialId_t
{
	NO_MATERIAL = 0,		// objects with no material
	DISCARD_MATERIAL = 1	// used for transparent object parts
};