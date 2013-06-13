//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitvec3d.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0
// Erweiterung des 3D-Vektors
//
// Datei    : unitvec2d.h
// Erstellt : 25.11.2007
// Stand    : 14.12.2008
// Copyright: � 2007-2009 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 2.00.000
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
//--------------------------------------------------------------------------*
// Diese Unit basiert auf dem Roboter mouse_2006
//
//    Copyright: (C) 2006-2007 Tim Foden
//
//--------------------------------------------------------------------------*
// Diese Version wurde mit MS Visual C++ 2005 Express Edition entwickelt.
//--------------------------------------------------------------------------*
// Das Programm wurde unter Windows XP entwickelt und getestet.
// Fehler sind nicht bekannt, dennoch gilt:
// Wer die Dateien verwendet erkennt an, dass f�r Fehler, Sch�den,
// Folgefehler oder Folgesch�den keine Haftung �bernommen wird.
//
// Im �brigen gilt f�r die Nutzung und/oder Weitergabe die
// GNU GPL (General Public License)
// Version 2 oder nach eigener Wahl eine sp�tere Version.
//--------------------------------------------------------------------------*
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//--------------------------------------------------------------------------*
#ifndef _UNITVEC3D_H_
#define _UNITVEC3D_H_

#include <tmath/v3_t.h>
//#include <v3_t.h>
#include <tgf.h>

#include "unitglobal.h"
//#include "unitcommon.h" NOT ALLOWERD HERE!!!
#include "unitvec2d.h"

//==========================================================================*
// Deklaration der Klasse TVec3d
//--------------------------------------------------------------------------*
class TVec3d : public v3t<double>
{
  public:
	TVec3d() {};
	TVec3d(const v3t<double>& V) : v3t<double>(V) {};
	TVec3d(double X, double Y, double Z) : v3t<double>(X, Y, Z) {};
	TVec3d(const t3Dd& V) : v3t<double>(V.x, V.y, V.z) {};

	TVec3d& operator= (const v3t<double>& V)
	{
	  v3t<double>::operator=(V);
	  return *this;
	};

	TVec2d GetXY() const {return TVec2d(x, y);};
	const float Len() const 
	  {return (float) sqrt(this->x * this->x + this->y * this->y);};

	const float Mult(const TVec3d* LHS, const TVec3d* RHS) const
      {return (float) (LHS->x*RHS->x + LHS->y*RHS->y + LHS->z*RHS->z);}

    const float CosPhi(const TVec3d* LHS, const TVec3d* RHS) const
      {return (float) (LHS->Mult(LHS,RHS) / (LHS->Len() * RHS->Len()));};
};
//==========================================================================*
#endif // _UNITVEC3D_H_
//--------------------------------------------------------------------------*
// end of file unitvec3d.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
