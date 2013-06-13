//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitcubicspline.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Kubischer Spline
//
// Datei    : unitcubicspline.cpp
// Erstellt : 25.11.2007
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
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

#include "unitglobal.h"
#include "unitcommon.h"

#include "unitcubicspline.h"

//==========================================================================*
// Constructor
//--------------------------------------------------------------------------*
TCubicSpline::TCubicSpline
  (int Count, const double* X, const double* Y, const double* S)
{
  oCount = Count;
  oSegs = new double[oCount];
  oCubics = new TCubic[oCount - 1];

  for (int I = 0; I < oCount; I++)
  {
 	oSegs[I] = X[I];
	if (I + 1 < oCount)
	oCubics[I].Set( X[I], Y[I], S[I], X[I+1], Y[I+1], S[I+1]);
  }
}
//==========================================================================*

//==========================================================================*
// Destructor
//--------------------------------------------------------------------------*
TCubicSpline::~TCubicSpline()
{
  delete [] oSegs;
  delete [] oCubics;
}
//==========================================================================*

//==========================================================================*
// Get offset
//--------------------------------------------------------------------------*
double TCubicSpline::CalcOffset(double X) const
{
  int I = FindSeg(X);
  return oCubics[I].CalcOffset(X);
}
//==========================================================================*

//==========================================================================*
// Get gradient
//--------------------------------------------------------------------------*
double TCubicSpline::CalcGradient(double X) const
{
  int I = FindSeg(X);
  return oCubics[I].CalcGradient(X);
}
//==========================================================================*

//==========================================================================*
// Is X valid in spline
//--------------------------------------------------------------------------*
bool TCubicSpline::IsValidX(double X) const
{
  return X >= oSegs[0] && X <= oSegs[oCount - 1];
}
//==========================================================================*

//==========================================================================*
// Find segment to X
//--------------------------------------------------------------------------*
int	TCubicSpline::FindSeg(double X) const
{
  // binary chop search for interval.
  int Lo = 0;
  int Hi = oCount;

  while (Lo + 1 < Hi)
  {
	int Mid = (Lo + Hi) / 2;
	if (X >= oSegs[Mid])
	  Lo = Mid;
	else
 	  Hi = Mid;
  }

  return Lo;
}
//==========================================================================*
// end of file unitcubicspline.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
