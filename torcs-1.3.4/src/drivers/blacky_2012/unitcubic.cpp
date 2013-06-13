//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitcubic.cpp
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Polynom dritten Grades
//
// Datei    : unitcubic.cpp
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

#include "unitcubic.h"

//==========================================================================*
// Default constructor
//--------------------------------------------------------------------------*
TCubic::TCubic()
{
}
//==========================================================================*

//==========================================================================*
// Parametric constructor
//--------------------------------------------------------------------------*
TCubic::TCubic(double C0, double C1, double C2, double C3)
{
  Set(C0, C1, C2, C3);
}
//==========================================================================*

//==========================================================================*
// Two point constructor
//--------------------------------------------------------------------------*
TCubic::TCubic
  (double X0, double Y0, double S0,	double X1, double Y1, double S1 )
{
  Set(X0, Y0, S0, X1, Y1, S1);
}
//==========================================================================*

//==========================================================================*
// Destructor
//--------------------------------------------------------------------------*
TCubic::~TCubic()
{
}
//==========================================================================*

//==========================================================================*
// Set coefficients
//--------------------------------------------------------------------------*
void TCubic::Set(double C0, double C1, double C2, double C3)
{
  oCoeffs[0] = C0;
  oCoeffs[1] = C1;
  oCoeffs[2] = C2;
  oCoeffs[3] = C3;
}
//==========================================================================*

//==========================================================================*
// Set coefficients from two points
//--------------------------------------------------------------------------*
void TCubic::Set
  (double X0, double Y0, double S0,	double X1, double Y1, double S1 )
{
	// uses Ferguson's Parametric Cubic Curve, which requires 2
	//	endpoints and 2 slopes.  here we define the endpoints to
	//	to be (x0,y0) & (x1,y1), and the slopes are given by s0 & s1.
	//
	// see: http://graphics.cs.ucdavis.edu/CAGDNotes/
	//			Catmull-Rom-Spline/Catmull-Rom-Spline.html
	//	for the equations used.

	// step 1. convert to parametric form (x in [0..1])
	//	(this basically only effects the slopes).
	double	Dx = X1 - X0;
	double	Dy = Y1 - Y0;
	S0 *= Dx;
	S1 *= Dx;

	// step 2. use Ferguson's method.
	double	C3 = Y0;
	double	C2 = S0;
	double	C1 = 3 * Dy - 2 * S0 - S1;
	double	C0 = -2 * Dy + S0 + S1;

	// step 3. convert back to real-world form (x in [x0..x1]).
	double	X02 = X0  * X0;
	double	X03 = X02 * X0;
	double	Dx2 = Dx  * Dx;
	double	Dx3 = Dx2 * Dx;
	oCoeffs[0] =      C0       / Dx3;
	oCoeffs[1] = -3 * C0 * X0  / Dx3 +     C1       / Dx2;
	oCoeffs[2] =  3 * C0 * X02 / Dx3 - 2 * C1 * X0  / Dx2 + C2      / Dx;
	oCoeffs[3] =     -C0 * X03 / Dx3 +     C1 * X02 / Dx2 - C2 * X0 / Dx + C3;
}
//==========================================================================*

//==========================================================================*
// Get offset
//--------------------------------------------------------------------------*
double TCubic::CalcOffset(double X) const
{
  return ((oCoeffs[0] * X + oCoeffs[1]) * X + oCoeffs[2]) * X + oCoeffs[3];
}
//==========================================================================*

//==========================================================================*
// Get gradient of offset
//--------------------------------------------------------------------------*
double TCubic::CalcGradient(double X) const
{
  return (3 * oCoeffs[0] * X + 2 * oCoeffs[1]) * X + oCoeffs[2];
}
//==========================================================================*

//==========================================================================*
// Get 2nd derivative
//--------------------------------------------------------------------------*
double TCubic::Calc2ndDerivative(double X) const
{
  return 6 * oCoeffs[0] * X + 2 * oCoeffs[1];
}
//==========================================================================*
// end of file unitcubic.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
