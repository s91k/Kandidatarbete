/***************************************************************************

    file                 : cubiccurve.cpp
    created              : Mon Sep 07 20:00:00 UTC 2009
    copyright            : (C) 2009 Daniel Schellhammer

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include "cubiccurve.h"


CubicCurve::CubicCurve()
{
}

// Parametric constructor
CubicCurve::CubicCurve(double C0, double C1, double C2, double C3)
{
  Set(C0, C1, C2, C3);
}

// Two point constructor
CubicCurve::CubicCurve(double X0, double Y0, double S0, double X1, double Y1, double S1)
{
  Set(X0, Y0, S0, X1, Y1, S1);
}

CubicCurve::~CubicCurve()
{
}

// Set coefficients
void CubicCurve::Set(double C0, double C1, double C2, double C3)
{
  mCoeffs[0] = C0;
  mCoeffs[1] = C1;
  mCoeffs[2] = C2;
  mCoeffs[3] = C3;
}

// Set coefficients from two points
void CubicCurve::Set(double X0, double Y0, double S0, double X1, double Y1, double S1)
{
  // uses Ferguson's Parametric Cubic Curve, which requires 2
  //  endpoints and 2 slopes.  here we define the endpoints to
  //  to be (x0,y0) & (x1,y1), and the slopes are given by s0 & s1.
  //
  // see: http://graphics.cs.ucdavis.edu/CAGDNotes/
  //      Catmull-Rom-Spline/Catmull-Rom-Spline.html
  //  for the equations used.

  // step 1. convert to parametric form (x in [0..1])
  //  (this basically only effects the slopes).
  double  Dx = X1 - X0;
  double  Dy = Y1 - Y0;
  S0 *= Dx;
  S1 *= Dx;

  // step 2. use Ferguson's method.
  double  C3 = Y0;
  double  C2 = S0;
  double  C1 = 3 * Dy - 2 * S0 - S1;
  double  C0 = -2 * Dy + S0 + S1;

  // step 3. convert back to real-world form (x in [x0..x1]).
  double  X02 = X0  * X0;
  double  X03 = X02 * X0;
  double  Dx2 = Dx  * Dx;
  double  Dx3 = Dx2 * Dx;
  mCoeffs[0] =      C0       / Dx3;
  mCoeffs[1] = -3 * C0 * X0  / Dx3 +     C1       / Dx2;
  mCoeffs[2] =  3 * C0 * X02 / Dx3 - 2 * C1 * X0  / Dx2 + C2      / Dx;
  mCoeffs[3] =     -C0 * X03 / Dx3 +     C1 * X02 / Dx2 - C2 * X0 / Dx + C3;
}

// Get offset
double CubicCurve::CalcOffset(double X)
{
  return ((mCoeffs[0] * X + mCoeffs[1]) * X + mCoeffs[2]) * X + mCoeffs[3];
}

// Get gradient of offset
double CubicCurve::CalcGradient(double X)
{
  return (3 * mCoeffs[0] * X + 2 * mCoeffs[1]) * X + mCoeffs[2];
}

// Get 2nd derivative
double CubicCurve::Calc2ndDerivative(double X)
{
  return 6 * mCoeffs[0] * X + 2 * mCoeffs[1];
}
