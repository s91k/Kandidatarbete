/***************************************************************************

    file                 : cubiccurve.h
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

#ifndef _CUBICCURVE_H_
#define _CUBICCURVE_H_


class CubicCurve
{
  public:
  CubicCurve();
  CubicCurve(double C0, double C1, double C2, double C3);  // Parametric constructor
  CubicCurve(double X0, double Y0, double S0,  double X1, double Y1, double S1); // Two point constructor
  ~CubicCurve();

  void Set(double C0, double C1, double C2, double C3); // Set coefficients 
  void Set(double X0, double Y0, double S0, double X1, double Y1, double S1); // Set two points 
  double CalcOffset(double X);        // Get offset
  double CalcGradient(double X);      // Get gradient
  double Calc2ndDerivative(double X); // Get 2nd derivative

  public:
  double mCoeffs[4]; // Coefficients
};


#endif // _CUBICCURVE_H_
