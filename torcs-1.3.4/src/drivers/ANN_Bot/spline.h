/***************************************************************************

    file                 : spline.h
    created              : Wed Mai 14 19:53:00 CET 2003
    copyright            : (C) 2003-2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: spline.h,v 1.3 2004/08/10 20:11:49 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _SPLINE_H_
#define _SPLINE_H_

class SplinePoint {
	public:
		float x;	// x coordinate.
		float y;	// y coordinate.
		float s;	// slope.
};


class Spline {
	public:
		Spline(int dim, SplinePoint *s);

		float evaluate(float z);

	private:
		SplinePoint *s;
		int dim;
};

// Structs to arrange data cache-friendly.
typedef struct {
	double a, b, c, d, h;
} SplineEquationData;

typedef struct {
	double a, b, c, d, h, x1, x2;
} SplineEquationData2;


/* solves tridiagonal matrix in linear time O(n) with Given's Rotations */
extern void tridiagonal(int dim, SplineEquationData *tmp, double *x);


/* solving tridiagonal nxn matrix for two vectors with Givens-Rotations in linear time O(n) */
extern void tridiagonal2(int dim, SplineEquationData2 *tmp);


/* compute the slopes of the spline points with periodic constraints */
extern void slopesp(int dim, const double *const x, const double *const y, double *const ys);


/* compute the slopes of the spline points with natural constraints */
extern void slopesn(int dim, const double *const x, const double *const y, double *const ys);


/* compute the slopes for 2-dim curve, sums euclidian distances as parameter, periodic */
extern void parametricslopesp(
	int dim,
	const double *const x,
	const double *const y,
	double *const xs,
	double *const ys,
	double *const s
);


/* compute the slopes for 2-dim curve, sums euclidian distances as parameter, natural */
extern void parametricslopesn(
	int dim,
	const double *const x,
	const double *const y,
	double *const xs,
	double *const ys,
	double *const s
);


/* compute the y value for a given z */
extern double spline(
	int dim,
	double z,
	const double *const x,
	const double *const y,
	const double *const ys
);


#endif // _SPLINE_H_

