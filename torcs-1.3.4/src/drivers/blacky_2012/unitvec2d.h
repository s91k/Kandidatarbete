//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitvec2d.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter für TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Erweiterung des 2D-Vektors
//
// Datei    : unitvec2d.h
// Erstellt : 25.11.2007
// Stand    : 10.06.2012
// Copyright: © 2007-2012 Wolf-Dieter Beelitz
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
// Wer die Dateien verwendet erkennt an, dass für Fehler, Schäden,
// Folgefehler oder Folgeschäden keine Haftung übernommen wird.
//
// Im übrigen gilt für die Nutzung und/oder Weitergabe die
// GNU GPL (General Public License)
// Version 2 oder nach eigener Wahl eine spätere Version.
//--------------------------------------------------------------------------*
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//--------------------------------------------------------------------------*
#ifndef _UNITVEC2D_H_
#define _UNITVEC2D_H_

#include <tmath/v2_t.h>
//#include <v2_t.h>
#include <tgf.h>

#include "unitglobal.h"
//#include "unitcommon.h" NOT ALLOWED HERE!!!

//==========================================================================*
// Deklaration der Klasse TVec2d
//--------------------------------------------------------------------------*
class TVec2d : public v2t<double>
{
  public:
	TVec2d() {}
	TVec2d( const v2t<double>& v ) : v2t<double>(v) {}
	TVec2d( double x, double y ) : v2t<double>(x, y) {};

	TVec2d&	operator=( const v2t<double>& v )
	{
		v2t<double>::operator=(v);
		return *this;
	}
};
//==========================================================================*
#endif // _UNITVEC2D_H_
//--------------------------------------------------------------------------*
// end of file unitvec2d.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
