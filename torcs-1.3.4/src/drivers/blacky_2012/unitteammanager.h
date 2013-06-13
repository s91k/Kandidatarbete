//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
// unitteammanager.h
//--------------------------------------------------------------------------*
// TORCS: "The Open Racing Car Simulator"
// Roboter f�r TORCS-Version 1.3.0/1.3.1/1.3.2/1.3.3/1.3.4
// Teammanager
//
// Datei    : unitteammanager.h
// Erstellt : 17.11.2007
// Stand    : 10.06.2012
// Copyright: � 2007-2012 Wolf-Dieter Beelitz
// eMail    : wdb@wdbee.de
// Version  : 3.04.000 (Championship 2012 Alpine-1)
//--------------------------------------------------------------------------*
// Ein erweiterter TORCS-Roboters
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
#ifndef _UNITTEAMMANAGER_H_
#define _UNITTEAMMANAGER_H_

#include <car.h>
#include "unitglobal.h"

//==========================================================================*
// Deklaration der Klasse TTeamManager
//--------------------------------------------------------------------------*
class TTeamManager
{
  public:
	struct TTeammate
	{
		int	Index;		                         // Index of car in race.
		TTeammate* Next;	                     // The next team member.
		CarElt*	Car;		                     // The car of this team member.
	};

	class TTeam
	{
	  public:
		const char*	TeamName;	                 // Name of team.
		int	PitState;	                         // Request for shared pit.
		TTeammate* Member;                       // The next team member.
		int FuelForLaps[MAX_NBBOTS];             // Fuel for laps
		CarElt* Cars[MAX_NBBOTS];                // Cars
		int Count;                               // Nbr of Teammates

	  TTeam():                                   // Default constructor
		PitState(PIT_IS_FREE),                   // Pit is free
		Member(NULL),                            // No members yet
		Count(0)                                 // Nbr of members
	  {
		TeamName = "Empty";	                     // Name of team
		for (int I = 0; I < MAX_NBBOTS; I++)     // Loop over all
		{                                        //   possible members
		  FuelForLaps[I] = 99;                   //   Fuel for laps
		  Cars[I] = NULL;                        //   No Cars
		}
	  }

	  int GetMinLaps(CarElt* oCar)               // Get Nbr of laps, all
	  {                                          //  teammates has fuel for
		int MinLaps = 99;                        // Assume much
		for (int I = 0; I < MAX_NBBOTS; I++)     // Loop over all possible
		  if (Cars[I] != oCar)                   // entries!
			MinLaps = MIN(MinLaps,FuelForLaps[I]); // If not self, calculate

		return MinLaps;
	  }
	};

  public:
	TTeamManager();                              // Default constructor
	~TTeamManager();                             // Destructor

	void Clear();                                // Clear all data
	TTeam* Add(CarElt* oCar);                    // Add a car to its team
	TTeam* Team(int Index);                      // Get a team

	bool IsTeamMate                              // Check to be a teammate
	  (const CarElt* Car0, const CarElt* Car1) const;

  private:
	int	oCount;                                  // Nbr of Teams
	TTeam** oTeams;                              // Pointer to Team array
};
//==========================================================================*
#endif // _UNITTEAMMANAGER_H_
//--------------------------------------------------------------------------*
// end of file unitteammanager.h
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
