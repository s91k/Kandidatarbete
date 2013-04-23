#include "test.h"
#include <portability.h>

#ifdef DMALLOC
#include "dmalloc.h"
#endif

/* function prototypes */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation * situation);
static void drive(int index, tCarElt* car, tSituation *situation);
static void newRace(int index, tCarElt* car, tSituation *situation);
static int  InitFuncPt(int index, void *pt);
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);


static const char* botname[BOTS] = {
	"test 1", "test 2", "test 3", "test 4", "test 5",
	"test 6", "test 7", "test 8", "test 9", "test 10"
};

static const char* botdesc[BOTS] = {
	"test 1", "test 2", "test 3", "test 4", "test 5",
	"test 6", "test 7", "test 8", "test 9", "test 10"
};

/* Module entry point */
extern "C" int test(tModInfo *modInfo)
{
	//char	buffer[BUFSIZE];

	for (int i = 0; i < BOTS; i++) {
		modInfo[i].name = strdup(botname[i]);	/* name of the module (short) */
		modInfo[i].desc = strdup(botdesc[i]);	/* description of the module (can be long) */
		modInfo[i].fctInit = InitFuncPt;		/* init function */
		modInfo[i].gfId    = ROB_IDENT;			/* supported framework version */
		modInfo[i].index   = i+1;
	}
	return 0;
}


/* initialize function (callback) pointers for torcs */
static int InitFuncPt(int index, void *pt)
{
	tRobotItf *itf = (tRobotItf *)pt;

	itf->rbNewTrack = initTrack;	/* init new track */
	itf->rbNewRace  = newRace;		/* init new race */
	itf->rbDrive    = drive;		/* drive during race */
	itf->rbShutdown	= shutdown;		/* called for cleanup per driver */
	itf->rbPitCmd   = pitcmd;		/* pit command */
	itf->index      = index;
	return 0;
}

/* release resources when the module gets unloaded */
static void shutdown(int index) 
{

}


/* initialize track data, called for every selected driver */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation * situation)
{
    *carParmHandle = NULL;

	char buffer[BUFSIZE];
	char* trackname = strrchr(track->filename, '/') + 1;

	snprintf(buffer, BUFSIZE, "drivers/test/%d/%s", index, trackname);
    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);

	if (*carParmHandle == NULL) {
		snprintf(buffer, BUFSIZE, "drivers/test/%d/default.xml", index);
	    *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	}
}


/* initialize driver for the race, called for every selected driver */
static void newRace(int index, tCarElt* car, tSituation *situation)
{

}

/* controls the car */
static void drive(int index, tCarElt* car, tSituation *situation)
{
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	float steer = 0.0f;
	float width = car->_trkPos.toLeft + car->_trkPos.toRight;

	steer = -1.0f * (car->_trkPos.toMiddle / width);

	car->ctrl.gear = 1;
	car->ctrl.accelCmd = 0.3f;
	car->ctrl.brakeCmd = 0.0f;
	car->ctrl.steer = steer;

	printf_s("Width: %f \t Distance to middle: %f \n", width, car->_trkPos.toMiddle);
}

/* pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
	return ROB_PIT_IM; /* return immediately */
}

