#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"

// #include "StopWatch.h"


// #define PLUGIN_DEBUG 1   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug
#define DRIVER_VERSION 0.80

#define AT_SIDEREAL_SPEED 15.04106864 // Arc sec/s required to maintain siderial tracking


enum AstroTracErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, PLUGIN_ERROR};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define PLUGIN_LOG_BUFFER_SIZE 256
#define ERR_PARSE   1

#define PLUGIN_NB_SLEW_SPEEDS 11

#define MAXSENDTRIES 3  // Maximum number of attempts to send a mesage to the mount


// Define Class for Astrometric Instruments AstroTrac controller.
class AstroTrac
{
public:
	AstroTrac();
	~AstroTrac();
	
	int Connect(char *pszPort);
	int Disconnect();
	bool isConnected() const { return m_bIsConnected; }

    void setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};
    void setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper;};

    int getFirmwareVersion(std::string &sFirmware);

    void    setMountMode(MountTypeInterface::Type mountType);
    MountTypeInterface::Type mountType();

    int getHaAndDec(double &dHa, double &dDec);
    int syncTo(double dHa, double dDec);
    int isAligned(bool &bAligned);
    
    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerSec, double dTrackDecArcSecPerSec);
    int getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerSec, double &dTrackDecArcSecPerSec);

    int startSlewTo(double dHa, double dDec, double dRa);
    int isSlewToComplete(bool &bComplete);
    int endSlewTo();

    int startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
    int stopOpenLoopMove();
    int getNbSlewRates();
    int getRateName(int nZeroBasedIndex, std::string &sOut);

    int gotoPark(double dHa, double dDEc);
    int GetIsParkingComplete(bool &bComplete);
    bool GetIsParked() const { return m_bisParked; }
    int unPark() {m_bisParked = false; return PLUGIN_OK; };


    bool GetIsBeyondThePole() const { return m_bIsBTP; }

    int Abort();

private:

    SerXInterface                       *m_pSerx;
    LoggerInterface                     *m_pLogger;
    TheSkyXFacadeForDriversInterface    *m_pTsx;
    SleeperInterface                    *m_pSleeper;

    bool    m_bDebugLog;
    char    m_szLogBuffer[PLUGIN_LOG_BUFFER_SIZE];

	bool    m_bIsConnected;                               // Connected to the mount?
    std::string m_sFirmwareVersion;
    
    bool    m_bNorthernHemisphere;

    MountTypeInterface::Type    m_mountType;

    
    // Latest RA and DEC encoder positions
    double m_dHAEncoder = 0.0;
    double m_dDecEncoder = 0.0;
	
    // Save the state of last tracking request
    bool m_bTracking = true;
    double m_dRATrackingRate = 0.0;
    double m_dDETrackingRate = 0.0;
    
    // Parking variables
    bool m_bisParked = false;
    bool m_bParkingInProgress = false;

    // Flag to tell slewing that aborted
    bool m_bSlewingAborted = false;
    
    // Variables to calculate slew time and improve Slew
    double m_dVSlewMax = 3 * 3600.0; // Maximum slew velocity - 3 deg/sec in arcsec/sec
    double m_dAslew = 3600.0;    // Slew Acceleration - arcsec/sec
    double m_dSlewOffset = 0.0;  // How wrong was last slew? Store and attempt to correct in next slew
    double  m_dGotoRATarget;     // Current Target RA - to allow slew offset to be calculated
    
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

    // limits don't change mid-course so we cache them
    bool    m_bIsBTP = false;
    bool    m_bLimitCached;
    double  m_dHoursEast;
    double  m_dHoursWest;
    
    int     AstroTracSendCommand(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen);
    int     AstroTracSendCommandInnerLoop(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen);
    int     AstroTracreadResponse(unsigned char *pszRespBuffer, unsigned int bufferLen);

    
    // Functions to encapsulate transform from drive 1 and drive 2 position angles to positions on the sky
    void EncoderValuesfromHAanDEC(double dHa, double dDec, double &RAEncoder, double &DEEncoder, bool bUseBTP);
    void HAandDECfromEncoderValues(double RAEncoder, double DEEncoder, double &dHa, double &dDec);
    
    // Function to calculate slew time
    double slewTime(double dDist);
    
    std::vector<std::string>    m_svSlewRateNames = {"0.5x", "1x (siderial)", "2x", "4x", "8x", "16x", "32x", "64x", "128x", "256x", "512x"};
    std::vector<double>    m_dvSlewRates = {0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0};
    
    // CStopWatch      timer;

    
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif
	
};


