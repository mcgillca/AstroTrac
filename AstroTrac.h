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

#include "StopWatch.h"


#define PLUGIN_DEBUG 2   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug
#define DRIVER_VERSION 1.00

enum AstroTracErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, PLUGIN_ERROR};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define PLUGIN_LOG_BUFFER_SIZE 256
#define ERR_PARSE   1

#define PLUGIN_NB_SLEW_SPEEDS 4


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

    int setSiteData(double dLongitude, double dLatitute, double dTimeZone);
    int getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone);

    int getFirmwareVersion(std::string &sFirmware);

    void    setMountMode(MountTypeInterface::Type mountType);
    MountTypeInterface::Type mountType();

    int getRaAndDec(double &dRa, double &dDec);
    int syncTo(double dRa, double dDec);
    int isAligned(bool &bAligned);
    
    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr);
    int getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr);

    int startSlewTo(double dRa, double dDec);
    int isSlewToComplete(bool &bComplete);

    int startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
    int stopOpenLoopMove();
    int getNbSlewRates();
    int getRateName(int nZeroBasedIndex, std::string &sOut);
    
    int setMaxSpeed(const int nSpeed);
    int getMaxSpeed(int &nSpeed);

    int setFindSpeed(const int nSpeed);
    int getFindSpeed(int &nSpeed);

    int setCenteringSpeed(const int nSpeed);
    int getCenteringSpeed(int &nSpeed);

    int setGuideSpeed(const int nSpeed);
    int getGuideSpeed(int &nSpeed);

    int gotoPark(double dRa, double dDEc);
    int markParkPosition();
    int getAtPark(bool &bParked);
    int unPark();


    int getLimits(double &dHoursEast, double &dHoursWest);

    int Abort();

    int getLocalTimeFormat(bool &b24h);
    int getDateFormat(bool &bDdMmYy);
    int getStandardTime(std::string &sTime);
    int getStandardDate(std::string &sDate);
    int syncTime();
    int syncDate();

private:

    SerXInterface                       *m_pSerx;
    LoggerInterface                     *m_pLogger;
    TheSkyXFacadeForDriversInterface    *m_pTsx;
    SleeperInterface                    *m_pSleeper;

    bool    m_bDebugLog;
    char    m_szLogBuffer[PLUGIN_LOG_BUFFER_SIZE];

	bool    m_bIsConnected;                               // Connected to the mount?
    std::string m_sFirmwareVersion;

    MountTypeInterface::Type    m_mountType;
    
    std::string     m_sTime;
    std::string     m_sDate;

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;
	
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

    // limits don't change mid-course so we cache them
    bool    m_bLimitCached;
    double  m_dHoursEast;
    double  m_dHoursWest;
    
    int     AstroTracSendCommand(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen);
    int     AstroTracreadResponse(unsigned char *pszRespBuffer, unsigned int bufferLen);

    int     setSiteLongitude(const char *szLongitude);
    int     setSiteLatitude(const char *szLatitude);
    int     setSiteTimezone(const char *szTimezone);

    int     getSiteLongitude(std::string &sLongitude);
    int     getSiteLatitude(std::string &sLatitude);
    int     getSiteTZ(std::string &sTimeZone);

    int     setTarget(double dRa, double dDec);
    int     slewTargetRA_DecEpochNow();

    int     getSoftLimitEastAngle(double &dAngle);
    int     getSoftLimitWestAngle(double &dAngle);

    void    convertDecDegToDDMMSS(double dDeg, char *szResult, char &cSign, unsigned int size);
    int     convertDDMMSSToDecDeg(const char *szStrDeg, double &dDecDeg);
    
    void    convertRaToHHMMSSt(double dRa, char *szResult, unsigned int size);
    int     convertHHMMSStToRa(const char *szStrRa, double &dRa);

    int     parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator);

    std::vector<std::string>    m_svSlewRateNames = {"Guide", "Centering", "Find", "Max"};
    CStopWatch      timer;

    
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif
	
};


