#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>
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


// Comment out PLUGIN_DEBUG entirely for a production build - Logfile/LogDebug then compile away to
// nothing (see LogDebug in AstroTrac.cpp). When defined, controls how much gets logged:
//   0: Open-loop-move tracing (startOpenLoopMove/stopOpenLoopMove) - relevant to guiding performance.
//   1: Notable/unexpected events worth a heads-up even outside active debugging - command outcome
//      summaries (succeeded after N retries / FAILED), a malformed device error reply, an
//      unexpectedly-mismatched extra reply, a response missing its closing '>', a read that timed
//      out waiting for bytes (with elapsed time), fewer bytes read than bytesWaitingRx reported, or
//      a writeFile error sending a command.
//   2: Full trace of the send-command machinery (AstroTracSendCommand/AstroTracSendCommandInnerLoop/
//      readResponse) - purge/resend decisions, stale/duplicate-reply draining, each batch of bytes
//      read in readResponse, how long WriteCommand's writeFile+flushTx took to send each command,
//      and every first-try-success command's round-trip time (for building a timing distribution -
//      the level 1 "succeeded after N retries" line only covers retried ones). Only useful when
//      actively debugging the comms protocol itself.
//   3: Everything else - connection lifecycle, coordinate/math tracing, slew lifecycle.
// #define PLUGIN_DEBUG 2
#define DRIVER_VERSION 1.7

// Changelog:
// Version  1.0: Initial release
//          1.1: Added pulseguide
//          1.2: Added setting to control guide rate and how much mount will track beyond the pole.
//          1.3: Fixed bug where command send ok but no response caused new command to be sent and two responses given, causing errors when parsing the next response.
//          1.4: Fixed bug in pulseguiding - selected rated was index+1.
//          1.5: Replaced sprintf with snprintf and added code to track timing of open loop slews and to send commands to Astrotrac (about 0.015s per axis). Also defined number of slew rates dynamically by reading from size of m_dvSlewRates.
//          1.6: Fixed command/response desync: always resend on retry instead of waiting for two failed reads, validate replies against the command sent, and drain stale/duplicate replies so command/response pairs stay in sync.
//          1.7: Added horizon limit setting, and send both it and the meridian/latitude settings to the mount
//               firmware (>= 2.35) as a last-resort backstop, padded with FIRMWARE_SAFETY_MARGIN_DEG so this
//               driver's own meridian/horizon checks in raDec() still take precedence.


#define AT_SIDEREAL_SPEED 15.04106864 // Arc sec/s required to maintain siderial tracking

// Firmware version (as reported by 'zv?') at which the 'lt'/'lh'/'la' safety-limit commands were introduced.
#define FIRMWARE_MIN_VER_SAFETY_LIMITS 2.35

enum AstroTracErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, PLUGIN_ERROR};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 300
#define PLUGIN_LOG_BUFFER_SIZE 256
#define ERR_PARSE   1

#define MAXSENDTRIES 3  // Maximum number of attempts to send a mesage to the mount
#define MAX_STALE_RESPONSE_TRIES 2  // Maximum number of stray/stale replies to discard while looking for the real response to a command
// 3ms was picked from measurement: at 25ms polling, replies clustered almost entirely in the
// first one or two poll windows; dropping to 3ms revealed the true round-trip is ~2-9ms for the
// overwhelming majority of commands, with no sign of a coarser scheduler floor forcing it back up.
#define READ_POLL_INTERVAL_MS 3  // How often AstroTracreadResponse re-checks bytesWaitingRx while waiting for a reply


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

    int getNumberGuideRates(void) {return m_iNumberGuideRates;};
    
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

    // Sends the firmware-level meridian/horizon safety backstop ('lt'/'lh'/'la'). No-ops (returns
    // PLUGIN_OK without sending anything) if the connected firmware predates FIRMWARE_MIN_VER_SAFETY_LIMITS -
    // older firmware doesn't understand these commands. dMeridianLimitDeg/dHorizonLimitDeg are sent as-is;
    // any margin over this driver's own limits is the caller's responsibility (see x2mount.h).
    int sendSafetyLimits(double dMeridianLimitDeg, double dHorizonLimitDeg, double dLatitudeDeg);

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
    double m_dAslewRA = 3600.0;    // RA/HA axis slew acceleration - arcsec/sec/sec - read from mount at connect, never set by us
    double m_dAslewDEC = 3600.0;   // DEC axis slew acceleration - arcsec/sec/sec - read from mount at connect, never set by us
                                    // Currently unused: DEC needs no sidereal lead-compensation (see startSlewTo), so nothing consumes this yet
    double m_dSlewOffset = 0.0;  // How wrong was last slew? Store and attempt to correct in next slew
    double  m_dGotoRATarget;     // Current Target RA - to allow slew offset to be calculated
    
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

    // limits don't change mid-course so we cache them
    bool    m_bIsBTP = false;
    bool    m_bLimitCached;
    double  m_dHoursEast;
    double  m_dHoursWest;
    
    int     AstroTracSendCommand(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen);
    int     AstroTracSendCommandInnerLoop(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen, bool bIsRetry);
    int     AstroTracreadResponse(unsigned char *pszRespBuffer, unsigned int bufferLen);
    bool    responseMatchesCommand(const char *pszCmd, const unsigned char *pszResp);

    // Helpers used by AstroTracSendCommandInnerLoop, broken out for readability - see definitions
    // for what each covers.
    bool    PreparePortForSend(const char *pszCmd, bool bIsRetry);
    int     WriteCommand(const char *pszCmd);
    int     DiscardStaleReplies(const char *pszCmd, unsigned char *pszResp, unsigned int nBufLen);
    void    DrainDuplicateReplies(const char *pszCmd, unsigned char *pszResp, unsigned int nBufLen, bool bIsRetry);
    void    LogDebug(int nLevel, const char *pszFormat, ...);

    
    // Functions to encapsulate transform from drive 1 and drive 2 position angles to positions on the sky
    void EncoderValuesfromHAanDEC(double dHa, double dDec, double &RAEncoder, double &DEEncoder, bool bUseBTP);
    void HAandDECfromEncoderValues(double RAEncoder, double DEEncoder, double &dHa, double &dDec);
    
    // Function to calculate slew time
    double slewTime(double dDist, double dAccel);
    
    std::vector<std::string>    m_svSlewRateNames = {"0.1x", "0.25x", "0.5x", "1x (siderial)", "2x", "4x", "8x", "16x", "32x", "64x", "128x", "256x", "512x"};
    std::vector<double>    m_dvSlewRates = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0};
    
    int const m_iNumberGuideRates = 4;
    
    struct timespec  m_OpenLoopStartTimeRA;
    struct timespec  m_OpenLoopStartTimeDEC;
    bool    m_bOpenLoopRA = false;
    bool    m_bOpenLoopDEC = false;

    
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif
	
};


