#pragma once
#include <string.h>
#include <math.h>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serialportparams2interface.h"
#include "../../licensedinterfaces/modalsettingsdialoginterface.h"
#include "../../licensedinterfaces/x2guiinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/slewtointerface.h"
#include "../../licensedinterfaces/mount/syncmountinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"
#include "../../licensedinterfaces/mount/openloopmoveinterface.h"
#include "../../licensedinterfaces/mount/needsrefractioninterface.h"
#include "../../licensedinterfaces/mount/trackingratesinterface.h"
#include "../../licensedinterfaces/parkinterface.h"
#include "../../licensedinterfaces/unparkinterface.h"
#include "../../licensedinterfaces/driverslewstoparkpositioninterface.h"
#include "../../licensedinterfaces/mount/pulseguideinterface2.h"

// Include files for AstroTrac mount
#include "AstroTrac.h"


#define PARENT_KEY			"AstroTracMount"
#define CHILD_KEY_PORT_NAME "PortName"
#define CHILD_KEY_GUIDERATE "GuideRate"
#define CHILD_KEY_HOURS_PAST_MERIDIAN "HPMeridian"
#define CHILD_KEY_HORIZON_LIMIT "HorizonLimit"

#define MAX_PORT_NAME_SIZE 120
//#define TRAC_PAST_MERIDIAN 1.0   // Allow mount to track this much beyond the Meridian - set to 1 hour for now
#define N_TRACK_STOP       4     // Require 4 successive location co-ordinates beyond limits (meridian or horizon) to stop tracking

// Firmware-level safety backstop ('lt'/'lh'/'la' commands - see AstroTrac::sendSafetyLimits(), which also
// defines the minimum firmware version that understands them). This driver's own meridian/horizon checks in
// raDec() take precedence and should stop tracking first; the firmware limits are sent with this much extra
// margin so they only trip if this driver's check somehow fails to (e.g. TheSkyX hangs or the connection drops).
#define FIRMWARE_SAFETY_MARGIN_DEG 3.0

// Comment out AstroTrac_X2_DEBUG entirely for a production build - LogFile/LogDebug then compile away to
// nothing (see LogDebug in x2mount.cpp). When defined, controls how much gets logged. Levels mirror the
// PLUGIN_DEBUG scheme in AstroTrac.h so the two log files read consistently (levels 0 and 2 have no sites
// here - the open-loop-move timing and send-command machinery they cover live in AstroTrac.cpp):
//   0: (unused here) - open-loop-move timing lives in AstroTrac.cpp under PLUGIN_DEBUG.
//   1: Open-loop-move tracing (relevant to guiding) plus notable/unexpected events worth a heads-up even
//      outside active debugging - command failures (open-loop move, slew, unpark) and the safety stops
//      that halt tracking (below horizon / past meridian).
//   2: (unused here) - the send-command machinery it would cover lives in AstroTrac.cpp.
//   3: Everything else - driver/connection lifecycle, coordinate/math tracing, slew and tracking lifecycle.
// #define AstroTrac_X2_DEBUG  3  // Uncomment to enable logging (levels 0-3, see above)

#if defined(SB_WIN_BUILD)
#define DEF_PORT_NAME					"COM1"
#elif defined(SB_LINUX_BUILD)
#define DEF_PORT_NAME					"/dev/mount"
#elif defined (SB_MAC_BUILD)
#define DEF_PORT_NAME					"/dev/cu.KeySerial1"
#endif


/*!
\brief The X2Mount example.

\ingroup Example

Use this example to write an X2Mount driver.
*/
class X2Mount : public MountDriverInterface 
						,public SyncMountInterface
						,public SlewToInterface
                        ,public AsymmetricalEquatorialInterface
						,public OpenLoopMoveInterface
						,public TrackingRatesInterface
						,public ParkInterface
						,public UnparkInterface
						,public ModalSettingsDialogInterface
                        ,public X2GUIEventInterface
                        ,public SerialPortParams2Interface
                        ,public DriverSlewsToParkPositionInterface
                        ,public PulseGuideInterface2
{
public:
	/*!Standard X2 constructor*/
	X2Mount(const char* pszDriverSelection,
				const int& nInstanceIndex,
				SerXInterface					* pSerX, 
				TheSkyXFacadeForDriversInterface	* pTheSkyX, 
				SleeperInterface					* pSleeper,
				BasicIniUtilInterface			* pIniUtil,
				LoggerInterface					* pLogger,
				MutexInterface					* pIOMutex,
				TickCountInterface				* pTickCount);

	~X2Mount();

// Operations
public:
	
	/*!\name DriverRootInterface Implementation
	 See DriverRootInterface.*/
	//@{
	virtual DeviceType							deviceType(void)							  {return DriverRootInterface::DT_MOUNT;}
	virtual int									queryAbstraction(const char* pszName, void** ppVal) ;
	//@}
	
	/* See LinkInterface.*/
	//@{
	virtual int									establishLink(void)						;
	virtual int									terminateLink(void)						;
	virtual bool								isLinked(void) const					;
	virtual bool								isEstablishLinkAbortable(void) const	;
	//@}
	
	/*!\name DriverInfoInterface Implementation
	 See DriverInfoInterface.*/
	//@{
	virtual void								driverInfoDetailedInfo(BasicStringInterface& str) const;
	virtual double								driverInfoVersion(void) const				;
	//@}
	
	/*!\name HardwareInfoInterface Implementation
	 See HardwareInfoInterface.*/
	//@{
	virtual void deviceInfoNameShort(BasicStringInterface& str) const				;
	virtual void deviceInfoNameLong(BasicStringInterface& str) const				;
	virtual void deviceInfoDetailedDescription(BasicStringInterface& str) const	;
	virtual void deviceInfoFirmwareVersion(BasicStringInterface& str)				;
	virtual void deviceInfoModel(BasicStringInterface& str)						;
	//@}
	
  virtual int raDec(double& ra, double& dec, const bool& bCached = false);
  virtual int abort(void);
	
	//Optional interfaces, uncomment and implement as required.
	
	//SyncMountInterface
	virtual int syncMount(const double& ra, const double& dec)									;
	virtual bool isSynced()																		;
	
	//SlewToInterface
	virtual int								startSlewTo(const double& dRa, const double& dDec)	;
	virtual int								isCompleteSlewTo(bool& bComplete) const				;
	virtual int								endSlewTo(void)										;
	
	//AsymmetricalEquatorialInterface
    virtual bool knowsBeyondThePole();
	virtual int beyondThePole(bool& bYes);
    virtual double flipHourAngle();
    virtual int gemLimits(double& dHoursEast, double& dHoursWest);

    // SymmetricalEquatorialInterface
    virtual MountTypeInterface::Type mountType();
	
	//OpenLoopMoveInterface
	virtual int								startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex);
	virtual int								endOpenLoopMove(void);
	virtual bool							allowDiagonalMoves() {return true;}
	virtual int								rateCountOpenLoopMove(void) const;
	virtual int								rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize);
	virtual int								rateIndexOpenLoopMove(void);
    
       
    //PulseGuideInterface
    virtual int useOpenLoopMoveInterface(int& nGuideRateIndex, OpenLoopMoveInterface** pOLSI)
    {
        nGuideRateIndex = m_iGuideRateIndex; 
        return queryAbstraction(OpenLoopMoveInterface_Name, (void**)pOLSI);
    }
	
	//NeedsRefractionInterface
	virtual bool							needsRefactionAdjustments(void);

    //TrackingRatesInterface
	virtual int setTrackingRates( const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec);
	virtual int trackingRates( bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec);
    virtual int siderealTrackingOn();
    virtual int trackingOff();

	/* Parking Interface */
	virtual bool							isParked(void);
	virtual int								startPark(const double& dAz, const double& dAlt);
	virtual int								isCompletePark(bool& bComplete) const;
	virtual int								endPark(void);

	/* Unparking Interface */
	int								startUnpark(void);
	int								isCompleteUnpark(bool& bComplete) const;
	int								endUnpark(void);
	
    //SerialPortParams2Interface
    virtual void            portName(BasicStringInterface& str) const            ;
    virtual void            setPortName(const char* szPort)                        ;
    virtual unsigned int    baudRate() const            {return 115200;};
    virtual void            setBaudRate(unsigned int)    {};
    virtual bool            isBaudRateFixed() const        {return true;}

    virtual SerXInterface::Parity    parity() const                {return SerXInterface::B_NOPARITY;}
    virtual void                    setParity(const SerXInterface::Parity& parity){};
    virtual bool                    isParityFixed() const        {return true;}

	// GUI Interface
    virtual int             initModalSettingsDialog(void) {return SB_OK;};
    virtual int             execModalSettingsDialog(void);
    virtual void            uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent);
	
	
	// Implementation
private:
	// Sky Interfaces
	SerXInterface 							*GetSerX() {return m_pSerX; }
	TheSkyXFacadeForDriversInterface		*GetTheSkyXFacadeForMounts() {return m_pTheSkyXForMounts;}
	SleeperInterface						*GetSleeper() {return m_pSleeper; }
	BasicIniUtilInterface					*GetSimpleIniUtil() {return m_pIniUtil; }
	LoggerInterface							*GetLogger() {return m_pLogger; }
	MutexInterface							*GetMutex()  {return m_pIOMutex;}
	TickCountInterface						*GetTickCountInterface() {return m_pTickCount;}
	
	// Variables to store Sky X interfaces
	int m_nPrivateMulitInstanceIndex;
	SerXInterface*							m_pSerX;
	TheSkyXFacadeForDriversInterface* 		m_pTheSkyXForMounts;
	SleeperInterface*						m_pSleeper;
	BasicIniUtilInterface*					m_pIniUtil;
	LoggerInterface*						m_pLogger;
	MutexInterface*							m_pIOMutex;
	TickCountInterface*						m_pTickCount;
	
    
    // Variables for 
	// Variables for AstroTrac
    AstroTrac mAstroTrac;

    bool m_bLinked;

    char m_PortName[MAX_PORT_NAME_SIZE];

    int m_CurrentRateIndex;

    void portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const;

    // Sends this driver's meridian/horizon settings (padded with FIRMWARE_SAFETY_MARGIN_DEG) plus the
    // current site latitude to the mount firmware, if it's new enough to support 'lt'/'lh'/'la'. Called
    // after establishLink() and whenever the settings dialog is accepted, so changes take effect without
    // needing a reconnect.
    void sendSafetyLimitsToFirmware();

    // Write a single debug log line if AstroTrac_X2_DEBUG is defined and at least nLevel, otherwise a
    // no-op. Centralizes the timestamp/fprintf/fflush boilerplate that used to be repeated at every log
    // site. Mirrors AstroTrac::LogDebug(); const so it can be called from const methods like isCompleteSlewTo().
    void LogDebug(int nLevel, const char *pszFormat, ...) const;

    int m_iNTrackingOff = 0;
    
    int m_iGuideRateIndex = 0; //Default - 0.1x siderial

    double m_dHoursPastMeridian = 0.0;
    double m_dHorizonLimitDeg = 0.0;
    
#ifdef AstroTrac_X2_DEBUG
    std::string m_sLogfilePath;
	FILE *LogFile;	  // LogFile
#endif
	
	
};
