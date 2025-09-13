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
#define MAX_PORT_NAME_SIZE 120
#define TRAC_PAST_MERIDIAN 1.0   // Allow mount to track this much beyond the Meridian - set to 1 hour for now
#define N_TRACK_STOP       4     // Require 4 successive location co-ordinates beyond limits (meridian or horizon) to stop tracking

// #define AstroTrac_X2_DEBUG  2  // Define this to have log files. 1 for just bad things, 2 for general stuff.

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
        nGuideRateIndex = 1; // 1 is rate "0.1x"
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
	virtual int initModalSettingsDialog(void) { return 0; }
	virtual int execModalSettingsDialog(void);
	void uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent); // Process a UI event
	
	
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
	
	// Variables for AstroTrac
  AstroTrac mAstroTrac;

  bool m_bLinked;

  char m_PortName[MAX_PORT_NAME_SIZE];
	
  int m_CurrentRateIndex;

  void portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const;

  int m_iNTrackingOff = 0;

#ifdef AstroTrac_X2_DEBUG
    std::string m_sLogfilePath;
    char *timestamp;
    time_t ltime;
	FILE *LogFile;	  // LogFile
#endif
	
	
};
