#include "x2mount.h"

X2Mount::X2Mount(const char* pszDriverSelection,
				 const int& nInstanceIndex,
				 SerXInterface					* pSerX,
				 TheSkyXFacadeForDriversInterface	* pTheSkyX,
				 SleeperInterface					* pSleeper,
				 BasicIniUtilInterface			* pIniUtil,
				 LoggerInterface					* pLogger,
				 MutexInterface					* pIOMutex,
				 TickCountInterface				* pTickCount)
{

	m_nPrivateMulitInstanceIndex	= nInstanceIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;
	
#ifdef AstroTrac_X2_DEBUG
    std::string sLogDir;
    std::string sPathSep;
#if defined(SB_WIN_BUILD)
    sLogDir = getenv("HOMEDRIVE");
    sLogDir += getenv("HOMEPATH");
    sPathSep = "\\";
#elif defined(SB_LINUX_BUILD)
    sLogDir = getenv("HOME");
    sPathSep = "/";
#elif defined(SB_MAC_BUILD)
    sLogDir = getenv("HOME");
    sPathSep = "/";
#endif

    // Name the log after the observing night, using the same "noon to noon"
    // convention TheSkyX itself uses for its guide-log folders, and the same
    // approach as AstroTrac.cpp's own log - see the comment there for why.
    time_t nowTime = time(nullptr);
    struct tm nightTm;
#if defined(SB_WIN_BUILD)
    localtime_s(&nightTm, &nowTime);
#else
    localtime_r(&nowTime, &nightTm);
#endif
    if (nightTm.tm_hour < 12) {
        nowTime -= 12 * 3600;
#if defined(SB_WIN_BUILD)
        localtime_s(&nightTm, &nowTime);
#else
        localtime_r(&nowTime, &nightTm);
#endif
    }
    char szNightDate[32];
    strftime(szNightDate, sizeof(szNightDate), "%B %d %Y", &nightTm);

    std::string sBaseName = std::string("AstroTrac_X2_Logfile_") + szNightDate;
    m_sLogfilePath = sLogDir + sPathSep + sBaseName + ".txt";
    int nVersion = 1;
    while (FILE *pExisting = fopen(m_sLogfilePath.c_str(), "r")) {
        fclose(pExisting);
        nVersion++;
        m_sLogfilePath = sLogDir + sPathSep + sBaseName + "_v" + std::to_string(nVersion) + ".txt";
    }

	LogFile = fopen(m_sLogfilePath.c_str(), "w");
#endif
	
	
    m_bLinked = false;

    mAstroTrac.setSerxPointer(m_pSerX);
    mAstroTrac.setTSX(m_pTheSkyXForMounts);
    mAstroTrac.setSleeper(m_pSleeper);

    m_CurrentRateIndex = 1;

	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
        m_iGuideRateIndex = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_GUIDERATE, 0);
        m_dHoursPastMeridian = m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_HOURS_PAST_MERIDIAN, 1.0);
        m_dHorizonLimitDeg = m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_HORIZON_LIMIT, 0.0);  // 0deg matches this driver's existing hardcoded dAlt<0.0 cutoff
	}
    
    
    // set mount alignement type and meridian avoidance mode.
    if(strstr(pszDriverSelection,"Single Arm")) {
        mAstroTrac.setMountMode(MountTypeInterface::Symmetrical_Equatorial);
    }
    else {
         mAstroTrac.setMountMode(MountTypeInterface::Asymmetrical_Equatorial);
    }
    
    LogDebug(3, "X2Mount constructor called pszDriverSelection: %s %d\n", pszDriverSelection, mAstroTrac.mountType());
}

X2Mount::~X2Mount()
{
	// Write the stored values

    if(m_bLinked) {
        X2MutexLocker ml(GetMutex());
        mAstroTrac.Disconnect();
    }

    if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;
	
#ifdef AstroTrac_X2_DEBUG
	// Close LogFile
	if (LogFile) {
        fflush(LogFile);
		fclose(LogFile);
	}
#endif

}

// Write a single debug log line if AstroTrac_X2_DEBUG is defined and at least nLevel, otherwise a no-op.
// Centralizes the timestamp/fprintf/fflush boilerplate that used to be repeated at every log site.
void X2Mount::LogDebug(int nLevel, const char *pszFormat, ...) const
{
#ifdef AstroTrac_X2_DEBUG
    if (!LogFile || nLevel > AstroTrac_X2_DEBUG)
        return;

    va_list args;

    // Millisecond precision (vs. asctime()'s whole-second resolution previously) so log timestamps
    // can be used directly for timing analysis, matching AstroTrac.cpp's own log.
    struct timespec tsNow;
    struct tm tmNow;
    char szTimestamp[32];
    clock_gettime(CLOCK_REALTIME, &tsNow);
#if defined(SB_WIN_BUILD)
    localtime_s(&tmNow, &tsNow.tv_sec);
#else
    localtime_r(&tsNow.tv_sec, &tmNow);
#endif
    strftime(szTimestamp, sizeof(szTimestamp), "%a %b %e %H:%M:%S", &tmNow);
    fprintf(LogFile, "[%s.%03ld %d] ", szTimestamp, tsNow.tv_nsec / 1000000, tmNow.tm_year + 1900);

    va_start(args, pszFormat);
    vfprintf(LogFile, pszFormat, args);
    va_end(args);

    fflush(LogFile);
#endif
}

int X2Mount::queryAbstraction(const char* pszName, void** ppVal)
{
	*ppVal = NULL;

    LogDebug(3, "queryAbstrcttion Called: pszName %s\n", pszName);


	if (!strcmp(pszName, SyncMountInterface_Name))
	    *ppVal = dynamic_cast<SyncMountInterface*>(this);
	if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
    // Only call this when mount is asymmmetrical equatorial, not when in single arm mode.
	if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name) && mountType() == MountTypeInterface::Asymmetrical_Equatorial)
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
    if (!strcmp(pszName, PulseGuideInterface2_Name))
        *ppVal = dynamic_cast<PulseGuideInterface2*>(this);
    if (!strcmp(pszName, NeedsRefractionInterface_Name))
	 	*ppVal = dynamic_cast<NeedsRefractionInterface*>(this);
	if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
		*ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    if (!strcmp(pszName, X2GUIEventInterface_Name))
	 	*ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    if (!strcmp(pszName, TrackingRatesInterface_Name))
		*ppVal = dynamic_cast<TrackingRatesInterface*>(this);
	if (!strcmp(pszName, ParkInterface_Name))
		*ppVal = dynamic_cast<ParkInterface*>(this);
	if (!strcmp(pszName, UnparkInterface_Name))
		*ppVal = dynamic_cast<UnparkInterface*>(this);
    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name))
       *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);

    return SB_OK;
}

#pragma mark - OpenLoopMoveInterface

int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

	m_CurrentRateIndex = nRateIndex;
    LogDebug(1, "startOpenLoopMove called Dir: %d , Rate: %d\n", Dir, nRateIndex);

    nErr = mAstroTrac.startOpenLoopMove(Dir, nRateIndex);
    if(nErr) {
        LogDebug(1, "startOpenLoopMove ERROR %d\n", nErr);
        m_pLogger->out("startOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    return SB_OK;
}

int X2Mount::endOpenLoopMove(void)
{
	int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    LogDebug(1, "endOpenLoopMove Called\n");

    nErr = mAstroTrac.stopOpenLoopMove();
    if(nErr) {
        LogDebug(1, "endOpenLoopMove ERROR %d\n", nErr);
        m_pLogger->out("endOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe->GetMutex());
	return pMe->mAstroTrac.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
    int nErr = SB_OK;
    std::string sTmp;
    
    nErr = mAstroTrac.getRateName(nZeroBasedIndex, sTmp);
    if(nErr) {
        LogDebug(1, "rateNameFromIndexOpenLoopMove ERROR %d\n", nErr);
        m_pLogger->out("rateNameFromIndexOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    strncpy(pszOut, sTmp.c_str(), nOutMaxSize);
    return nErr;
}

int X2Mount::rateIndexOpenLoopMove(void)
{
	return m_CurrentRateIndex;
}

#pragma mark - UI binding

int X2Mount::execModalSettingsDialog(void)
{
	int nErr = SB_OK;
	X2ModalUIUtil uiutil(this, m_pTheSkyXForMounts);
	X2GUIInterface*					ui = uiutil.X2UI();
	X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    int i; // Counter
	bool bPressedOK = false;
    std::string sTmp;

	if (NULL == ui) return ERR_POINTER;
	
	if ((nErr = ui->loadUserInterface("AstroTrac.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;
	
	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}

    X2MutexLocker ml(GetMutex());

	// Set values in the userinterface
    // First add the possible slew rates as guide rates (most will be way too big, but...)
    for (i = 0; i < mAstroTrac.getNumberGuideRates(); i++) {
        mAstroTrac.getRateName(i, sTmp);
        dx->comboBoxAppendString("comboBox", sTmp.c_str());
    }
    dx->setCurrentIndex("comboBox", m_iGuideRateIndex);
    
    // Now display the hours past the meridian, and the horizon limit.
    dx->setPropertyDouble("doubleSpinBox", "value", m_dHoursPastMeridian);
    dx->setPropertyDouble("doubleSpinBox_2", "value", m_dHorizonLimitDeg);

	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;

	//Retreive values from the user interface
	if (bPressedOK) {
        m_iGuideRateIndex = dx->currentIndex("comboBox");
        dx->propertyDouble("doubleSpinBox", "value", m_dHoursPastMeridian);
        dx->propertyDouble("doubleSpinBox_2", "value", m_dHorizonLimitDeg);
        m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_GUIDERATE, m_iGuideRateIndex);
        m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_HOURS_PAST_MERIDIAN, m_dHoursPastMeridian);
        m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_HORIZON_LIMIT, m_dHorizonLimitDeg);
        if (m_bLinked) sendSafetyLimitsToFirmwareCore();  //Apply immediately rather than waiting for a reconnect
	}
	return nErr;
}

// See x2mount.h declaration for the overall design: this driver's own raDec() meridian/horizon checks take
// precedence, firmware limits are last-resort only and padded with FIRMWARE_SAFETY_MARGIN_DEG so they trip
// after this driver's own check would have. No-ops harmlessly (via AstroTrac::sendSafetyLimits()) on
// firmware older than FIRMWARE_MIN_VER_SAFETY_LIMITS.
void X2Mount::sendSafetyLimitsToFirmwareCore()
{
    double dMeridianLimitDeg = m_dHoursPastMeridian * 15.0 + FIRMWARE_SAFETY_MARGIN_DEG;
    double dHorizonLimitDeg = m_dHorizonLimitDeg - FIRMWARE_SAFETY_MARGIN_DEG;
    double dLatitudeDeg = m_pTheSkyXForMounts->latitude();

    mAstroTrac.sendSafetyLimits(dMeridianLimitDeg, dHorizonLimitDeg, dLatitudeDeg);
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
	return;
}

#pragma mark - LinkInterface
int X2Mount::establishLink(void)
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

	X2MutexLocker ml(GetMutex());
	// get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);

	nErr =  mAstroTrac.Connect(szPort);

    LogDebug(3, "establishLink Called. nErr %d\n", nErr);
    if(nErr) {
        m_bLinked = false;
    }
    else {
        m_bLinked = true;
        sendSafetyLimitsToFirmwareCore();
    }

    return nErr;
}

int X2Mount::terminateLink(void)
{
    int nErr = SB_OK;

	X2MutexLocker ml(GetMutex());

    nErr = mAstroTrac.Disconnect();
    m_bLinked = false;

    return nErr;
}

bool X2Mount::isLinked(void) const
{

	return mAstroTrac.isConnected();;
}

bool X2Mount::isEstablishLinkAbortable(void) const
{
    return false;
}

#pragma mark - AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
	str = "AstroTrac X2 plugin by Colin McGill";
}

double	X2Mount::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const
{
    if(m_bLinked) {
        str = "AstroTrac";
    }
    else
        str = "Not connected1";
}
void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "AstroTrac360 Mount";
	
}
void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = "AstroTrac360 Telescope Control System";
	
}
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
    if(m_bLinked) {
        std::string sFirmware;
        X2MutexLocker ml(GetMutex());
        mAstroTrac.getFirmwareVersion(sFirmware);
        str = sFirmware.c_str();
    }
    else
        str = "Not connected";
}
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
    if(m_bLinked) {
        str = "AstroTrac";
    }
    else
        str = "Not connected";
}

#pragma mark - Common Mount specifics
int X2Mount::raDec(double& ra, double& dec, const bool& bCached)
{
  int nErr = 0;
    double dAz, dAlt, Ha;
    bool  bComplete;
    
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    // Get the HA and DEC from the mount
    nErr = mAstroTrac.getHaAndDec(Ha, dec);
    if(nErr) nErr = ERR_CMDFAILED;
    
    // Subtract HA from lst to get ra;
    ra = m_pTheSkyXForMounts->lst()-Ha;
    
    // Ensure in range 0 to 24
    if (ra < 0) {
        ra += 24.0;
    }
    else if (ra > 24.0) {
        ra -= 24.0;
    }


    
    LogDebug(3, "raDec Called. ha : %f , Ra : %f Dec : %f BTP %d\n", Ha, ra, dec, mAstroTrac.GetIsBeyondThePole());
    LogDebug(3, "nErr = %d \n", nErr);


    // Now check if have exceeded the tracking limits
    // First check to see if currently slewing - if so, then can return since no limits imposed during slews
    nErr = isCompleteSlewToCore(bComplete); if (nErr || !bComplete) return nErr;

    // Now check to see if below the horizon.
    // Beyond the pole must be false (pointing west of meridian) for this to be true
    
    nErr = m_pTheSkyXForMounts->EqToHz(ra, dec, dAz, dAlt); if (nErr) return nErr;
    
    if (!mAstroTrac.GetIsBeyondThePole() && dAlt < m_dHorizonLimitDeg) {
      // Were getting random problems with positions, to ensure we have several measurements
      m_iNTrackingOff++;
      if (m_iNTrackingOff >= N_TRACK_STOP) {

	LogDebug(1, "raDec Called. Below horizon limit. dAlt %f, limit %f, ha %f dec %f m_INTrackingOff %d\n",
		 dAlt, m_dHorizonLimitDeg, Ha, dec, m_iNTrackingOff);
	nErr = setTrackingRatesCore(false, true, 0.0, 0.0); if (nErr) return ERR_CMDFAILED; // Stop tracking since now too low and setting
      }
    }
    // Now see if tracking beyond the meridian.
    // Must be beyond the pole (pointing east of meridian) for this to occur
    // or have pointing west of Merdidian and gone beyond Ha = 12.
    // Value of TRAC_PAST_MERIDIAN set in x2mount.h
    else if ((mAstroTrac.GetIsBeyondThePole() && Ha > m_dHoursPastMeridian) ||
             (!mAstroTrac.GetIsBeyondThePole()  && Ha > 12 + m_dHoursPastMeridian)) {
      // Were getting random problems with positions, to ensure we have several measurements
      m_iNTrackingOff++;
      if (m_iNTrackingOff >= N_TRACK_STOP) {
        nErr = setTrackingRatesCore(false, true, 0.0, 0.0);    // Stop tracking since these have been exceeded

	LogDebug(1, "raDec Called. Too far past meridian. Ha %f m_iNTrackingOff %d\n", Ha, m_iNTrackingOff);
        if (nErr) return ERR_CMDFAILED;
      }
    }
    // Else reset stop tracking counter
    else {
      m_iNTrackingOff = 0;
    }

    LogDebug(3, "raDec Called. dAz %f, dAlt %f\n", dAz, dAlt);

	return nErr;
}

int X2Mount::abort()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    LogDebug(3, "abort Called\n");

    nErr = mAstroTrac.Abort();
    if(nErr)
        nErr = ERR_CMDFAILED;

    LogDebug(3, "Abort nErr = %d \n", nErr);

    return nErr;
}

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
	int nErr = SB_OK;
    double dHA;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    // Start tracking since mount remembers tracking state before slewing
    siderealTrackingOnCore();

    LogDebug(3, "startSlewTo Called %f %f\n", dRa, dDec);

    // Calulate HA using the Sky interface:
    dHA = m_pTheSkyXForMounts->hourAngle(dRa);

    nErr = mAstroTrac.startSlewTo(dHA, dDec, dRa);
    if(nErr) {
        LogDebug(1, "startSlewTo nErr = %d \n", nErr);
        m_pLogger->out("startSlewTo ERROR");
        return ERR_CMDFAILED;
    }

    return nErr;
}

int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    return pMe->isCompleteSlewToCore(bComplete);
}

// Same as isCompleteSlewTo(), minus the link check and mutex - for raDec(), which already holds the lock
// itself and would otherwise re-lock GetMutex() reentrantly.
int X2Mount::isCompleteSlewToCore(bool& bComplete) const
{
    int nErr = SB_OK;

    X2Mount* pMe = (X2Mount*)this;
    nErr = pMe->mAstroTrac.isSlewToComplete(bComplete);

    if(nErr)
        return ERR_CMDFAILED;

    LogDebug(3, "isCompleteSlewTo %d nErr = %d i\n", bComplete, nErr);

	return nErr;
}

int X2Mount::endSlewTo(void)
{
    int nErr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    LogDebug(3, "endSlewTo Called\n");

    nErr = mAstroTrac.endSlewTo();
    
    if(nErr) {
        return ERR_CMDFAILED;
    } else {
        return SB_OK;
    }
}


int X2Mount::syncMount(const double& ra, const double& dec)
{
	int nErr = SB_OK;
    double Ha;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    
    // Convert ra to Ha
    Ha = m_pTheSkyXForMounts->hourAngle(ra);

    LogDebug(3, "syncMount Called : %f\t%f\n", ra, dec);

    nErr = mAstroTrac.syncTo(Ha, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

    LogDebug(3, "syncMount nErr = %d \n", nErr);

    return nErr;
}

bool X2Mount::isSynced(void)
{   // As per documentation, always returns true since the mount does not know if it has been synced
    
    return true;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    return setTrackingRatesCore(bTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
}

// Same as setTrackingRates(), minus the link check and mutex - for callers (raDec, siderealTrackingOnCore,
// trackingOff) that already hold the lock themselves and would otherwise re-lock GetMutex() reentrantly.
int X2Mount::setTrackingRatesCore(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;

    nErr = mAstroTrac.setTrackingRates(bTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);

    LogDebug(3, "setTrackingRates Called. Tracking On: %s , Ra rate : %f , Dec rate: %f nerr %d\n", bTrackingOn?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec, nErr);
    if(nErr)
        return ERR_CMDFAILED;

    return nErr;

}

int X2Mount::trackingRates(bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
    // This simply reads the previously stored rates
    int nErr = SB_OK;
    
    X2MutexLocker ml(GetMutex());
    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = mAstroTrac.getTrackRates(bTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec); if (nErr) return ERR_CMDFAILED;

    LogDebug(3, "trackingRates Called. Tracking On: %s , Ra rate : %f , Dec rate: %f nerr %d\n", bTrackingOn?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec, nErr);

	return nErr;
}

int X2Mount::siderealTrackingOn()
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    return siderealTrackingOnCore();
}

// Same as siderealTrackingOn(), minus the link check and mutex - for startSlewTo(), which already holds
// the lock itself and would otherwise re-lock GetMutex() reentrantly.
int X2Mount::siderealTrackingOnCore()
{
    int nErr = SB_OK;

    LogDebug(3, "siderealTrackingOn Called \n");

    nErr = setTrackingRatesCore( true, true, 0.0, 0.0);
    if(nErr)
        return ERR_CMDFAILED;

    LogDebug(3, "siderealTrackingOn nErr = %d \n", nErr);

    return nErr;
}

int X2Mount::trackingOff()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    LogDebug(3, "trackingOff Called \n");

    nErr = setTrackingRatesCore( false, true, 0.0, 0.0);
    if(nErr)
        nErr = ERR_CMDFAILED;

    LogDebug(3, "trackingOff nErr = %d \n", nErr);

    return nErr;
}


#pragma mark - NeedsRefractionInterface
bool X2Mount::needsRefactionAdjustments(void)
{
    return true;
}

#pragma mark - Parking Interface
bool X2Mount::isParked(void)
{
    return mAstroTrac.GetIsParked();
 
}

int X2Mount::startPark(const double& dAz, const double& dAlt)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;
	
    X2MutexLocker ml(GetMutex());

    // No choice of park position so can ignore co-ordinates
    // Will park towards north or south pole with weights down
    nErr = mAstroTrac.gotoPark(0.0, 0.0);
    if(nErr)
        nErr = ERR_CMDFAILED;

    LogDebug(3, "startPark  mAstroTrac.gotoPark nErr = %d \n", nErr);

	return nErr;
}


int X2Mount::isCompletePark(bool& bComplete) const
{
    int nErr = SB_OK;
    X2Mount* pMe = (X2Mount*)this;
    
    X2MutexLocker ml(pMe->GetMutex());
    
    if(!m_bLinked)
        return ERR_NOLINK;
    
    nErr = pMe->mAstroTrac.GetIsParkingComplete(bComplete); if (nErr) return ERR_CMDFAILED;
    
    return SB_OK;
}

int X2Mount::endPark(void)
{
    return SB_OK;
}

int X2Mount::startUnpark(void)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mAstroTrac.unPark();
    if(nErr) {
        LogDebug(1, "startUnpark : mAstroTrac.unPark() failed !\n");
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

/*!Called to monitor the unpark process.
 \param bComplete Set to true if the unpark is complete, otherwise set to false.
*/
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
    bComplete = true;
    return SB_OK;
}

/*!Called once the unpark is complete.
 This is called once for every corresponding startUnpark() allowing software implementations of unpark.
 */
int		X2Mount::endUnpark(void)
{
	return SB_OK;
}

#pragma mark - AsymmetricalEquatorialInterface

bool X2Mount::knowsBeyondThePole()
{
    LogDebug(3, "knowBeyondThePole called\n");
    return true;
}

int X2Mount::beyondThePole(bool& bYes) {
    if(!m_bLinked)
        return ERR_NOLINK;

    bYes = mAstroTrac.GetIsBeyondThePole();
	return SB_OK;
}


double X2Mount::flipHourAngle() {

	return 0.0;
}


int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
	dHoursEast = 0.0;
	dHoursWest = m_dHoursPastMeridian;
	return SB_OK;
}

MountTypeInterface::Type X2Mount::mountType()
{
    return  mAstroTrac.mountType();
}


#pragma mark - SerialPortParams2Interface

void X2Mount::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Mount::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort);

}


void X2Mount::portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort, pszPort, nMaxSize);

}



