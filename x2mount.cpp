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
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\AstroTrac_X2_Logfile.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/AstroTrac_X2_Logfile.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/AstroTrac_X2_Logfile.txt";
#endif
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
	}
    
    
    // set mount alignement type and meridian avoidance mode.
    if(strstr(pszDriverSelection,"Single Arm")) {
        mAstroTrac.setMountMode(MountTypeInterface::Symmetrical_Equatorial);
    }
    else {
         mAstroTrac.setMountMode(MountTypeInterface::Asymmetrical_Equatorial);
    }
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] X2Mount constructor called pszDriverSelection: %s %d\n", timestamp, pszDriverSelection, mAstroTrac.mountType());
        fflush(LogFile);
    }
#endif
}

X2Mount::~X2Mount()
{
	// Write the stored values

    if(m_bLinked)
        mAstroTrac.Disconnect();
    
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

int X2Mount::queryAbstraction(const char* pszName, void** ppVal)
{
	*ppVal = NULL;
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] queryAbstrcttion Called: pszName %s\n", timestamp, pszName);
        fflush(LogFile);
    }
#endif
    
	if (!strcmp(pszName, SyncMountInterface_Name))
	    *ppVal = dynamic_cast<SyncMountInterface*>(this);
	if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
    // Only call this when mount is asymmmetrical equatorial, not when in single arm mode.
	if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name) && mountType() == MountTypeInterface::Asymmetrical_Equatorial)
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
    // if (!strcmp(pszName, NeedsRefractionInterface_Name))
	// 	*ppVal = dynamic_cast<NeedsRefractionInterface*>(this);
	//if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
	//	*ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    //if (!strcmp(pszName, X2GUIEventInterface_Name))
	// 	*ppVal = dynamic_cast<X2GUIEventInterface*>(this);
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
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startOpenLoopMove called Dir: %d , Rate: %d\n", timestamp, Dir, nRateIndex);
        fflush(LogFile);
	}
#endif

    nErr = mAstroTrac.startOpenLoopMove(Dir, nRateIndex);
    if(nErr) {
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
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

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
	if (LogFile){
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] endOpenLoopMove Called\n", timestamp);
        fflush(LogFile);
	}
#endif

    nErr = mAstroTrac.stopOpenLoopMove();
    if(nErr) {
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] endOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
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
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] rateNameFromIndexOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
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
	bool bPressedOK = false;
    std::string sTmp;
    std::string sTime;
    std::string sDate;
    std::string sLongitude;
    std::string sLatitude;
    std::string sTimeZone;
	if (NULL == ui) return ERR_POINTER;
	
	if ((nErr = ui->loadUserInterface("AstroTrac.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;
	
	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}

    X2MutexLocker ml(GetMutex());

	// Set values in the userinterface
    if(m_bLinked) {
        dx->setEnabled("pushButton",true);
        dx->setEnabled("pushButton_2",true);
        dx->setEnabled("pushButton_3",true);
        dx->setEnabled("alignmentType",true);
        dx->setEnabled("pushButton_4",true);

        if(!nErr) {
            sTmp =sDate + " - " + sTime;
            dx->setText("time_date", sTmp.c_str());
        }

    }
    else {
        dx->setText("time_date", "");
        dx->setText("siteName", "");
        dx->setText("longitude", "");
        dx->setText("latitude", "");
        dx->setText("timezone", "");
        dx->setEnabled("pushButton",false);
        dx->setEnabled("pushButton_2",false);
        dx->setEnabled("pushButton_3",false);
        dx->setEnabled("alignmentType",false);
        dx->setEnabled("pushButton_4",false);
    }
	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;
	
	//Retreive values from the user interface
	if (bPressedOK) {
	}
	return nErr;
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    std::string sTmpBuf;
    std::string sTime;
    std::string sDate;

    if(!m_bLinked)
        return ;

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
	time_t ltime;
	char *timestamp;
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] uievent %s\n", timestamp, pszEvent);
        fflush(LogFile);
	}
#endif
	if (!strcmp(pszEvent, "on_timer")) {

	}

    if (!strcmp(pszEvent, "on_pushButton_clicked")) {

    }
    

    if (!strcmp(pszEvent, "on_pushButton_2_clicked")) {
    }

    if (!strcmp(pszEvent, "on_pushButton_3_clicked")) {
    }

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
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] establishLink Called. nErr %d\n", timestamp, nErr);
        fflush(LogFile);
    }
#endif
    if(nErr) {
        m_bLinked = false;
    }
    else {
        m_bLinked = true;
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


    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] raDec Called. ha : %f , Ra : %f Dec : %f BTP %d\n", timestamp, Ha, ra, dec, mAstroTrac.GetIsBeyondThePole());
        fprintf(LogFile, "[%s] nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif


    // Now check if have exceeded the tracking limits
    // First check to see if currently slewing - if so, then can return since no limits imposed during slews
    nErr = isCompleteSlewTo(bComplete); if (nErr || ! bComplete) return nErr;
    
    // Now see if tracking beyond the meridian. Must be beyond the pole (pointing east of meridian) for this to occur.
    // Value of TRAC_PAST_MERIDIAN set in x2mount.h
    if (mAstroTrac.GetIsBeyondThePole() && Ha > TRAC_PAST_MERIDIAN) {
        nErr = setTrackingRates(false, true, 0.0, 0.0);    // Stop tracking since these have been exceeded
        if (nErr) return ERR_CMDFAILED;
    }

    // Now check to see if below the horizon. Beyond the pole must be false (pointing west of meridian) for this to be true
    nErr = m_pTheSkyXForMounts->EqToHz(ra, dec, dAz, dAlt); if (nErr) return nErr;
    
    if (!mAstroTrac.GetIsBeyondThePole() && dAlt < 0.0) {
        nErr = setTrackingRates(false, true, 0.0, 0.0);    if (nErr) return ERR_CMDFAILED; // Stop tracking since now too low and setting
    }

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] raDec Called. dAz %f, dAlt %f\n", timestamp, dAz, dAlt);
        fflush(LogFile);
    }
#endif
    
	return nErr;
}

int X2Mount::abort()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef AstroTrac_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] abort Called\n", timestamp);
        fflush(LogFile);
	}
#endif

    nErr = mAstroTrac.Abort();
    if(nErr)
        nErr = ERR_CMDFAILED;

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] Abort nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

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
    siderealTrackingOn();

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] startSlewTo Called %f %f\n", timestamp, dRa, dDec);
        fflush(LogFile);
	}
#endif
    
    // Calulate HA using the Sky interface:
    dHA = m_pTheSkyXForMounts->hourAngle(dRa);
    
    nErr = mAstroTrac.startSlewTo(dHA, dDec, dRa);
    if(nErr) {
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startSlewTo nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("startSlewTo ERROR");
        return ERR_CMDFAILED;
    }

    return nErr;
}

int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    nErr = pMe->mAstroTrac.isSlewToComplete(bComplete);

    if(nErr)
        return ERR_CMDFAILED;

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompleteSlewTo %d nErr = %d i\n", timestamp, bComplete, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::endSlewTo(void)
{
    int nErr;
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] endSlewTo Called\n", timestamp);
        fflush(LogFile);
    }
#endif
    
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

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] syncMount Called : %f\t%f\n", timestamp, ra, dec);
        fflush(LogFile);
    }
#endif
    
    nErr = mAstroTrac.syncTo(Ha, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] syncMount nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

bool X2Mount::isSynced(void)
{   // As per documentation, always returns true since the mount does not know if it has been synced
    
    return true;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mAstroTrac.setTrackingRates(bTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] setTrackingRates Called. Tracking On: %s , Ra rate : %f , Dec rate: %f nerr %d\n", timestamp, bTrackingOn?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec, nErr);
        fflush(LogFile);
    }
#endif
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
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingRates Called. Tracking On: %s , Ra rate : %f , Dec rate: %f nerr %d\n", timestamp, bTrackingOn?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::siderealTrackingOn()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] siderealTrackingOn Called \n", timestamp);
        fflush(LogFile);
    }
#endif

    nErr = setTrackingRates( true, true, 0.0, 0.0);
    if(nErr)
        return ERR_CMDFAILED;

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] siderealTrackingOn nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

int X2Mount::trackingOff()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingOff Called \n", timestamp);
        fflush(LogFile);
    }
#endif
    
    nErr = setTrackingRates( false, true, 0.0, 0.0);
    if(nErr)
        nErr = ERR_CMDFAILED;

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingOff nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

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

#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startPark  mAstroTrac.gotoPark nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

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
        
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startUnpark : mAstroTrac.unPark() failled !\n", timestamp);
            fflush(LogFile);
        }
#endif
        
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
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 1
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] knowBeyondThePole called\n", timestamp);
            fflush(LogFile);
        }
#endif
    return true;
}

int X2Mount::beyondThePole(bool& bYes) {
    if(!m_bLinked)
        return ERR_NOLINK;

    bYes = mAstroTrac.GetIsBeyondThePole();
	return SB_OK;
}


double X2Mount::flipHourAngle() {
    
#if defined AstroTrac_X2_DEBUG && AstroTrac_X2_DEBUG >= 2
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		// fprintf(LogFile, "[%s] flipHourAngle called\n", timestamp);
        fflush(LogFile);
	}
#endif

	return 0.0;
}


int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
	dHoursEast = 0.0;
	dHoursWest = TRAC_PAST_MERIDIAN; // Defined in x2mount.h
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




