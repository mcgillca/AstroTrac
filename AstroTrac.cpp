#include "AstroTrac.h"

// Constructor for AstroTrac
AstroTrac::AstroTrac()
{
	m_bIsConnected = false;

    m_bDebugLog = true;
    m_bLimitCached = false;
    
#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\AstroTracLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/AstroTracLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/AstroTracLog.txt";
#endif
	Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::AstroTrac] Version %3.2f build 2020_09_05_1140.\n", timestamp, DRIVER_VERSION);
	fprintf(Logfile, "[%s] AstroTrac New Constructor Called\n", timestamp);
    fflush(Logfile);
#endif

}


AstroTrac::~AstroTrac(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] AstroTrac Destructor Called\n", timestamp );
    fflush(Logfile);
#endif
#ifdef PLUGIN_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int AstroTrac::Connect(char *pszPort)
{
    int nErr = SB_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] AstroTrac::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 115.2K 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::Connect m_mountType %d\n", timestamp, m_mountType);
    fflush(Logfile);
#endif

    
    // Set flat to indicate whether north or south latitude
    m_bNorthernHemisphere = (m_pTsx->latitude() > 0);
    // Set axis direction to AstroTrac
    nErr = AstroTracSendCommand(m_bNorthernHemisphere? "<1d1>" : "<1d-1>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    
    // Set approximate slew offset - either plus or minus 25 arcsecs depending on the hempisphere
    m_dSlewOffset = 25.0/3600.0 * (m_bNorthernHemisphere ? 1.0: -1.0);
    
    // Read the maximum slew velocity
    nErr = AstroTracSendCommand("<1zs?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    
    // Remove the last character (>)
    szResp[strlen(szResp) - 1] = '\0';
    
    // Read max slew rate from 5th character of response
    m_dVSlewMax = atof(szResp+4);
    
    // Get the RA velocity to set the initial tracking rates
    nErr = AstroTracSendCommand("<1v?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    
    // Remove the last character (>)
    szResp[strlen(szResp) - 1] = '\0';

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::Connect RAVel response %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    // Read tracking rate from 4th character of response
    m_dRATrackingRate = atof(szResp+3);
    
    // Now repeat for DEC velocity
    // Get the RA velocity to set the initial tracking rates
    nErr = AstroTracSendCommand("<2v?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    
    // Remove the last character (>)
    szResp[strlen(szResp) - 1] = '\0';
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::Connect DECVel response %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    // Read tracking rate from 4th character of response
    m_dDETrackingRate = atof(szResp+3);
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::Connect RAVel %f DECVel %f\n", timestamp, m_dRATrackingRate, m_dDETrackingRate);
    fflush(Logfile);
#endif
    
    
    // Now tidy up the readings since there is some uncertainty
    // First see if there is any DEC velocity
    if (fabs(m_dDETrackingRate) < 0.001) {
        // Close to zero DEC velocity, so standard tracking or off
        if (fabs(m_dRATrackingRate)< 0.001) {
            nErr = setTrackingRates(false, true, 0.0, 0.0); if (nErr) return ERR_CMDFAILED;
        } else {
            nErr = setTrackingRates(true, true, 0.0, 0.0); if (nErr) return ERR_CMDFAILED;
        }
    } else {
        // Non zero tracking velocity, so liklely to be non-Siderial tracking. Store rates and set tracking on.
        // First, subtract siderial velocity since measured against that
        m_dRATrackingRate -= m_bNorthernHemisphere ? AT_SIDEREAL_SPEED: - AT_SIDEREAL_SPEED;
        m_bTracking = true;
    }

    return nErr;
}


int AstroTrac::Disconnect(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] AstroTrac::Disconnect Called\n", timestamp);
    fflush(Logfile);
#endif
	if (m_bIsConnected) {
        if(m_pSerx){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] AstroTrac::Disconnect closing serial port\n", timestamp);
            fflush(Logfile);
#endif
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
        }
    }
	m_bIsConnected = false;

	return SB_OK;
}




#pragma mark - AstroTrac communication
int AstroTrac::AstroTracSendCommand(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen)
{
    int itries;
    int nErr = PLUGIN_OK;

    *pszResult = 0; // Clear pszResult

    for (itries = 0; itries < MAXSENDTRIES; itries++) {
        nErr = AstroTracSendCommandInnerLoop(pszCmd, pszResult, nResultMaxLen);
        if (nErr == PLUGIN_OK) return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::AstroTracSendCommand itries %d Cmd: %s Result: %s \n", timestamp, itries, pszCmd, pszResult);
    fflush(Logfile);
#endif
    }

    return nErr;

}

int AstroTrac::AstroTracSendCommandInnerLoop(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen)
{
    int nErr = PLUGIN_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] Sending %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    // Always (re)send the command - any stray reply left over from a previous send is caught
    // and discarded below by responseMatchesCommand / the bytesWaitingRx drain.
    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLooop] error %d sending command : %s\n", timestamp, nErr, pszCmd);
            fflush(Logfile);
#endif
        return nErr;
    }

    // read response

    if(pszResult) {
        nErr = AstroTracreadResponse(szResp, SERIAL_BUFFER_SIZE);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] error %d reading response : %s\n", timestamp, nErr, szResp);
            fflush(Logfile);
#endif
            return nErr;
        }

        // Discard any stale reply left over from an earlier command (see responseMatchesCommand)
        // and keep reading until we find the response that actually belongs to pszCmd.
        if (!responseMatchesCommand(pszCmd, szResp)) {
            int nStaleTries;

            for (nStaleTries = 0; nStaleTries < MAX_STALE_RESPONSE_TRIES && !responseMatchesCommand(pszCmd, szResp); nStaleTries++) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] Mismatched response for Cmd: %s -- got stale reply: '%s', discarding (attempt %d/%d)\n",
                        timestamp, pszCmd, szResp, nStaleTries + 1, MAX_STALE_RESPONSE_TRIES);
                fflush(Logfile);
#endif
                nErr = AstroTracreadResponse(szResp, SERIAL_BUFFER_SIZE);
                if (nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] error %d re-reading response while discarding stale reply for Cmd: %s\n",
                            timestamp, nErr, pszCmd);
                    fflush(Logfile);
#endif
                    return nErr;
                }
            }

            if (!responseMatchesCommand(pszCmd, szResp)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] Gave up after %d stale replies, still mismatched for Cmd: %s last reply: '%s'\n",
                        timestamp, MAX_STALE_RESPONSE_TRIES, pszCmd, szResp);
                fflush(Logfile);
#endif
                return PLUGIN_BAD_CMD_RESPONSE;
            }
        }

        // We now have a reply that matches pszCmd, but if this same command was sent more than
        // once (e.g. a repeated poll where an earlier attempt was resent after a read failure),
        // an extra reply to an earlier send may already be sitting in the receive buffer right
        // behind this one - and prefix-matching alone cannot tell it apart from a fresh reply,
        // since it is a reply to the very same command. Only drain what has PROVABLY already
        // arrived (bytesWaitingRx is non-blocking) and keep the most recent one - never wait
        // around on the chance that another reply might still be coming.
        {
            unsigned char szExtra[SERIAL_BUFFER_SIZE];
            int nExtraTries;

            for (nExtraTries = 0; nExtraTries < MAX_STALE_RESPONSE_TRIES; nExtraTries++) {
                int nBytesWaiting = 0;
                int nErrPeek = m_pSerx->bytesWaitingRx(nBytesWaiting);

                if (nErrPeek || nBytesWaiting <= 0)
                    break;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] %d more byte(s) already waiting after matching reply for Cmd: %s -- reading likely duplicate reply (attempt %d/%d)\n",
                        timestamp, nBytesWaiting, pszCmd, nExtraTries + 1, MAX_STALE_RESPONSE_TRIES);
                fflush(Logfile);
#endif
                if (AstroTracreadResponse(szExtra, SERIAL_BUFFER_SIZE) || !responseMatchesCommand(pszCmd, szExtra)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] extra reply for Cmd: %s did not match or failed to read - keeping previous reply: '%s'\n",
                            timestamp, pszCmd, szResp);
                    fflush(Logfile);
#endif
                    break;
                }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] discarding stale duplicate reply: '%s', keeping newer reply: '%s' for Cmd: %s\n",
                        timestamp, szResp, szExtra, pszCmd);
                fflush(Logfile);
#endif
                memcpy(szResp, szExtra, SERIAL_BUFFER_SIZE);
            }
        }

        strncpy(pszResult, (const char *)szResp, nResultMaxLen);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
        // Check that the returned message is good
        // If second letter is e, this indicates an error code
        if (szResp[2] == 'e') {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] Poorly formed reply: '%s'\n", timestamp, szResp);
            fflush(Logfile);
            return PLUGIN_BAD_CMD_RESPONSE;
        }
#endif
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [AstroTrac::AstroTracSendCommandInnerLoop] got response : '%s'\n", timestamp, szResp);
        fflush(Logfile);
#endif
    }
    return nErr;
}


// Every command response echoes back the "<axis><code>" prefix of the command that
// triggered it (e.g. "<1p32.4>" -> "<1p#", "<1zv?>" -> "<1zv2.01"). If a stray reply
// from an earlier command (e.g. left over after a resend) is sitting in the buffer,
// its prefix will belong to that earlier command instead and won't match here.
// Device-reported errors ("<axis>e<code>") are always accepted as a genuine match,
// since they are a real reply to the command just sent, just carrying an error code.
bool AstroTrac::responseMatchesCommand(const char *pszCmd, const unsigned char *pszResp)
{
    size_t i;

    if (!pszCmd || !pszResp)
        return false;

    if (pszCmd[0] != '<' || pszResp[0] != '<' || !isdigit((unsigned char)pszCmd[1]))
        return false;

    if (pszCmd[1] != (char)pszResp[1]) // axis number mismatch
        return false;

    if (pszResp[2] == 'e' && pszCmd[2] != 'e') // genuine device error reply
        return true;

    i = 2;
    while (pszCmd[i] && isalpha((unsigned char)pszCmd[i]))
        i++;

    return strncmp(pszCmd, (const char *)pszResp, i) == 0;
}

int AstroTrac::AstroTracreadResponse(unsigned char *pszRespBuffer, unsigned int nBufferLen)
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
        struct timespec rfStart, rfEnd;
        clock_gettime(CLOCK_MONOTONIC, &rfStart);
#endif
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr || ulBytesRead != 1) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
            clock_gettime(CLOCK_MONOTONIC, &rfEnd);
            double rfElapsed = (rfEnd.tv_sec - rfStart.tv_sec) + (rfEnd.tv_nsec - rfStart.tv_nsec) * 1e-9;
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            if (nErr)
                fprintf(Logfile, "[%s] [AstroTrac::readResponse] readFile error %d after %.3f seconds\n", timestamp, nErr, rfElapsed);
            else
                fprintf(Logfile, "[%s] [AstroTrac::readResponse] readFile timed out (0 bytes received) after %.3f seconds\n", timestamp, rfElapsed);
            fflush(Logfile);
#endif
            if (nErr) return nErr;
        }

 #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [AstroTrac::readResponse] *pszBufPtr = 0x%02X ulBytesRead %d\n", timestamp, *pszBufPtr, ulBytesRead);
        fflush(Logfile);
#endif

        if (ulBytesRead != 1) {// timeout
            nErr = PLUGIN_BAD_CMD_RESPONSE;
	    return nErr;
        }
        ulTotalBytesRead += ulBytesRead;

    } while (*pszBufPtr++ != '>' && ulTotalBytesRead < nBufferLen );


    // Last character should be a '>' - if not send error message
    if (*(pszBufPtr -1) != '>') {
    // Ensure string closed
        if (ulTotalBytesRead < nBufferLen) *pszBufPtr = 0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [AstroTrac::readResponse] No closing bracket: *pszRespBuffer = %s bytes read %lu\n", timestamp, pszRespBuffer, ulTotalBytesRead);
        fflush(Logfile);
#endif
        
        return PLUGIN_BAD_CMD_RESPONSE;
    }
    
    
    if(ulTotalBytesRead && *(pszBufPtr-1) == '>')
        *(pszBufPtr-1) = 0; //remove the > to zero terminate the string

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
       ltime = time(NULL);
       timestamp = asctime(localtime(&ltime));
       timestamp[strlen(timestamp) - 1] = 0;
       fprintf(Logfile, "[%s] [AstroTrac::readResponse] *pszRespBuffer = %s nErr %d\n", timestamp, pszRespBuffer, nErr);
       fflush(Logfile);
#endif

    
    return nErr;
}


#pragma mark - dome controller informations

int AstroTrac::getFirmwareVersion(std::string &sFirmware)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = AstroTracSendCommand("<1zv?>", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // Remove first four characters of reply "<1zv"
    sFirmware.assign(szResp+4);
    m_sFirmwareVersion.assign(szResp+4);
    return nErr;
}

#pragma mark - Mount Coordinates
void AstroTrac::setMountMode(MountTypeInterface::Type mountType)
{
    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [AstroTrac::setMountMode] mountType = %d\n", timestamp, mountType);
        fflush(Logfile);
    #endif
    
    m_mountType = mountType;
}

MountTypeInterface::Type AstroTrac::mountType()
{
    return m_mountType;
}


int AstroTrac::getHaAndDec(double &dHa, double &dDec)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    // get Ha encoder values
    nErr = AstroTracSendCommand("<1p?>", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::getHaAndDec] szResp = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    // Remove closing ">" and convert to float - ignoring first 3 characters
    szResp[strlen(szResp)-1] = '\0';
    
    // RA Encoder measured in degrees - stored in private variable too
    m_dHAEncoder = atof(szResp+3);

    // get DEC encoder values
    nErr = AstroTracSendCommand("<2p?>", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    // Remove closing ">" and convert to float - ignoring first 3 characters
    szResp[strlen(szResp)-1] = '\0';
    
    // Dec Encoder measured in degrees - stored in private variable too
    m_dDecEncoder = atof(szResp+3);
    
    // Now convert encoder values to HA and DEC
    HAandDECfromEncoderValues(m_dHAEncoder, m_dDecEncoder, dHa, dDec);
    
    //Set flag to indicate if beyond the pole
    // If this is an asymettrical mount, set flag to indicate if beyond the pole:
    if (m_mountType == MountTypeInterface::Asymmetrical_Equatorial) {
        m_bIsBTP = m_bNorthernHemisphere ? (m_dDecEncoder  > 0): (m_dDecEncoder < 0);
    } else {
        m_bIsBTP = false;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::getHaAndDec] RAEncoder %f DEencoder %f Ha %f Dec %f BTP %d\n", timestamp, m_dHAEncoder, m_dDecEncoder, dHa, dDec, m_bIsBTP);
    fflush(Logfile);
#endif
    return nErr;
}
void AstroTrac::EncoderValuesfromHAanDEC(double dHa, double dDec, double &HAEncoder, double &DEEncoder, bool bUseBTP)
{
    // For symmetrical mount, use pre-merdian positioning.
    // If bUseBTP is true, take value from current state of BTP to determine which side of the merdian we are on - only used to sync the mount
    if (m_bNorthernHemisphere) {
        if (m_mountType == MountTypeInterface::Symmetrical_Equatorial || (bUseBTP && m_bIsBTP) || (!bUseBTP && dHa < 0.0)) {
            DEEncoder = - (dDec - 90.0);
            HAEncoder = + (dHa + 6.0) * 360.0 / 24.0;
        }
        else {    // Post-Meridian
            DEEncoder = + (dDec - 90.0);
            HAEncoder = + (dHa - 6.0) * 360.0 / 24.0;
        }
    }
    else {
        if (m_mountType == MountTypeInterface::Symmetrical_Equatorial || (bUseBTP && m_bIsBTP) || (!bUseBTP && dHa < 0.0)) {
            DEEncoder = - (dDec + 90.0);
            HAEncoder = - (dHa + 6.0) * 360.0 / 24.0;
        }
        else {    // Post-Meridian
            DEEncoder = + (dDec + 90.0);
            HAEncoder = - (dHa - 6.0) * 360 / 24.0;
        }
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::EncodervaluefromHAandDec (asymetrical) called %f %f %f %f bUseBTP %d IsBeyondThePole %d\n", timestamp, HAEncoder, DEEncoder,  dHa, dDec, bUseBTP, m_bIsBTP);
    fflush(Logfile);
#endif
        

}

void AstroTrac::HAandDECfromEncoderValues(double RAEncoder, double DEEncoder, double &dHa, double &dDec)
{
    // Convert from encoder values
    // For symmetrical mount, use pre-merdian positioning.
    if (m_bNorthernHemisphere) {
        if (m_mountType == MountTypeInterface::Symmetrical_Equatorial || DEEncoder > 0.0) {      //  Pre-meridian
            // Constrain to maximum of 90 for cases where DEEncoder is slightly positive in Symmetrical case
            dDec = std::min(90.0 - DEEncoder, 90.0);
            dHa = -6.0 + RAEncoder / 360.0 * 24.0;
        }
        else {
            dDec = 90.0 + DEEncoder;
            dHa = 6.0 + RAEncoder /360.0 * 24.0;
        }
    }
    else {
        if (m_mountType == MountTypeInterface::Symmetrical_Equatorial || DEEncoder < 0.0) {      //  Pre-meridian
            // Constrain to minimum of -90 for cases where DEEncoder is slightly positive in Symmetrical case
            dDec = std::max(-90.0 - DEEncoder, -90.0);
            dHa = -6.0 - RAEncoder / 360.0 * 24.0;
        }
        else {
            dDec = -90.0 + DEEncoder;
            dHa = 6.0 - RAEncoder / 360.0 * 24.0;
        }
        
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] AstroTrac::HAandDecfromEncodervalue called %f %f %f %f IsBeyondThePole %d\n", timestamp, RAEncoder, DEEncoder,  dHa, dDec, m_bIsBTP);
    fflush(Logfile);
#endif
}



#pragma mark - Sync and Cal
int AstroTrac::syncTo(double dHa, double dDec)
{
    int nErr = PLUGIN_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    double SyncHAEncoderValue;
    double SyncDECEncoderValue;

    // Convert dHA and dDec to encoder values
    // Note have set use beyond the pole as true - not used at the moment.
    EncoderValuesfromHAanDEC(dHa, dDec, SyncHAEncoderValue, SyncDECEncoderValue, true);
    
    // Set mount values to the Syncencoder values
    snprintf(szCmd, sizeof(szCmd), "<1y%f>", SyncHAEncoderValue);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;
    
    // Set mount values to the Syncencoder values
    snprintf(szCmd, sizeof(szCmd), "<2y%f>", SyncDECEncoderValue);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;
    
    return nErr;
}

int AstroTrac::isAligned(bool &bAligned)
{
    int nErr = PLUGIN_OK;
    return nErr;
}

#pragma mark - tracking rates
int AstroTrac::setTrackingRates(const bool bTrackingOn, const bool bIgnoreRates, const double dRaRateArcSecPerSec, const double dDecRateArcSecPerSec)
{
    // dRaRateArcSecPerSec and dDecRateArcSecPerSec contain the rate of change of RA and DEC of the object in arcsec/sec.
    // To turn this into tracking rates:
    // RA:    since HA = LST - RA, must subtract this from Sidereal speed
    //        If in Southern hemisphere, rates are negative.
    // DEC: If pre-meridian, increasing DEC means increasing steps, so sign should be the same as dDecRateArcSecPerSec.
    //        If post-medidian, increasing DEC decreases steps, so this should be the opposite sign.
    // We are pre-meridian if mbIsBTP is true, otherwise post-meridian
    
    int nErr = PLUGIN_OK;
    double RARate, DECRate;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
   
    //Set flag to indicate if beyond the pole


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [AstroTrac::setTrackingRates] Tracking on: %d, Ignorrates %d, dRARate %f dDECrate %f\n", timestamp, bTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
        fflush(Logfile);
#endif
    
    if (bTrackingOn) { // set tracking
        if (bIgnoreRates) { // No movement in DEC and siderial for RA
            RARate = m_bNorthernHemisphere ? AT_SIDEREAL_SPEED : -AT_SIDEREAL_SPEED;
            DECRate = 0.0;
            // Now save tracking rates for TSX interface - no difference from Siderial speed since rates ignored
            m_bTracking = true;
            m_dRATrackingRate = 0.0;
            m_dDETrackingRate = 0.0;
        }
        else {
            if (m_bNorthernHemisphere) {
                RARate = AT_SIDEREAL_SPEED - dRaRateArcSecPerSec;
                DECRate = (m_mountType == MountTypeInterface::Symmetrical_Equatorial || m_bIsBTP) ? -dDecRateArcSecPerSec : dDecRateArcSecPerSec;
            }
            else {
                RARate = -(AT_SIDEREAL_SPEED - dRaRateArcSecPerSec);
                DECRate = (m_mountType == MountTypeInterface::Symmetrical_Equatorial ||m_bIsBTP) ? -dDecRateArcSecPerSec : dDecRateArcSecPerSec;
            }
            // Now save tracking rates for TSX interface - must capture rates
            m_bTracking = true;
            m_dRATrackingRate = dRaRateArcSecPerSec;
            m_dDETrackingRate = dDecRateArcSecPerSec;
        }
    }
    else {
        // Tracking is off
        RARate = 0.0;
        DECRate = 0.0;
        m_bTracking = false;
        m_dRATrackingRate = 15.0410681; // Convention to say tracking is off - see TSX documentation
        m_dDETrackingRate = 0.0;
    }
    
    // Send set velocity commands to the RA and DEC axes - use the ve variant to ensure encoder is turned on
    snprintf(szCmd, sizeof(szCmd), "<1ve%f>", RARate);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;

    snprintf(szCmd, sizeof(szCmd), "<2ve%f>", DECRate);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::setTrackingRates] Tracking on: %d, Ignorrates %d, RARate %f DECrate %f\n", timestamp, bTrackingOn, bIgnoreRates, RARate, DECRate);
    fflush(Logfile);
#endif
    
    return nErr;
}

int AstroTrac::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecperSec, double &dTrackDecArcSecPerSec)
{
    // Simply report previously stored tracking variables
    bTrackingOn = m_bTracking;
    dTrackRaArcSecperSec = m_dRATrackingRate;
    dTrackDecArcSecPerSec  = m_dDETrackingRate;
    
    return PLUGIN_OK;
}



#pragma mark - Slew

// Function to estimate time to slew - distance in degrees
double AstroTrac::slewTime(double dDist)
{
    double tslew;  // Estimate of time for slew
    double accelndist; // Estimate of distance covered by acceleration and deceleration period
    
    // Firstly throw away sign of distance - don't care about direction - and convert to arcsec
    dDist = fabs(dDist) * 3600.0;
    
    // Now estimate how far mount travels during accelertion and deceleration period
    accelndist = m_dVSlewMax * m_dVSlewMax / m_dAslew;
    
    // If distance less than this, then calulate using accleration forumlae:
    if (dDist < accelndist) {
        tslew = 2 * sqrt(dDist/m_dAslew);
    } else {
        // Time is equal to twice the time required to accelerate or decelerate, plus the remaining distance at max slew speed
        tslew = 2.0 * m_dVSlewMax/m_dAslew + (dDist-accelndist)/m_dVSlewMax;
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] slewTime: dDist %f Time %f\n", timestamp, dDist, tslew);
        fflush(Logfile);
#endif
    return tslew;
}
        
    
int AstroTrac::startSlewTo(double dHa, double dDec, double dRa)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char out[SERIAL_BUFFER_SIZE];
    double HAEncoder;
    double DEEncoder;
    bool bUseBTP = false;
    double tHa; // Time to slew in HA direction
    
    // Reset slewing aborted flag
    m_bSlewingAborted = false;
    
    // Store slew target for use later
    m_dGotoRATarget = dRa;

    // Convert dHA and dDec to encoder positions
    EncoderValuesfromHAanDEC(dHa, dDec, HAEncoder, DEEncoder, bUseBTP);

    // Calculate time required to slew for each axis:
    tHa = slewTime(HAEncoder - m_dHAEncoder);
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] startSlewTo: dHa: %f HAEncoder %f, m_dHAEncoder %f, tHA %f\n", timestamp, dHa, HAEncoder, m_dHAEncoder, tHa);
        fflush(Logfile);
#endif
    
    // Formulate and send command to set acceleration to use during the slew - can get changed during guiding
    snprintf(out, sizeof(out), "<1a%d>",  (int) m_dAslew);
    nErr = AstroTracSendCommand(out, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;
    
    // Formulate and send command to slew for RA axis - adding on time it takes to slew in the RA axis (remember to convert from arcsec to degrees)
    // m_dSlewOffset is initially zero, then set to the difference between actual and target RA after the slew has completed
    
    snprintf(out, sizeof(out), "<1p%f>", HAEncoder + m_dSlewOffset + (m_bNorthernHemisphere ? 1.0: -1.0) * tHa * AT_SIDEREAL_SPEED/3600.0);
    
    nErr = AstroTracSendCommand(out, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;
 
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] startSlewTo: dHa: %f command: %s response %s\n", timestamp, dHa, out, szResp);
        fflush(Logfile);
#endif
    
    // Formulate and send command to set acceleration to use during the slew - can get changed during guiding
    snprintf(out, sizeof(out), "<2a%d>",  (int) m_dAslew);
    
    nErr = AstroTracSendCommand(out, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;
    // Formulate and send command to slew for DEC axis
    snprintf(out, sizeof(out), "<2p%f>", DEEncoder);
    nErr = AstroTracSendCommand(out, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] startSlewTo: dDec: %f command: %s response %s\n", timestamp, dDec, out, szResp);
        fflush(Logfile);
#endif

    return nErr;
}

//  At end of slew, work out how far away the slew was from the target. Save for next time to allow better slew
int AstroTrac::endSlewTo(){
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    double dHa;
    double dDec = 0.0;
    double dHAEncoder, dDecEncoder;
    
    // Was slewing aborted? If so, just return so don't capture slew offset
    if (m_bSlewingAborted) return PLUGIN_OK;
    
    // Firstly, get current Ha encoder value
    nErr = AstroTracSendCommand("<1p?>", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    // Remove closing ">" and convert to float - ignoring first 3 characters
    szResp[strlen(szResp)-1] = '\0';
    
    // RA Encoder measured in degrees - stored in private variable
    m_dHAEncoder = atof(szResp+3);
    
    // Calculate current HA of slew target
    dHa = m_pTsx->hourAngle(m_dGotoRATarget);
    
    // Calculate Encoder Values
    EncoderValuesfromHAanDEC(dHa, dDec, dHAEncoder, dDecEncoder, false);
    
    // Add difference between the two as additional offset. The offset has already been used to get this close
    // Only do this if the difference is less than 100" - otherwise likely to be some sort of error
    if (fabs(dHAEncoder - m_dHAEncoder) * 3600.0 < 100) m_dSlewOffset += dHAEncoder - m_dHAEncoder;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] endSlewTo: m_dHAEncoder %f Target Encoder %f Offset %f\"\n", timestamp, m_dHAEncoder, dHAEncoder, m_dSlewOffset*3600);
        fflush(Logfile);
#endif
    
    return nErr;
    
}

int AstroTrac::isSlewToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bComplete = false;

    // Is RA drive finished slew?
    nErr = AstroTracSendCommand("<1t?>", szResp, SERIAL_BUFFER_SIZE);    if(nErr) return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::isSlewToComplete] RA szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    // Third character tells whether is still slewing - 1 yes, 0 no
    if (szResp[3] == '1') return nErr;
    
    // RA drive has finished slew - try DEC drive
    nErr = AstroTracSendCommand("<2t?>", szResp, SERIAL_BUFFER_SIZE);    if(nErr) return nErr;
    // Third character tells whether is still slewing - 1 yes, 0 no
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::isSlewToComplete] DEC szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    if (szResp[3] == '1') return nErr;
    
    // Both drives indicate slew has finished so flag bComplete to be true.
    bComplete = true;

    return nErr;
}

int AstroTrac::getNbSlewRates()
{
    return (int)m_dvSlewRates.size();
}

// returns rate name from lit in Astrotrac.h

int AstroTrac::getRateName(int nZeroBasedIndex, std::string &sOut)
{
    if (nZeroBasedIndex < 0 || nZeroBasedIndex >= (int)m_svSlewRateNames.size())
        return PLUGIN_ERROR;

    sOut.assign(m_svSlewRateNames[nZeroBasedIndex]);
    return PLUGIN_OK;
}


int AstroTrac::startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate)
{
    int nErr = PLUGIN_OK;
    double rate;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::startOpenSlew] setting to Dir %d\n", timestamp, Dir);
    fprintf(Logfile, "[%s] [AstroTrac::startOpenSlew] Setting rate to %d\n", timestamp, nRate);
    fflush(Logfile);
#endif

    // select rate
    m_nOpenLoopDir = Dir;
    rate = m_dvSlewRates[nRate] * AT_SIDEREAL_SPEED;
    
    // figure out direction
    switch(Dir){
            // Easy for DEC move - just positive or negative rate
        case MountDriverInterface::MD_NORTH:
            snprintf(szCmd, sizeof(szCmd), "<2v%f>", -rate);
            break;
        case MountDriverInterface::MD_SOUTH:
            snprintf(szCmd, sizeof(szCmd), "<2v%f>", rate);
            break;
            // Harder for RA move - must be with reference to the tracking speed. Sign of tracking speed depends on hemisphere
            // Work out tracking speed, then add or subtract move rate to get resulting rate to move mount at.
        case MountDriverInterface::MD_EAST:
            snprintf(szCmd, sizeof(szCmd), "<1v%f>", (m_bNorthernHemisphere ? + AT_SIDEREAL_SPEED : - AT_SIDEREAL_SPEED) + rate);
            break;
        case MountDriverInterface::MD_WEST:
            snprintf(szCmd, sizeof(szCmd), "<1v%f>", (m_bNorthernHemisphere ? + AT_SIDEREAL_SPEED : - AT_SIDEREAL_SPEED) - rate);
            break;
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    struct timespec cmdStart, cmdEnd;
    clock_gettime(CLOCK_MONOTONIC, &cmdStart);
#endif
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    clock_gettime(CLOCK_MONOTONIC, &cmdEnd);
    double cmdElapsed = (cmdEnd.tv_sec - cmdStart.tv_sec) + (cmdEnd.tv_nsec - cmdStart.tv_nsec) * 1e-9;
    fprintf(Logfile, "[%s] [AstroTrac::startOpenLoopMove] AstroTracSendCommand took %.3f seconds, nErr = %d\n", timestamp, cmdElapsed, nErr);
    fflush(Logfile);
#endif
    
    // Start timer to measure open loop slew duration for the appropriate axis
    if (Dir == MountDriverInterface::MD_NORTH || Dir == MountDriverInterface::MD_SOUTH) {
        m_bOpenLoopDEC = true;
        clock_gettime(CLOCK_MONOTONIC, &m_OpenLoopStartTimeDEC);
    } else {
        m_bOpenLoopRA = true;
        clock_gettime(CLOCK_MONOTONIC, &m_OpenLoopStartTimeRA);
    }
    
    return nErr;
}

int AstroTrac::stopOpenLoopMove()
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (m_bOpenLoopRA) {
        double elapsedRA = (now.tv_sec - m_OpenLoopStartTimeRA.tv_sec) + (now.tv_nsec - m_OpenLoopStartTimeRA.tv_nsec) * 1e-9;
        fprintf(Logfile, "[%s] [AstroTrac::stopOpenLoopMove] RA (East/West) duration %.3f seconds\n", timestamp, elapsedRA);
    }
    if (m_bOpenLoopDEC) {
        double elapsedDEC = (now.tv_sec - m_OpenLoopStartTimeDEC.tv_sec) + (now.tv_nsec - m_OpenLoopStartTimeDEC.tv_nsec) * 1e-9;
        fprintf(Logfile, "[%s] [AstroTrac::stopOpenLoopMove] DEC (North/South) duration %.3f seconds\n", timestamp, elapsedDEC);
    }
    fflush(Logfile);
#endif
    m_bOpenLoopRA = false;
    m_bOpenLoopDEC = false;
    // Set tracking on to end slew
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    struct timespec cmdStart, cmdEnd;
    clock_gettime(CLOCK_MONOTONIC, &cmdStart);
#endif
    nErr = setTrackingRates(true, true, 0.0, 0.0);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
    clock_gettime(CLOCK_MONOTONIC, &cmdEnd);
    double cmdElapsed = (cmdEnd.tv_sec - cmdStart.tv_sec) + (cmdEnd.tv_nsec - cmdStart.tv_nsec) * 1e-9;
    fprintf(Logfile, "[%s] [AstroTrac::stopOpenLoopMove] setTrackingRates took %.3f seconds, nErr = %d\n", timestamp, cmdElapsed, nErr);
    fflush(Logfile);
#endif

    return nErr;
}


int AstroTrac::gotoPark(double dHa, double dDec)
{
    // Sends to park postion - Ha and Dec are set to 0.0 position so ignores any value sent by the driver
    char szResp[SERIAL_BUFFER_SIZE];
    int nErr;
    
    // Reset abort flag
    m_bSlewingAborted = false;
    
    // Turn tracking off - remembered by drive after slew
    nErr = setTrackingRates(false, true, 0.0, 0.0); if (nErr) return COMMAND_FAILED;
    
    nErr = AstroTracSendCommand("<1p0.0>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;
    nErr = AstroTracSendCommand("<2p0.0>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;
    
    // Flag the parking is in progress
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [AstroTrac::gotoPark] Called!\n", timestamp);
    fflush(Logfile);
#endif
    
    m_bParkingInProgress = true;
    return nErr;
}

int AstroTrac::GetIsParkingComplete(bool &bComplete){
    int nErr = SB_OK;
    
    // If no parking in progress then complete
    if (!m_bParkingInProgress) {
        bComplete = true;
        return nErr;
    }
    
    // Else, see if slewing has finished
    nErr = isSlewToComplete(bComplete);     if (nErr) return COMMAND_FAILED;
    
    // If complete, mark that parking is finished.
    // Unless slewing was aborted, also set m_bisParked to true i.e. to opposite of is m_bSlewingAborted
    if (bComplete) {
        m_bParkingInProgress = false;
        m_bisParked = !m_bSlewingAborted;
    }
    
    return nErr;
}

int AstroTrac::Abort()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = AstroTracSendCommand("<1x>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;
    nErr = AstroTracSendCommand("<2x>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;
    
    // Set flag to say that slew was aborted
    m_bSlewingAborted = true;
    
    return nErr;
}

#pragma mark - time and site methods

