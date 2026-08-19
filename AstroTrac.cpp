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

    LogDebug(3, "[AstroTrac::AstroTrac] Version %3.2f build 2020_09_05_1140.\n", DRIVER_VERSION);
    LogDebug(3, "AstroTrac New Constructor Called\n");

}


AstroTrac::~AstroTrac(void)
{
    LogDebug(3, "AstroTrac Destructor Called\n");
#ifdef PLUGIN_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int AstroTrac::Connect(char *pszPort)
{
    int nErr = SB_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    LogDebug(3, "AstroTrac::Connect Called %s\n", pszPort);

    // 115.2K 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    LogDebug(3, "AstroTrac::Connect m_mountType %d\n", m_mountType);

    // Set flat to indicate whether north or south latitude
    m_bNorthernHemisphere = (m_pTsx->latitude() > 0);
    // Set axis direction to AstroTrac
    nErr = AstroTracSendCommand(m_bNorthernHemisphere? "<1d1>" : "<1d-1>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    
    // Set approximate slew offset - either plus or minus 25 arcsecs depending on the hempisphere
    m_dSlewOffset = 25.0/3600.0 * (m_bNorthernHemisphere ? 1.0: -1.0);
    
    // Read the maximum slew velocity
    nErr = AstroTracSendCommand("<1zs?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;

    // Read max slew rate from 5th character of response
    m_dVSlewMax = atof(szResp+4);

    // Read the per-axis slew acceleration already configured on the mount. We only ever read
    // this - startSlewTo no longer writes an acceleration value back to the device - so a user's
    // own setting (e.g. tuned for a heavier rig) is left alone; we just use it to estimate slew
    // duration.
    nErr = AstroTracSendCommand("<1a?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    m_dAslewRA = atof(szResp+3);

    nErr = AstroTracSendCommand("<2a?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;
    m_dAslewDEC = atof(szResp+3);

    // Get the RA velocity to set the initial tracking rates
    nErr = AstroTracSendCommand("<1v?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;

    LogDebug(3, "AstroTrac::Connect RAVel response %s\n", szResp);

    // Read tracking rate from 4th character of response
    m_dRATrackingRate = atof(szResp+3);
    
    // Now repeat for DEC velocity
    // Get the RA velocity to set the initial tracking rates
    nErr = AstroTracSendCommand("<2v?>", szResp, SERIAL_BUFFER_SIZE); if (nErr) return ERR_CMDFAILED;

    LogDebug(3, "AstroTrac::Connect DECVel response %s\n", szResp);

    // Read tracking rate from 4th character of response
    m_dDETrackingRate = atof(szResp+3);

    LogDebug(3, "AstroTrac::Connect RAVel %f DECVel %f\n", m_dRATrackingRate, m_dDETrackingRate);


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
    LogDebug(3, "AstroTrac::Disconnect Called\n");
	if (m_bIsConnected) {
        if(m_pSerx){
            LogDebug(3, "AstroTrac::Disconnect closing serial port\n");
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
        }
    }
	m_bIsConnected = false;

	return SB_OK;
}




#pragma mark - AstroTrac communication

// Write a single debug log line if PLUGIN_DEBUG is defined and at least nLevel, otherwise a no-op.
// Centralizes the timestamp/fprintf/fflush boilerplate that used to be repeated at every log site.
void AstroTrac::LogDebug(int nLevel, const char *pszFormat, ...)
{
#ifdef PLUGIN_DEBUG
    if (nLevel > PLUGIN_DEBUG)
        return;

    va_list args;

    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] ", timestamp);

    va_start(args, pszFormat);
    vfprintf(Logfile, pszFormat, args);
    va_end(args);

    fflush(Logfile);
#endif
}

// Length of the "<axis><code>" prefix of a command string, e.g. 3 for "<1p...", 4 for
// "<1zv...". Assumes pszCmd starts with '<' followed by an axis digit.
static size_t CommandPrefixLen(const char *pszCmd)
{
    size_t i = 2;
    while (pszCmd[i] && isalpha((unsigned char)pszCmd[i]))
        i++;
    return i;
}

int AstroTrac::AstroTracSendCommand(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen)
{
    int itries;
    int nErr = PLUGIN_OK;
    struct timespec cmdStart, cmdNow;
    clock_gettime(CLOCK_MONOTONIC, &cmdStart);

    *pszResult = 0; // Clear pszResult

    for (itries = 0; itries < MAXSENDTRIES; itries++) {
        nErr = AstroTracSendCommandInnerLoop(pszCmd, pszResult, nResultMaxLen, itries > 0);
        if (nErr == PLUGIN_OK) {
            // Only log the ones that needed a retry - this is the total time from the first send
            // attempt to a working reply, which is what a longer read timeout would need to beat.
            if (itries > 0) {
                clock_gettime(CLOCK_MONOTONIC, &cmdNow);
                double cmdElapsed = (cmdNow.tv_sec - cmdStart.tv_sec) + (cmdNow.tv_nsec - cmdStart.tv_nsec) * 1e-9;
                LogDebug(1, "AstroTrac::AstroTracSendCommand Cmd: %s succeeded after %d retries, %.3f seconds total\n",
                         pszCmd, itries, cmdElapsed);
            }
            return nErr;
        }

        LogDebug(2, "AstroTrac::AstroTracSendCommand itries %d Cmd: %s Result: %s \n", itries, pszCmd, pszResult);
    }

    clock_gettime(CLOCK_MONOTONIC, &cmdNow);
    {
        double cmdElapsed = (cmdNow.tv_sec - cmdStart.tv_sec) + (cmdNow.tv_nsec - cmdStart.tv_nsec) * 1e-9;
        LogDebug(1, "AstroTrac::AstroTracSendCommand Cmd: %s FAILED after %d tries, %.3f seconds total\n",
                 pszCmd, itries, cmdElapsed);
    }

    return nErr;

}

int AstroTrac::AstroTracSendCommandInnerLoop(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen, bool bIsRetry)
{
    int nErr = PLUGIN_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    // Decide whether to purge and (re)send, or read directly (see PreparePortForSend for why).
    bool bSkipResend = PreparePortForSend(pszCmd, bIsRetry);

    // Only write if PreparePortForSend didn't tell us a reply is already waiting to be read.
    if (!bSkipResend) {
        nErr = WriteCommand(pszCmd);
        if (nErr)
            return nErr;
    }

    // Caller doesn't want the reply text (e.g. fire-and-forget) - nothing left to do.
    if (!pszResult)
        return nErr;

    // Read the framed "<...>" response for this command.
    nErr = AstroTracreadResponse(szResp, SERIAL_BUFFER_SIZE);
    if (nErr) {
        LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] error %d reading response : %s\n", nErr, szResp);
        return nErr;
    }

    // Discard any stale reply left over from an earlier, different command before trusting this one.
    nErr = DiscardStaleReplies(pszCmd, szResp, SERIAL_BUFFER_SIZE);
    if (nErr)
        return nErr;

    // If this was a retry, an extra reply to the earlier attempt may still be arriving - keep the
    // freshest one (see DrainDuplicateReplies for why prefix-matching alone can't catch this case).
    DrainDuplicateReplies(pszCmd, szResp, SERIAL_BUFFER_SIZE, bIsRetry);

    strncpy(pszResult, (const char *)szResp, nResultMaxLen);

    // Check that the returned message is good - if second letter is e, this indicates an error code.
    // This check must always run, not just under PLUGIN_DEBUG - only the log line is diagnostic.
    if (szResp[2] == 'e') {
        LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] Poorly formed reply: '%s'\n", szResp);
        return PLUGIN_BAD_CMD_RESPONSE;
    }

    LogDebug(2, "[AstroTrac::AstroTracSendCommandInnerLoop] got response : '%s'\n", szResp);

    return nErr;
}

// Decide whether to purge and (re)send pszCmd, or skip both and let the caller read directly. On a
// retry, a reply already sitting in the buffer is very likely the delayed answer to the ORIGINAL
// send finally arriving - purging it and resending would throw away a valid reply and pay for an
// entirely unnecessary extra round trip. On a first attempt (not a retry), anything already waiting
// must be a leftover from an earlier, different command - never a valid answer to what we're about
// to send - so purge and send as normal.
bool AstroTrac::PreparePortForSend(const char *pszCmd, bool bIsRetry)
{
    int nBytesBeforePurge = 0, nBytesAfterPurge = 0;

    m_pSerx->bytesWaitingRx(nBytesBeforePurge);

    bool bSkipResend = bIsRetry && nBytesBeforePurge > 0;

    if (!bSkipResend) {
        m_pSerx->purgeTxRx();
        m_pSerx->bytesWaitingRx(nBytesAfterPurge);
    }

    if (nBytesBeforePurge > 0 || nBytesAfterPurge > 0) {
        if (bSkipResend)
            LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] %d byte(s) already waiting for Cmd: %s -- reading without resending\n",
                     nBytesBeforePurge, pszCmd);
        else
            LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] purgeTxRx for Cmd: %s -- bytesWaitingRx before: %d, after: %d\n",
                     pszCmd, nBytesBeforePurge, nBytesAfterPurge);
    }

    return bSkipResend;
}

// Write pszCmd to the port. Any stray reply left over from a previous send is caught and discarded
// by DiscardStaleReplies / DrainDuplicateReplies once the caller gets to reading.
int AstroTrac::WriteCommand(const char *pszCmd)
{
    int nErr;
    unsigned long ulBytesWrite;

    LogDebug(2, "[AstroTrac::AstroTracSendCommandInnerLoop] Sending %s\n", pszCmd);

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();

    if (nErr)
        LogDebug(2, "[AstroTrac::AstroTracSendCommandInnerLooop] error %d sending command : %s\n", nErr, pszCmd);

    return nErr;
}

// Keep reading until pszResp matches pszCmd (see responseMatchesCommand), or give up after
// MAX_STALE_RESPONSE_TRIES - handles a stray reply left over from an earlier, different command
// still being in the pipe when we start reading this one's response.
int AstroTrac::DiscardStaleReplies(const char *pszCmd, unsigned char *pszResp, unsigned int nBufLen)
{
    int nStaleTries;
    int nErr = PLUGIN_OK;

    for (nStaleTries = 0; nStaleTries < MAX_STALE_RESPONSE_TRIES && !responseMatchesCommand(pszCmd, pszResp); nStaleTries++) {
        LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] Mismatched response for Cmd: %s -- got stale reply: '%s', discarding (attempt %d/%d)\n",
                 pszCmd, pszResp, nStaleTries + 1, MAX_STALE_RESPONSE_TRIES);

        nErr = AstroTracreadResponse(pszResp, nBufLen);
        if (nErr) {
            LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] error %d re-reading response while discarding stale reply for Cmd: %s\n",
                     nErr, pszCmd);
            return nErr;
        }
    }

    if (!responseMatchesCommand(pszCmd, pszResp)) {
        LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] Gave up after %d stale replies, still mismatched for Cmd: %s last reply: '%s'\n",
                 MAX_STALE_RESPONSE_TRIES, pszCmd, pszResp);
        return PLUGIN_BAD_CMD_RESPONSE;
    }

    return PLUGIN_OK;
}

// If bIsRetry, an extra reply to the earlier, resent attempt may still be sitting in, or arriving
// into, the receive buffer, and prefix-matching alone cannot tell it apart from pszResp, since it
// is a reply to the very same command. Only worth checking when a resend has actually happened; a
// first-try success never created an extra reply to worry about.
//
// waitForBytesRx(1, EXTRA_REPLY_WAIT_MS) was tried here previously in place of a fixed sleep+peek,
// to give a genuinely in-flight duplicate a short bounded window to land, and measured (before the
// x2mount.cpp mutex-nesting fixes) to not honor the requested timeout - it returned instantly when
// data was already there, but took ~0.9s regardless of the requested value otherwise, same as
// readFile's MAX_TIMEOUT. Re-trying it now that the reentrant-lock paths are fixed, with the wait
// itself timed and logged so the two can be compared directly; bytesWaitingRx() below is kept as
// the actual source of truth regardless of what waitForBytesRx's own return code reports.
void AstroTrac::DrainDuplicateReplies(const char *pszCmd, unsigned char *pszResp, unsigned int nBufLen, bool bIsRetry)
{
    unsigned char szExtra[SERIAL_BUFFER_SIZE];
    int nExtraTries;

    if (!bIsRetry)
        return;

    for (nExtraTries = 0; nExtraTries < MAX_STALE_RESPONSE_TRIES; nExtraTries++) {
        int nBytesWaiting = 0;
        int nErrPeek;
        int nErrWait;
        struct timespec wStart, wEnd;

        clock_gettime(CLOCK_MONOTONIC, &wStart);
        nErrWait = m_pSerx->waitForBytesRx(1, EXTRA_REPLY_WAIT_MS);
        clock_gettime(CLOCK_MONOTONIC, &wEnd);
        {
            double wElapsed = (wEnd.tv_sec - wStart.tv_sec) + (wEnd.tv_nsec - wStart.tv_nsec) * 1e-9;
            LogDebug(1, "[AstroTrac::DrainDuplicateReplies] waitForBytesRx(1, %dms) returned %d after %.3f seconds\n",
                     EXTRA_REPLY_WAIT_MS, nErrWait, wElapsed);
        }

        nErrPeek = m_pSerx->bytesWaitingRx(nBytesWaiting);

        if (nErrPeek || nBytesWaiting <= 0)
            break;

        LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] %d more byte(s) already waiting after matching reply for Cmd: %s -- reading likely duplicate reply (attempt %d/%d)\n",
                 nBytesWaiting, pszCmd, nExtraTries + 1, MAX_STALE_RESPONSE_TRIES);

        if (AstroTracreadResponse(szExtra, nBufLen) || !responseMatchesCommand(pszCmd, szExtra)) {
            LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] extra reply for Cmd: %s did not match or failed to read - keeping previous reply: '%s'\n",
                     pszCmd, pszResp);
            break;
        }

        LogDebug(1, "[AstroTrac::AstroTracSendCommandInnerLoop] discarding stale duplicate reply: '%s', keeping newer reply: '%s' for Cmd: %s\n",
                 pszResp, szExtra, pszCmd);

        memcpy(pszResp, szExtra, nBufLen);
    }
}


// Every command response echoes back the "<axis><code>" prefix of the command that
// triggered it (e.g. "<1p32.4>" -> "<1p#", "<1zv?>" -> "<1zv2.01"). If a stray reply
// from an earlier command (e.g. left over after a resend) is sitting in the buffer,
// its prefix will belong to that earlier command instead and won't match here.
// Device-reported errors ("<axis>e<code>") are always accepted as a genuine match,
// since they are a real reply to the command just sent, just carrying an error code.
// Some set/action command acks come back with the '#' and the code letters swapped
// versus what the manual documents, e.g. "<1a3600>" ("Set Acceleration") is
// acknowledged as "<1#a>" rather than "<1a#>" - accept that ordering too.
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

    i = CommandPrefixLen(pszCmd);

    if (strncmp(pszCmd, (const char *)pszResp, i) == 0)
        return true;

    if (pszResp[2] == '#' && strncmp(pszCmd + 2, (const char *)pszResp + 3, i - 2) == 0)
        return true;

    return false;
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
        int nErrWait;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
        struct timespec rfStart, rfEnd;
        clock_gettime(CLOCK_MONOTONIC, &rfStart);
#endif
        // Wait for the byte to actually be there before asking readFile for it, instead of relying
        // on readFile's own timeout - testing whether this behaves differently now that the
        // x2mount.cpp mutex-nesting fixes are in (see DrainDuplicateReplies for the earlier,
        // pre-fix measurement of waitForBytesRx not honoring its requested timeout). waitForBytesRx
        // doesn't deliver the byte itself, so a readFile call still follows once it reports ready.
        nErrWait = m_pSerx->waitForBytesRx(1, MAX_TIMEOUT);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
        {
            clock_gettime(CLOCK_MONOTONIC, &rfEnd);
            double waitElapsed = (rfEnd.tv_sec - rfStart.tv_sec) + (rfEnd.tv_nsec - rfStart.tv_nsec) * 1e-9;
            // Only log a genuine failure - unlike readFile's silent "0 bytes, no error" timeout,
            // waitForBytesRx reliably reports a real error code here (e.g. 209 = ERR_RXTIMEOUT), so
            // that alone is a clean signal; no need for an elapsed-time heuristic on top of it.
            if (nErrWait)
                LogDebug(1, "[AstroTrac::readResponse] waitForBytesRx(1, %dms) returned %d after %.3f seconds (had %lu byte(s) so far: '%s')\n",
                         MAX_TIMEOUT, nErrWait, waitElapsed, ulTotalBytesRead, pszRespBuffer);
        }
#endif
        if (nErrWait) {
            nErr = PLUGIN_BAD_CMD_RESPONSE;
            return nErr;
        }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
        clock_gettime(CLOCK_MONOTONIC, &rfStart);
#endif
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr || ulBytesRead != 1) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 0
            clock_gettime(CLOCK_MONOTONIC, &rfEnd);
            double rfElapsed = (rfEnd.tv_sec - rfStart.tv_sec) + (rfEnd.tv_nsec - rfStart.tv_nsec) * 1e-9;
            if (nErr)
                LogDebug(1, "[AstroTrac::readResponse] readFile error %d after %.3f seconds (of %dms requested timeout, had %lu byte(s) so far: '%s')\n",
                         nErr, rfElapsed, MAX_TIMEOUT, ulTotalBytesRead, pszRespBuffer);
            else
                LogDebug(1, "[AstroTrac::readResponse] readFile got %lu byte(s) (wanted 1) after %.3f seconds DESPITE waitForBytesRx reporting ready (of %dms requested timeout, had %lu byte(s) so far: '%s')\n",
                         ulBytesRead, rfElapsed, MAX_TIMEOUT, ulTotalBytesRead, pszRespBuffer);
#endif
            if (nErr) return nErr;
        }

        LogDebug(3, "[AstroTrac::readResponse] *pszBufPtr = 0x%02X ulBytesRead %d\n", *pszBufPtr, ulBytesRead);

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
        LogDebug(1, "[AstroTrac::readResponse] No closing bracket: *pszRespBuffer = %s bytes read %lu\n", pszRespBuffer, ulTotalBytesRead);

        return PLUGIN_BAD_CMD_RESPONSE;
    }


    if(ulTotalBytesRead && *(pszBufPtr-1) == '>')
        *(pszBufPtr-1) = 0; //remove the > to zero terminate the string

    LogDebug(3, "[AstroTrac::readResponse] *pszRespBuffer = %s nErr %d\n", pszRespBuffer, nErr);

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

int AstroTrac::sendSafetyLimits(double dMeridianLimitDeg, double dHorizonLimitDeg, double dLatitudeDeg)
{
    int nErr = PLUGIN_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    std::string sFirmware;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getFirmwareVersion(sFirmware);
    if(nErr) return nErr;

    if (atof(sFirmware.c_str()) < FIRMWARE_MIN_VER_SAFETY_LIMITS) {
        LogDebug(1, "[AstroTrac::sendSafetyLimits] Firmware %s predates %.2f - firmware safety limits not supported, skipping\n", sFirmware.c_str(), FIRMWARE_MIN_VER_SAFETY_LIMITS);
        return PLUGIN_OK;
    }

    // Latitude first - the firmware's horizon/altitude math depends on it being set before 'lh' is meaningful.
    snprintf(szCmd, sizeof(szCmd), "<1la%f>", dLatitudeDeg);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;

    snprintf(szCmd, sizeof(szCmd), "<1lt%f>", dMeridianLimitDeg);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;

    snprintf(szCmd, sizeof(szCmd), "<1lh%f>", dHorizonLimitDeg);
    nErr = AstroTracSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE); if (nErr) return COMMAND_FAILED;

    LogDebug(3, "[AstroTrac::sendSafetyLimits] Sent la=%f lt=%f lh=%f\n", dLatitudeDeg, dMeridianLimitDeg, dHorizonLimitDeg);
    return nErr;
}

#pragma mark - Mount Coordinates
void AstroTrac::setMountMode(MountTypeInterface::Type mountType)
{
    LogDebug(3, "[AstroTrac::setMountMode] mountType = %d\n", mountType);

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
    LogDebug(3, "[AstroTrac::getHaAndDec] szResp = %s\n", szResp);

    // RA Encoder measured in degrees - stored in private variable too, ignoring first 3 characters
    m_dHAEncoder = atof(szResp+3);

    // get DEC encoder values
    nErr = AstroTracSendCommand("<2p?>", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // Dec Encoder measured in degrees - stored in private variable too, ignoring first 3 characters
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

    LogDebug(3, "[AstroTrac::getHaAndDec] RAEncoder %f DEencoder %f Ha %f Dec %f BTP %d\n", m_dHAEncoder, m_dDecEncoder, dHa, dDec, m_bIsBTP);
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
    
    LogDebug(3, "AstroTrac::EncodervaluefromHAandDec (asymetrical) called %f %f %f %f bUseBTP %d IsBeyondThePole %d\n", HAEncoder, DEEncoder, dHa, dDec, bUseBTP, m_bIsBTP);
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
    
    LogDebug(3, "AstroTrac::HAandDecfromEncodervalue called %f %f %f %f IsBeyondThePole %d\n", RAEncoder, DEEncoder, dHa, dDec, m_bIsBTP);
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


    LogDebug(3, "[AstroTrac::setTrackingRates] Tracking on: %d, Ignorrates %d, dRARate %f dDECrate %f\n", bTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);

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
    
    LogDebug(3, "[AstroTrac::setTrackingRates] Tracking on: %d, Ignorrates %d, RARate %f DECrate %f\n", bTrackingOn, bIgnoreRates, RARate, DECRate);

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

// Function to estimate time to slew - distance in degrees, dAccel is the acceleration (arcsec/sec/sec) of the axis being slewed
double AstroTrac::slewTime(double dDist, double dAccel)
{
    double tslew;  // Estimate of time for slew
    double accelndist; // Estimate of distance covered by acceleration and deceleration period

    // Firstly throw away sign of distance - don't care about direction - and convert to arcsec
    dDist = fabs(dDist) * 3600.0;

    // Now estimate how far mount travels during accelertion and deceleration period
    accelndist = m_dVSlewMax * m_dVSlewMax / dAccel;

    // If distance less than this, then calulate using accleration forumlae:
    if (dDist < accelndist) {
        tslew = 2 * sqrt(dDist/dAccel);
    } else {
        // Time is equal to twice the time required to accelerate or decelerate, plus the remaining distance at max slew speed
        tslew = 2.0 * m_dVSlewMax/dAccel + (dDist-accelndist)/m_dVSlewMax;
    }
    
    LogDebug(3, "slewTime: dDist %f Time %f\n", dDist, tslew);
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

    // Calculate time required to slew in HA direction. Only RA needs this: the mount never stops
    // the previously-set tracking velocity around a position slew (confirmed by testing), so RA
    // resumes sidereal tracking the instant its own slew finishes, regardless of how long the DEC
    // axis takes - no need to account for DEC's slew time here.
    tHa = slewTime(HAEncoder - m_dHAEncoder, m_dAslewRA);

    LogDebug(3, "startSlewTo: dHa: %f HAEncoder %f, m_dHAEncoder %f, tHa %f\n", dHa, HAEncoder, m_dHAEncoder, tHa);

    // Formulate and send command to slew for RA axis - adding on time it takes to slew in the RA axis (remember to convert from arcsec to degrees)
    // m_dSlewOffset is initially zero, then set to the difference between actual and target RA after the slew has completed
    snprintf(out, sizeof(out), "<1p%f>", HAEncoder + m_dSlewOffset + (m_bNorthernHemisphere ? 1.0: -1.0) * tHa * AT_SIDEREAL_SPEED/3600.0);

    nErr = AstroTracSendCommand(out, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;

    LogDebug(3, "startSlewTo: dHa: %f command: %s response %s\n", dHa, out, szResp);

    // Formulate and send command to slew for DEC axis
    snprintf(out, sizeof(out), "<2p%f>", DEEncoder);
    nErr = AstroTracSendCommand(out, szResp, SERIAL_BUFFER_SIZE); if (nErr) return nErr;

    LogDebug(3, "startSlewTo: dDec: %f command: %s response %s\n", dDec, out, szResp);

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

    // RA Encoder measured in degrees - stored in private variable, ignoring first 3 characters
    m_dHAEncoder = atof(szResp+3);
    
    // Calculate current HA of slew target
    dHa = m_pTsx->hourAngle(m_dGotoRATarget);
    
    // Calculate Encoder Values
    EncoderValuesfromHAanDEC(dHa, dDec, dHAEncoder, dDecEncoder, false);
    
    // Add difference between the two as additional offset. The offset has already been used to get this close
    // Only do this if the difference is less than 100" - otherwise likely to be some sort of error
    if (fabs(dHAEncoder - m_dHAEncoder) * 3600.0 < 100) m_dSlewOffset += dHAEncoder - m_dHAEncoder;
    
    LogDebug(3, "endSlewTo: m_dHAEncoder %f Target Encoder %f Offset %f\"\n", m_dHAEncoder, dHAEncoder, m_dSlewOffset*3600);

    return nErr;
    
}

int AstroTrac::isSlewToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bComplete = false;

    // Is RA drive finished slew?
    nErr = AstroTracSendCommand("<1t?>", szResp, SERIAL_BUFFER_SIZE);    if(nErr) return nErr;

    LogDebug(3, "[AstroTrac::isSlewToComplete] RA szResp : %s\n", szResp);

    // Third character tells whether is still slewing - 1 yes, 0 no
    if (szResp[3] == '1') return nErr;

    // RA drive has finished slew - try DEC drive
    nErr = AstroTracSendCommand("<2t?>", szResp, SERIAL_BUFFER_SIZE);    if(nErr) return nErr;
    // Third character tells whether is still slewing - 1 yes, 0 no

    LogDebug(3, "[AstroTrac::isSlewToComplete] DEC szResp : %s\n", szResp);

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

    LogDebug(0, "[AstroTrac::startOpenSlew] setting to Dir %d\n", Dir);
    LogDebug(0, "[AstroTrac::startOpenSlew] Setting rate to %d\n", nRate);

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
    LogDebug(0, "[AstroTrac::startOpenLoopMove] AstroTracSendCommand took %.3f seconds, nErr = %d\n", cmdElapsed, nErr);
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
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (m_bOpenLoopRA) {
            double elapsedRA = (now.tv_sec - m_OpenLoopStartTimeRA.tv_sec) + (now.tv_nsec - m_OpenLoopStartTimeRA.tv_nsec) * 1e-9;
            LogDebug(0, "[AstroTrac::stopOpenLoopMove] RA (East/West) duration %.3f seconds\n", elapsedRA);
        }
        if (m_bOpenLoopDEC) {
            double elapsedDEC = (now.tv_sec - m_OpenLoopStartTimeDEC.tv_sec) + (now.tv_nsec - m_OpenLoopStartTimeDEC.tv_nsec) * 1e-9;
            LogDebug(0, "[AstroTrac::stopOpenLoopMove] DEC (North/South) duration %.3f seconds\n", elapsedDEC);
        }
    }
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
    LogDebug(0, "[AstroTrac::stopOpenLoopMove] setTrackingRates took %.3f seconds, nErr = %d\n", cmdElapsed, nErr);
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
    
    LogDebug(3, "[AstroTrac::gotoPark] Called!\n");

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

