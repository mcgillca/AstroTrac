// Standalone diagnostic: sends the driver's four routine polling commands (<1t?>, <2t?>, <1p?>,
// <2p?>) directly to the AstroTrac360 over TCP, back to back in a tight loop, with NO TheSkyX
// process involved at all. Ports AstroTrac.cpp's own read/retry/stale-reply-recovery logic
// (AstroTracreadResponse, DiscardStaleReplies, RecoverMatchingReply, responseMatchesCommand)
// almost verbatim - same 3-try staged timeout (300ms, 300ms, 1600ms) - just swapping
// SerXInterface calls for plain POSIX sockets. Purpose: isolate whether the periodic comms
// stalls seen in the field (e.g. the recurring ~:00-second-aligned bursts) still happen with
// TheSkyX entirely out of the picture. If they do, that points at the Pi/network/mount; if they
// don't, that points at TheSkyX itself.
//
// Deliberately NOT ported: DrainDuplicateReplies (the fixed-200ms extra-duplicate-reply check) -
// a secondary refinement, not needed to answer the core question here.
//
// Log format matches AstroTrac.cpp's own LogDebug (same millisecond timestamp style, same
// "Sending/got response/succeeded.../TIMED OUT/FAILED after" phrasing) so analyze_timing_log.py
// works on this tool's output unchanged.
//
// Addition beyond the driver: logs how late each poll-loop sleep() actually returns vs what was
// requested - if THIS process's own sleeps run late even with nothing else competing for the
// mount's attention, that's independent evidence the Pi's scheduler (not the network or mount)
// is the root cause.
//
// Usage: astrotrac_standalone_test [host] [port] [logfile]
//   Defaults: host=10.39.63.74 port=23 logfile=AstroTracStandaloneTest_<observing night>.txt
// Ctrl-C to stop (SIGINT is caught for a clean shutdown; the log is fflush()ed after every line
// regardless, so nothing is lost even on an abrupt kill).

#include <cctype>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 300              // ms, first MAXSENDTRIES-1 tries
#define MAX_TIMEOUT_FINAL_TRY 1600   // ms, last try only - matches AstroTrac.h v2.03
#define MAXSENDTRIES 3
#define MAX_STALE_RESPONSE_TRIES 2
#define READ_POLL_INTERVAL_MS 3

enum { OK = 0, BAD_CMD_RESPONSE = 1 };

static int g_fd = -1;
static FILE *g_logfile = nullptr;
static volatile sig_atomic_t g_stop = 0;

static void onSigint(int) { g_stop = 1; }

// ---------------------------------------------------------------------------
// Logging - matches AstroTrac.cpp's LogDebug timestamp format exactly.
// ---------------------------------------------------------------------------
static void LogLine(const char *pszFormat, ...)
{
    struct timespec tsNow;
    struct tm tmNow;
    char szTimestamp[32];
    clock_gettime(CLOCK_REALTIME, &tsNow);
    localtime_r(&tsNow.tv_sec, &tmNow);
    strftime(szTimestamp, sizeof(szTimestamp), "%a %b %e %H:%M:%S", &tmNow);
    fprintf(g_logfile, "[%s.%03ld %d] ", szTimestamp, tsNow.tv_nsec / 1000000, tmNow.tm_year + 1900);

    va_list args;
    va_start(args, pszFormat);
    vfprintf(g_logfile, pszFormat, args);
    va_end(args);
    fflush(g_logfile);
}

// ---------------------------------------------------------------------------
// Low-level socket helpers standing in for SerXInterface's bytesWaitingRx/readFile/writeFile.
// ---------------------------------------------------------------------------
static int bytesWaitingRx(int &nBytes)
{
    int n = 0;
    if (ioctl(g_fd, FIONREAD, &n) < 0) {
        nBytes = 0;
        return -1;
    }
    nBytes = n;
    return 0;
}

static int readSome(unsigned char *buf, unsigned long toRead, unsigned long &bytesRead)
{
    ssize_t n = recv(g_fd, buf, toRead, 0);
    if (n < 0) {
        bytesRead = 0;
        return -1;
    }
    bytesRead = (unsigned long)n;
    return 0;
}

static int writeCmd(const char *pszCmd)
{
    ssize_t n = send(g_fd, pszCmd, strlen(pszCmd), 0);
    return (n < 0) ? -1 : 0;
}

static void purgeRx()
{
    unsigned char discard[SERIAL_BUFFER_SIZE];
    int nBytes = 0;
    while (bytesWaitingRx(nBytes) == 0 && nBytes > 0) {
        unsigned long got = 0;
        unsigned long toRead = (unsigned long)nBytes;
        if (toRead > sizeof(discard))
            toRead = sizeof(discard);
        if (readSome(discard, toRead, got) || got == 0)
            break;
    }
}

static void sleepMs(int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, nullptr);
}

// ---------------------------------------------------------------------------
// Ported near-verbatim from AstroTrac.cpp.
// ---------------------------------------------------------------------------
static size_t CommandPrefixLen(const char *pszCmd)
{
    size_t i = 2;
    while (pszCmd[i] && isalpha((unsigned char)pszCmd[i]))
        i++;
    return i;
}

static bool responseMatchesCommand(const char *pszCmd, const unsigned char *pszResp)
{
    if (!pszCmd || !pszResp)
        return false;
    if (pszCmd[0] != '<' || pszResp[0] != '<' || !isdigit((unsigned char)pszCmd[1]))
        return false;
    if (pszCmd[1] != (char)pszResp[1])
        return false;
    if (pszResp[2] == 'e' && pszCmd[2] != 'e')
        return true;
    size_t i = CommandPrefixLen(pszCmd);
    if (strncmp(pszCmd, (const char *)pszResp, i) == 0)
        return true;
    if (pszResp[2] == '#' && strncmp(pszCmd + 2, (const char *)pszResp + 3, i - 2) == 0)
        return true;
    return false;
}

static bool RecoverMatchingReply(const char *pszCmd, unsigned char *pszResp)
{
    unsigned char *pszLastFrame = pszResp;
    for (unsigned char *p = pszResp; *p; p++) {
        if (*p == '<')
            pszLastFrame = p;
    }
    if (!responseMatchesCommand(pszCmd, pszLastFrame))
        return false;
    if (pszLastFrame != pszResp) {
        memmove(pszResp, pszLastFrame, strlen((const char *)pszLastFrame) + 1);
        LogLine("Recovered matching reply for Cmd: %s from end of stale backlog: '%s'\n", pszCmd, pszResp);
    }
    return true;
}

static int readResponse(unsigned char *buf, unsigned int bufLen, unsigned int timeoutMs)
{
    unsigned long totalRead = 0;
    unsigned char *ptr;
    int msWaited = 0;
    struct timespec rfStart, rfEnd;
    clock_gettime(CLOCK_MONOTONIC, &rfStart);

    memset(buf, 0, bufLen);
    ptr = buf;

    do {
        int nWaiting = 0;
        int errPeek = bytesWaitingRx(nWaiting);

        if (errPeek || nWaiting <= 0) {
            if (msWaited >= (int)timeoutMs) {
                clock_gettime(CLOCK_MONOTONIC, &rfEnd);
                double elapsed = (rfEnd.tv_sec - rfStart.tv_sec) + (rfEnd.tv_nsec - rfStart.tv_nsec) * 1e-9;
                LogLine("[readResponse] TIMED OUT: no bytes waiting after %.3f seconds (of %dms requested timeout, had %lu byte(s) so far: '%s')\n",
                        elapsed, timeoutMs, totalRead, buf);
                return BAD_CMD_RESPONSE;
            }
            struct timespec sleepStart, sleepEnd;
            clock_gettime(CLOCK_MONOTONIC, &sleepStart);
            sleepMs(READ_POLL_INTERVAL_MS);
            clock_gettime(CLOCK_MONOTONIC, &sleepEnd);
            double sleptMs = (sleepEnd.tv_sec - sleepStart.tv_sec) * 1000.0 +
                              (sleepEnd.tv_nsec - sleepStart.tv_nsec) / 1e6;
            // Diagnostic addition (not in the driver): flag this process's own scheduling being
            // late, independent of anything happening on the wire.
            if (sleptMs > READ_POLL_INTERVAL_MS * 5.0)
                LogLine("[readResponse] SCHED LATE: sleep(%dms) actually took %.3fms\n", READ_POLL_INTERVAL_MS, sleptMs);
            msWaited += READ_POLL_INTERVAL_MS;
            continue;
        }

        unsigned long toRead = (unsigned long)nWaiting;
        if (totalRead + toRead > bufLen)
            toRead = bufLen - totalRead;

        unsigned long got = 0;
        int errRead = readSome(ptr, toRead, got);
        if (errRead) {
            LogLine("[readResponse] readFile error reading %lu waiting byte(s) (had %lu byte(s) so far: '%s')\n",
                    toRead, totalRead, buf);
            return BAD_CMD_RESPONSE;
        }

        LogLine("[readResponse] bytesWaitingRx=%d, read %lu byte(s)\n", nWaiting, got);

        msWaited = 0;
        totalRead += got;
        ptr += got;

    } while ((totalRead == 0 || *(ptr - 1) != '>') && totalRead < bufLen);

    if (*(ptr - 1) != '>') {
        if (totalRead < bufLen)
            *ptr = 0;
        LogLine("[readResponse] No closing bracket: *buf = %s bytes read %lu\n", buf, totalRead);
        return BAD_CMD_RESPONSE;
    }

    if (totalRead && *(ptr - 1) == '>')
        *(ptr - 1) = 0;

    return OK;
}

static int discardStaleReplies(const char *pszCmd, unsigned char *pszResp, unsigned int bufLen, unsigned int timeoutMs)
{
    int nStaleTries;
    int nErr = OK;

    for (nStaleTries = 0; nStaleTries < MAX_STALE_RESPONSE_TRIES && !RecoverMatchingReply(pszCmd, pszResp); nStaleTries++) {
        LogLine("Mismatched response for Cmd: %s -- got stale reply: '%s', discarding (attempt %d/%d)\n",
                pszCmd, pszResp, nStaleTries + 1, MAX_STALE_RESPONSE_TRIES);
        nErr = readResponse(pszResp, bufLen, timeoutMs);
        if (nErr)
            return nErr;
    }

    if (!RecoverMatchingReply(pszCmd, pszResp)) {
        LogLine("Gave up after %d stale replies, still mismatched for Cmd: %s last reply: '%s'\n",
                MAX_STALE_RESPONSE_TRIES, pszCmd, pszResp);
        return BAD_CMD_RESPONSE;
    }

    return OK;
}

static bool prepareForSend(bool isRetry)
{
    int before = 0, after = 0;
    bytesWaitingRx(before);
    bool skipResend = isRetry && before > 0;
    if (!skipResend) {
        purgeRx();
        bytesWaitingRx(after);
    }
    return skipResend;
}

static int sendCommandInnerLoop(const char *pszCmd, unsigned char *pszResp, unsigned int bufLen, bool isRetry, unsigned int timeoutMs)
{
    bool skipResend = prepareForSend(isRetry);

    if (!skipResend) {
        LogLine("Sending %s\n", pszCmd);
        struct timespec wcStart, wcEnd;
        clock_gettime(CLOCK_MONOTONIC, &wcStart);
        int err = writeCmd(pszCmd);
        clock_gettime(CLOCK_MONOTONIC, &wcEnd);
        double wcElapsed = (wcEnd.tv_sec - wcStart.tv_sec) + (wcEnd.tv_nsec - wcStart.tv_nsec) * 1e-9;
        LogLine("writeFile+flushTx for Cmd: %s took %.3f seconds\n", pszCmd, wcElapsed);
        if (err)
            return -1;
    }

    int err = readResponse(pszResp, bufLen, timeoutMs);
    if (err) {
        LogLine("error %d reading response : %s\n", err, pszResp);
        return err;
    }

    err = discardStaleReplies(pszCmd, pszResp, bufLen, timeoutMs);
    if (err)
        return err;

    if (pszResp[2] == 'e') {
        LogLine("Poorly formed reply: '%s'\n", pszResp);
        return BAD_CMD_RESPONSE;
    }

    LogLine("got response : '%s'\n", pszResp);
    return OK;
}

static int sendCommand(const char *pszCmd)
{
    unsigned char resp[SERIAL_BUFFER_SIZE];
    int itries;
    int err = OK;
    struct timespec cmdStart, cmdNow;
    clock_gettime(CLOCK_MONOTONIC, &cmdStart);

    for (itries = 0; itries < MAXSENDTRIES; itries++) {
        unsigned int timeoutMs = (itries == MAXSENDTRIES - 1) ? MAX_TIMEOUT_FINAL_TRY : MAX_TIMEOUT;
        err = sendCommandInnerLoop(pszCmd, resp, SERIAL_BUFFER_SIZE, itries > 0, timeoutMs);
        if (err == OK) {
            clock_gettime(CLOCK_MONOTONIC, &cmdNow);
            double elapsed = (cmdNow.tv_sec - cmdStart.tv_sec) + (cmdNow.tv_nsec - cmdStart.tv_nsec) * 1e-9;
            if (itries > 0)
                LogLine("Cmd: %s succeeded after %d retries, %.3f seconds total\n", pszCmd, itries, elapsed);
            else
                LogLine("Cmd: %s succeeded first try, %.3f seconds total\n", pszCmd, elapsed);
            return err;
        }
        LogLine("itries %d Cmd: %s\n", itries, pszCmd);
    }

    clock_gettime(CLOCK_MONOTONIC, &cmdNow);
    double elapsed = (cmdNow.tv_sec - cmdStart.tv_sec) + (cmdNow.tv_nsec - cmdStart.tv_nsec) * 1e-9;
    LogLine("Cmd: %s FAILED after %d tries, %.3f seconds total\n", pszCmd, itries, elapsed);
    return err;
}

// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    const char *host = argc > 1 ? argv[1] : "10.39.63.74";
    int port = argc > 2 ? atoi(argv[2]) : 23;

    // Name the log after the observing night, same noon-to-noon convention as the driver's own
    // log (see AstroTrac.cpp's constructor) - lets this be matched up with the driver's log from
    // the same night at a glance.
    time_t nowTime = time(nullptr);
    struct tm nightTm;
    localtime_r(&nowTime, &nightTm);
    if (nightTm.tm_hour < 12) {
        nowTime -= 12 * 3600;
        localtime_r(&nowTime, &nightTm);
    }
    char szNightDate[32];
    strftime(szNightDate, sizeof(szNightDate), "%B %d %Y", &nightTm);

    std::string logPath;
    if (argc > 3) {
        // An explicit path was given - use it exactly as named, no versioning.
        logPath = argv[3];
    } else {
        // Same _v2/_v3/... versioning as the driver's own log (see AstroTrac.cpp's constructor) -
        // a second run on the same observing night must not silently overwrite the first.
        std::string base = std::string("AstroTracStandaloneTest_") + szNightDate;
        logPath = base + ".txt";
        int version = 1;
        while (FILE *pExisting = fopen(logPath.c_str(), "r")) {
            fclose(pExisting);
            version++;
            logPath = base + "_v" + std::to_string(version) + ".txt";
        }
    }

    g_logfile = fopen(logPath.c_str(), "w");
    if (!g_logfile) {
        fprintf(stderr, "Could not open log file %s\n", logPath.c_str());
        return 1;
    }
    setvbuf(g_logfile, nullptr, _IOLBF, 0);

    signal(SIGINT, onSigint);

    g_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_fd < 0) {
        fprintf(stderr, "socket() failed: %s\n", strerror(errno));
        return 1;
    }

    int nodelay = 1;
    setsockopt(g_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        fprintf(stderr, "invalid host address: %s\n", host);
        return 1;
    }

    printf("Connecting to %s:%d ...\n", host, port);
    if (connect(g_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "connect() failed: %s\n", strerror(errno));
        return 1;
    }
    // From here on, the socket is non-blocking - readResponse's own poll loop controls timing,
    // same as AstroTracreadResponse only calling readFile once bytesWaitingRx confirms data.
    fcntl(g_fd, F_SETFL, O_NONBLOCK);

    printf("Connected. Logging to %s. Ctrl-C to stop.\n", logPath.c_str());
    LogLine("=== AstroTrac standalone comms test started, connected to %s:%d (no TheSkyX involved) ===\n", host, port);

    const char *commands[] = {"<1t?>", "<2t?>", "<1p?>", "<2p?>"};
    unsigned long cycles = 0;
    while (!g_stop) {
        for (int i = 0; i < 4 && !g_stop; i++)
            sendCommand(commands[i]);
        cycles++;
        if (cycles % 1000 == 0)
            printf("%lu cycles (%lu commands) sent...\n", cycles, cycles * 4);
    }

    LogLine("=== Stopped (SIGINT) after %lu cycles ===\n", cycles);
    printf("\nStopped after %lu cycles. Log: %s\n", cycles, logPath.c_str());
    fclose(g_logfile);
    close(g_fd);
    return 0;
}
