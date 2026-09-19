#pragma once
// All of the driver's clock/timing code lives here, so the platform-specific parts are in one place.
// Built on <chrono> and time()/localtime, which behave the same on Windows (MSVC), macOS and Linux
// (x86 and ARM) - unlike clock_gettime(), which MSVC doesn't provide.
//
// Nothing in the driver's real logic uses these: the read timeout counts milliseconds itself and the
// sleeping goes through the SDK's SleeperInterface. They only feed debug logging, so every call site
// is inside #ifdef PLUGIN_DEBUG (AstroTrac.cpp) or #ifdef AstroTrac_X2_DEBUG (x2mount.cpp). The
// definitions below are deliberately NOT guarded: those two macros are defined in different headers,
// and unused inline functions generate no code anyway.

#include <chrono>
#include <stdio.h>
#include <time.h>

typedef std::chrono::steady_clock::time_point AtTime;

// Monotonic clock, for measuring intervals (immune to the wall clock being adjusted).
inline AtTime AtNow()
{
    return std::chrono::steady_clock::now();
}

inline double AtSecondsSince(const AtTime &start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

// Thread-safe time_t -> local struct tm (localtime_s on Windows takes the same arguments in this order).
inline void AtLocalTime(time_t t, struct tm &out)
{
#if defined(SB_WIN_BUILD)
    localtime_s(&out, &t);
#else
    localtime_r(&t, &out);
#endif
}

// Wall-clock timestamp with millisecond precision for log lines, e.g. "Wed Sep  9 20:03:39.724 2026".
// The format is parsed by analyze_timing_log.py / analyze_guiding_session.py - keep them in step.
inline void AtTimestampNow(char *pszBuf, size_t nLen)
{
    using namespace std::chrono;
    system_clock::time_point tp = system_clock::now();
    time_t secs = system_clock::to_time_t(tp);
    int ms = (int)(duration_cast<milliseconds>(tp.time_since_epoch()).count() % 1000);
    if (ms < 0) ms = 0;    // only possible for pre-1970 times
    struct tm tmNow;
    AtLocalTime(secs, tmNow);
    char szBase[32];
    strftime(szBase, sizeof(szBase), "%a %b %e %H:%M:%S", &tmNow);
    int year = tmNow.tm_year + 1900;
    if (year < 0 || year > 9999) year = 0;
    // ms and year are clamped so the output is provably at most 31+1+3+1+4 = 40 chars, which lets the
    // compiler see it fits (gcc otherwise warns about possible truncation, assuming a huge int).
    snprintf(pszBuf, nLen, "%s.%03d %04d", szBase, ms, year);
}

// Observing-night date used to name the debug logs, e.g. "September 04 2026". Uses the same
// "noon to noon" convention TheSky itself uses for its guide-log folders (that name covers the night
// from midday Sep 4 to midday Sep 5), so a log can be matched to its guiding data by date at a glance.
// The time_t overload exists so the noon boundary can be tested with a fixed time.
inline void AtObservingNightDate(time_t now, char *pszBuf, size_t nLen)
{
    struct tm tmNight;
    AtLocalTime(now, tmNight);
    if (tmNight.tm_hour < 12) {
        now -= 12 * 3600;
        AtLocalTime(now, tmNight);
    }
    strftime(pszBuf, nLen, "%B %d %Y", &tmNight);
}

inline void AtObservingNightDate(char *pszBuf, size_t nLen)
{
    AtObservingNightDate(time(nullptr), pszBuf, nLen);
}
