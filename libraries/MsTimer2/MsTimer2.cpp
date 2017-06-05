#include "Arduino.h"
#include <interrupt_handlers.h>
#include <libraries/MsTimer2/MsTimer2.h>

static void (*millisIntervalFunc)() = NULL;
static unsigned long millisIntervalTime;
static unsigned long millisIntervalCount;
static void (*intervalFunc)() = NULL;

static void MillisIntervalFunc()
{
    if (++millisIntervalCount >= millisIntervalTime) {
        if (millisIntervalFunc) {
            millisIntervalFunc();
        }
        millisIntervalCount = 0;
    }
}

void MsTimer2::set(unsigned long ms, void (*f)())
{
    attachMicroIntervalTimerHandler(NULL, 1000);
    millisIntervalCount = 0;
    millisIntervalTime = ms;
    millisIntervalFunc = f;
    intervalFunc = MillisIntervalFunc;
}

void MsTimer2::setMicros(unsigned short us, void (*f)())
{
    attachMicroIntervalTimerHandler(NULL, us);
    intervalFunc = f;
}

void MsTimer2::setClock(unsigned short fclk, void (*f)())
{
    attachClockIntervalTimerHandler(NULL, fclk);
    intervalFunc = f;
}

void MsTimer2::start()
{
    INT_TM_HOOK = intervalFunc;
}

void MsTimer2::stop()
{
    INT_TM_HOOK = NULL;
}
