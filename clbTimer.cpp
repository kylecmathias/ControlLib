#include "clbTimer.h"



clb::Timer::Timer() {
    _clockSource = 0b000; 
}

void clb::Timer::deactivate() { CRITICAL("Timer superclass called deactivate(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }


//global timer methods


void clb::Timer::resetSynchronousPrescalers() { //reset the prescalers for timers 0, 1, 3, 4 and 5
    cli();
    WARNING("Resetting synchronous prescalers will break delay(), millis() and micros() functions from Timer0");
    
    uint8_t _GTCCR = GTCCR; 

    _GTCCR &= ~(BIT0 << TSM);       
    _GTCCR |= (BIT0 << PSRSYNC); 

    GTCCR = _GTCCR;   
    
    sei();       
}
void clb::Timer::resetAsynchronousPrescalers() { //reset the prescaler for timer 2
    cli();
    WARNING("Resetting asynchronous prescalers will break tone() and noTone() functions from Timer2");
    
    uint8_t _GTCCR = GTCCR; 

    _GTCCR &= ~(BIT0 << TSM);       
    _GTCCR |= (BIT0 << PSRASY);  

    GTCCR = _GTCCR;   
    
    sei();
}
void clb::Timer::resetAllPrescalers() { //reset all prescalers
    cli();
    WARNING("Resetting all prescalers will break delay(), millis() and micros() functions from Timer0 and tone() and noTone() from Timer2");
    
    uint8_t _GTCCR = GTCCR;

    _GTCCR &= ~(BIT0 << TSM);      
    _GTCCR |= (BIT0 << PSRASY) | (BIT0 << PSRSYNC); 

    GTCCR = _GTCCR;
    
    sei();
}
void clb::Timer::startTimerSynchronization() { //start all timers in sync
    cli();
    WARNING("Starting all timers in sync will break delay(), millis() and micros() functions from Timer0 and tone() and noTone() from Timer2");
    
    uint8_t _GTCCR = GTCCR; 

    _GTCCR |= (BIT0 << TSM | BIT0 << PSRASY | BIT0 << PSRSYNC);
    
    GTCCR = _GTCCR; 

    sei();
}
void clb::Timer::stopTimerSynchronization() { //stop all timers in sync
    cli();
    WARNING("Stopping all timers in sync will break delay(), millis() and micros() functions from Timer0 and tone() and noTone() from Timer2");
    
    uint8_t _GTCCR = GTCCR;

    _GTCCR &= ~(BIT0 << TSM);
    
    GTCCR = _GTCCR; 

    sei();
}

//setup methods


void clb::Timer::setMode(clb::TMode8 mode) { CRITICAL("Timer superclass called setMode(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setMode(clb::TMode16 mode) { CRITICAL("Timer superclass called setMode(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setClock(clb::TSyncClock clock) { CRITICAL("Timer superclass called setClock(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setClock(clb::TAsynClock clock) { CRITICAL("Timer superclass called setClock(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchOutputModeA(clb::TCMOM mode) { CRITICAL("Timer superclass called setCompareMatchOutputModeA(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }   
void clb::Timer::setCompareMatchOutputModeB(clb::TCMOM mode) { CRITICAL("Timer superclass called setCompareMatchOutputModeB(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchOutputModeC(clb::TCMOM mode) { CRITICAL("Timer superclass called setCompareMatchOutputModeC(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchValueA(uint8_t value) { CRITICAL("Timer superclass called setCompareMatchValueA(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchValueB(uint8_t value) { CRITICAL("Timer superclass called setCompareMatchValueB(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchValueA(uint16_t value) { CRITICAL("Timer superclass called setCompareMatchValueA(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchValueB(uint16_t value) { CRITICAL("Timer superclass called setCompareMatchValueB(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setCompareMatchValueC(uint16_t value) { CRITICAL("Timer superclass called setCompareMatchValueC(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }


//overflow handling methods

/*
void clb::Timer::overflow() { CRITICAL("Timer superclass called overflow(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setOverflowCount(uint32_t count) { CRITICAL("Timer superclass called setOverflowCount(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
uint32_t clb::Timer::getOverflowCount() { CRITICAL("Timer superclass called getOverflowCount(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); return 0; }
bool clb::Timer::isLastOverflow() { CRITICAL("Timer superclass called isLastOverflow(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); return false; }
*/

//operation methods


void clb::Timer::startTimer() { CRITICAL("Timer superclass called startTimer(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::stopTimer() { CRITICAL("Timer superclass called stopTimer(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
uint8_t clb::Timer::getTimerValue8() { CRITICAL("Timer superclass called getTimerValue(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
uint16_t clb::Timer::getTimerValue16() { CRITICAL("Timer superclass called getTimerValue(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setTimerValue(uint8_t value) { CRITICAL("Timer superclass called setTimerValue(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setTimerValue(uint16_t value) { CRITICAL("Timer superclass called setTimerValue(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::forceOutputCompareA() { CRITICAL("Timer superclass called forceOutputCompareA(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::forceOutputCompareB() { CRITICAL("Timer superclass called forceOutputCompareB(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::forceOutputCompareC() { CRITICAL("Timer superclass called forceOutputCompareC(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }

//interrupt control methods


void clb::Timer::setInterruptCallback(clb::TInterrupt8 type, void (*callback)()) { CRITICAL("Timer superclass called setInterruptCallback(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::enableInterrupt(clb::TInterrupt8 type) { CRITICAL("Timer superclass called enableInterrupt(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::disableInterrupt(clb::TInterrupt8 type) { CRITICAL("Timer superclass called disableInterrupt(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
bool clb::Timer::getInterruptFlag(clb::TInterrupt8 type) { CRITICAL("Timer superclass called getInterruptFlag(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::clearInterruptFlag(clb::TInterrupt8 type) { CRITICAL("Timer superclass called clearInterruptFlag(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::setInterruptCallback(clb::TInterrupt16 type, void (*callback)()) { CRITICAL("Timer superclass called setInterruptCallback(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::enableInterrupt(clb::TInterrupt16 type) { CRITICAL("Timer superclass called enableInterrupt(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::disableInterrupt(clb::TInterrupt16 type) { CRITICAL("Timer superclass called disableInterrupt(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
bool clb::Timer::getInterruptFlag(clb::TInterrupt16 type) { CRITICAL("Timer superclass called getInterruptFlag(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::clearInterruptFlag(clb::TInterrupt16 type) { CRITICAL("Timer superclass called clearInterruptFlag(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }

//input capture methods


void clb::Timer::inputCaptureNoiseCancelEnable(bool enable) { CRITICAL("Timer superclass called inputCaptureNoiseCancelEnable(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::inputCaptureEdgeSelect(bool rising) { CRITICAL("Timer superclass called inputCaptureEdgeSelect(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }


//direct blocking delay methods
void clb::Timer::syncDelay(uint32_t time) { CRITICAL("Timer superclass called syncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::syncDelay(uint32_t time, clb::TTimeUnit timeUnit) { CRITICAL("Timer superclass called syncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::syncDelay(uint32_t time, clb::TTimeUnit timeUnit, clb::TOutputChannel channel) { CRITICAL("Timer superclass called syncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }


//direct non blocking delay methods
void clb::Timer::asyncDelay(uint32_t time) { CRITICAL("Timer superclass called asyncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit) { CRITICAL("Timer superclass called asyncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit, clb::TOutputChannel channel) { CRITICAL("Timer superclass called asyncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }

//async delay control methods
bool clb::Timer::isAsyncDelayFinished() { CRITICAL("Timer superclass called isAsyncDelayFinished(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); } //returns true if the asynchronous delay is finished
void clb::Timer::stopAsyncDelay() { CRITICAL("Timer superclass called stopAsyncDelay(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); } //stops the asynchronous delay

//delay logic methods
void clb::Timer::syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) { CRITICAL("Timer superclass called syncDelayLogic(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }
void clb::Timer::asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) { CRITICAL("Timer superclass called asyncDelayLogic(): Use clb::Timer::createTimer() to create a timer corresponding to one of the hardware timers 0-5"); }