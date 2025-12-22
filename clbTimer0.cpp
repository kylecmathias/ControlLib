#include "clbTimer.h"

static clb::Timer0* s_active_timer0_instance = nullptr;

static uint32_t getPrescaler(clb::TSyncClock clock);
static uint64_t calculateTicks(uint32_t value, clb::TTimeUnit unit, uint32_t prescaler);
volatile uint8_t* getOcrRegister(clb::TOutputChannel channel);
uint8_t getOcFlagBit(clb::TOutputChannel channel);

static struct Timer0InterruptHandlers {
    void (*compareMatchACallback)() = nullptr;
    void (*compareMatchBCallback)() = nullptr;
    void (*overflowCallback)() = nullptr; //should not be used 
} s_timer0_handlers; 

//global ISRs for Timer0
ISR(TIMER0_COMPA_vect) {
    if (s_active_timer0_instance && s_active_timer0_instance->_asyncDelayActive && s_active_timer0_instance->_asyncDelayActiveChannel == clb::TOutputChannel::A) {
        s_active_timer0_instance->_asyncOverflowsCount--; 
        if (s_active_timer0_instance->_asyncOverflowsCount == 0) {
            s_active_timer0_instance->_asyncDelayActive = false; 
    
            TIMSK0 &= ~(BIT0 << OCIE0A);

            uint8_t temp_sreg = SREG;
            cli();
            TCCR0A = s_active_timer0_instance->_asyncSavedTCCR0A;
            TCCR0B = s_active_timer0_instance->_asyncSavedTCCR0B;
            TCNT0 = s_active_timer0_instance->_asyncSavedTCNT0;
            OCR0A = s_active_timer0_instance->_asyncSavedOCR0A;
            OCR0B = s_active_timer0_instance->_asyncSavedOCR0B;
            TIMSK0 = s_active_timer0_instance->_asyncSavedTIMSK0;
            TIFR0 = s_active_timer0_instance->_asyncSavedTIFR0;
            SREG = temp_sreg;

            if (s_timer0_handlers.compareMatchACallback) {
                s_timer0_handlers.compareMatchACallback();
            }
        } 
        else {
            if (s_active_timer0_instance->_asyncOverflowsCount == 1) {
                OCR0A = s_active_timer0_instance->_asyncRemainingTicksValue;
            } 
            else {
                OCR0A = 255; 
            }
        }
    } 
    else {
        if (s_timer0_handlers.compareMatchACallback) {
            s_timer0_handlers.compareMatchACallback();
        }
    }
}

ISR(TIMER0_COMPB_vect) {
    if (s_active_timer0_instance && s_active_timer0_instance->_asyncDelayActive && s_active_timer0_instance->_asyncDelayActiveChannel == clb::TOutputChannel::B) {
        s_active_timer0_instance->_asyncOverflowsCount--;
        if (s_active_timer0_instance->_asyncOverflowsCount == 0) {
            s_active_timer0_instance->_asyncDelayActive = false;
            TIMSK0 &= ~(BIT0 << OCIE0B);

            uint8_t temp_sreg = SREG;
            cli();
            TCCR0A = s_active_timer0_instance->_asyncSavedTCCR0A;
            TCCR0B = s_active_timer0_instance->_asyncSavedTCCR0B;
            TCNT0 = s_active_timer0_instance->_asyncSavedTCNT0;
            OCR0A = s_active_timer0_instance->_asyncSavedOCR0A;
            OCR0B = s_active_timer0_instance->_asyncSavedOCR0B;
            TIMSK0 = s_active_timer0_instance->_asyncSavedTIMSK0;
            TIFR0 = s_active_timer0_instance->_asyncSavedTIFR0;
            SREG = temp_sreg;

            if (s_timer0_handlers.compareMatchBCallback) {
                s_timer0_handlers.compareMatchBCallback();
            }
        } 
        else {
            if (s_active_timer0_instance->_asyncOverflowsCount == 1) {
                OCR0B = s_active_timer0_instance->_asyncRemainingTicksValue;
            } else {
                OCR0B = 255; 
            }
        }
    } 
    else {
        if (s_timer0_handlers.compareMatchBCallback) {
            s_timer0_handlers.compareMatchBCallback();
        }
    }
}

/* commented out because timer0 overflow interrupt is used by delay(), millis() and micros() functions
 * only uncomment if completely removing those functions and using timer 0 overflow

ISR(TIMER0_OVF_vect) {
    if (s_timer0_handlers.overflowCallback) { 
        s_timer0_handlers.overflowCallback();
    }
}
*/

clb::Timer0::Timer0() {
    WARNING("Using Timer0 is not recommended since any change will basically break the delay(), millis(), and micros() functions.");
    
    if (s_active_timer0_instance != nullptr) {
        FATAL("There is already an instance of Timer0. Only one instance is allowed.");
    }
    
    s_active_timer0_instance = this; 
    _asyncDelayActive = false;
    _asyncTargetTicks = 0;
    _asyncOverflowsCount = 0;
    _asyncRemainingTicksValue = 0;
    _asyncDelayActiveChannel = clb::TOutputChannel::A; 

    _asyncSavedTCCR0A = 0;
    _asyncSavedTCCR0B = 0;
    _asyncSavedTCNT0 = 0;
    _asyncSavedOCR0A = 0;
    _asyncSavedOCR0B = 0;
    _asyncSavedTIMSK0 = 0;
    _asyncSavedTIFR0 = 0;
    _asyncSavedSREG = 0;
}

clb::Timer0::~Timer0() {
    deactivate();
}

void clb::Timer0::deactivate() {
    if (_asyncDelayActive) {
        stopAsyncDelay();
    }

    cli();

    TIMSK0 = 0;
    TIFR0 = (BIT0 << OCF0A) | (BIT0 << OCF0B) | (BIT0 << TOV0);
    TCCR0B = 0;

    TCCR0A = 0;
    OCR0A = 0;
    OCR0B = 0;
    TCNT0 = 0;

    sei(); 

    s_active_timer0_instance = nullptr;
}

//set the mode in TCCR0A and TCCR0B
void clb::Timer0::setMode(TMode8 mode) {
    uint8_t _TCCR0A = TCCR0A;
    uint8_t _TCCR0B = TCCR0B;
    
    uint8_t _mode = static_cast<uint8_t>(mode) & 0x07;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;
    uint8_t _bit2 = (_mode >> 2) & BIT0;

    _bit0 <<= WGM00;
    _bit1 <<= WGM01;
    _bit2 <<= WGM02;

    _TCCR0A &= ~(BIT1 | BIT0);
    _TCCR0B &= ~(BIT3);
    _TCCR0A |= (_bit1 | _bit0);
    _TCCR0B |= (_bit2);

    TCCR0A = _TCCR0A;
    TCCR0B = _TCCR0B;
}

//set the clock in TCCR0B
void clb::Timer0::setClock(TSyncClock clock) { 
    WARNING("Modifying the clock source of Timer0 will affect delay(), millis() and micros() so watch out. calling startTimer() sets the change");
    _clockSource = static_cast<uint8_t>(clock) & 0x07; 
}

//set the compare match output mode for OC0A
void clb::Timer0::setCompareMatchOutputModeA(TCMOM mode) {
    uint8_t _TCCR0A = TCCR0A;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x03;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;

    _bit0 <<= COM0A0;
    _bit1 <<= COM0A1;

    _TCCR0A &= ~(BIT7 | BIT6);
    _TCCR0A |= (_bit1 | _bit0);

    TCCR0A = _TCCR0A;
}

//set the compare match output mode for OC0B
void clb::Timer0::setCompareMatchOutputModeB(TCMOM mode) {
    uint8_t _TCCR0A = TCCR0A;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x03;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;

    _bit0 <<= COM0B0;
    _bit1 <<= COM0B1;

    _TCCR0A &= ~(BIT5 | BIT4);
    _TCCR0A |= (_bit1 | _bit0);

    TCCR0A = _TCCR0A;
}

//set the compare match value in OCR0A
void clb::Timer0::setCompareMatchValueA(uint8_t value) { OCR0A = value; }

//set the compare match value in OCR0B
void clb::Timer0::setCompareMatchValueB(uint8_t value) { OCR0B = value; }

//set the interrupt callback for the timer 
void clb::Timer0::setInterruptCallback(TInterrupt8 type, void (*callback)()) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            s_timer0_handlers.compareMatchACallback = callback; 
            break;
        case TInterrupt8::COMPMATCHB:
            s_timer0_handlers.compareMatchBCallback = callback; 
            break;
        case TInterrupt8::OVERFLOW:
            FATAL("Timer0 overflow reserved for delay(), millis() or micros()");
            //s_timer0_handlers.overflowCallback = callback; //commented out see ISR(TIMER0_OVF_vect) 
            break;
        default:
            CRITICAL("Invalid interrupt type for setting callback");
            break;
    }
}

void clb::Timer0::enableInterrupt(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            TIMSK0 |= BIT0 << OCIE0A; 
            break;
        case TInterrupt8::COMPMATCHB:
            TIMSK0 |= BIT0 << OCIE0B; 
            break;
        case TInterrupt8::OVERFLOW:
            //TIMSK0 |= BIT0 << TOIE0; // commented out see ISR(TIMER0_OVF_vect)
            FATAL("Timer0 overflow reserved for delay(), millis() or micros()");
            break;
        default:
            CRITICAL("Invalid interrupt type for enabling interrupt");
            break;
    }
}

void clb::Timer0::disableInterrupt(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            TIMSK0 &= ~(BIT0 << OCIE0A); 
            break;
        case TInterrupt8::COMPMATCHB:
            TIMSK0 &= ~(BIT0 << OCIE0B); 
            break;
        case TInterrupt8::OVERFLOW:
            //TIMSK0 &= ~(BIT0 << TOIE0); // commented out see ISR(TIMER0_OVF_vect)
            FATAL("Timer0 overflow reserved for delay(), millis() or micros()");
            break;
        default:
            CRITICAL("Invalid interrupt type for disabling interrupt");
            break;
    }
}

bool clb::Timer0::getInterruptFlag(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            return (TIFR0 & BIT0 << OCF0A);
        case TInterrupt8::COMPMATCHB:
            return (TIFR0 & BIT0 << OCF0B);
        case TInterrupt8::OVERFLOW:
            WARNING("Timer0 overflow interrupt flag is cleared constantly by ISR, not much use in checking it");
            return (TIFR0 & BIT0 << TOV0);
        default:
            CRITICAL("Invalid interrupt type for getting interrupt flag");
            break;
    }
}

void clb::Timer0::clearInterruptFlag(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            TIFR0 |= BIT0 << OCF0A; 
            break;
        case TInterrupt8::COMPMATCHB:
            TIFR0 |= BIT0 << OCF0B; 
            break;
        case TInterrupt8::OVERFLOW:
            TIFR0 |= BIT0 << TOV0; 
            CRITICAL("Timer0 overflow interrupt flag is cleared by ISR, and probably already cleared not much use in clearing it");
            break;
        default:
            CRITICAL("Invalid interrupt type for clearing interrupt flag");
            break;
    }
}

void clb::Timer0::startTimer() {
    if (_clockSource == 0) {
        FATAL("Clock source was not set, timer doesnt start");
    }
    WARNING("startTimer() modifies the clock source which affects delay(), millis() and micros() functions, so watch out");
    uint8_t _TCCR0B = TCCR0B;

    _TCCR0B &= ~(BIT2 | BIT1 | BIT0);
    _TCCR0B |= _clockSource;

    TCCR0B = _TCCR0B;
}

void clb::Timer0::stopTimer() {
    WARNING("stopTimer() modifies the clock source which halts delay(), millis() and micros() functions, so watch out");
    if (_asyncDelayActive) {
        stopAsyncDelay();
    }

    uint8_t _TCCR0B = TCCR0B;

    _TCCR0B &= ~(BIT2 | BIT1 | BIT0);

    TCCR0B = _TCCR0B;
}

uint8_t clb::Timer0::getTimerValue8() { return TCNT0; }

void clb::Timer0::setTimerValue(uint8_t value) { TCNT0 = value; }

void clb::Timer0::forceOutputCompareA() { TCCR0B |= BIT0 << FOC0A; }

void clb::Timer0::forceOutputCompareB() { TCCR0B |= BIT0 << FOC0B; }

void clb::Timer0::syncDelay(uint32_t time) {
    syncDelay(time, clb::TTimeUnit::MILLISECONDS, clb::TOutputChannel::B);
}

void clb::Timer0::syncDelay(uint32_t time, clb::TTimeUnit unit) {
    syncDelay(time, unit, clb::TOutputChannel::B);
}

void clb::Timer0::syncDelay(uint32_t time, clb::TTimeUnit unit, clb::TOutputChannel channel) {
    uint32_t _prescaler;
    clb::TSyncClock _clockSourceEnum = static_cast<clb::TSyncClock>(this->_clockSource);

    if (_clockSourceEnum == clb::TSyncClock::STOPPED) {
        _prescaler = 256;
    } else {
        _prescaler = getPrescaler(_clockSourceEnum);
    }

    uint64_t _ticks = calculateTicks(time, unit, _prescaler);

    syncDelayLogic(_ticks, channel);
}

void clb::Timer0::asyncDelay(uint32_t time) {
    asyncDelay(time, clb::TTimeUnit::MILLISECONDS, clb::TOutputChannel::A);
}

void clb::Timer0::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit) {
    asyncDelay(time, timeUnit, clb::TOutputChannel::A);
}

void clb::Timer0::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit, clb::TOutputChannel channel) {
    if (channel != clb::TOutputChannel::A && channel != clb::TOutputChannel::B) {
        CRITICAL("Timer0 only supports TOutputChannel::A and TOutputChannel::B for asyncDelay.");
        return;
    }

    uint32_t _prescaler;
    clb::TSyncClock _clockSourceEnum = static_cast<clb::TSyncClock>(this->_clockSource);

    if (_clockSourceEnum == clb::TSyncClock::STOPPED) {
        WARNING("Timer0 clock source not explicitly set for asyncDelay calculation. Assuming prescaler 64.");
        _prescaler = 64; 
    } else {
        _prescaler = getPrescaler(_clockSourceEnum);
    }

    if (_prescaler == 0) {
        FATAL("Calculated prescaler for Timer0 asyncDelay is 0 (STOPPED clock source), cannot calculate ticks.");
        return;
    }

    uint64_t calculatedTicks = calculateTicks(time, timeUnit, _prescaler);

    asyncDelayLogic(calculatedTicks, channel);
}



//helpers 
void clb::Timer0::syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) {
    uint8_t _sreg = SREG; 
    cli(); 

    uint8_t _tccr0a = TCCR0A;
    uint8_t _tccr0b = TCCR0B;
    uint8_t _tcnt0 = TCNT0;
    uint8_t _ocr0a = OCR0A;
    uint8_t _ocr0b = OCR0B;
    uint8_t _timsk0 = TIMSK0; 
    uint8_t _tifr0 = TIFR0;

    volatile uint8_t* _ocr_reg = getOcrRegister(channel);
    uint8_t _oc_flag_bit = getOcFlagBit(channel);

    TCCR0A = 0;
    TCCR0B = 0;

    uint32_t _prescaler = getPrescaler(static_cast<clb::TSyncClock>(this->_clockSource));
    if (_prescaler == 0) {
        TCCR0B = (BIT0 << CS02);
    } 
    else {
        TCCR0B = this->_clockSource; 
    }

    TCNT0 = 0;
    TIFR0 = (BIT0 << _oc_flag_bit) | (BIT0 << TOV0); 

    const uint16_t MAX_TIMER0_TICKS = 256;

    uint32_t _overflows =  ticks / MAX_TIMER0_TICKS;
    uint16_t _remaining_ticks = ticks % MAX_TIMER0_TICKS;

    for (uint32_t i = 0; i < _overflows; i++) {
        while (!(TIFR0 & (BIT0 << TOV0))) {
        }
        TIFR0 |= (BIT0 << TOV0); 
    }
    if (_remaining_ticks > 0) {
        *_ocr_reg = _remaining_ticks - 1; 
        while (!(TIFR0 & (BIT0 << _oc_flag_bit))) { }
        TIFR0 |= (BIT0 << _oc_flag_bit);
    }

    TCCR0A = _tccr0a;
    TCCR0B = _tccr0b;
    TCNT0 = _tcnt0;
    OCR0A = _ocr0a;
    OCR0B = _ocr0b;
    TIMSK0 = _timsk0;
    TIFR0 = _tifr0; 

    SREG = _sreg;
}

void clb::Timer0::asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) {
    if (_asyncDelayActive) {
        WARNING("An asynchronous delay is already active on Timer0. Cannot start a new one.");
        return;
    }
    if (ticks == 0) {
        WARNING("asyncDelay(0) called. Delay will complete immediately.");
        _asyncDelayActive = false; 
        return;
    }

    _asyncSavedSREG = SREG;
    cli(); 

    _asyncSavedTCCR0A = TCCR0A;
    _asyncSavedTCCR0B = TCCR0B;
    _asyncSavedTCNT0 = TCNT0;
    _asyncSavedOCR0A = OCR0A;
    _asyncSavedOCR0B = OCR0B;
    _asyncSavedTIMSK0 = TIMSK0;
    _asyncSavedTIFR0 = TIFR0;

    TCCR0A = 0;
    TCCR0B = 0; 
    TCNT0 = 0; 

    const uint16_t MAX_TIMER0_TICKS = 256; 

    _asyncTargetTicks = ticks; 

    uint32_t numFullCycles = ticks / MAX_TIMER0_TICKS;
    uint8_t remainderTicks = ticks % MAX_TIMER0_TICKS;

    if (remainderTicks == 0) {
        _asyncOverflowsCount = numFullCycles;
        _asyncRemainingTicksValue = MAX_TIMER0_TICKS - 1;
    } 
    else {
        _asyncOverflowsCount = numFullCycles + 1;
        _asyncRemainingTicksValue = remainderTicks - 1; 
    }

    TCCR0A |= (BIT0 << WGM01);

    uint32_t prescaler_val_for_setup = getPrescaler(static_cast<clb::TSyncClock>(this->_clockSource));
    if (prescaler_val_for_setup == 0) { 
        TCCR0B |= (BIT0 << CS01) | (BIT0 << CS00); 
    } 
    else {
        TCCR0B |= this->_clockSource; 
    }

    if (channel == clb::TOutputChannel::A) {
        TIFR0 |= (BIT0 << OCF0A); 
        if (numFullCycles == 0 && remainderTicks > 0) { 
            OCR0A = _asyncRemainingTicksValue; 
        } 
        else {
            OCR0A = MAX_TIMER0_TICKS - 1;
        }
        TIMSK0 |= (BIT0 << OCIE0A); 
    } 
    else { 
        TIFR0 |= (BIT0 << OCF0B); 
        if (numFullCycles == 0 && remainderTicks > 0) {
            OCR0B = _asyncRemainingTicksValue;
        } 
        else {
            OCR0B = MAX_TIMER0_TICKS - 1;
        }
        TIMSK0 |= (BIT0 << OCIE0B); 
    }

    _asyncDelayActive = true;
    _asyncDelayActiveChannel = channel;

    SREG = _asyncSavedSREG; 
}

bool clb::Timer0::isAsyncDelayFinished() {
    return !_asyncDelayActive;
}

void clb::Timer0::stopAsyncDelay() {
    if (_asyncDelayActive) {
        WARNING("Stopping active asynchronous delay on Timer0.");

        uint8_t temp_sreg = SREG;
        cli();

        if (_asyncDelayActiveChannel == clb::TOutputChannel::A) {
            TIMSK0 &= ~(BIT0 << OCIE0A); 
            TIFR0 |= (BIT0 << OCF0A); 
        } 
        else {
            TIMSK0 &= ~(BIT0 << OCIE0B); 
            TIFR0 |= (BIT0 << OCF0B); 
        }

        TCCR0A = _asyncSavedTCCR0A;
        TCCR0B = _asyncSavedTCCR0B;
        TCNT0 = _asyncSavedTCNT0;
        OCR0A = _asyncSavedOCR0A;
        OCR0B = _asyncSavedOCR0B;
        TIMSK0 = _asyncSavedTIMSK0;
        TIFR0 = _asyncSavedTIFR0;
        SREG = _asyncSavedSREG; 

        _asyncDelayActive = false;
        _asyncCurrentTicks = 0;
        _asyncOverflowsCount = 0;
        _asyncRemainingTicksValue = 0;
    }
}

static uint32_t getPrescaler(clb::TSyncClock clock) {
    switch (clock) {
        case clb::TSyncClock::STOPPED: return 0;
        case clb::TSyncClock::DIV_1: return 1;
        case clb::TSyncClock::DIV_8: return 8;
        case clb::TSyncClock::DIV_64: return 64;
        case clb::TSyncClock::DIV_256: return 256;
        case clb::TSyncClock::DIV_1024: return 1024;
        default: return 1; 
    }
}

static uint64_t calculateTicks(uint32_t value, clb::TTimeUnit unit, uint32_t prescaler) {
    uint64_t _microseconds = 0;
    switch (unit) {
        case clb::TTimeUnit::SECONDS:      _microseconds = (uint64_t)value * 1000000UL; break;
        case clb::TTimeUnit::MILLISECONDS: _microseconds = (uint64_t)value * 1000UL; break;
        case clb::TTimeUnit::MICROSECONDS: _microseconds = (uint64_t)value; break;
        default: return 0; 
    }

    //ticks = (total_microseconds * F_CPU) / (prescaler_value * 1,000,000)
    uint64_t _ticks = (_microseconds * F_CPU) / (prescaler * 1000000UL);

    return _ticks;
}

volatile uint8_t* getOcrRegister(clb::TOutputChannel channel) {
    switch (channel) {
        case clb::TOutputChannel::A: return &OCR0A;
        case clb::TOutputChannel::B: return &OCR0B;
        default: return nullptr; 
    }
}

uint8_t getOcFlagBit(clb::TOutputChannel channel) {
    switch (channel) {
        case clb::TOutputChannel::A: return OCF0A;
        case clb::TOutputChannel::B: return OCF0B;
        default: return 0;
    }
}