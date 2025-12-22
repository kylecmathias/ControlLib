#include "clbTimer.h"

static clb::Timer2* s_active_timer2_instance = nullptr;

static uint32_t getPrescaler(clb::TAsynClock clock);
static uint64_t calculateTicks(uint32_t value, clb::TTimeUnit unit, uint32_t prescaler);
volatile uint8_t* getOcrRegister(clb::TOutputChannel channel);
uint8_t getOcFlagBit(clb::TOutputChannel channel);

static struct Timer2InterruptHandlers {
    void (*compareMatchACallback)() = nullptr;
    void (*compareMatchBCallback)() = nullptr;
    void (*overflowCallback)() = nullptr;
} s_timer2_handlers;

//global ISRs for Timer2
ISR(TIMER2_COMPA_vect) {
    if (s_active_timer2_instance && s_active_timer2_instance->_asyncDelayActive && s_active_timer2_instance->_asyncDelayActiveChannel == clb::TOutputChannel::A) {
        s_active_timer2_instance->_asyncOverflowsCount--;
        if (s_active_timer2_instance->_asyncOverflowsCount == 0) {
            s_active_timer2_instance->_asyncDelayActive = false;

            TIMSK2 &= ~(BIT0 << OCIE2A);

            uint8_t temp_sreg = SREG;
            cli();
            TCCR2A = s_active_timer2_instance->_asyncSavedTCCR2A;
            TCCR2B = s_active_timer2_instance->_asyncSavedTCCR2B;
            TCNT2  = s_active_timer2_instance->_asyncSavedTCNT2;
            OCR2A  = s_active_timer2_instance->_asyncSavedOCR2A;
            OCR2B  = s_active_timer2_instance->_asyncSavedOCR2B;
            TIMSK2 = s_active_timer2_instance->_asyncSavedTIMSK2;
            TIFR2  = s_active_timer2_instance->_asyncSavedTIFR2;
            SREG   = temp_sreg;

            if (s_timer2_handlers.compareMatchACallback) {
                s_timer2_handlers.compareMatchACallback();
            }
        }
        else {
            if (s_active_timer2_instance->_asyncOverflowsCount == 1) {
                OCR2A = s_active_timer2_instance->_asyncRemainingTicksValue;
            }
            else {
                OCR2A = 255;
            }
        }
    }
    else {
        if (s_timer2_handlers.compareMatchACallback) {
            s_timer2_handlers.compareMatchACallback();
        }
    }
}

ISR(TIMER2_COMPB_vect) {
    if (s_active_timer2_instance && s_active_timer2_instance->_asyncDelayActive && s_active_timer2_instance->_asyncDelayActiveChannel == clb::TOutputChannel::B) {
        s_active_timer2_instance->_asyncOverflowsCount--;
        if (s_active_timer2_instance->_asyncOverflowsCount == 0) {
            s_active_timer2_instance->_asyncDelayActive = false;
            TIMSK2 &= ~(BIT0 << OCIE2B);

            uint8_t temp_sreg = SREG;
            cli();
            TCCR2A = s_active_timer2_instance->_asyncSavedTCCR2A;
            TCCR2B = s_active_timer2_instance->_asyncSavedTCCR2B;
            TCNT2  = s_active_timer2_instance->_asyncSavedTCNT2;
            OCR2A  = s_active_timer2_instance->_asyncSavedOCR2A;
            OCR2B  = s_active_timer2_instance->_asyncSavedOCR2B;
            TIMSK2 = s_active_timer2_instance->_asyncSavedTIMSK2;
            TIFR2  = s_active_timer2_instance->_asyncSavedTIFR2;
            SREG   = temp_sreg;

            if (s_timer2_handlers.compareMatchBCallback) {
                s_timer2_handlers.compareMatchBCallback();
            }
        }
        else {
            if (s_active_timer2_instance->_asyncOverflowsCount == 1) {
                OCR2B = s_active_timer2_instance->_asyncRemainingTicksValue;
            }
            else {
                OCR2B = 255;
            }
        }
    }
    else {
        if (s_timer2_handlers.compareMatchBCallback) {
            s_timer2_handlers.compareMatchBCallback();
        }
    }
}

// Timer2 constructor/destructor
clb::Timer2::Timer2() {
    WARNING("Using Timer2 is not recommended since any change will basically break the tone() and noTone() functions.");
    if (s_active_timer2_instance != nullptr) {
        FATAL("There is already an instance of Timer2. Only one instance is allowed.");
    }
    s_active_timer2_instance = this;
    _asyncDelayActive = false;
    _asyncTargetTicks = 0;
    _asyncOverflowsCount = 0;
    _asyncRemainingTicksValue = 0;
    _asyncDelayActiveChannel = clb::TOutputChannel::A;

    _asyncSavedTCCR2A = 0;
    _asyncSavedTCCR2B = 0;
    _asyncSavedTCNT2 = 0;
    _asyncSavedOCR2A = 0;
    _asyncSavedOCR2B = 0;
    _asyncSavedTIMSK2 = 0;
    _asyncSavedTIFR2 = 0;
    _asyncSavedSREG = 0;
}

clb::Timer2::~Timer2() {
    deactivate();
}

void clb::Timer2::deactivate() {
    if (_asyncDelayActive) {
        stopAsyncDelay();
    }
    
    cli();

    TIMSK2 = 0;
    TIFR2 = (BIT0 << OCF2A) | (BIT0 << OCF2B) | (BIT0 << TOV2);
    TCCR2B = 0;

    TCCR2A = 0;
    OCR2A = 0;
    OCR2B = 0;
    TCNT2 = 0;

    sei();

    s_active_timer2_instance = nullptr;
}

//set the mode in TCCR2A and TCCR2B
void clb::Timer2::setMode(TMode8 mode) {
    uint8_t _TCCR2A = TCCR2A;
    uint8_t _TCCR2B = TCCR2B;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x07;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;
    uint8_t _bit2 = (_mode >> 2) & BIT0;

    _bit0 <<= WGM20;
    _bit1 <<= WGM21;
    _bit2 <<= WGM22;

    _TCCR2A &= ~(BIT1 | BIT0);
    _TCCR2B &= ~(BIT3);
    _TCCR2A |= (_bit1 | _bit0);
    _TCCR2B |= (_bit2);

    TCCR2A = _TCCR2A;
    TCCR2B = _TCCR2B;
}

//set the clock in TCCR2B
void clb::Timer2::setClock(TAsynClock clock) {
    _clockSource = static_cast<uint8_t>(clock) & 0x07;
}

//set the compare match output mode for OC2A
void clb::Timer2::setCompareMatchOutputModeA(TCMOM mode) {
    uint8_t _TCCR2A = TCCR2A;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x03;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;

    _bit0 <<= COM2A0;
    _bit1 <<= COM2A1;

    _TCCR2A &= ~(BIT7 | BIT6);
    _TCCR2A |= (_bit1 | _bit0);

    TCCR2A = _TCCR2A;
}

//set the compare match output mode for OC2B
void clb::Timer2::setCompareMatchOutputModeB(TCMOM mode) {
    uint8_t _TCCR2A = TCCR2A;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x03;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;

    _bit0 <<= COM2B0;
    _bit1 <<= COM2B1;

    _TCCR2A &= ~(BIT5 | BIT4);
    _TCCR2A |= (_bit1 | _bit0);

    TCCR2A = _TCCR2A;
}

//set the compare match value in OCR2A
void clb::Timer2::setCompareMatchValueA(uint8_t value) { OCR2A = value; }

//set the compare match value in OCR2B
void clb::Timer2::setCompareMatchValueB(uint8_t value) { OCR2B = value; }

//set the interrupt callback for the timer
void clb::Timer2::setInterruptCallback(TInterrupt8 type, void (*callback)()) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            s_timer2_handlers.compareMatchACallback = callback;
            break;
        case TInterrupt8::COMPMATCHB:
            s_timer2_handlers.compareMatchBCallback = callback;
            break;
        case TInterrupt8::OVERFLOW:
            s_timer2_handlers.overflowCallback = callback;
            break;
        default:
            CRITICAL("Invalid interrupt type for setting callback");
            break;
    }
}

void clb::Timer2::enableInterrupt(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            TIMSK2 |= BIT0 << OCIE2A;
            break;
        case TInterrupt8::COMPMATCHB:
            TIMSK2 |= BIT0 << OCIE2B;
            break;
        case TInterrupt8::OVERFLOW:
            TIMSK2 |= BIT0 << TOIE2;
            break;
        default:
            CRITICAL("Invalid interrupt type for enabling interrupt");
            break;
    }
}

void clb::Timer2::disableInterrupt(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            TIMSK2 &= ~(BIT0 << OCIE2A);
            break;
        case TInterrupt8::COMPMATCHB:
            TIMSK2 &= ~(BIT0 << OCIE2B);
            break;
        case TInterrupt8::OVERFLOW:
            TIMSK2 &= ~(BIT0 << TOIE2);
            break;
        default:
            CRITICAL("Invalid interrupt type for disabling interrupt");
            break;
    }
}

bool clb::Timer2::getInterruptFlag(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            return (TIFR2 & BIT0 << OCF2A);
        case TInterrupt8::COMPMATCHB:
            return (TIFR2 & BIT0 << OCF2B);
        case TInterrupt8::OVERFLOW:
            return (TIFR2 & BIT0 << TOV2);
        default:
            CRITICAL("Invalid interrupt type for getting interrupt flag");
            break;
    }
    return false;
}

void clb::Timer2::clearInterruptFlag(TInterrupt8 type) {
    switch (type) {
        case TInterrupt8::COMPMATCHA:
            TIFR2 |= BIT0 << OCF2A;
            break;
        case TInterrupt8::COMPMATCHB:
            TIFR2 |= BIT0 << OCF2B;
            break;
        case TInterrupt8::OVERFLOW:
            TIFR2 |= BIT0 << TOV2;
            break;
        default:
            CRITICAL("Invalid interrupt type for clearing interrupt flag");
            break;
    } 
}

void clb::Timer2::startTimer() {
    if (_clockSource == 0) {
        FATAL("Clock source was not set, timer doesn't start");
    }
    uint8_t _TCCR2B = TCCR2B;

    _TCCR2B &= ~(BIT2 | BIT1 | BIT0);
    _TCCR2B |= _clockSource;

    TCCR2B = _TCCR2B;
}

void clb::Timer2::stopTimer() {
    if (_asyncDelayActive) {
        stopAsyncDelay();
    }

    uint8_t _TCCR2B = TCCR2B;

    _TCCR2B &= ~(BIT2 | BIT1 | BIT0);

    TCCR2B = _TCCR2B;
}

uint8_t clb::Timer2::getTimerValue8() { return TCNT2; }

void clb::Timer2::setTimerValue(uint8_t value) { TCNT2 = value; }

void clb::Timer2::forceOutputCompareA() { TCCR2B |= BIT0 << FOC2A; }

void clb::Timer2::forceOutputCompareB() { TCCR2B |= BIT0 << FOC2B; }

void clb::Timer2::setAsynchronousClock(clb::TACLK clk) {
    cli();
    uint8_t _CS2 = TCCR2B & (BIT0 | BIT1 | BIT2);
    uint8_t _TCNT2 = TCNT2;
    uint8_t _OCR2A = OCR2A;
    uint8_t _OCR2B = OCR2B;
    uint8_t _TCCR2A = TCCR2A;
    uint8_t _TCCR2B = TCCR2B;

    TCCR2B &= ~(BIT0 | BIT1 | BIT2);

    switch (clk) {
        case clb::TACLK::CLKIO:
            ASSR &= ~(BIT0 << EXCLK);
            ASSR &= ~(BIT0 << AS2);
            break;
        case clb::TACLK::OSC:
            ASSR &= ~(BIT0 << EXCLK);
            ASSR |= (BIT0 << AS2);
            while (ASSR & ((BIT0 << TCN2UB) | (BIT0 << OCR2AUB) | (BIT0 << OCR2BUB) | (BIT0 << TCR2AUB) | (BIT0 << TCR2BUB))) {
                // Wait for the registers to be updated
            }
            WARNING("Modifying TCNT2, OCR2A, OCR2B, TCCR2A, and TCCR2B must be done after polling their corresponding busy flags in async mode");
            break;
        case clb::TACLK::SQRWAVE:
            ASSR |= (BIT0 << EXCLK);
            ASSR |= (BIT0 << AS2);
            while (ASSR & ((BIT0 << TCN2UB) | (BIT0 << OCR2AUB) | (BIT0 << OCR2BUB) | (BIT0 << TCR2AUB) | (BIT0 << TCR2BUB))) {
                // Wait for the registers to be updated
            }
            WARNING("Modifying TCNT2, OCR2A, OCR2B, TCCR2A, and TCCR2B must be done after polling their corresponding busy flags in async mode");
            break;
        default:
            CRITICAL("Invalid TACLK value for Timer2");
            break;
    }
    TCNT2 = _TCNT2;
    if (ASSR & (BIT0 << AS2)) {while (ASSR & (BIT0 << TCN2UB));}

    OCR2A = _OCR2A;
    if (ASSR & (BIT0 << AS2)) {while (ASSR & (BIT0 << OCR2AUB));}

    OCR2B = _OCR2B;
    if (ASSR & (BIT0 << AS2)) {while (ASSR & (BIT0 << OCR2BUB));}

    TCCR2A = _TCCR2A;
    if (ASSR & (BIT0 << AS2)) {while (ASSR & (BIT0 << TCR2AUB));}

    TCCR2B = _TCCR2B | _CS2;
    if (ASSR & (BIT0 << AS2)) {while (ASSR & (BIT0 << TCR2BUB));}

    sei();
}

bool clb::Timer2::getBusyFlag(clb::TBusyFlag flag) {
    return (ASSR & (BIT0 << static_cast<uint8_t>(flag))) != 0;
}

void clb::Timer2::syncDelay(uint32_t time) {
    syncDelay(time, clb::TTimeUnit::MILLISECONDS, clb::TOutputChannel::B);
}

void clb::Timer2::syncDelay(uint32_t time, clb::TTimeUnit unit) {
    syncDelay(time, unit, clb::TOutputChannel::B);
}

void clb::Timer2::syncDelay(uint32_t time, clb::TTimeUnit unit, clb::TOutputChannel channel) {
    uint32_t _prescaler;
    clb::TAsynClock _clockSourceEnum = static_cast<clb::TAsynClock>(this->_clockSource);

    if (_clockSourceEnum == clb::TAsynClock::STOPPED) {
        _prescaler = 256;
    } else {
        _prescaler = getPrescaler(_clockSourceEnum);
    }

    uint64_t _ticks = calculateTicks(time, unit, _prescaler);

    syncDelayLogic(_ticks, channel);
}

void clb::Timer2::asyncDelay(uint32_t time) {
    asyncDelay(time, clb::TTimeUnit::MILLISECONDS, clb::TOutputChannel::A);
}

void clb::Timer2::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit) {
    asyncDelay(time, timeUnit, clb::TOutputChannel::A);
}

void clb::Timer2::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit, clb::TOutputChannel channel) {
    if (channel != clb::TOutputChannel::A && channel != clb::TOutputChannel::B) {
        CRITICAL("Timer2 only supports TOutputChannel::A and TOutputChannel::B for asyncDelay.");
        return;
    }

    uint32_t _prescaler;
    clb::TAsynClock _clockSourceEnum = static_cast<clb::TAsynClock>(this->_clockSource);

    if (_clockSourceEnum == clb::TAsynClock::STOPPED) {
        _prescaler = 64;
    } else {
        _prescaler = getPrescaler(_clockSourceEnum);
    }

    if (_prescaler == 0) {
        FATAL("Calculated prescaler for Timer2 asyncDelay is 0 (STOPPED clock source), cannot calculate ticks.");
        return;
    }

    uint64_t calculatedTicks = calculateTicks(time, timeUnit, _prescaler);

    asyncDelayLogic(calculatedTicks, channel);
}

//helpers
void clb::Timer2::syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) {
    uint8_t _sreg = SREG;
    cli();

    uint8_t _tccr2a = TCCR2A;
    uint8_t _tccr2b = TCCR2B;
    uint8_t _tcnt2 = TCNT2;
    uint8_t _ocr2a = OCR2A;
    uint8_t _ocr2b = OCR2B;
    uint8_t _timsk2 = TIMSK2;
    uint8_t _tifr2 = TIFR2;

    volatile uint8_t* _ocr_reg = getOcrRegister(channel);
    uint8_t _oc_flag_bit = getOcFlagBit(channel);

    TCCR2A = 0;
    TCCR2B = 0;

    uint32_t _prescaler = getPrescaler(static_cast<clb::TAsynClock>(this->_clockSource));
    if (_prescaler == 0) {
        TCCR2B = (BIT0 << CS22);
    }
    else {
        TCCR2B = this->_clockSource;
    }

    TCNT2 = 0;
    TIFR2 = (BIT0 << _oc_flag_bit) | (BIT0 << TOV2);

    const uint16_t MAX_TIMER2_TICKS = 256;

    uint32_t _overflows = ticks / MAX_TIMER2_TICKS;
    uint16_t _remaining_ticks = ticks % MAX_TIMER2_TICKS;

    for (uint32_t i = 0; i < _overflows; i++) {
        while (!(TIFR2 & (BIT0 << TOV2))) {
        }
        TIFR2 |= (BIT0 << TOV2);
    }
    if (_remaining_ticks > 0) {
        *_ocr_reg = _remaining_ticks - 1;
        while (!(TIFR2 & (BIT0 << _oc_flag_bit))) { }
        TIFR2 |= (BIT0 << _oc_flag_bit);
    }

    TCCR2A = _tccr2a;
    TCCR2B = _tccr2b;
    TCNT2 = _tcnt2;
    OCR2A = _ocr2a;
    OCR2B = _ocr2b;
    TIMSK2 = _timsk2;
    TIFR2 = _tifr2;

    SREG = _sreg;
}

void clb::Timer2::asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) {
    if (_asyncDelayActive) {
        WARNING("An asynchronous delay is already active on Timer2. Cannot start a new one.");
        return;
    }
    if (ticks == 0) {
        WARNING("asyncDelay(0) called. Delay will complete immediately.");
        _asyncDelayActive = false;
        return;
    }

    _asyncSavedSREG = SREG;
    cli();

    _asyncSavedTCCR2A = TCCR2A;
    _asyncSavedTCCR2B = TCCR2B;
    _asyncSavedTCNT2 = TCNT2;
    _asyncSavedOCR2A = OCR2A;
    _asyncSavedOCR2B = OCR2B;
    _asyncSavedTIMSK2 = TIMSK2;
    _asyncSavedTIFR2 = TIFR2;

    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;

    const uint16_t MAX_TIMER2_TICKS = 256;

    _asyncTargetTicks = ticks;

    uint32_t numFullCycles = ticks / MAX_TIMER2_TICKS;
    uint8_t remainderTicks = ticks % MAX_TIMER2_TICKS;

    if (remainderTicks == 0) {
        _asyncOverflowsCount = numFullCycles;
        _asyncRemainingTicksValue = MAX_TIMER2_TICKS - 1;
    }
    else {
        _asyncOverflowsCount = numFullCycles + 1;
        _asyncRemainingTicksValue = remainderTicks - 1;
    }

    TCCR2A |= (BIT0 << WGM21);

    uint32_t prescaler_val_for_setup = getPrescaler(static_cast<clb::TAsynClock>(this->_clockSource));
    if (prescaler_val_for_setup == 0) {
        TCCR2B |= (BIT0 << CS21) | (BIT0 << CS20);
    }
    else {
        TCCR2B |= this->_clockSource;
    }

    if (channel == clb::TOutputChannel::A) {
        TIFR2 |= (BIT0 << OCF2A);
        if (numFullCycles == 0 && remainderTicks > 0) {
            OCR2A = _asyncRemainingTicksValue;
        }
        else {
            OCR2A = MAX_TIMER2_TICKS - 1;
        }
        TIMSK2 |= (BIT0 << OCIE2A);
    }
    else {
        TIFR2 |= (BIT0 << OCF2B);
        if (numFullCycles == 0 && remainderTicks > 0) {
            OCR2B = _asyncRemainingTicksValue;
        }
        else {
            OCR2B = MAX_TIMER2_TICKS - 1;
        }
        TIMSK2 |= (BIT0 << OCIE2B);
    }

    _asyncDelayActive = true;
    _asyncDelayActiveChannel = channel;

    SREG = _asyncSavedSREG;
}

bool clb::Timer2::isAsyncDelayFinished() {
    return !_asyncDelayActive;
}

void clb::Timer2::stopAsyncDelay() {
    if (_asyncDelayActive) {
        WARNING("Stopping active asynchronous delay on Timer2.");

        uint8_t temp_sreg = SREG;
        cli();

        if (_asyncDelayActiveChannel == clb::TOutputChannel::A) {
            TIMSK2 &= ~(BIT0 << OCIE2A);
            TIFR2 |= (BIT0 << OCF2A);
        }
        else {
            TIMSK2 &= ~(BIT0 << OCIE2B);
            TIFR2 |= (BIT0 << OCF2B);
        }

        TCCR2A = _asyncSavedTCCR2A;
        TCCR2B = _asyncSavedTCCR2B;
        TCNT2 = _asyncSavedTCNT2;
        OCR2A = _asyncSavedOCR2A;
        OCR2B = _asyncSavedOCR2B;
        TIMSK2 = _asyncSavedTIMSK2;
        TIFR2 = _asyncSavedTIFR2;
        SREG = _asyncSavedSREG;

        _asyncDelayActive = false;
        _asyncCurrentTicks = 0;
        _asyncOverflowsCount = 0;
        _asyncRemainingTicksValue = 0;
    }
}

static uint32_t getPrescaler(clb::TAsynClock clock) {
    switch (clock) {
        case clb::TAsynClock::STOPPED: return 0;
        case clb::TAsynClock::DIV_1: return 1;
        case clb::TAsynClock::DIV_8: return 8;
        case clb::TAsynClock::DIV_32: return 32;
        case clb::TAsynClock::DIV_64: return 64;
        case clb::TAsynClock::DIV_128: return 128;
        case clb::TAsynClock::DIV_256: return 256;
        case clb::TAsynClock::DIV_1024: return 1024;
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
        case clb::TOutputChannel::A: return &OCR2A;
        case clb::TOutputChannel::B: return &OCR2B;
        default: return nullptr;
    }
}

uint8_t getOcFlagBit(clb::TOutputChannel channel) {
    switch (channel) {
        case clb::TOutputChannel::A: return OCF2A;
        case clb::TOutputChannel::B: return OCF2B;
        default: return 0;
    }
}

