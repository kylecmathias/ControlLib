#include "clbTimer.h"

static clb::Timer1* s_active_timer1_instance = nullptr;

static uint32_t getPrescaler(clb::TAsynClock clock);
static uint64_t calculateTicks(uint32_t value, clb::TTimeUnit unit, uint32_t prescaler);
volatile uint16_t* getOcrRegister(clb::TOutputChannel channel);
uint16_t getOcFlagBit(clb::TOutputChannel channel);

static struct Timer1InterruptHandlers {
    void (*compareMatchACallback)() = nullptr;
    void (*compareMatchBCallback)() = nullptr;
    void (*overflowCallback)() = nullptr;
} s_timer1_handlers;

//global ISRs for Timer1
ISR(TIMER1_COMPA_vect) {
    if (s_active_timer1_instance && s_active_timer1_instance->_asyncDelayActive && s_active_timer1_instance->_asyncDelayActiveChannel == clb::TOutputChannel::A) {
        s_active_timer1_instance->_asyncOverflowsCount--;
        if (s_active_timer1_instance->_asyncOverflowsCount == 0) {
            s_active_timer1_instance->_asyncDelayActive = false;

            TIMSK1 &= ~(BIT0 << OCIE1A);

            uint8_t temp_sreg = SREG;
            cli();
            TCCR1A = s_active_timer1_instance->_asyncSavedTCCR1A;
            TCCR1B = s_active_timer1_instance->_asyncSavedTCCR1B;
            TCNT1  = s_active_timer1_instance->_asyncSavedTCNT1;
            OCR1A  = s_active_timer1_instance->_asyncSavedOCR1A;
            OCR1B  = s_active_timer1_instance->_asyncSavedOCR1B;
            TIMSK1 = s_active_timer1_instance->_asyncSavedTIMSK1;
            TIFR1  = s_active_timer1_instance->_asyncSavedTIFR1;
            SREG   = temp_sreg;

            if (s_timer1_handlers.compareMatchACallback) {
                s_timer1_handlers.compareMatchACallback();
            }
        }
        else {
            if (s_active_timer1_instance->_asyncOverflowsCount == 1) {
                OCR1A = s_active_timer1_instance->_asyncRemainingTicksValue;
            }
            else {
                OCR1A = 255;
            }
        }
    }
    else {
        if (s_timer1_handlers.compareMatchACallback) {
            s_timer1_handlers.compareMatchACallback();
        }
    }
}

ISR(TIMER1_COMPB_vect) {
    if (s_active_timer1_instance && s_active_timer1_instance->_asyncDelayActive && s_active_timer1_instance->_asyncDelayActiveChannel == clb::TOutputChannel::B) {
        s_active_timer1_instance->_asyncOverflowsCount--;
        if (s_active_timer1_instance->_asyncOverflowsCount == 0) {
            s_active_timer1_instance->_asyncDelayActive = false;
            TIMSK1 &= ~(BIT0 << OCIE1B);

            uint8_t temp_sreg = SREG;
            cli();
            TCCR1A = s_active_timer1_instance->_asyncSavedTCCR1A;
            TCCR1B = s_active_timer1_instance->_asyncSavedTCCR1B;
            TCNT1  = s_active_timer1_instance->_asyncSavedTCNT1;
            OCR1A  = s_active_timer1_instance->_asyncSavedOCR1A;
            OCR1B  = s_active_timer1_instance->_asyncSavedOCR1B;
            TIMSK1 = s_active_timer1_instance->_asyncSavedTIMSK1;
            TIFR1  = s_active_timer1_instance->_asyncSavedTIFR1;
            SREG   = temp_sreg;

            if (s_timer1_handlers.compareMatchBCallback) {
                s_timer1_handlers.compareMatchBCallback();
            }
        }
        else {
            if (s_active_timer1_instance->_asyncOverflowsCount == 1) {
                OCR1B = s_active_timer1_instance->_asyncRemainingTicksValue;
            }
            else {
                OCR1B = 255;
            }
        }
    }
    else {
        if (s_timer1_handlers.compareMatchBCallback) {
            s_timer1_handlers.compareMatchBCallback();
        }
    }
}

// Timer1 constructor/destructor
clb::Timer1::Timer1() {
    if (s_active_timer1_instance != nullptr) {
        FATAL("There is already an instance of Timer1. Only one instance is allowed.");
    }
    s_active_timer1_instance = this;
    _asyncDelayActive = false;
    _asyncTargetTicks = 0;
    _asyncOverflowsCount = 0;
    _asyncRemainingTicksValue = 0;
    _asyncDelayActiveChannel = clb::TOutputChannel::A;

    _asyncSavedTCCR1A = 0;
    _asyncSavedTCCR1B = 0;
    _asyncSavedTCNT1 = 0;
    _asyncSavedOCR1A = 0;
    _asyncSavedOCR1B = 0;
    _asyncSavedTIMSK1 = 0;
    _asyncSavedTIFR1 = 0;
    _asyncSavedSREG = 0;
}

clb::Timer1::~Timer1() {
    deactivate();
}

void clb::Timer1::deactivate() {
    if (_asyncDelayActive) {
        stopAsyncDelay();
    }
    
    cli();

    TIMSK1 = 0;
    TIFR1 = (BIT0 << OCF1A) | (BIT0 << OCF1B) | (BIT0 << TOV1);
    TCCR1B = 0;

    TCCR1A = 0;
    OCR1A = 0;
    OCR1B = 0;
    TCNT1 = 0;

    sei();

    s_active_timer1_instance = nullptr;
}

//set the mode in TCCR1A and TCCR1B
void clb::Timer1::setMode(TMode16 mode) {
    uint8_t _TCCR1A = TCCR1A;
    uint8_t _TCCR1B = TCCR1B;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x07;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;
    uint8_t _bit2 = (_mode >> 2) & BIT0;

    _bit0 <<= WGM10;
    _bit1 <<= WGM11;
    _bit2 <<= WGM12;

    _TCCR1A &= ~(BIT1 | BIT0);
    _TCCR1B &= ~(BIT3);
    _TCCR1A |= (_bit1 | _bit0);
    _TCCR1B |= (_bit2);

    TCCR1A = _TCCR1A;
    TCCR1B = _TCCR1B;
}

//set the clock in TCCR1B
void clb::Timer1::setClock(TSyncClock clock) {
    _clockSource = static_cast<uint8_t>(clock) & 0x07;
}

//set the compare match output mode for OC1A
void clb::Timer1::setCompareMatchOutputModeA(TCMOM mode) {
    uint8_t _TCCR1A = TCCR1A;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x03;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;

    _bit0 <<= COM1A0;
    _bit1 <<= COM1A1;

    _TCCR1A &= ~(BIT7 | BIT6);
    _TCCR1A |= (_bit1 | _bit0);

    TCCR1A = _TCCR1A;
}

//set the compare match output mode for OC1B
void clb::Timer1::setCompareMatchOutputModeB(TCMOM mode) {
    uint8_t _TCCR1A = TCCR1A;

    uint8_t _mode = static_cast<uint8_t>(mode) & 0x03;
    uint8_t _bit0 = (_mode >> 0) & BIT0;
    uint8_t _bit1 = (_mode >> 1) & BIT0;

    _bit0 <<= COM1B0;
    _bit1 <<= COM1B1;

    _TCCR1A &= ~(BIT5 | BIT4);
    _TCCR1A |= (_bit1 | _bit0);

    TCCR1A = _TCCR1A;
}

//set the compare match value in OCR1A
void clb::Timer1::setCompareMatchValueA(uint16_t value) { OCR1A = value; }

//set the compare match value in OCR1B
void clb::Timer1::setCompareMatchValueB(uint16_t value) { OCR1B = value; }

//set the interrupt callback for the timer
void clb::Timer1::setInterruptCallback(TInterrupt16 type, void (*callback)()) {
    switch (type) {
        case TInterrupt16::COMPMATCHA:
            s_timer1_handlers.compareMatchACallback = callback;
            break;
        case TInterrupt16::COMPMATCHB:
            s_timer1_handlers.compareMatchBCallback = callback;
            break;
        case TInterrupt16::OVERFLOW:
            s_timer1_handlers.overflowCallback = callback;
            break;
        default:
            CRITICAL("Invalid interrupt type for setting callback");
            break;
    }
}

void clb::Timer1::enableInterrupt(TInterrupt16 type) {
    switch (type) {
        case TInterrupt16::COMPMATCHA:
            TIMSK1 |= BIT0 << OCIE1A;
            break;
        case TInterrupt16::COMPMATCHB:
            TIMSK1 |= BIT0 << OCIE1B;
            break;
        case TInterrupt16::OVERFLOW:
            TIMSK1 |= BIT0 << TOIE1;
            break;
        default:
            CRITICAL("Invalid interrupt type for enabling interrupt");
            break;
    }
}

void clb::Timer1::disableInterrupt(TInterrupt16 type) {
    switch (type) {
        case TInterrupt16::COMPMATCHA:
            TIMSK1 &= ~(BIT0 << OCIE1A);
            break;
        case TInterrupt16::COMPMATCHB:
            TIMSK1 &= ~(BIT0 << OCIE1B);
            break;
        case TInterrupt16::OVERFLOW:
            TIMSK1 &= ~(BIT0 << TOIE1);
            break;
        default:
            CRITICAL("Invalid interrupt type for disabling interrupt");
            break;
    }
}

bool clb::Timer1::getInterruptFlag(TInterrupt16 type) {
    switch (type) {
        case TInterrupt16::COMPMATCHA:
            return (TIFR1 & BIT0 << OCF1A);
        case TInterrupt16::COMPMATCHB:
            return (TIFR1 & BIT0 << OCF1B);
        case TInterrupt16::OVERFLOW:
            return (TIFR1 & BIT0 << TOV1);
        default:
            CRITICAL("Invalid interrupt type for getting interrupt flag");
            break;
    }
    return false;
}

void clb::Timer1::clearInterruptFlag(TInterrupt16 type) {
    switch (type) {
        case TInterrupt16::COMPMATCHA:
            TIFR1 |= BIT0 << OCF1A;
            break;
        case TInterrupt16::COMPMATCHB:
            TIFR1 |= BIT0 << OCF1B;
            break;
        case TInterrupt16::OVERFLOW:
            TIFR1 |= BIT0 << TOV1;
            break;
        default:
            CRITICAL("Invalid interrupt type for clearing interrupt flag");
            break;
    } 
}

void clb::Timer1::startTimer() {
    if (_clockSource == 0) {
        FATAL("Clock source was not set, timer doesn't start");
    }
    uint8_t _TCCR1B = TCCR1B;

    _TCCR1B &= ~(BIT2 | BIT1 | BIT0);
    _TCCR1B |= _clockSource;

    TCCR1B = _TCCR1B;
}

void clb::Timer1::stopTimer() {
    if (_asyncDelayActive) {
        stopAsyncDelay();
    }

    uint8_t _TCCR1B = TCCR1B;

    _TCCR1B &= ~(BIT2 | BIT1 | BIT0);

    TCCR1B = _TCCR1B;
}

uint16_t clb::Timer1::getTimerValue16() { return TCNT1; }

void clb::Timer1::setTimerValue(uint16_t value) { TCNT1 = value; }

void clb::Timer1::forceOutputCompareA() { TCCR1B |= BIT0 << FOC1A; }

void clb::Timer1::forceOutputCompareB() { TCCR1B |= BIT0 << FOC1B; }

void clb::Timer1::syncDelay(uint32_t time) {
    syncDelay(time, clb::TTimeUnit::MILLISECONDS, clb::TOutputChannel::B);
}

void clb::Timer1::syncDelay(uint32_t time, clb::TTimeUnit unit) {
    syncDelay(time, unit, clb::TOutputChannel::B);
}

void clb::Timer1::syncDelay(uint32_t time, clb::TTimeUnit unit, clb::TOutputChannel channel) {
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

void clb::Timer1::asyncDelay(uint32_t time) {
    asyncDelay(time, clb::TTimeUnit::MILLISECONDS, clb::TOutputChannel::A);
}

void clb::Timer1::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit) {
    asyncDelay(time, timeUnit, clb::TOutputChannel::A);
}

void clb::Timer1::asyncDelay(uint32_t time, clb::TTimeUnit timeUnit, clb::TOutputChannel channel) {
    channel = static_cast<clb::TOutputChannel>(static_cast<uint8_t>(channel) & 0b11);

    uint32_t _prescaler;
    clb::TSyncClock _clockSourceEnum = static_cast<clb::TSyncClock>(this->_clockSource);

    if (_clockSourceEnum == clb::TSyncClock::STOPPED) {
        _prescaler = 64;
    } else {
        _prescaler = getPrescaler(_clockSourceEnum);
    }

    if (_prescaler == 0) {
        FATAL("Calculated prescaler for Timer1 asyncDelay is 0 (STOPPED clock source), cannot calculate ticks.");
        return;
    }

    uint64_t calculatedTicks = calculateTicks(time, timeUnit, _prescaler);

    asyncDelayLogic(calculatedTicks, channel);
}

//helpers
void clb::Timer1::syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) {
    uint8_t _sreg = SREG;
    cli();

    uint8_t _tccr1a = TCCR1A;
    uint8_t _tccr1b = TCCR1B;
    uint8_t _tcnt1 = TCNT1;
    uint8_t _ocr1a = OCR1A;
    uint8_t _ocr1b = OCR1B;
    uint8_t _timsk1 = TIMSK1;
    uint8_t _tifr1 = TIFR1;

    volatile uint16_t* _ocr_reg = getOcrRegister(channel);
    uint8_t _oc_flag_bit = getOcFlagBit(channel);

    TCCR1A = 0;
    TCCR1B = 0;

    uint32_t _prescaler = getPrescaler(static_cast<clb::TSyncClock>(this->_clockSource));
    if (_prescaler == 0) {
        TCCR1B = (BIT0 << CS12);
    }
    else {
        TCCR1B = this->_clockSource;
    }

    TCNT1 = 0;
    TIFR1 = (BIT0 << _oc_flag_bit) | (BIT0 << TOV1);

    const uint32_t MAX_TIMER1_TICKS = 65536;

    uint32_t _overflows = ticks / MAX_TIMER1_TICKS;
    uint16_t _remaining_ticks = ticks % MAX_TIMER1_TICKS;

    for (uint32_t i = 0; i < _overflows; i++) {
        while (!(TIFR1 & (BIT0 << TOV1))) {
        }
        TIFR1 |= (BIT0 << TOV1);
    }
    if (_remaining_ticks > 0) {
        *_ocr_reg = _remaining_ticks - 1;
        while (!(TIFR1 & (BIT0 << _oc_flag_bit))) { }
        TIFR1 |= (BIT0 << _oc_flag_bit);
    }

    TCCR1A = _tccr1a;
    TCCR1B = _tccr1b;
    TCNT1 = _tcnt1;
    OCR1A = _ocr1a;
    OCR1B = _ocr1b;
    TIMSK1 = _timsk1;
    TIFR1 = _tifr1;

    SREG = _sreg;
}

void clb::Timer1::asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) {
    if (_asyncDelayActive) {
        WARNING("An asynchronous delay is already active on Timer1. Cannot start a new one.");
        return;
    }
    if (ticks == 0) {
        WARNING("asyncDelay(0) called. Delay will complete immediately.");
        _asyncDelayActive = false;
        return;
    }

    _asyncSavedSREG = SREG;
    cli();

    _asyncSavedTCCR1A = TCCR1A;
    _asyncSavedTCCR1B = TCCR1B;
    _asyncSavedTCNT1 = TCNT1;
    _asyncSavedOCR1A = OCR1A;
    _asyncSavedOCR1B = OCR1B;
    _asyncSavedTIMSK1 = TIMSK1;
    _asyncSavedTIFR1 = TIFR1;

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    const uint32_t MAX_TIMER1_TICKS = 65536;

    _asyncTargetTicks = ticks;

    uint32_t numFullCycles = ticks / MAX_TIMER1_TICKS;
    uint16_t remainderTicks = ticks % MAX_TIMER1_TICKS;

    if (remainderTicks == 0) {
        _asyncOverflowsCount = numFullCycles;
        _asyncRemainingTicksValue = MAX_TIMER1_TICKS - 1;
    }
    else {
        _asyncOverflowsCount = numFullCycles + 1;
        _asyncRemainingTicksValue = remainderTicks - 1;
    }

    TCCR1A |= (BIT0 << WGM11);

    uint32_t prescaler_val_for_setup = getPrescaler(static_cast<clb::TAsynClock>(this->_clockSource));
    if (prescaler_val_for_setup == 0) {
        TCCR1B |= (BIT0 << CS11) | (BIT0 << CS10);
    }
    else {
        TCCR1B |= this->_clockSource;
    }

    if (channel == clb::TOutputChannel::A) {
        TIFR1 |= (BIT0 << OCF1A);
        if (numFullCycles == 0 && remainderTicks > 0) {
            OCR1A = _asyncRemainingTicksValue;
        }
        else {
            OCR1A = MAX_TIMER1_TICKS - 1;
        }
        TIMSK1 |= (BIT0 << OCIE1A);
    }
    else if (channel == clb::TOutputChannel::B) {
        TIFR1 |= (BIT0 << OCF1B);
        if (numFullCycles == 0 && remainderTicks > 0) {
            OCR1B = _asyncRemainingTicksValue;
        }
        else {
            OCR1B = MAX_TIMER1_TICKS - 1;
        }
        TIMSK1 |= (BIT0 << OCIE1B);
    }
    else {
        TIFR1 |= (BIT0 << OCF1C);
        if (numFullCycles == 0 && remainderTicks > 0) {
            OCR1C = _asyncRemainingTicksValue;
        }
        else {
            OCR1C = MAX_TIMER1_TICKS - 1;
        }
        TIMSK1 |= (BIT0 << OCIE1C);
    }

    _asyncDelayActive = true;
    _asyncDelayActiveChannel = channel;

    SREG = _asyncSavedSREG;
}

bool clb::Timer1::isAsyncDelayFinished() {
    return !_asyncDelayActive;
}

void clb::Timer1::stopAsyncDelay() {
    if (_asyncDelayActive) {
        WARNING("Stopping active asynchronous delay on Timer1.");

        uint8_t temp_sreg = SREG;
        cli();

        if (_asyncDelayActiveChannel == clb::TOutputChannel::A) {
            TIMSK1 &= ~(BIT0 << OCIE1A);
            TIFR1 |= (BIT0 << OCF1A);
        }
        else if (_asyncDelayActiveChannel == clb::TOutputChannel::B) {
            TIMSK1 &= ~(BIT0 << OCIE1B);
            TIFR1 |= (BIT0 << OCF1B);
        }
        else {
            TIMSK1 &= ~(BIT0 << OCIE1C);
            TIFR1 |= (BIT0 << OCF1C);
        }

        TCCR1A = _asyncSavedTCCR1A;
        TCCR1B = _asyncSavedTCCR1B;
        TCNT1 = _asyncSavedTCNT1;
        OCR1A = _asyncSavedOCR1A;
        OCR1B = _asyncSavedOCR1B;
        TIMSK1 = _asyncSavedTIMSK1;
        TIFR1 = _asyncSavedTIFR1;
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
        case clb::TSyncClock::DIV_64: return 32;
        case clb::TSyncClock::DIV_256: return 64;
        case clb::TSyncClock::DIV_1024: return 128;
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

volatile uint16_t* getOcrRegister(clb::TOutputChannel channel) {
    switch (channel) {
        case clb::TOutputChannel::A: return &OCR1A;
        case clb::TOutputChannel::B: return &OCR1B;
        case clb::TOutputChannel::C: return &OCR1C; 
        default: return nullptr;
    }
}

uint16_t getOcFlagBit(clb::TOutputChannel channel) {
    switch (channel) {
        case clb::TOutputChannel::A: return OCF1A;
        case clb::TOutputChannel::B: return OCF1B;
        case clb::TOutputChannel::C: return OCF1C;
        default: return 0;
    }
}

