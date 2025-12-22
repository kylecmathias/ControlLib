/* IMPORTANT NOTES
 * 
 * Timer0 is used by the Arduino core for delay(), millis() and micros() functions. Dont use it unless you will not be using these functions.
 * Timer1 is used by the Arduino core for Servo library, so if you use Servo library, dont use Timer1.
 * Timer2 is used by the Arduino core for tone() function, so if you use tone() function, dont use Timer2.
 * Timers 3, 4 and 5 are not used by the Arduino core, so you can use them freely, although some libraries may use Timer5.
 * 
 * These classes are meant for very low level control over the timers, so you cant use the builtin arduino functions that use the timers you use.
 * 
 * 
 * These are the functions and what timers are used by them:
 * 
 * Timer0 (8 bit) (used by Arduino core for timekeeping)
 * - delay(ms)
 * - delayMicroseconds(us)
 * - millis()
 * - micros()
 * - analogWrite(4, value)
 * - analogWrite(13, value)
 * 
 * Timer1 (16 bit) (used by Arduino for PWM and Servo control)
 * - analogWrite(11, value)
 * - analogWrite(12, value)
 * 
 * Timer2 (8 bit) (used by Arduino for tone generation)
 * - tone()
 * - noTone()
 * - analogWrite(9, value)
 * - analogWrite(10, value)
 */
#ifndef TIMERREG_H
#define TIMERREG_H

#define F_CPU 16000000UL 

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>

#include "clbBits.h"
#include "clbException.h"


//short for ControlLib
namespace clb { 
    class Timer;
    class Timer0;
    class Timer1;
    class Timer2;
    class Timer3;
    class Timer4;
    class Timer5;

    //mode enum for 8 bit timers
    enum class TMode8 : uint8_t {
        NORMAL = 0b000,                  // WGM2=0, WGM1=0, WGM0=0, TOP = 0xFF, counts to 0xFF, overflows, and resets, OCR0A updates immediately
        PWM_PHASE_CORRECT = 0b001,       // WGM2=0, WGM1=0, WGM0=1, TOP = 0xFF, counts to 0xFF, then down to 0x00
        CTC_OCR_A = 0b010,               // WGM2=0, WGM1=1, WGM0=0, TOP = OCRnA, counts to OCR0A, then resets
        FAST_PWM = 0b011,                // WGM2=0, WGM1=1, WGM0=1, TOP = 0xFF, counts to 0xFF, then resets
        RESERVED_0 = 0b100,              // WGM2=1, WGM1=0, WGM0=0, reserved
        PWM_PHASE_CORRECT_OCR_A = 0b101, // WGM2=1, WGM1=0, WGM0=1, TOP = OCRnA, counts to OCR0A, then down to 0x00
        RESERVED_1 = 0b110,              // WGM2=1, WGM1=1, WGM0=0, reserved
        FAST_PWM_OCR_A = 0b111           // WGM2=1, WGM1=1, WGM0=1, TOP = OCRnA, counts to OCR0A, then resets
    };
    //mode enum for 16 bit timers
    enum class TMode16 : uint8_t {
        NORMAL = 0b0000,                            // WGM13=0, WGM12=0, WGM11=0, WGM10=0, TOP = 0xFFFF, Update OCRnx Immediate, TOV1 Flag Set on MAX
        PWM_PHASE_CORRECT_8BIT = 0b0001,            // WGM13=0, WGM12=0, WGM11=0, WGM10=1, TOP = 0x00FF, Update OCRnx on TOP, TOV1 Flag Set on BOTTOM
        PWM_PHASE_CORRECT_9BIT = 0b0010,            // WGM13=0, WGM12=0, WGM11=1, WGM10=0, TOP = 0x01FF, Update OCRnx on TOP, TOV1 Flag Set on BOTTOM
        PWM_PHASE_CORRECT_10BIT = 0b0011,           // WGM13=0, WGM12=0, WGM11=1, WGM10=1, TOP = 0x03FF, Update OCRnx on TOP, TOV1 Flag Set on BOTTOM
        CTC_OCR_A = 0b0100,                         // WGM13=0, WGM12=1, WGM11=0, WGM10=0, TOP = OCR1A, Update OCRnx Immediate, TOV1 Flag Set on MAX
        FAST_PWM_8BIT = 0b0101,                     // WGM13=0, WGM12=1, WGM11=0, WGM10=1, TOP = 0x00FF, Update OCRnx on BOTTOM, TOV1 Flag Set on TOP
        FAST_PWM_9BIT = 0b0110,                     // WGM13=0, WGM12=1, WGM11=1, WGM10=0, TOP = 0x01FF, Update OCRnx on BOTTOM, TOV1 Flag Set on TOP
        FAST_PWM_10BIT = 0b0111,                    // WGM13=0, WGM12=1, WGM11=1, WGM10=1, TOP = 0x03FF, Update OCRnx on BOTTOM, TOV1 Flag Set on TOP
        PWM_PHASE_FREQUENCY_CORRECT_ICR = 0b1000,   // WGM13=1, WGM12=0, WGM11=0, WGM10=0, TOP = ICR1, Update OCRnx on BOTTOM, TOV1 Flag Set on BOTTOM
        PWM_PHASE_FREQUENCY_CORRECT_OCR_A = 0b1001, // WGM13=1, WGM12=0, WGM11=0, WGM10=1, TOP = OCR1A, Update OCRnx on BOTTOM, TOV1 Flag Set on BOTTOM
        PWM_PHASE_CORRECT_ICR = 0b1010,             // WGM13=1, WGM12=0, WGM11=1, WGM10=0, TOP = ICR1, Update OCRnx on TOP, TOV1 Flag Set on BOTTOM
        PWM_PHASE_CORRECT_OCR_A = 0b1011,           // WGM13=1, WGM12=0, WGM11=1, WGM10=1, TOP = OCR1A, Update OCRnx on TOP, TOV1 Flag Set on BOTTOM
        CTC_ICR = 0b1100,                           // WGM13=1, WGM12=1, WGM11=0, WGM10=0, TOP = ICR1, Update OCRnx Immediate, TOV1 Flag Set on MAX
        RESERVED = 0b1101,                          // WGM13=1, WGM12=1, WGM11=0, WGM10=1, Reserved
        FAST_PWM_ICR = 0b1110,                      // WGM13=1, WGM12=1, WGM11=1, WGM10=0, TOP = ICR1, Update OCRnx on BOTTOM, TOV1 Flag Set on TOP
        FAST_PWM_OCR_A = 0b1111                     // WGM13=1, WGM12=1, WGM11=1, WGM10=1, TOP = OCR1A, Update OCRnx on BOTTOM, TOV1 Flag Set on TOP
    };
    //clock enum for synchronous timers
    enum class TSyncClock : uint8_t {
        STOPPED = 0b000,
        DIV_1 = 0b001,
        DIV_8 = 0b010,
        DIV_64 = 0b011,
        DIV_256 = 0b100,
        DIV_1024 = 0b101,
        EXT_CLK_FE = 0b110,
        EXT_CLK_RE = 0b111
    };
    //clock enum for asynchronous timers
    enum class TAsynClock : uint8_t {
        STOPPED = 0b000,
        DIV_1 = 0b001,
        DIV_8 = 0b010,
        DIV_32 = 0b011,
        DIV_64 = 0b100,
        DIV_128 = 0b101,
        DIV_256 = 0b110,
        DIV_1024 = 0b111
    };
    //compare match output modes enum
    enum class TCMOM : uint8_t {
        NORMAL = 0b00,
        TOGGLE = 0b01, //sometimes restricted for B
        CLEAR = 0b10,
        SET = 0b11
    };
    //asynchronous clock input (timer 2 only)
    enum class TACLK : uint8_t {
        CLKIO = 0b00, //regular clock
        OSC = 0b10, //TOSC1 and TOSC2 external oscillator input
        SQRWAVE = 0b11 //square wave input on TOSC1
    };
    //asynchronous registers with busy flags (timer 2 only)
    enum class TBusyFlag : uint8_t {
        TCCRB = 0b000,
        TCCRA = 0b001,
        OCRB = 0b010,
        OCRA = 0b011,
        TCNT = 0b100,
    };
    //8 bit timer interrupt types
    enum class TInterrupt8 : uint8_t {
        COMPMATCHA = 0b000,
        COMPMATCHB = 0b001,
        OVERFLOW = 0b010
    };
    //16 bit timer interrupt types
    enum class TInterrupt16 : uint8_t {
        COMPMATCHA = 0b000,
        COMPMATCHB = 0b001,
        COMPMATCHC = 0b010,
        OVERFLOW = 0b011,
        INPUTCAPTURE = 0b100
    };
    //time units for delays
    enum class TTimeUnit : uint8_t {
        SECONDS = 0b000, //seconds
        MILLISECONDS = 0b001, //milliseconds
        MICROSECONDS = 0b010, //microseconds
    };
    //output channel for selecting register
    enum class TOutputChannel : uint8_t {
        A = 0b000, //OCRA
        B = 0b001, //OCRB
        C = 0b010  //OCRC if timer has it
    };

    //superclass implementation of a timer with direct register control
    class Timer { 
        protected:
            uint8_t _clockSource = 0;
            volatile uint32_t _overflowCount = 0;
            volatile uint32_t _overflowTarget = 0;
        public:
            Timer();
            virtual void deactivate() = 0; //deactivates the timer and resets the registers
            //global timer methods
            static void resetSynchronousPrescalers(); //resets prescalers for timers 0, 1, 3, 4 and 5
            static void resetAsynchronousPrescalers(); //resets prescaler for timer 2
            static void resetAllPrescalers(); //clears all prescalers, stops all timers
            static void startTimerSynchronization(); //starts all timers in synchronous mode
            static void stopTimerSynchronization(); //stops all timers in synchronous mode

            //setup methods
            virtual void setMode(TMode8 mode); //sets the waveform generation mode of the timer (8 bit)
            virtual void setMode(TMode16 mode); //sets the waveform generation mode of the timer (16 bit)
            virtual void setClock(TSyncClock clock); //sets the clock source and prescaler of the timer (synchronous)
            virtual void setClock(TAsynClock clock); //sets the clock source and prescaler of the timer (asynchronous)
            virtual void setCompareMatchOutputModeA(TCMOM mode) = 0; //sets the output mode for pin OC0A
            virtual void setCompareMatchOutputModeB(TCMOM mode) = 0; //sets the output mode for pin OC0B
            virtual void setCompareMatchOutputModeC(TCMOM mode); //sets the output mode for pin OC0C
            virtual void setCompareMatchValueA(uint8_t value); //sets the compare match value for pin OC0A (8 bits)
            virtual void setCompareMatchValueB(uint8_t value); //sets the compare match value for pin OC0B (8 bits)
            virtual void setCompareMatchValueA(uint16_t value); //sets the compare match value for pin OC0A (16 bits)
            virtual void setCompareMatchValueB(uint16_t value); //sets the compare match value for pin OC0B (16 bits)
            virtual void setCompareMatchValueC(uint16_t value); //sets the compare match value for pin OC0C (16 bits)

            /*//overflow handling methods
            virtual void overflow() = 0; //called when the timer overflows, increments the overflow count
            virtual void setOverflowCount(uint32_t count) = 0; //sets the overflow count for the timer, used for delay calculations
            virtual uint32_t getOverflowCount() = 0; //returns the overflow count for the timer, used for delay calculations
            virtual bool isLastOverflow() = 0; //returns true if the last overflow was the last one, used for delay calculations
            */

            //operation methods
            virtual void startTimer() = 0; //starts the timer
            virtual void stopTimer() = 0; //stops the timer
            virtual uint8_t getTimerValue8(); //returns the current value of the timer (8 bit)
            virtual uint16_t getTimerValue16(); //returns the current value of the timer (16 bit)
            virtual void setTimerValue(uint8_t value); //sets the timer value (8 bit), not a good idea to use since can cause a race condition if compare match was about to occur
            virtual void setTimerValue(uint16_t value); //sets the timer value (16 bit), not a good idea to use since can cause a race condition if compare match was about to occur
            virtual void forceOutputCompareA() = 0; //forces a compare match on OC0A
            virtual void forceOutputCompareB() = 0; //forces a compare match on OC0B
            virtual void forceOutputCompareC(); //forces a compare match on OC0C

            //interrupt control methods
            virtual void setInterruptCallback(TInterrupt8 type, void (*callback)()); //sets the callback for the interrupt (8 bit)
            virtual void enableInterrupt(TInterrupt8 type); //enables the interrupt for the timer (8 bit)
            virtual void disableInterrupt(TInterrupt8 type); //disables the interrupt for the timer (8 bit)
            virtual bool getInterruptFlag(TInterrupt8 type); //returns the interrupt flag for the timer (8 bit)
            virtual void clearInterruptFlag(TInterrupt8 type); //clears the interrupt flag for the timer (8 bit)
            virtual void setInterruptCallback(TInterrupt16 type, void (*callback)()); //sets the callback for the interrupt (16 bit)
            virtual void enableInterrupt(TInterrupt16 type); //enables the interrupt for the timer (16 bit)
            virtual void disableInterrupt(TInterrupt16 type); //disables the interrupt for the timer (16 bit)
            virtual bool getInterruptFlag(TInterrupt16 type); //returns the interrupt flag for the timer (16 bit)
            virtual void clearInterruptFlag(TInterrupt16 type); //clears the interrupt flag for the timer (16 bit)    

            //input capture methods
            virtual void inputCaptureNoiseCancelEnable(bool enable); //enables the noise canceler for the input capture
            virtual void inputCaptureEdgeSelect(bool rising); //selects the edge for the input capture

            //direct blocking delay methods
            virtual void syncDelay(uint32_t time) = 0; //delays for a specified time in milliseconds
            virtual void syncDelay(uint32_t time, TTimeUnit timeUnit) = 0; //delays for a specified time in seconds, milliseconds or microseconds
            virtual void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) = 0; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)
        
            //direct non blocking delay methods
            virtual void asyncDelay(uint32_t time) = 0; //delays for a specified time in milliseconds, non-blocking
            virtual void asyncDelay(uint32_t time, TTimeUnit timeUnit) = 0; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            virtual void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) = 0; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            virtual bool isAsyncDelayFinished() = 0; //returns true if the asynchronous delay is finished
            virtual void stopAsyncDelay() = 0; //stops the asynchronous delay
        private:
            //delay logic methods
            virtual void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) = 0; //logic for the delay methods
            virtual void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) = 0; //logic for the non-blocking delay methods
    };
    //subclass timer 0 (8 bits)
    class Timer0 : public Timer {
        public:
            //setup methods
            Timer0();
            ~Timer0();
            void deactivate() override; //deactivates the timer and resets the registers

            void setMode(TMode8 mode) override; //sets the waveform generation mode of the timer
            void setClock(TSyncClock clock) override; //sets the clock source and prescaler of the timer
            void setCompareMatchOutputModeA(TCMOM mode) override; //sets the output mode for pin OC0A
            void setCompareMatchOutputModeB(TCMOM mode) override; //sets the output mode for pin OC0B
            void setCompareMatchValueA(uint8_t value) override; //sets the compare match value for pin OC0A
            void setCompareMatchValueB(uint8_t value) override; //sets the compare match value for pin OC0B

            /*//overflow handling methods
            void overflow() override; //called when the timer overflows, increments the overflow count
            void setOverflowCount(uint32_t count) override; //sets the overflow count for the timer, used for delay calculations
            uint32_t getOverflowCount() override; //returns the overflow count for the timer, used for delay calculations
            bool isLastOverflow() override; //returns true if the last overflow was the last one, used for delay calculations
            */

            //operation methods
            void startTimer() override; //starts the timer
            void stopTimer() override; //stops the timer
            uint8_t getTimerValue8() override; //returns the current value of the timer
            void setTimerValue(uint8_t value) override; //sets the timer value, not a good idea to use since can cause a race condition if compare match was about to occur
            void forceOutputCompareA() override; //forces a compare match on OC0A
            void forceOutputCompareB() override; //forces a compare match on OC0B

            //interrupt control methods
            void setInterruptCallback(TInterrupt8 type, void (*callback)()) override; //sets the callback for the interrupt (8 bit)
            void enableInterrupt(TInterrupt8 type) override; //enables the interrupt for the timer (8 bit)
            void disableInterrupt(TInterrupt8 type) override; //disables the interrupt for the timer (8 bit)
            bool getInterruptFlag(TInterrupt8 type) override; //returns the interrupt flag for the timer (8 bit)
            void clearInterruptFlag(TInterrupt8 type) override; //clears the interrupt flag for the timer (8 bit)

            //direct blocking delay methods
            void syncDelay(uint32_t time) override; //delays for a specified time in milliseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)
        
            //direct non blocking delay methods
            void asyncDelay(uint32_t time) override; //delays for a specified time in milliseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            bool isAsyncDelayFinished() override; //returns true if the asynchronous delay is finished
            void stopAsyncDelay() override; //stops the asynchronous delay
        private:
            void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the delay methods
            void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the non-blocking delay methods
        public:
            volatile uint64_t _asyncTargetTicks; //total delay ticks
            volatile uint64_t _asyncCurrentTicks; //counter for elapsed ticks
            volatile uint32_t _asyncOverflowsCount; //number of tick cycles
            volatile uint16_t _asyncRemainingTicksValue; //last ocr value
            volatile bool _asyncDelayActive; //async delay flag
            volatile clb::TOutputChannel _asyncDelayActiveChannel; //channel for async delay

            //register save variables for asynchronous mode
            volatile uint8_t _asyncSavedTCCR0A;
            volatile uint8_t _asyncSavedTCCR0B;
            volatile uint8_t _asyncSavedTCNT0;
            volatile uint8_t _asyncSavedOCR0A;
            volatile uint8_t _asyncSavedOCR0B;
            volatile uint8_t _asyncSavedTIMSK0;
            volatile uint8_t _asyncSavedTIFR0;
            volatile uint8_t _asyncSavedSREG;
    }; 
    //subclass timer 2 (8 bits)
    class Timer2 : public Timer {
        public:
            //setup methods
            Timer2();
            ~Timer2();
            void deactivate() override; //deactivates the timer and resets the registers

            void setMode(TMode8 mode) override; //sets the waveform generation mode of the timer
            void setClock(TAsynClock clock) override; //sets the clock source and prescaler of the timer
            void setCompareMatchOutputModeA(TCMOM mode) override; //sets the output mode for pin OC0A
            void setCompareMatchOutputModeB(TCMOM mode) override; //sets the output mode for pin OC0B
            void setCompareMatchValueA(uint8_t value) override; //sets the compare match value for pin OC0A
            void setCompareMatchValueB(uint8_t value) override; //sets the compare match value for pin OC0B
            
            /*//overflow handling methods
            void overflow() override; //called when the timer overflows, increments the overflow count
            void setOverflowCount(uint32_t count) override; //sets the overflow count for the timer, used for delay calculations
            uint32_t getOverflowCount() override; //returns the overflow count for the timer, used for delay calculations
            bool isLastOverflow() override; //returns true if the last overflow was the last one, used for delay calculations
            */
            //operation methods
            void startTimer() override; //starts the timer
            void stopTimer() override; //stops the timer
            uint8_t getTimerValue8() override; //returns the current value of the timer
            void setTimerValue(uint8_t value) override; //sets the timer value, not a good idea to use since can cause a race condition if compare match was about to occur
            void forceOutputCompareA() override; //forces a compare match on OC0A
            void forceOutputCompareB() override; //forces a compare match on OC0B

            //interrupt control methods
            void setInterruptCallback(TInterrupt8 type, void (*callback)()) override; //sets the callback for the interrupt (8 bit)
            void enableInterrupt(TInterrupt8 type) override; //enables the interrupt for the timer (8 bit)
            void disableInterrupt(TInterrupt8 type) override; //disables the interrupt for the timer (8 bit)
            bool getInterruptFlag(TInterrupt8 type) override; //returns the interrupt flag for the timer (8 bit)
            void clearInterruptFlag(TInterrupt8 type) override; //clears the interrupt flag for the timer (8 bit)

            //asynchonous clock methods
            void setAsynchronousClock(TACLK clk); //enables the external clock for the timer, input on TOSC1 pin
            bool getBusyFlag(TBusyFlag flag); //returns the busy flag for the specified register
            
            //direct blocking delay methods
            void syncDelay(uint32_t time) override; //delays for a specified time in milliseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)

            //direct non blocking delay methods
            void asyncDelay(uint32_t time) override; //delays for a specified time in milliseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            bool isAsyncDelayFinished() override; //returns true if the asynchronous delay is finished
            void stopAsyncDelay() override; //stops the asynchronous delay
        private:
            void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the delay methods
            void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the non-blocking delay methods
        public:
            volatile uint64_t _asyncTargetTicks; //total delay ticks
            volatile uint64_t _asyncCurrentTicks; //counter for elapsed ticks
            volatile uint32_t _asyncOverflowsCount; //number of tick cycles
            volatile uint16_t _asyncRemainingTicksValue; //last ocr value
            volatile bool _asyncDelayActive; //async delay flag
            volatile clb::TOutputChannel _asyncDelayActiveChannel; //channel for async delay

            //register save variables for asynchronous mode
            volatile uint8_t _asyncSavedTCCR2A;
            volatile uint8_t _asyncSavedTCCR2B;
            volatile uint8_t _asyncSavedTCNT2;
            volatile uint8_t _asyncSavedOCR2A;
            volatile uint8_t _asyncSavedOCR2B;
            volatile uint8_t _asyncSavedTIMSK2;
            volatile uint8_t _asyncSavedTIFR2;
            volatile uint8_t _asyncSavedSREG;
    };
    //subclass timer 1 (16 bits)
    class Timer1 : public Timer {
        public:
            //setup methods
            Timer1();
            ~Timer1();
            void deactivate() override; //deactivates the timer and resets the registers

            void setMode(TMode16 mode) override; //sets the waveform generation mode of the timer
            void setClock(TSyncClock clock) override; //sets the clock source and prescaler of the timer
            void setCompareMatchOutputModeA(TCMOM mode) override; //sets the output mode for pin OC0A
            void setCompareMatchOutputModeB(TCMOM mode) override; //sets the output mode for pin OC0B
            void setCompareMatchOutputModeC(TCMOM mode) override; //sets the output mode for pin OC0C
            void setCompareMatchValueA(uint16_t value) override; //sets the compare match value for pin OC0A
            void setCompareMatchValueB(uint16_t value) override; //sets the compare match value for pin OC0B
            void setCompareMatchValueC(uint16_t value) override; //sets the compare match value for pin OC0C

            /*//overflow handling methods
            void overflow() override; //called when the timer overflows, increments the overflow count
            void setOverflowCount(uint32_t count) override; //sets the overflow count for the timer, used for delay calculations
            uint32_t getOverflowCount() override; //returns the overflow count for the timer, used for delay calculations
            bool isLastOverflow() override; //returns true if the last overflow was the last one, used for delay calculations
            */       

            //operation methods
            void startTimer() override; //starts the timer
            void stopTimer() override; //stops the timer
            uint16_t getTimerValue16() override; //returns the current value of the timer
            void setTimerValue(uint16_t value) override; //sets the timer value, not a good idea to use since can cause a race condition if compare match was about to occur
            void forceOutputCompareA() override; //forces a compare match on OC0A
            void forceOutputCompareB() override; //forces a compare match on OC0B
            void forceOutputCompareC() override; //forces a compare match on OC0C

            //interrupt control methods
            void setInterruptCallback(TInterrupt16 type, void (*callback)()) override; //sets the callback for the interrupt (16 bit)
            void enableInterrupt(TInterrupt16 type) override; //enables the interrupt for the timer (16 bit)
            void disableInterrupt(TInterrupt16 type) override; //disables the interrupt for the timer (16 bit)
            bool getInterruptFlag(TInterrupt16 type) override; //returns the interrupt flag for the timer (16 bit)
            void clearInterruptFlag(TInterrupt16 type) override; //clears the interrupt flag for the timer (16 bit)

            //input capture methods
            void inputCaptureNoiseCancelEnable(bool enable) override; //enables the noise canceler for the input capture
            void inputCaptureEdgeSelect(bool rising) override; //selects the edge for the input capture

            //direct blocking delay methods
            void syncDelay(uint32_t time) override; //delays for a specified time in milliseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)
        
            //direct non blocking delay methods
            void asyncDelay(uint32_t time) override; //delays for a specified time in milliseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            bool isAsyncDelayFinished() override; //returns true if the asynchronous delay is finished
            void stopAsyncDelay() override; //stops the asynchronous delay
        private:
            void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the delay methods
            void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the non-blocking delay methods
        public:
            volatile uint64_t _asyncTargetTicks; //total delay ticks
            volatile uint64_t _asyncCurrentTicks; //counter for elapsed ticks
            volatile uint32_t _asyncOverflowsCount; //number of tick cycles
            volatile uint16_t _asyncRemainingTicksValue; //last ocr value
            volatile bool _asyncDelayActive; //async delay flag
            volatile clb::TOutputChannel _asyncDelayActiveChannel; //channel for async delay

            //register save variables for asynchronous mode
            volatile uint16_t _asyncSavedTCCR1A;
            volatile uint16_t _asyncSavedTCCR1B;
            volatile uint16_t _asyncSavedTCNT1;
            volatile uint16_t _asyncSavedOCR1A;
            volatile uint16_t _asyncSavedOCR1B;
            volatile uint16_t _asyncSavedTIMSK1;
            volatile uint16_t _asyncSavedTIFR1;
            volatile uint16_t _asyncSavedSREG;
            
    };
    //subclass timer 3 (16 bits)
    class Timer3 : public Timer {
        public:
            //setup methods
            Timer3();
            ~Timer3();
            void deactivate() override; //deactivates the timer and resets the registers

            void setMode(TMode16 mode) override; //sets the waveform generation mode of the timer
            void setClock(TSyncClock clock) override; //sets the clock source and prescaler of the timer
            void setCompareMatchOutputModeA(TCMOM mode) override; //sets the output mode for pin OC0A
            void setCompareMatchOutputModeB(TCMOM mode) override; //sets the output mode for pin OC0B
            void setCompareMatchOutputModeC(TCMOM mode) override; //sets the output mode for pin OC0C
            void setCompareMatchValueA(uint16_t value) override; //sets the compare match value for pin OC0A
            void setCompareMatchValueB(uint16_t value) override; //sets the compare match value for pin OC0B
            void setCompareMatchValueC(uint16_t value) override; //sets the compare match value for pin OC0C

            /*//overflow handling methods
            void overflow() override; //called when the timer overflows, increments the overflow count
            void setOverflowCount(uint32_t count) override; //sets the overflow count for the timer, used for delay calculations
            uint32_t getOverflowCount() override; //returns the overflow count for the timer, used for delay calculations
            bool isLastOverflow() override; //returns true if the last overflow was the last one, used for delay calculations
            */

            //operation methods
            void startTimer() override; //starts the timer
            void stopTimer() override; //stops the timer
            uint16_t getTimerValue16() override; //returns the current value of the timer
            void setTimerValue(uint16_t value) override; //sets the timer value, not a good idea to use since can cause a race condition if compare match was about to occur
            void forceOutputCompareA() override; //forces a compare match on OC0A
            void forceOutputCompareB() override; //forces a compare match on OC0B
            void forceOutputCompareC() override; //forces a compare match on OC0C

            //interrupt control methods
            void setInterruptCallback(TInterrupt16 type, void (*callback)()) override; //sets the callback for the interrupt (16 bit)
            void enableInterrupt(TInterrupt16 type) override; //enables the interrupt for the timer (16 bit)
            void disableInterrupt(TInterrupt16 type) override; //disables the interrupt for the timer (16 bit)
            bool getInterruptFlag(TInterrupt16 type) override; //returns the interrupt flag for the timer (16 bit)
            void clearInterruptFlag(TInterrupt16 type) override; //clears the interrupt flag for the timer (16 bit)

            //input capture methods
            void inputCaptureNoiseCancelEnable(bool enable) override; //enables the noise canceler for the input capture
            void inputCaptureEdgeSelect(bool rising) override; //selects the edge for the input capture
            
            //direct blocking delay methods
            void syncDelay(uint32_t time) override; //delays for a specified time in milliseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)
 
            //direct non blocking delay methods
            void asyncDelay(uint32_t time) override; //delays for a specified time in milliseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            bool isAsyncDelayFinished() override; //returns true if the asynchronous delay is finished
            void stopAsyncDelay() override; //stops the asynchronous delay
        private:
            void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the delay methods
            void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the non-blocking delay methods
    };
    //subclass timer 4 (16 bits)
    class Timer4 : public Timer {
        public:
            //setup methods
            Timer4();
            ~Timer4();
            void deactivate() override; //deactivates the timer and resets the registers

            void setMode(TMode16 mode) override; //sets the waveform generation mode of the timer
            void setClock(TSyncClock clock) override; //sets the clock source and prescaler of the timer
            void setCompareMatchOutputModeA(TCMOM mode) override; //sets the output mode for pin OC0A
            void setCompareMatchOutputModeB(TCMOM mode) override; //sets the output mode for pin OC0B
            void setCompareMatchOutputModeC(TCMOM mode) override; //sets the output mode for pin OC0C
            void setCompareMatchValueA(uint16_t value) override; //sets the compare match value for pin OC0A
            void setCompareMatchValueB(uint16_t value) override; //sets the compare match value for pin OC0B
            void setCompareMatchValueC(uint16_t value) override; //sets the compare match value for pin OC0C

            /*//overflow handling methods
            void overflow() override; //called when the timer overflows, increments the overflow count
            void setOverflowCount(uint32_t count) override; //sets the overflow count for the timer, used for delay calculations
            uint32_t getOverflowCount() override; //returns the overflow count for the timer, used for delay calculations
            bool isLastOverflow() override; //returns true if the last overflow was the last one, used for delay calculations
            */

            //operation methods
            void startTimer() override; //starts the timer
            void stopTimer() override; //stops the timer
            uint16_t getTimerValue16() override; //returns the current value of the timer
            void setTimerValue(uint16_t value) override; //sets the timer value, not a good idea to use since can cause a race condition if compare match was about to occur
            void forceOutputCompareA() override; //forces a compare match on OC0A
            void forceOutputCompareB() override; //forces a compare match on OC0B
            void forceOutputCompareC() override; //forces a compare match on OC0C

            //interrupt control methods
            void setInterruptCallback(TInterrupt16 type, void (*callback)()) override; //sets the callback for the interrupt (16 bit)
            void enableInterrupt(TInterrupt16 type) override; //enables the interrupt for the timer (16 bit)
            void disableInterrupt(TInterrupt16 type) override; //disables the interrupt for the timer (16 bit)
            bool getInterruptFlag(TInterrupt16 type) override; //returns the interrupt flag for the timer (16 bit)
            void clearInterruptFlag(TInterrupt16 type) override; //clears the interrupt flag for the timer (16 bit)

            //input capture methods
            void inputCaptureNoiseCancelEnable(bool enable) override; //enables the noise canceler for the input capture
            void inputCaptureEdgeSelect(bool rising) override; //selects the edge for the input capture 
            
            //direct blocking delay methods
            void syncDelay(uint32_t time) override; //delays for a specified time in milliseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)

            //direct non blocking delay methods
            void asyncDelay(uint32_t time) override; //delays for a specified time in milliseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            bool isAsyncDelayFinished() override; //returns true if the asynchronous delay is finished
            void stopAsyncDelay() override; //stops the asynchronous delay
        private:
            void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the delay methods
            void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the non-blocking delay methods
    };
    //subclass timer 5 (16 bits)
    class Timer5 : public Timer {
        public:
            //setup methods
            Timer5();
            ~Timer5();
            void deactivate() override; //deactivates the timer and resets the registers
            
            void setMode(TMode16 mode) override; //sets the waveform generation mode of the timer
            void setClock(TSyncClock clock) override; //sets the clock source and prescaler of the timer
            void setCompareMatchOutputModeA(TCMOM mode) override; //sets the output mode for pin OC0A
            void setCompareMatchOutputModeB(TCMOM mode) override; //sets the output mode for pin OC0B
            void setCompareMatchOutputModeC(TCMOM mode) override; //sets the output mode for pin OC0C
            void setCompareMatchValueA(uint16_t value) override; //sets the compare match value for pin OC0A
            void setCompareMatchValueB(uint16_t value) override; //sets the compare match value for pin OC0B
            void setCompareMatchValueC(uint16_t value) override; //sets the compare match value for pin OC0C

            /*//overflow handling methods
            void overflow() override; //called when the timer overflows, increments the overflow count
            void setOverflowCount(uint32_t count) override; //sets the overflow count for the timer, used for delay calculations
            uint32_t getOverflowCount() override; //returns the overflow count for the timer, used for delay calculations
            bool isLastOverflow() override; //returns true if the last overflow was the last one, used for delay calculations
            */

            //operation methods
            void startTimer() override; //starts the timer
            void stopTimer() override; //stops the timer
            uint16_t getTimerValue16() override; //returns the current value of the timer
            void setTimerValue(uint16_t value) override; //sets the timer value, not a good idea to use since can cause a race condition if compare match was about to occur
            void forceOutputCompareA() override; //forces a compare match on OC0A
            void forceOutputCompareB() override; //forces a compare match on OC0B
            void forceOutputCompareC() override; //forces a compare match on OC0C

            //interrupt control methods
            void setInterruptCallback(TInterrupt16 type, void (*callback)()) override; //sets the callback for the interrupt (16 bit)
            void enableInterrupt(TInterrupt16 type) override; //enables the interrupt for the timer (16 bit)
            void disableInterrupt(TInterrupt16 type) override; //disables the interrupt for the timer (16 bit)
            bool getInterruptFlag(TInterrupt16 type) override; //returns the interrupt flag for the timer (16 bit)
            void clearInterruptFlag(TInterrupt16 type) override; //clears the interrupt flag for the timer (16 bit)

            //input capture methods
            void inputCaptureNoiseCancelEnable(bool enable) override; //enables the noise canceler for the input capture
            void inputCaptureEdgeSelect(bool rising) override; //selects the edge for the input capture   
            
            //direct blocking delay methods
            void syncDelay(uint32_t time) override; //delays for a specified time in milliseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds
            void syncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C)
    
            //direct non blocking delay methods
            void asyncDelay(uint32_t time) override; //delays for a specified time in milliseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit) override; //delays for a specified time in seconds, milliseconds or microseconds, non-blocking
            void asyncDelay(uint32_t time, TTimeUnit timeUnit, TOutputChannel channel) override; //delays for a specified time in seconds, milliseconds or microseconds on one of the compare registers (A, B or C), non-blocking
        
            //asynchronous control methods
            bool isAsyncDelayFinished() override; //returns true if the asynchronous delay is finished
            void stopAsyncDelay() override; //stops the asynchronous delay
        private:
            void syncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the delay methods
            void asyncDelayLogic(uint64_t ticks, clb::TOutputChannel channel) override; //logic for the non-blocking delay methods
    };
};

#endif