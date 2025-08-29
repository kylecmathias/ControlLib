# ControlLib

An Arduino library for the Arduino Mega 2560 allowing custom timer settings and interrupts. 

### Currently Supports only timer0 and timer2

## IMPORTANT NOTES
 
Timer0 is used by the Arduino core for delay(), millis() and micros() functions. Dont use it unless you will not be using these functions.
Timer1 is used by the Arduino core for Servo library, so if you use Servo library, dont use Timer1.
Timer2 is used by the Arduino core for tone() function, so if you use tone() function, dont use Timer2.
Timers 3, 4 and 5 are not used by the Arduino core, so you can use them freely, although some libraries may use Timer5.

These classes are meant for very low level control over the timers, so you cant use the builtin arduino functions that use the timers you use.

These are the functions and what timers are used by them:

Timer0 (8 bit) (used by Arduino core for timekeeping)
- ```delay(ms)```
- ```delayMicroseconds(us)```
- ```millis()```
- ```micros()```
- ```analogWrite(4, value)```
- ```analogWrite(13, value)```

Timer1 (16 bit) (used by Arduino for PWM and Servo control)
- ```analogWrite(11, value)```
- ```analogWrite(12, value)```

Timer2 (8 bit) (used by Arduino for tone generation)
- ```tone()```
- ```noTone()```
 * - analogWrite(9, value)
 * - analogWrite(10, value)
 */
