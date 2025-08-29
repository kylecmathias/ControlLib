#ifndef CLBEXCEPTION_HPP
#define CLBEXCEPTION_HPP

#include <Arduino.h>
#include <assert.h> 

#define WARNING(msg) clb::Warning::warn(msg)
#define CRITICAL(msg) clb::Critical::log(msg)
#define FATAL(msg) clb::Fatal::halt(msg)  

/* 
 IMPORTANT NOTE:
 never use these inside an ISR since it can corrupt the static buffers and print broken error messages.
 theres not really any reason to use these inside a user program either, theyre just for use by clb libraries.
 but if used in user programs, error messages get truncated to ~247 characters (after including error type in string).

*/

namespace clb {
    //warning means you can continue but just be aware of what you're doing. lots of warnings wont break your system, but you should definitely know what youre doing.
    class Warning {
    public:
        static void warn(const char* message) {
            static char buffer[256]; 
            snprintf(buffer, sizeof(buffer), "WARNING: %s", message);

            Serial.println(buffer);
            Serial.flush();
        }
    };
    //critical means something is wrong, you can still continue but you should definitely know what youre doing. lots of criticals mean you should probably try to fix some stuff so your system doesnt break.
    class Critical {
    public:
        static void log(const char* message) {
            static char buffer[256];
            snprintf(buffer, sizeof(buffer), "CRITICAL: %s", message); 

            Serial.println(buffer);
            Serial.flush();
        }
    };
    //fatal means you did something seriously wrong and you cannot continue unless you fix it.
    class Fatal {
    public:
        static void halt(const char* message) {
            static char buffer[256];
            snprintf(buffer, sizeof(buffer), "FATAL: %s", message); 

            Serial.println(buffer);
            Serial.flush();

            assert(0); 

            while (true) { }
        }
    };

}      

#endif