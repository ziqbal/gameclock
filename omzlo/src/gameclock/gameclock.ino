
// Game Clock
// r1.0.20181030 Zafar Iqbal
// http://omzlo.com/articles/the-game-clock

// MIT License

// Copyright (c) 2018 Zafar Iqbal, Alain Pannetrat

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


///////////////////////////////////////


bool power_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 60000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define power_modulename F( "power_" )


void power_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( power_modulename , k ) ;

    log_print( i ) ;

}

void power_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( power_modulename , k ) ;
    log_print( i ) ;

}

void power_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( power_modulename , k ) ;
    log_print( i ) ;

}

void power_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( power_modulename , k ) ;
    log_print( i ) ;

}

void power_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( power_modulename , k ) ;
    log_print( i ) ;

}

void power_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( power_modulename , k ) ;
    log_print( i ) ;

}



void power_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( power_modulename , k ) ;
    log_print( i ) ;

}



void power_Setups( ) {

    power_setup( ) ;

}




void power_Loops( ) {

    if( ! power_Trigger( ) ) return ;

    power_loop( ) ;

}


///////////////////////////////////////


///////////////////////////////////////


const int power_pin = A3 ;

int power_val = -1 ;

void power_setup( ) {

    //pinMode( power_pin , INPUT ) ;
    power_val = -1 ;

}

///////////////////////////////////////

void power_loop( ) {

    power_val = analogRead( power_pin ) ;

    power_Log( F( "BAT" ) , power_val ) ;

}

///////////////////////////////////////


///////////////////////////////////////


bool watchdog_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 1000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define watchdog_modulename F( "watchdog_" )


void watchdog_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( watchdog_modulename , k ) ;

    log_print( i ) ;

}

void watchdog_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( watchdog_modulename , k ) ;
    log_print( i ) ;

}

void watchdog_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( watchdog_modulename , k ) ;
    log_print( i ) ;

}

void watchdog_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( watchdog_modulename , k ) ;
    log_print( i ) ;

}

void watchdog_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( watchdog_modulename , k ) ;
    log_print( i ) ;

}

void watchdog_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( watchdog_modulename , k ) ;
    log_print( i ) ;

}



void watchdog_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( watchdog_modulename , k ) ;
    log_print( i ) ;

}



void watchdog_Setups( ) {

    watchdog_setup( ) ;

}




void watchdog_Loops( ) {

    if( ! watchdog_Trigger( ) ) return ;

    watchdog_loop( ) ;

}


///////////////////////////////////////


///////////////////////////////////////

byte watchdog_counter = 0 ;

void softReset( ) {

    asm volatile ( "  jmp 0" ) ;

}

void watchdog_setup( ) {

    watchdog_counter = 0 ;

}

///////////////////////////////////////

void watchdog_loop( ) {

    watchdog_counter++ ;

    if( watchdog_counter == 10 ) {

        softReset( ) ;

        while( true ) {

            delay( 1000 ) ;

        }

    }

}

///////////////////////////////////////

void watchdog_reset( ) {

    watchdog_counter = 0 ;

}




///////////////////////////////////////


bool sleep_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 1000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define sleep_modulename F( "sleep_" )


void sleep_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( sleep_modulename , k ) ;

    log_print( i ) ;

}

void sleep_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( sleep_modulename , k ) ;
    log_print( i ) ;

}

void sleep_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( sleep_modulename , k ) ;
    log_print( i ) ;

}

void sleep_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( sleep_modulename , k ) ;
    log_print( i ) ;

}

void sleep_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( sleep_modulename , k ) ;
    log_print( i ) ;

}

void sleep_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( sleep_modulename , k ) ;
    log_print( i ) ;

}



void sleep_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( sleep_modulename , k ) ;
    log_print( i ) ;

}



void sleep_Setups( ) {

    sleep_setup( ) ;

}




void sleep_Loops( ) {

    if( ! sleep_Trigger( ) ) return ;

    sleep_loop( ) ;

}


///////////////////////////////////////


///////////////////////////////////////

#include "LowPower.h"

bool sleep_shutdown = false ;

byte sleep_counter = 0 ;
bool sleep_enabled = false ;


bool flagAwakeMessage = false ;

void sleep_setup( ) {

    sleep_counter = 0 ;

    sleep_enabled = false ;
    sleep_shutdown = false ;

}

///////////////////////////////////////

void sleep_loop( ) {

    if(sleep_shutdown) return ;

    if( flagAwakeMessage ) {

        sleep_Log( F( "S" ) , F( "AWAKE" ) ) ;

        flagAwakeMessage = false ;
        
    }


    if( ! sleep_enabled ) {

        return ;

    }

    sleep_enabled = false ;

    if( sleep_counter < 65 ) {

        sleep_counter++ ;

        return ;

    } 

    sleep_counter = 0 ;

    display_poweroff( ) ;

    sleep_Log( F( "S" ) , F( "SLEEP" ) ) ;
    delay( 1000 ) ;
    LowPower.idle( SLEEP_FOREVER , ADC_OFF , TIMER2_OFF , TIMER1_OFF , TIMER0_OFF , SPI_OFF , USART0_OFF , TWI_OFF ) ;

    display_poweron( ) ;

    delay( 1000 ) ;

    flagAwakeMessage = true ;

}

void sleep_enable( ) {

    sleep_enabled = true ;

}

///////////////////////////////////////






#include <EEPROM.h>

///////////////////////////////////////

void eeprom_write( int address , byte val ) {

    EEPROM.write( address , val ) ;

}






bool serial_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 10000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



///////////////////////////////////////////////////////////////////


#define serial_modulename F( "serial_" )


void serial_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( serial_modulename , k ) ;

    log_print( i ) ;

}

void serial_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( serial_modulename , k ) ;
    log_print( i ) ;

}

void serial_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( serial_modulename , k ) ;
    log_print( i ) ;

}

void serial_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( serial_modulename , k ) ;
    log_print( i ) ;

}

void serial_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( serial_modulename , k ) ;
    log_print( i ) ;

}

void serial_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( serial_modulename , k ) ;
    log_print( i ) ;

}



void serial_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( serial_modulename , k ) ;
    log_print( i ) ;

}



void serial_Setups( ) {

    serial_setup( ) ;

}




void serial_Loops( ) {

    if( ! serial_Trigger( ) ) return ;

    serial_loop( ) ;

}







void serial_print( const __FlashStringHelper * i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}

void serial_print( const char * i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}


void serial_print( byte i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}


void serial_print( int i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}

void serial_print( unsigned int i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}

void serial_print( long i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}


void serial_print( unsigned long i , bool newline ) {

    if( newline ) {

        Serial.println( i ) ;

    } else {

        Serial.print( i ) ;

    }

}

///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////

const byte serial_numChars = 32;

char serial_receivedChars[ serial_numChars ] ;
char serial_tempChars[ serial_numChars ] ; 

boolean serial_newData = false ;

///////////////////////////////////////////////////////////////////

void serial_setup( ) {

    Serial.begin( 57600 ) ;

}

///////////////////////////////////////////////////////////////////

void serial_loop( ) {

    serial_recvWithStartEndMarkers( ) ;

    if( serial_newData == true ) {

        strcpy( serial_tempChars , serial_receivedChars ) ;

        wizard_incomingParse( serial_tempChars ) ;

        serial_newData = false ;

    }

}

///////////////////////////////////////////////////////////////////

void serial_recvWithStartEndMarkers( ) {

    static boolean recvInProgress = false ;
    static byte ndx = 0 ;

    char startMarker = '<' ;
    char endMarker = '>' ;

    char rc ;

    while( Serial.available( ) > 0 && serial_newData == false ) {

        rc = Serial.read( ) ;

        if( rc == '\n' ) {

            serial_receivedChars[ 0 ] = '\0' ;

            recvInProgress = false ;
            
            ndx = 0 ;

            serial_newData = false ;

            continue ;

        }

        if( recvInProgress == true ) {

            if( rc != endMarker ) {

                serial_receivedChars[ ndx ] = rc ; 

                ndx++ ;

                if( ndx >= serial_numChars ) {

                    ndx = serial_numChars - 1 ;

                }

            } else {

                serial_receivedChars[ ndx ] = '\0' ;

                recvInProgress = false ;

                ndx = 0 ;

                serial_newData = true ;

            }

        } else if( rc == startMarker ) {

            recvInProgress = true ;

        }

    }

}







bool log_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 60000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}





void log_Setups( ) {

    log_setup( ) ;

}




void log_Loops( ) {

    if( ! log_Trigger( ) ) return ;

    log_loop( ) ;

}




#define log_clientmodulebufSize 20

const char log_clientmodulebuf[ log_clientmodulebufSize ] ;


void log_setup( ) {

}

void log_loop( ) {

    log_initClientModule( F( "log_" ) , F( "hb" ) ) ;

    log_print( F( "@" ) ) ;

    wizard_debugMem( ) ;

}

void log_initClientModule( const __FlashStringHelper * c , const __FlashStringHelper * m ) {

    snprintf_P( log_clientmodulebuf , log_clientmodulebufSize , PSTR( "%S|%S" ) , c , m ) ;

}




void log_print( const __FlashStringHelper * i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}


void log_print( char * i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}


void log_print( byte i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}


void log_print( int i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}


void log_print( unsigned int i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}

void log_print( long i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}

void log_print( unsigned long i ) {

    log_timeclientstamp( ) ;
    wizard_serialPrint( i , true ) ;

}

void log_timeclientstamp( ) {

    wizard_serialPrint( clock_getTimeMillis( ) , false ) ;
    //wizard_serialPrint( rtc_millis() , false ) ;

    wizard_serialPrint( F( "|" )  , false ) ;
    wizard_serialPrint( log_clientmodulebuf , false ) ;
    wizard_serialPrint( F( "|" ) , false ) ;

}







///////////////////////////////////////



void clock_Setups( ) {

    clock_setup( ) ;

}



// DO NOT YSE LOOPS MACRO!
// This defines its own Loop


///////////////////////////////////////

unsigned long clock_timeMillis = millis( ) ;
unsigned long clock_timeMicros = micros( ) ;

///////////////////////////////////////

void clock_setup( ) {


}

void clock_Loops( ) {


    clock_timeMillis = millis( ) ;
    clock_timeMicros = micros( ) ;

    delay( 1 ) ;

}

///////////////////////////////////////

unsigned long clock_getTimeMillis( ) {

    return( clock_timeMillis ) ;

}

unsigned long clock_getTimeMicros( ) {

    return( clock_timeMicros ) ;

}







///////////////////////////////////////


bool buzzer_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 1000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define buzzer_modulename F( "buzzer_" )


void buzzer_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( buzzer_modulename , k ) ;

    log_print( i ) ;

}

void buzzer_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( buzzer_modulename , k ) ;
    log_print( i ) ;

}

void buzzer_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( buzzer_modulename , k ) ;
    log_print( i ) ;

}

void buzzer_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( buzzer_modulename , k ) ;
    log_print( i ) ;

}

void buzzer_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( buzzer_modulename , k ) ;
    log_print( i ) ;

}

void buzzer_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( buzzer_modulename , k ) ;
    log_print( i ) ;

}



void buzzer_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( buzzer_modulename , k ) ;
    log_print( i ) ;

}



void buzzer_Setups( ) {

    buzzer_setup( ) ;

}




void buzzer_Loops( ) {

    if( ! buzzer_Trigger( ) ) return ;

    buzzer_loop( ) ;

}


///////////////////////////////////////


///////////////////////////////////////

//const byte buzzer_pin = 6 ;

#define buzzer_pin 6

bool buzzer_enabled = false;
bool buzzer_state = false ;

unsigned long buzzer_timeON = millis( ) ;
unsigned long buzzer_timeDelay = 0 ;



///////////////////////////////////////

void buzzer_setup( ) {

    pinMode( buzzer_pin , OUTPUT ) ;

    buzzer_enabled = true ;

}

///////////////////////////////////////

void buzzer_loop( ) {


    if( !buzzer_state ) {

        return ;

    }

    if( ( clock_getTimeMillis( ) - buzzer_timeON ) < buzzer_timeDelay ) {

        return ;

    }

    if( buzzer_enabled ) {

        noTone( buzzer_pin ) ;

    }

    buzzer_state = false ;

}


void buzzer_beep( int freq , unsigned long d ) {

    buzzer_timeON = millis( ) ;

    buzzer_timeDelay = d ;

    buzzer_state = true ;

    if( buzzer_enabled ) {

        tone( buzzer_pin , freq ) ;
        
    }

}

bool buzzer_isOff( ) {

    return( ! buzzer_state ) ;

}





bool display_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 50000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}




#define display_modulename F( "display_" )


void display_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( display_modulename , k ) ;

    log_print( i ) ;

}

void display_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( display_modulename , k ) ;
    log_print( i ) ;

}

void display_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( display_modulename , k ) ;
    log_print( i ) ;

}

void display_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( display_modulename , k ) ;
    log_print( i ) ;

}

void display_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( display_modulename , k ) ;
    log_print( i ) ;

}

void display_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( display_modulename , k ) ;
    log_print( i ) ;

}



void display_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( display_modulename , k ) ;
    log_print( i ) ;

}



void display_Setups( ) {

    display_setup( ) ;

}




void display_Loops( ) {

    if( ! display_Trigger( ) ) return ;

    display_loop( ) ;

}



#include "LedControl.h"

// SPI
LedControl display_lc = LedControl( 10 , 1 ) ; 

byte display_buffer[ ] = { ' ' , ' ' , ' ' , ' ' , ' ' , ' ' , ' ' , ' ' } ;
bool display_bufferChanged[ ] = {  false , false , false , false , false , false , false , false } ;     
bool display_bufferDots[ ] = { false , false , false , false , false , false , false , false } ;

//byte display_bufferDisplayMap[ ] = { 4 , 5 , 6 , 7 , 0 , 1 , 2 , 3 } ;
byte display_bufferDisplayMap[ ] = { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 } ;

bool display_dirty = true ;

byte display_brightness = 7 ;

/////////////////////////////////////////////

void display_setup( ) {

    display_lc.setScanLimit( 0 , 8 ) ;
    display_lc.shutdown( 0 , false ) ; 
    display_lc.setIntensity( 0 , display_brightness ) ;
    display_lc.clearDisplay( 0 ) ;

}

void display_loop( ) {

    if( ! display_dirty ) return ;

    // FIXME TODO
    // this is called many times
    // check that draw handles this well

//    display_Log(F("zzz"),F("zzz"));

    display_draw( ) ;

    display_dirty = false ;

}

void display_draw( ) {

    for( int i = 0 ; i < 8 ; i++ ) {

        if( display_bufferChanged[ i ] ) {

            display_lc.setChar( 0 , display_bufferDisplayMap[ 7 - i ] , display_buffer[ i ] , display_bufferDots[ i ] ) ;

            display_bufferChanged[ i ] = false ;

        }

    }

}





void display_printNumberPad( unsigned long v , byte p , byte pad ) {

    byte digit ;

    byte pos = p ;

    while( v > 0 ) {

        digit = v % 10 ;
        v = v / 10 ;

        if( display_buffer[ pos ] != digit ) {

            display_buffer[ pos ] = digit ;

            display_bufferChanged[ pos ] = true ;

        }

        pos++ ;

    }

    while( ( pos - p ) < pad ) {

        // FIXME TODO

        if( display_buffer[ pos ] != 0 ) {

            display_buffer[ pos ] = 0 ;

            display_bufferChanged[ pos ] = true ;

        }


        pos++ ;

    }

    display_dirty = true ;

}


// FIXME TODO - more optimum changed flags
void display_clearBuffers( ) {

    for( byte i = 0 ; i < 8 ; i++ ) {

        if( display_buffer[ i ] != ' ') {

            display_buffer[ i ] = ' ' ;
            display_bufferChanged[ i ] = true ;

        }


        if(display_bufferDots[ i ] != false ){

            display_bufferDots[ i ] = false ; 
            display_bufferChanged[ i ] = true ;


        }

    }
    
    display_dirty = true ;

}

void display_refresh( ) {

    for( byte i = 0 ; i < 8 ; i++ ) {

        display_bufferChanged[ i ] = true ;

    }
    
    display_dirty = true ;

}


void display_setDot( byte i , bool v ) {

    if(display_bufferDots[ i ] != v ){
        display_bufferDots[ i ] = v ; 
        display_bufferChanged[ i ] = true ;
        display_dirty = true ;

    }

}

void display_printChar( char c , byte p ) {

    display_buffer[ p ] = c ;


}

void display_printByte( byte c , byte p ) {

    display_lc.setRow( 0 , display_bufferDisplayMap[ p ] , c ) ;


}


void display_brightnessUpdate( ) {

    display_lc.setIntensity( 0 , display_brightness ) ;


}


void display_ready( ) {

    display_lc.setRow( 0 , random( 0 , 8 ) , random( 0 , 256 ) ) ;

}


void display_reset( ) {

    for( byte i = 0 ; i < 8 ; i++ ) {

        display_lc.setRow( 0 , i , 0 ) ;

    }

    // TODO FIXME - expected bevaiour is to clear the display and then keep it
    // Unless some code is relying on blanking out some chars...?
    display_dirty = false ;

}

void display_poweroff( ) {
    display_lc.shutdown(0,true);
}

void display_poweron( ) {
    display_lc.shutdown(0,false);
}




bool rtc_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 10000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define rtc_modulename F( "rtc_" )


void rtc_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( rtc_modulename , k ) ;

    log_print( i ) ;

}

void rtc_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( rtc_modulename , k ) ;
    log_print( i ) ;

}

void rtc_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( rtc_modulename , k ) ;
    log_print( i ) ;

}

void rtc_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( rtc_modulename , k ) ;
    log_print( i ) ;

}

void rtc_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( rtc_modulename , k ) ;
    log_print( i ) ;

}

void rtc_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( rtc_modulename , k ) ;
    log_print( i ) ;

}



void rtc_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( rtc_modulename , k ) ;
    log_print( i ) ;

}





void rtc_Setups( ) {

    rtc_setup( ) ;

}




void rtc_Loops( ) {

    if( ! rtc_Trigger( ) ) return ;

    rtc_loop( ) ;

}



///////////////////////////////////////

#include <Time.h>

#include <Wire.h>
#include "RTClib.h"

///////////////////////////////////////

RTC_DS3231 rtc_rtc ;
DateTime rtc_now ;

unsigned long rtc_startunixtime = 0 ;
unsigned long rtc_lastunixtime = 0 ;
unsigned long rtc_lastmillis = 0 ;

bool rtc_hasupdated = false ;

bool rtc_enabled = false ;

const byte rtc_sqwPin = 8 ;
bool rtc_sqwLast = false ;
unsigned long rtc_sqwCounter = 0 ;

///////////////////////////////////////

void rtc_setup( ) {

    if( !rtc_rtc.begin( ) ) {

        rtc_Log( F( "WARNING" ) , F( "begin?" ) ) ;

    } else {

        rtc_enabled = true ;

    }

    ///////////////////////////////////////

    if( rtc_enabled && rtc_rtc.lostPower( ) ) {

        rtc_Log( F( "WARNING" ) , F( "Lost Power" ) ) ;

        rtc_rtc.adjust( DateTime( F( __DATE__ ) , F( __TIME__ ) ) ) ;

    }


    ///////////////////////////////////////

    if( rtc_enabled ) {

        rtc_now = rtc_rtc.now( ) ;

        rtc_startunixtime = rtc_now.unixtime( ) ;

        rtc_lastunixtime = rtc_startunixtime ;
        rtc_lastmillis = clock_getTimeMillis( )  ;


        pinMode( rtc_sqwPin , INPUT ) ; 
        rtc_rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

    }

}

///////////////////////////////////////

void rtc_loop( ) {

    if( ! rtc_enabled ) return ;

    if( digitalRead( rtc_sqwPin ) != rtc_sqwLast ) {

        if( rtc_sqwLast ) {

            rtc_sqwCounter++ ;

            //rtc_Log( F( "!" ) , rtc_sqwCounter ) ;

        }

        rtc_sqwLast = ! rtc_sqwLast ;

    }

    if( ( clock_getTimeMillis( ) - rtc_lastmillis ) < 900 ) {

        return ;

    }

    rtc_now = rtc_rtc.now( ) ;

    if( rtc_now.unixtime( ) != rtc_lastunixtime ) {

        rtc_lastunixtime = rtc_now.unixtime( ) ;

        rtc_lastmillis = clock_getTimeMillis( ) ;

        rtc_hasupdated = true ;

    }


}

bool rtc_updated( ) {

    if( rtc_hasupdated ) {

        rtc_hasupdated = false ;

        return( true ) ;

    } else {

        return( false ) ;

    }

}

unsigned long rtc_millis( ) {

    if(  rtc_enabled ) {

        return( ( ( rtc_lastunixtime - rtc_startunixtime ) * 1000L ) + ( clock_getTimeMillis( ) - rtc_lastmillis ) ) ;

    } else {

        return( clock_getTimeMillis( ) ) ;

    }

}




//////////////////////////////////////

bool button1_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 100L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define button1_modulename F( "button1_" )


void button1_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( button1_modulename , k ) ;

    log_print( i ) ;

}

void button1_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( button1_modulename , k ) ;
    log_print( i ) ;

}

void button1_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( button1_modulename , k ) ;
    log_print( i ) ;

}

void button1_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( button1_modulename , k ) ;
    log_print( i ) ;

}

void button1_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( button1_modulename , k ) ;
    log_print( i ) ;

}

void button1_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( button1_modulename , k ) ;
    log_print( i ) ;

}



void button1_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( button1_modulename , k ) ;
    log_print( i ) ;

}



void button1_Setups( ) {

    button1_setup( ) ;

}




void button1_Loops( ) {

    if( ! button1_Trigger( ) ) return ;

    button1_loop( ) ;

}


///////////////////////////////////////


//////////////////////////////////////

byte button1_pin = 3 ;
bool button1_normallyOpen = false ;
unsigned long button1_debounceTime = 50000 ;

//////////////////////////////////////



bool button1_val_ = false ;

//////////////////////////////////////

bool button1_state_ = false ;

//////////////////////////////////////

bool button1_hasChanged_ = false ;
bool button1_hasChangedDownUp_ = false ;
bool button1_hasChangedUpDown_ = false ;

bool button1_isPressed_ = false ;

bool button1_isLongPress_ = false ;

bool button1_wasPressed_ = false ;
bool button1_wasShortPress_ = false ;
bool button1_wasLongPress_ = false ;
bool button1_wasSinglePress_ = false ;
bool button1_wasDoublePress_ = false ;

//////////////////////////////////////

unsigned long button1_timeLongPressStart_ = 500000 ;
unsigned long button1_timeDoublePressEnd_ = 500000 ;

//////////////////////////////////////

int button1_eventsTotal_ = 5 ;
int button1_eventPos_ = 0 ;
bool button1_eventType_[ 5 ] = { false ,  false , false , false , false } ;
unsigned long button1_eventTime_[ 5 ] = { 0 , 0 , 0 , 0 , 0 } ;

void button1_setup_( ) {

    pinMode( button1_pin , INPUT_PULLUP ) ;

    button1_reset_( ) ;

}

void button1_loop_( ) {

    if( digitalRead( button1_pin ) ^ button1_normallyOpen ) {

        button1_actionDown_( ) ;

    } else {

        button1_actionUp_( ) ;

    }

}

void button1_reset_( ) {

    for( int i = 0 ; i < button1_eventsTotal_ ; i++ ) {

        button1_eventType_[ i ] = false ;
        button1_eventTime_[ i ] = clock_getTimeMicros( ) ;

    }

    button1_eventPos_ = 0 ;

    button1_val_ = false ;
    button1_state_ = false ;
    button1_hasChanged_ = false ;
    button1_hasChangedDownUp_ = false ;
    button1_hasChangedUpDown_ = false ;
    button1_isPressed_ = false ;
    button1_isLongPress_ = false ;
    button1_wasPressed_ = false ;
    button1_wasShortPress_ = false ;
    button1_wasLongPress_ = false ;
    button1_wasSinglePress_ = false ;
    button1_wasDoublePress_ = false ;

}


void button1_actionUp_( ) {

    button1_val_ = false ;

    // button1_eventAdd( button1_val ) ;


    if( !button1_debounced_( ) ) {

        return ;

    }

    if( button1_state_ ) {



       if( ( clock_getTimeMicros( ) - button1_eventTime_[ button1_eventPos_ ] ) < button1_timeLongPressStart_ )  {

            button1_wasShortPress_ = true ;
            button1_wasLongPress_ = false ;

       } else {

            button1_wasShortPress_ = false ;
            button1_wasLongPress_ = true ;

       }

        button1_state_ = false ;
        button1_eventAdd_( false ) ;

        button1_isPressed_ = false ;




        
        //button1_Log( "button1_UP" ) ;

    }

}


void button1_actionDown_( ) {

    button1_val_ = true ;

    if( !button1_debounced_( ) ) {

        return ;

    }

    if( ! button1_state_  ) {

        button1_state_ = true ;
        button1_eventAdd_( true ) ;

        button1_isPressed_ = true ;

        //button1_Log( "button1_DOWN" ) ;
        button1_hasChangedUpDown_ = true ;

    }

    // button1_eventAdd( button1_val ) ;


}

void button1_eventAdd_( bool type ) {

    button1_eventPos_++ ;

    if( button1_eventPos_ == button1_eventsTotal_ ) {

        button1_eventPos_ = 0 ;

    }

    button1_eventType_[ button1_eventPos_ ] = type ;
    button1_eventTime_[ button1_eventPos_ ] = clock_getTimeMicros( ) ;

}

bool button1_debounced_( ) {

    if( ( clock_getTimeMicros( ) - button1_eventTime_[ button1_eventPos_ ] ) < button1_debounceTime ) {

        return( false ) ;

    }

    return( true ) ;

}

void button1_wakeup( ) {

    //button1_Log(F("PIN " ) , button1_pin  ) ;
    
}





//////////////////////////////////////

void button1_setup( ) {

    button1_setup_( ) ;

    attachInterrupt(digitalPinToInterrupt(button1_pin), button1_wakeup , CHANGE ) ;

}

///////////////////////////////////////

void button1_loop( ) {

    button1_loop_( ) ;

}



//////////////////////////////////////

bool button2_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 100L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define button2_modulename F( "button2_" )


void button2_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( button2_modulename , k ) ;

    log_print( i ) ;

}

void button2_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( button2_modulename , k ) ;
    log_print( i ) ;

}

void button2_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( button2_modulename , k ) ;
    log_print( i ) ;

}

void button2_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( button2_modulename , k ) ;
    log_print( i ) ;

}

void button2_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( button2_modulename , k ) ;
    log_print( i ) ;

}

void button2_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( button2_modulename , k ) ;
    log_print( i ) ;

}



void button2_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( button2_modulename , k ) ;
    log_print( i ) ;

}



void button2_Setups( ) {

    button2_setup( ) ;

}




void button2_Loops( ) {

    if( ! button2_Trigger( ) ) return ;

    button2_loop( ) ;

}


///////////////////////////////////////


//////////////////////////////////////

byte button2_pin = 2 ;
bool button2_normallyOpen = false ;
unsigned long button2_debounceTime = 50000 ;

//////////////////////////////////////



bool button2_val_ = false ;

//////////////////////////////////////

bool button2_state_ = false ;

//////////////////////////////////////

bool button2_hasChanged_ = false ;
bool button2_hasChangedDownUp_ = false ;
bool button2_hasChangedUpDown_ = false ;

bool button2_isPressed_ = false ;

bool button2_isLongPress_ = false ;

bool button2_wasPressed_ = false ;
bool button2_wasShortPress_ = false ;
bool button2_wasLongPress_ = false ;
bool button2_wasSinglePress_ = false ;
bool button2_wasDoublePress_ = false ;

//////////////////////////////////////

unsigned long button2_timeLongPressStart_ = 500000 ;
unsigned long button2_timeDoublePressEnd_ = 500000 ;

//////////////////////////////////////

int button2_eventsTotal_ = 5 ;
int button2_eventPos_ = 0 ;
bool button2_eventType_[ 5 ] = { false ,  false , false , false , false } ;
unsigned long button2_eventTime_[ 5 ] = { 0 , 0 , 0 , 0 , 0 } ;

void button2_setup_( ) {

    pinMode( button2_pin , INPUT_PULLUP ) ;

    button2_reset_( ) ;

}

void button2_loop_( ) {

    if( digitalRead( button2_pin ) ^ button2_normallyOpen ) {

        button2_actionDown_( ) ;

    } else {

        button2_actionUp_( ) ;

    }

}

void button2_reset_( ) {

    for( int i = 0 ; i < button2_eventsTotal_ ; i++ ) {

        button2_eventType_[ i ] = false ;
        button2_eventTime_[ i ] = clock_getTimeMicros( ) ;

    }

    button2_eventPos_ = 0 ;

    button2_val_ = false ;
    button2_state_ = false ;
    button2_hasChanged_ = false ;
    button2_hasChangedDownUp_ = false ;
    button2_hasChangedUpDown_ = false ;
    button2_isPressed_ = false ;
    button2_isLongPress_ = false ;
    button2_wasPressed_ = false ;
    button2_wasShortPress_ = false ;
    button2_wasLongPress_ = false ;
    button2_wasSinglePress_ = false ;
    button2_wasDoublePress_ = false ;

}


void button2_actionUp_( ) {

    button2_val_ = false ;

    // button2_eventAdd( button2_val ) ;


    if( !button2_debounced_( ) ) {

        return ;

    }

    if( button2_state_ ) {



       if( ( clock_getTimeMicros( ) - button2_eventTime_[ button2_eventPos_ ] ) < button2_timeLongPressStart_ )  {

            button2_wasShortPress_ = true ;
            button2_wasLongPress_ = false ;

       } else {

            button2_wasShortPress_ = false ;
            button2_wasLongPress_ = true ;

       }

        button2_state_ = false ;
        button2_eventAdd_( false ) ;

        button2_isPressed_ = false ;




        
        //button2_Log( "button2_UP" ) ;

    }

}


void button2_actionDown_( ) {

    button2_val_ = true ;

    if( !button2_debounced_( ) ) {

        return ;

    }

    if( ! button2_state_  ) {

        button2_state_ = true ;
        button2_eventAdd_( true ) ;

        button2_isPressed_ = true ;

        //button2_Log( "button2_DOWN" ) ;
        button2_hasChangedUpDown_ = true ;

    }

    // button2_eventAdd( button2_val ) ;


}

void button2_eventAdd_( bool type ) {

    button2_eventPos_++ ;

    if( button2_eventPos_ == button2_eventsTotal_ ) {

        button2_eventPos_ = 0 ;

    }

    button2_eventType_[ button2_eventPos_ ] = type ;
    button2_eventTime_[ button2_eventPos_ ] = clock_getTimeMicros( ) ;

}

bool button2_debounced_( ) {

    if( ( clock_getTimeMicros( ) - button2_eventTime_[ button2_eventPos_ ] ) < button2_debounceTime ) {

        return( false ) ;

    }

    return( true ) ;

}

void button2_wakeup( ) {

    //button2_Log(F("PIN " ) , button2_pin  ) ;
    
}





//////////////////////////////////////

void button2_setup( ) {

    button2_setup_( ) ;
    attachInterrupt(digitalPinToInterrupt(button2_pin), button2_wakeup , CHANGE ) ;

}

///////////////////////////////////////

void button2_loop( ) {


    button2_loop_( ) ;


}



//////////////////////////////////////

bool button3_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 100L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define button3_modulename F( "button3_" )


void button3_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( button3_modulename , k ) ;

    log_print( i ) ;

}

void button3_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( button3_modulename , k ) ;
    log_print( i ) ;

}

void button3_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( button3_modulename , k ) ;
    log_print( i ) ;

}

void button3_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( button3_modulename , k ) ;
    log_print( i ) ;

}

void button3_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( button3_modulename , k ) ;
    log_print( i ) ;

}

void button3_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( button3_modulename , k ) ;
    log_print( i ) ;

}



void button3_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( button3_modulename , k ) ;
    log_print( i ) ;

}



void button3_Setups( ) {

    button3_setup( ) ;

}




void button3_Loops( ) {

    if( ! button3_Trigger( ) ) return ;

    button3_loop( ) ;

}


///////////////////////////////////////


//////////////////////////////////////

byte button3_pin = 4 ;
bool button3_normallyOpen = true ;
unsigned long button3_debounceTime = 50000 ;



bool button3_val_ = false ;

//////////////////////////////////////

bool button3_state_ = false ;

//////////////////////////////////////

bool button3_hasChanged_ = false ;
bool button3_hasChangedDownUp_ = false ;
bool button3_hasChangedUpDown_ = false ;

bool button3_isPressed_ = false ;

bool button3_isLongPress_ = false ;

bool button3_wasPressed_ = false ;
bool button3_wasShortPress_ = false ;
bool button3_wasLongPress_ = false ;
bool button3_wasSinglePress_ = false ;
bool button3_wasDoublePress_ = false ;

//////////////////////////////////////

unsigned long button3_timeLongPressStart_ = 500000 ;
unsigned long button3_timeDoublePressEnd_ = 500000 ;

//////////////////////////////////////

int button3_eventsTotal_ = 5 ;
int button3_eventPos_ = 0 ;
bool button3_eventType_[ 5 ] = { false ,  false , false , false , false } ;
unsigned long button3_eventTime_[ 5 ] = { 0 , 0 , 0 , 0 , 0 } ;

void button3_setup_( ) {

    pinMode( button3_pin , INPUT_PULLUP ) ;

    button3_reset_( ) ;

}

void button3_loop_( ) {

    if( digitalRead( button3_pin ) ^ button3_normallyOpen ) {

        button3_actionDown_( ) ;

    } else {

        button3_actionUp_( ) ;

    }

}

void button3_reset_( ) {

    for( int i = 0 ; i < button3_eventsTotal_ ; i++ ) {

        button3_eventType_[ i ] = false ;
        button3_eventTime_[ i ] = clock_getTimeMicros( ) ;

    }

    button3_eventPos_ = 0 ;

    button3_val_ = false ;
    button3_state_ = false ;
    button3_hasChanged_ = false ;
    button3_hasChangedDownUp_ = false ;
    button3_hasChangedUpDown_ = false ;
    button3_isPressed_ = false ;
    button3_isLongPress_ = false ;
    button3_wasPressed_ = false ;
    button3_wasShortPress_ = false ;
    button3_wasLongPress_ = false ;
    button3_wasSinglePress_ = false ;
    button3_wasDoublePress_ = false ;

}


void button3_actionUp_( ) {

    button3_val_ = false ;

    // button3_eventAdd( button3_val ) ;


    if( !button3_debounced_( ) ) {

        return ;

    }

    if( button3_state_ ) {



       if( ( clock_getTimeMicros( ) - button3_eventTime_[ button3_eventPos_ ] ) < button3_timeLongPressStart_ )  {

            button3_wasShortPress_ = true ;
            button3_wasLongPress_ = false ;

       } else {

            button3_wasShortPress_ = false ;
            button3_wasLongPress_ = true ;

       }

        button3_state_ = false ;
        button3_eventAdd_( false ) ;

        button3_isPressed_ = false ;




        
        //button3_Log( "button3_UP" ) ;

    }

}


void button3_actionDown_( ) {

    button3_val_ = true ;

    if( !button3_debounced_( ) ) {

        return ;

    }

    if( ! button3_state_  ) {

        button3_state_ = true ;
        button3_eventAdd_( true ) ;

        button3_isPressed_ = true ;

        //button3_Log( "button3_DOWN" ) ;
        button3_hasChangedUpDown_ = true ;

    }

    // button3_eventAdd( button3_val ) ;


}

void button3_eventAdd_( bool type ) {

    button3_eventPos_++ ;

    if( button3_eventPos_ == button3_eventsTotal_ ) {

        button3_eventPos_ = 0 ;

    }

    button3_eventType_[ button3_eventPos_ ] = type ;
    button3_eventTime_[ button3_eventPos_ ] = clock_getTimeMicros( ) ;

}

bool button3_debounced_( ) {

    if( ( clock_getTimeMicros( ) - button3_eventTime_[ button3_eventPos_ ] ) < button3_debounceTime ) {

        return( false ) ;

    }

    return( true ) ;

}

void button3_wakeup( ) {

    //button3_Log(F("PIN " ) , button3_pin  ) ;
    
}





//////////////////////////////////////

void button3_setup( ) {

    button3_setup_( ) ;

}

///////////////////////////////////////

void button3_loop( ) {

    button3_loop_( ) ;

}





bool player1_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 1000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}




#define player1_modulename F( "player1_" )


void player1_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( player1_modulename , k ) ;

    log_print( i ) ;

}

void player1_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( player1_modulename , k ) ;
    log_print( i ) ;

}

void player1_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( player1_modulename , k ) ;
    log_print( i ) ;

}

void player1_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( player1_modulename , k ) ;
    log_print( i ) ;

}

void player1_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( player1_modulename , k ) ;
    log_print( i ) ;

}

void player1_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( player1_modulename , k ) ;
    log_print( i ) ;

}



void player1_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( player1_modulename , k ) ;
    log_print( i ) ;

}




void player1_Setups( ) {

    player1_setup( ) ;

}




void player1_Loops( ) {

    if( ! player1_Trigger( ) ) return ;

    player1_loop( ) ;

}




int player1_timeInitialM = 0 ;
int player1_timeInitialS = 0 ;
int player1_timeInitialI = 0 ;

int player1_totalMoves = 0 ;

// in MS
long player1_timeRemaining = 0 ;
// in MS
long player1_timeRemainingCurrent = 0 ;

bool player1_active = false ;


// in MS
unsigned long player1_ctime = 0 ;



///////////////////////////////////////

void player1_setup( ) {

}

void player1_loop( ) {

}





bool player2_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 1000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}




#define player2_modulename F( "player2_" )


void player2_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( player2_modulename , k ) ;

    log_print( i ) ;

}

void player2_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( player2_modulename , k ) ;
    log_print( i ) ;

}

void player2_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( player2_modulename , k ) ;
    log_print( i ) ;

}

void player2_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( player2_modulename , k ) ;
    log_print( i ) ;

}

void player2_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( player2_modulename , k ) ;
    log_print( i ) ;

}

void player2_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( player2_modulename , k ) ;
    log_print( i ) ;

}



void player2_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( player2_modulename , k ) ;
    log_print( i ) ;

}




void player2_Setups( ) {

    player2_setup( ) ;

}




void player2_Loops( ) {

    if( ! player2_Trigger( ) ) return ;

    player2_loop( ) ;

}




int player2_timeInitialM = 0 ;
int player2_timeInitialS = 0 ;
int player2_timeInitialI = 0 ;

int player2_totalMoves = 0 ;

// in MS
long player2_timeRemaining = 0 ;
// in MS
long player2_timeRemainingCurrent = 0 ;

bool player2_active = false ;

// in MS
unsigned long player2_ctime = 0 ;



///////////////////////////////////////

void player2_setup( ) {

}

void player2_loop( ) {

}



///////////////////////////////////////


bool comms_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 10000000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}



#define comms_modulename F( "comms_" )


void comms_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( comms_modulename , k ) ;

    log_print( i ) ;

}

void comms_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( comms_modulename , k ) ;
    log_print( i ) ;

}

void comms_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( comms_modulename , k ) ;
    log_print( i ) ;

}

void comms_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( comms_modulename , k ) ;
    log_print( i ) ;

}

void comms_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( comms_modulename , k ) ;
    log_print( i ) ;

}

void comms_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( comms_modulename , k ) ;
    log_print( i ) ;

}



void comms_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( comms_modulename , k ) ;
    log_print( i ) ;

}



void comms_Setups( ) {

    comms_setup( ) ;

}




void comms_Loops( ) {

    if( ! comms_Trigger( ) ) return ;

    comms_loop( ) ;

}


///////////////////////////////////////


#define comms_pin 7 
bool comms_enabled = false ;

///////////////////////////////////////

void comms_setup( ) {


    pinMode( comms_pin , OUTPUT ) ;
    digitalWrite( comms_pin , comms_enabled ) ;

}

///////////////////////////////////////

void comms_loop( ) {

}

///////////////////////////////////////


void comms_updateEnabled( ) {

    digitalWrite( comms_pin , comms_enabled ) ;

}





#define fsm_modulename F( "fsm_" )


void fsm_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( fsm_modulename , k ) ;

    log_print( i ) ;

}

void fsm_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( fsm_modulename , k ) ;
    log_print( i ) ;

}

void fsm_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( fsm_modulename , k ) ;
    log_print( i ) ;

}

void fsm_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( fsm_modulename , k ) ;
    log_print( i ) ;

}

void fsm_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( fsm_modulename , k ) ;
    log_print( i ) ;

}

void fsm_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( fsm_modulename , k ) ;
    log_print( i ) ;

}



void fsm_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( fsm_modulename , k ) ;
    log_print( i ) ;

}


bool fsm_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 100L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}





void fsm_Setups( ) {

    fsm_setup( ) ;

}




void fsm_Loops( ) {

    if( ! fsm_Trigger( ) ) return ;

    fsm_loop( ) ;

}




bool fsm_enabled = false ;

byte fsm_mode = 0 ;
byte fsm_state = 0 ;
byte fsm_step = 0 ;

bool fsm_changed = true ;


void fsm_setup( ) {

    fsm_Log( F("MODULE"), F( "1,booting" ) ) ;
fsm_Log( F("MODULE"), F( "2,playgame" ) ) ;
fsm_Log( F("MODULE"), F( "3,menu" ) ) ;
fsm_Log( F("MODULE"), F( "4,showingclock" ) ) ;

}


void fsm_loop( ) {

    if( !fsm_enabled ) return ;

    fsm_update( ) ;

    if( fsm_changed ) {

        static char buff[ 20 ] ;

        sprintf_P( buff , PSTR( "%i,%i,%i" ) , fsm_mode , fsm_state , fsm_step ) ;

        fsm_Log( F( "MSS" ) , buff ) ;
        fsm_changed = false ;
        

    }

}



void fsm_update( ) {

    ////////////////////////////////

    
    //////////////////////////////////////
        if( fsm_mode == 1 ) {

            //fsm_Log( F( "fsm_ PRE MODE?" ) ) ;

            wizard_fsmModePre( ) ;

            
if( fsm_state == 1 ) {

    
// EEPROM update runs

if( fsm_step == 1 ) {

    //wizard_storageRunsReset( ) ;

    wizard_buttonsInit( ) ;

    wizard_storageRunsInc( ) ;

    

    fsm_stateSet( 2 ) ;

    return ;

}




    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 2 ) {

    

if( fsm_step == 1 ) {




    //wizard_programSetup( ) ;


    wizard_storageStashGet( ) ;

    wizard_storageSync( ) ;






    //wizard_extrotimeout
    //wizard_button init


    fsm_stateSet( 3 ) ;

    return ;

}


    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 3 ) {

    
// Play boooop-bip sound

if( fsm_step == 1 ) {

    display_clearBuffers( ) ;

    wizard_sfxPlay( 300 , 400 ) ; 

    fsm_stepNext( ) ;

    return ;

}


if( fsm_step == 2 ) {

    wizard_displayReady( ) ;

    if( wizard_sfxIsOff( ) ) {

        wizard_sfxPlay( 600 , 100 ) ; 

        fsm_stepNext( ) ;

        return ;

    }

    return ;

}

if( fsm_step == 3 ) {

    wizard_displayReady( ) ;

    if( wizard_sfxIsOff( ) ) {

        display_reset( ) ;

        sleep_counter = 0 ;


        fsm_modeSet( 4 ) ;

        return ;

    }

    return ;

}



    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


            ////
            fsm_stateSet( 1 ) ;

            return ;

        }
    //////////////////////////////////////

    
    //////////////////////////////////////
        if( fsm_mode == 2 ) {

            //fsm_Log( F( "fsm_ PRE MODE?" ) ) ;

            wizard_fsmModePre( ) ;

            
if( fsm_state == 1 ) {

    
if( fsm_step == 1 ) {

    wizard_sfxPlay( 440 , 500 ) ; 

    fsm_stepNext( ) ;

    return ;

}




if( fsm_step == 2 ) {

    if( wizard_buttonsIsPressed( 1 ) ) {

        return ;

    }

    if( wizard_buttonsIsPressed( 2 ) ) {

        return ;

    } 

    wizard_buttonsInit( ) ;
    
    wizard_playersInit( ) ;

    fsm_Log( F("GAME"),F("START") ) ;

    fsm_stateSet( 2 ) ;

    return ;

}

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 2 ) {

    
if( fsm_step == 1 ) {

    wizard_playersUpdateTimeRemaining( ) ;

    wizard_displayClear( ) ;

    wizard_displayPlayer1TimeRemaining( ) ;
    wizard_displayPlayer2TimeRemaining( ) ;

    fsm_stepNext( ) ;

    return ;

}

if( fsm_step == 2 ) {


    if( wizard_playersDetectMoveAndProcess( ) ) {

        fsm_stepNext( ) ;

        return ;

    }


    ////////////////////////////////////////////////////

    if( wizard_playersDetectEnd( ) ) {

        fsm_stateSet( 4 ) ;

        return ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 3 ) ;

        return ;

    }


    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;


    }

    fsm_stepFirst( ) ;

    return ;

}


if( fsm_step == 3 ) {


    wizard_buttonsUpDownReset( ) ;

    if( wizard_buttonsNotPressed( ) )  {

        //fsm_stepFirst( ) ;

        //return;

    }

    wizard_sfxPlay( 220 , 50 ) ; 
    fsm_stepFirst( ) ;

    return ;

}

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 3 ) {

    
if( fsm_step == 1 ) {

    wizard_playersPauseStart( ) ;

    wizard_sfxPlay( 200 , 100 ) ;

    wizard_countdownSet( 0 , 1000000 ) ;

    // TODO FIXME - seems that display is dirty here...
    // Due to slow refresh of display?
    wizard_displayPaused( ) ;

    fsm_stepNext( ) ;

    return ;

}


if( fsm_step == 2 ) {

    if( wizard_buttonsWasShortPress( 3 ) ) {

        wizard_playersPauseEnd( ) ;

        wizard_sfxPlay( 200 , 100 ) ;

        wizard_buttonsInit( ) ; 

        fsm_stateSet( 2 ) ;

        return ;

    }

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;


    }

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 4 ) {

    

if( fsm_step == 1 ) {

    //fsm_Log( "END" ) ;
    fsm_Log( F("GAME"),F("END") ) ;
    fsm_Log(F("P1"),player1_timeRemainingCurrent);
    fsm_Log(F("P2"),player2_timeRemainingCurrent);
    
    wizard_sfxPlay( 880 , 1500 ) ; 

    fsm_stepNext( ) ;

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_sfxIsOff( ) ) {

        wizard_countdownSet( 0 , 5000000 ) ;
        fsm_stepNext( ) ;

        return ;

    }

    return ;

}

if( fsm_step == 3 ) {

    if( wizard_countdownDone( 0 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    return ;

}









    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


            ////
            fsm_stateSet( 1 ) ;

            return ;

        }
    //////////////////////////////////////

    
    //////////////////////////////////////
        if( fsm_mode == 3 ) {

            //fsm_Log( F( "fsm_ PRE MODE?" ) ) ;

            wizard_fsmModePre( ) ;

            
if( fsm_state == 1 ) {

    
if( fsm_step == 1 ) {

    wizard_displayChoosePrograms( ) ;

    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 2 ) ;

        return ;

    }     

    if( wizard_buttonsWasShortPress( 2 ) ) {

        fsm_stateSet( 2 ) ;

        return ;

    }     


    ///////////////////////////////////////////////
    
    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepNext( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     


    return ;

}

if( fsm_step == 2 ) {


    wizard_displayChangeTime( ) ;


    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 9 ) ;

        return ;

    }     


    ///////////////////////////////////////////////

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepNext( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     

    return ;

}

if( fsm_step == 3 ) {


    wizard_displayChangeBrightness( ) ;


    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 13 ) ;

        return ;

    }     


    ///////////////////////////////////////////////

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepNext( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     

    return ;

}


if( fsm_step == 4 ) {


    wizard_displayMenuControlComms( ) ;


    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 15 ) ;

        return ;

    }     


    ///////////////////////////////////////////////

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepNext( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     

    return ;

}

if( fsm_step == 5 ) {


    wizard_displayMenuControlSound( ) ;


    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 17 ) ;

        return ;

    }     


    ///////////////////////////////////////////////

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepNext( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     

    return ;

}


if( fsm_step == 6 ) {

    wizard_displayMenuControlSleep( ) ;

    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 19 ) ;

        return ;

    }     

    ///////////////////////////////////////////////

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepNext( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     

    return ;

}

if( fsm_step == 7 ) {


    wizard_displayMenuStash( ) ;

    if( wizard_buttonsWasShortPress( 1 ) ) {

        fsm_stateSet( 21 ) ;

        return ;

    }     


    ///////////////////////////////////////////////

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stepFirst( ) ;

        return ;

    }     

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }     

    return ;

}






    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 2 ) {

    


if( fsm_step == 1 ) {

    wizard_displayCurrentProgram( ) ;

    fsm_stepNext( ) ;


    return ;

}


if( fsm_step == 2 ) {


    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programNext( ) ;

        return ;

    }


    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programPrev( ) ;

        return ;

    }


    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 3 ) ;

        return ;

    }

    if( wizard_buttonsWasLongPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    


    fsm_stepFirst( ) ;


    return ;

}



    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 3 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer1Settings( true , true , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;
        fsm_stepNext( ) ;

    }


    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer1Settings( false , true , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;
        fsm_stepNext( ) ;


    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programCurrentPlayer1MinutesAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programCurrentPlayer1MinutesAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 4 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 4 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer1Settings( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer1Settings( true , false , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programCurrentPlayer1SecondsAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programCurrentPlayer1SecondsAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 5 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 5 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer1Settings( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer1Settings( true , true , false ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programCurrentPlayer1IncrementAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programCurrentPlayer1IncrementAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 6 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 6 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer2Settings( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer2Settings( false , true , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programCurrentPlayer2MinutesAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programCurrentPlayer2MinutesAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 7 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 7 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer2Settings( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer2Settings( true , false , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programCurrentPlayer2SecondsAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programCurrentPlayer2SecondsAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 8 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 8 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer2Settings( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayPlayer2Settings( true , true , false ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_programCurrentPlayer2IncrementAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_programCurrentPlayer2IncrementAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 9 ) {

    

if( fsm_step == 1 ) {

    fsm_stateSet( 10 ) ;
    
   return ;
   
} 
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 10 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayClock( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayClock( false , true , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_rtcAdd( 3600 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_rtcAdd( -3600 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 11 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 11 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayClock( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayClock( true , false , true ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_rtcAdd( 60 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_rtcAdd( -60 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_stateSet( 12 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 12 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayClock( true , true , true ) ;
        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;
        wizard_displayClock( true , true , false ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        //wizard_programCurrentPlayer1MinutesAdd( 1 ) ;
        wizard_rtcAdd( 1 ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        //wizard_programCurrentPlayer1MinutesAdd( -1 ) ;
        wizard_rtcAdd( -1 ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 13 ) {

    

if( fsm_step == 1 ) {

    fsm_stateSet( 14 ) ;
    
   return ;
   
} 
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 14 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        //wizard_displayClock( true , true , true ) ;
        wizard_displayBrightness( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        //wizard_displayClock( false , true , true ) ;


        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        //wizard_rtcAdd( 3600 ) ;
        wizard_displayBrightnessInc( ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        //wizard_rtcAdd( -3600 ) ;
        wizard_displayBrightnessDec( ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 15 ) {

    

if( fsm_step == 1 ) {

    fsm_stateSet( 16 ) ;
    
   return ;
   
} 

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 16 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        wizard_displayCommsSwitchValue( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;
        

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_commsSwitchToggle( ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_commsSwitchToggle( ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 17 ) {

    

if( fsm_step == 1 ) {

    fsm_stateSet( 18 ) ;
    
   return ;
   
} 

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 18 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        wizard_displaySoundSwitchValue( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;
        

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_sfxSwitchToggle( ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_sfxSwitchToggle( ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 19 ) {

    

if( fsm_step == 1 ) {

    fsm_stateSet( 20 ) ;
    
   return ;
   
} 

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 20 ) {

    

if( fsm_step == 1 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        wizard_displaySleepSwitchValue( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;
        

    }

    return ;

}

if( fsm_step == 2 ) {

    if( wizard_countdownDone( 0 ) ) {

        display_clearBuffers( ) ;

        wizard_countdownSet( 0 , 100000 ) ;

        fsm_stepNext( ) ;

    }

    return ;

}


if( fsm_step == 3 ) {

    if( wizard_buttonsWasShortPress( 1 ) ) {

        wizard_sleepSwitchToggle( ) ;

    }

    if( wizard_buttonsWasShortPress( 2 ) ) {

        wizard_sleepSwitchToggle( ) ;

    }

    if( wizard_buttonsWasShortPress( 3 ) ) {

        fsm_modeSet( 1 ) ;

        return ;

    }

    fsm_stepFirst( ) ;

    return ;

}
    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


if( fsm_state == 21 ) {

    

if( fsm_step == 1 ) {


    //fsm_Log( F( "MSS" ) , F( "STASH!!!" ) ) ;

    wizard_storageStashPut( ) ;

    fsm_modeSet( 1 ) ;
    //fsm_modeSet( 1 ) ;
    
   return ;
   
} 

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


            ////
            fsm_stateSet( 1 ) ;

            return ;

        }
    //////////////////////////////////////

    
    //////////////////////////////////////
        if( fsm_mode == 4 ) {

            //fsm_Log( F( "fsm_ PRE MODE?" ) ) ;

            wizard_fsmModePre( ) ;

            
if( fsm_state == 1 ) {

    

if( fsm_step == 1 ) {

    sleep_enable( ) ;

    display_clearBuffers( ) ;

    wizard_displayTime( ) ;

    fsm_stepNext( ) ;

    return ;

}

if( fsm_step == 2 ) {


    if( wizard_buttonsWasShortPress( 3 ) ) {
        fsm_modeSet( 3 ) ;
        return;
    }



    if( wizard_buttonsHaveOrBeenPressed( 1 ) ) {

        wizard_playersActive( 2 ) ;
        fsm_modeSet( 2 ) ;
        return ;

    }


    if( wizard_buttonsHaveOrBeenPressed( 2 ) ) {

        wizard_playersActive( 1 ) ;
        fsm_modeSet( 2 ) ;
        return ;

    }
    fsm_stepFirst( ) ;

    return ;

}

    ////

    fsm_stepSet( 1 ) ;
    
    return ;

}


            ////
            fsm_stateSet( 1 ) ;

            return ;

        }
    //////////////////////////////////////

    
////////////////////////////////

//    fsm_Log( "BOOTING" ) ;

    //fsm_Log( "fsm_ POST MODE?" ) ;

    fsm_modeSet( 1 ) ;

}

void fsm_modeSet( byte mode ) {

    if( fsm_mode == mode ) {

        return ;

    }
    
    fsm_mode = mode ;
    fsm_state = 1 ;
    fsm_step = 1 ;

    fsm_changed = true ;

    wizard_fsmModeChange( ) ;

}

void fsm_stateSet( byte state ) {
    
    if( fsm_state == state ) {

        return ;

    }

    fsm_state = state ;
    fsm_step = 1 ;

    fsm_changed = true ;

    wizard_fsmStateChange( ) ;

}

void fsm_stepSet( byte step ) {
    
    if( fsm_step == step ) {

        return ;

    }

    fsm_step = step ;

    // to many messages to include step changes
    //fsm_changed = true ;

    wizard_fsmStepChange( ) ;

}

void fsm_stepNext( ) {

    fsm_stepSet( fsm_step + 1 ) ;

}

void fsm_stepPrevious( ) {

    fsm_stepSet( fsm_step - 1 ) ;

}

void fsm_stepFirst( ) {

    fsm_stepSet( 1 ) ;

}









#define wizard_modulename F( "wizard_" )


void wizard_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( wizard_modulename , k ) ;

    log_print( i ) ;

}

void wizard_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( wizard_modulename , k ) ;
    log_print( i ) ;

}

void wizard_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( wizard_modulename , k ) ;
    log_print( i ) ;

}

void wizard_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( wizard_modulename , k ) ;
    log_print( i ) ;

}

void wizard_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( wizard_modulename , k ) ;
    log_print( i ) ;

}

void wizard_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( wizard_modulename , k ) ;
    log_print( i ) ;

}



void wizard_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( wizard_modulename , k ) ;
    log_print( i ) ;

}





void wizard_serialPrint( const char * i , bool newline ) {

    serial_print( i , newline ) ;

}


void wizard_serialPrint( byte i , bool newline ) {

    serial_print( i , newline ) ;

}

void wizard_serialPrint( int i , bool newline ) {

    serial_print( i , newline ) ;

}

void wizard_serialPrint( unsigned int i , bool newline ) {

    serial_print( i , newline ) ;

}

void wizard_serialPrint( long i , bool newline ) {

    serial_print( i , newline ) ;

}

void wizard_serialPrint( unsigned long i , bool newline ) {

    serial_print( i , newline ) ;

}

void wizard_serialPrint( const __FlashStringHelper * i , bool newline ) {

    serial_print( i , newline ) ;

}




void wizard_incomingParse( char * buffer ) {

    char messageFromPC[ sizeof( buffer ) ] = { 0 } ;
    int integerFromPC1 = 0 ;
    int integerFromPC2 = 0 ;

    char * strtokIndx ;

    strtokIndx = strtok( buffer , "," ) ;    
    strcpy( messageFromPC , strtokIndx ) ; 
 
    strtokIndx = strtok( NULL , "," ) ; 
    integerFromPC1 = atoi( strtokIndx ) ;    

    strtokIndx = strtok( NULL , "," ) ;
    integerFromPC2 = atoi( strtokIndx ) ;   

    //////////////////////////////////////////////////////////////

    if( strcmp( messageFromPC , "B" ) == 0 ) {

        wizard_buttonsSimulateWasPressed( integerFromPC1 ) ;

        return ;

    }

    if( strcmp( messageFromPC , "S" ) == 0 ) {

        wizard_sfxPlay( integerFromPC1 , integerFromPC2 ) ;

        return ;

    }

    // SLEEP SHUTDOWN (1) TRUE/FALSE (0/1)
    if( strcmp( messageFromPC , "SLEEP" ) == 0 ) {

        if( integerFromPC1 == 0 ) {

            if( integerFromPC2 == 0 ) {

                sleep_shutdown = false ;

            }

            if( integerFromPC2 == 1 ) {

                sleep_shutdown = true ;

            }

        }

        return ;

    }


}

void wizard_debugMem( ) {

    wizard_Log( F( "MEM" ) , wizard_freeMemory( )  ) ;

}

#ifdef __arm__
extern "C" char* sbrk(int incr);
#else  
extern char *__brkval;
#endif  

int wizard_freeMemory( ) {

    static char top ;

#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  

}



void wizard_fsmModePre( ) {

    if( rtc_updated( ) ) {

        //wizard_Log( F( "MODEPRERTC" ) , rtc_millis( ) ) ;

    }

}


void wizard_fsmModeChange( ) {

    //wizard_Log("wizard_fsmModeChange"));

}

void wizard_fsmStateChange( ) {
    //wizard_Log(F("wizard_fsmStateChange"));

}

void wizard_fsmStepChange( ) {
    //wizard_Log(F("wizard_fsmStepChange"));

}



const unsigned int wizard_storageMem = 2 ;

bool wizard_storageLoaded = false ;
bool wizard_storageSynced = false ;

////////////////////////////////////////////////////////

struct wizard_storageProgram {

  int player1TimeInitialM ;
  int player1TimeInitialS ;
  int player1TimeInitialI ;

  int player2TimeInitialM ;
  int player2TimeInitialS ;
  int player2TimeInitialI ;

} ;

const int wizard_storageProgramTotal = 5 ;

////////////////////////////////////////////////////////

struct wizard_storageObject {

    unsigned long magic ;

    unsigned int release ;

    byte brightness ;
    byte sound ;
    byte node ;

    byte sleep ;

    int program ;

    wizard_storageProgram programs[ wizard_storageProgramTotal ] ;

} ;


wizard_storageObject wizard_storageStash ;


void wizard_storageInit( ) {

    wizard_storageStash.magic = 0x41c4ebe8 ;

    wizard_storageStash.release = 1  ;

    wizard_storageStash.brightness = 7 ;
    wizard_storageStash.sound = 1 ;
    wizard_storageStash.node = 0 ;

    wizard_storageStash.sleep = 0 ;

    wizard_storageStash.program = 0 ;

    wizard_storageStash.programs[ 0 ] = { 3 , 0 , 2 , 3 , 0 , 2 } ;
    wizard_storageStash.programs[ 1 ] = { 5 , 0 , 2 , 5 , 0 , 2 } ;
    wizard_storageStash.programs[ 2 ] = { 10 , 0 , 2 , 10 , 0 , 2 } ;
    wizard_storageStash.programs[ 3 ] = { 15 , 0 , 10 , 15 , 0 , 10 } ;
    wizard_storageStash.programs[ 4 ] = { 90 , 0 , 30 , 90 , 0 , 30 } ;

}

void wizard_storageSync( ) {

    if( wizard_storageSynced ) return ;

    display_brightness = wizard_storageStash.brightness ;
    display_brightnessUpdate( ) ;

    if( wizard_storageStash.sound == 1 ) {

        buzzer_enabled = true ;

    } else {

        buzzer_enabled = false ;

    }

    if( wizard_storageStash.node == 1 ) {

        comms_enabled = true ;

    } else {

        comms_enabled = false ;
        
    }
    comms_updateEnabled( ) ;

    if( wizard_storageStash.sleep == 1 ) {

        sleep_shutdown = true ;

    } else {

        sleep_shutdown = false ;

    }


    wizard_programSetCurrent( wizard_storageStash.program ) ;

    wizard_storageSynced = true ;

}


void wizard_storageStashGet( ) {

    if( wizard_storageLoaded ) return;

    wizard_storageInit( ) ;

    wizard_storageObject customVar ;

    EEPROM.get( wizard_storageMem, customVar );

    if( customVar.magic == wizard_storageStash.magic ) {

        wizard_Log( F( "STASH" ) , "loading" ) ; 

        memcpy( &wizard_storageStash , &customVar , sizeof( wizard_storageStash ) ) ;

        //wizard_Log( F("OBJ"),wizard_StorageStash.magic);

    } else {

        wizard_Log( F( "STASH" ) , "saving" ) ;

        wizard_storageStashPut( ) ; 
    
    }

    wizard_storageLoaded = true ;

}

void wizard_storageStashPut( ) {


    wizard_storageStash.brightness = display_brightness ;

    if( buzzer_enabled ) {

        wizard_storageStash.sound = 1 ;
    
    } else {

        wizard_storageStash.sound = 0 ;
    }

    if( comms_enabled ) {

        wizard_storageStash.node = 1 ;
    
    } else {
        
        wizard_storageStash.node = 0 ;
    }


    if( sleep_shutdown ) {

        wizard_storageStash.sleep = 1 ;
    
    } else {
        
        wizard_storageStash.sleep = 0 ;

    }

    EEPROM.put( wizard_storageMem ,  wizard_storageStash );

}

void wizard_storageRunsInc( ) {

    static bool doneOnce = false ;

    static const unsigned int mem = 0 ;
    static unsigned int val = 0 ;

    if( doneOnce ) {

        //wizard_Log( F( "already stored runs ???" ) ) ;

        return ;

    }

    byte ee1 = EEPROM.read( mem ) ;
    byte ee2 = EEPROM.read( mem + 1 ) ;

    if( ee2 == 255 ) {

        ee2 = 0 ;

        if( ee1 == 255 ) {

            ee1 = 0 ;

        } else {

            ee1++ ;

        }

        // write now to reduce number of writes
        eeprom_write( mem , ee1 ) ;

        //eeprom_write( mem , 255 ) ;
        //eeprom_write( mem + 1 , 255 ) ;


    } else {

        ee2++ ;

    }

    eeprom_write( mem + 1 , ee2 ) ;

    val = ( ee1 << 8 ) + ee2 ;

    wizard_Log( F( "RUNS" ) , val ) ;

    doneOnce = true ;
    

}







void wizard_sfxPlay( int freq , int duration ) {

    buzzer_beep( freq , duration ) ;

}


bool wizard_sfxIsOff( ) {


    return( buzzer_isOff( ) ) ;


}

void wizard_sfxSwitchToggle( ) {


    buzzer_enabled = ! buzzer_enabled ;


}




// FIXME TODO - more optimum? rather than refreshing each time??
void wizard_displayClear( ) {

    display_clearBuffers( ) ;
    display_refresh( ) ;

}

void wizard_displayTime( ) {

    int h = rtc_now.hour( ) ;

    if( h > 12 ) h = h - 12 ;

    if( h == 0 ) h = 12 ;
 
    if( h > 9 ) {

        display_printNumberPad( rtc_now.minute( )  , 2 , 2 ) ;
        display_printNumberPad( h , 4 , 0 ) ;

    } else {

        display_printNumberPad( rtc_now.minute( )  , 2 , 2 ) ;
        display_printNumberPad( h , 4 , 0 ) ;

    }

    display_setDot( 4 , true ) ;
    //wizard_displayDebug( ) ;

    //int s = rtc_now.second( ) ;
    unsigned long s = rtc_millis( )/1000 ;

    if( ( s % 2 ) == 0 ) {
        display_setDot( 1 , true ) ;
    }else{
        display_setDot( 6 , true ) ;
    }


    unsigned long os = clock_getTimeMillis( ) ;
    //unsigned long os = rtc_millis( ) ;
    os = os / 1000 ;

    if( ( os % 2 ) == 0 ) {

        display_setDot( 0 , true ) ;

    } else {

        display_setDot( 7 , true ) ;
        
    }

}


void wizard_displayPlayer1TimeRemaining( ) {

    //wizard_Log(String(player2_timeRemaining));
    if(player1_timeRemainingCurrent<=0){
        display_printNumberPad( 0 , 0 , 1 ) ;
        return;
    }

    int minutes = ( player1_timeRemainingCurrent / 1000 )  / 60 ;
    int seconds = ( player1_timeRemainingCurrent / 1000 ) % 60 ;
    int millis = int( ( player1_timeRemainingCurrent / 100 ) % 10 );



    if( minutes > 0 ) {  

        display_printNumberPad( minutes , 2 , 1 ) ;
        display_printNumberPad( seconds , 0 , 2 ) ;
        display_setDot( 2 , true ) ;

    } else {

        display_printNumberPad( seconds , 1 , 1 ) ;
        display_printNumberPad( millis , 0 , 1 ) ;
        display_setDot( 1 , true ) ;

    }


}



void wizard_displayPlayer2TimeRemaining( ) {

    //wizard_Log(String(player2_timeRemaining));

    if(player2_timeRemainingCurrent<=0){
        display_printNumberPad( 0 , 7 , 1 ) ;
        return;
    }

    int minutes = ( player2_timeRemainingCurrent / 1000 )  / 60 ;
    int seconds = ( player2_timeRemainingCurrent / 1000 ) % 60 ;
    int millis = int( ( player2_timeRemainingCurrent / 100 ) % 10 );

    if( minutes > 0 ) {  

        if( minutes < 10 ) {

            display_printNumberPad( minutes , 7 , 1 ) ;
            display_printNumberPad( seconds , 5 , 2 ) ;

            display_setDot( 7 , true ) ;

        } else {
            
            display_printNumberPad( minutes , 6 , 1 ) ;
            display_printNumberPad( seconds , 4 , 2 ) ;

            display_setDot( 6 , true ) ;

            
        }

    }else{


        if( seconds < 10 ) {

            display_printNumberPad( seconds , 7 , 1 ) ;
            display_printNumberPad( millis , 6 , 1 ) ;

            display_setDot( 7 , true ) ;

        } else {
            
            display_printNumberPad( seconds , 6 , 2 ) ;
            display_printNumberPad( millis , 5 , 1 ) ;

            display_setDot( 6 , true ) ;

            
        }



    }

}


void wizard_displayPlayer1Settings( bool minutesvisible,bool secondsvisible,bool incrementvisible ) {

    if(minutesvisible){
        display_printNumberPad( player1_timeInitialM , 4 , 2 ) ;
    }
    if(secondsvisible){
        display_printNumberPad( player1_timeInitialS , 2 , 2 ) ;
    }
    if(incrementvisible){
        display_printNumberPad( player1_timeInitialI , 0 , 2 ) ;
    }

    display_setDot( 2 , true ) ;
    display_setDot( 4 , true ) ;

}


void wizard_displayPlayer2Settings( bool minutesvisible,bool secondsvisible,bool incrementvisible ) {

    if(minutesvisible){
        display_printNumberPad( player2_timeInitialM , 6 , 2 ) ;
    }
    if(secondsvisible){
        display_printNumberPad( player2_timeInitialS , 4 , 2 ) ;
    }
    if(incrementvisible){
        display_printNumberPad( player2_timeInitialI , 2 , 2 ) ;
    }

    display_setDot( 4 , true ) ;
    display_setDot( 6 , true ) ;

}


void wizard_displayPaused( ) {

    display_reset( ) ;

    display_printByte( B1100111 , 1 ) ;
    display_printByte( B1110111 , 2 ) ;
    display_printByte( B0011100 , 3 ) ;
    display_printByte( B1011011 , 4 ) ;
    display_printByte( B1001111 , 5 ) ;
    display_printByte( B0111101 , 6 ) ;

}





void wizard_displayChoosePrograms( ) {

    wizard_displayClear( ) ;

    display_printByte( B1100111 , 0 ) ;
    display_printByte( B0000101 , 1 ) ;
    display_printByte( B0011101 , 2 ) ;
    display_printByte( B1111011 , 3 ) ;

}

void wizard_displayChangeTime( ) {

    wizard_displayClear( ) ;

    display_printByte( B0111101 , 0 ) ;
    display_printByte( B1110111 , 1 ) ;
    display_printByte( B0001111 , 2 ) ;
    display_printByte( B1101111 , 3 ) ;


}


void wizard_displayChangeBrightness( ) {

    wizard_displayClear( ) ;
    display_printByte( B0001110 , 0 ) ;
    display_printByte( B0010000 , 1 ) ;
    display_printByte( B1111011 , 2 ) ;
    display_printByte( B0010111 , 3 ) ;
    display_printByte( B0001111 , 4 ) ;


}

void wizard_displayMenuControlComms( ) {

    wizard_displayClear( ) ;

    display_printByte( B0010101 , 0 ) ;
    display_printByte( B0011101 , 1 ) ;
    display_printByte( B0111101 , 2 ) ;
    display_printByte( B1101111 , 3 ) ;


}

void wizard_displayMenuControlSound( ) {

    wizard_displayClear( ) ;
    display_printByte( B1011011 , 0 ) ;
    display_printByte( B0011101 , 1 ) ;
    display_printByte( B0011100 , 2 ) ;
    display_printByte( B0010101 , 3 ) ;
    display_printByte( B0111101 , 4 ) ;


}

void wizard_displayMenuStash( ) {

    wizard_displayClear( ) ;
    display_printByte( B1011011 , 0 ) ;
    display_printByte( B0001111 , 1 ) ;
    display_printByte( B1110111 , 2 ) ;
    display_printByte( B1011011 , 3 ) ;
    display_printByte( B0010111 , 4 ) ;

}

void wizard_displayMenuControlSleep( ) {

    wizard_displayClear( ) ;

    display_printByte( B1011011 , 0 ) ;
    display_printByte( B0001110 , 1 ) ;
    display_printByte( B1001111 , 2 ) ;
    display_printByte( B1001111 , 3 ) ;
    display_printByte( B1100111 , 4 ) ;


}

////////

void wizard_displayCurrentProgram( ) {

    wizard_displayClear( ) ;

    display_printNumberPad( wizard_programGetCurrent( ) + 1 , 3 , 1 ) ;

    display_printByte( B1100111 , 0 ) ;
    display_printByte( B0000101 , 1 ) ;
    display_printByte( B0011101 , 2 ) ;
    display_printByte( B1111011 , 3 ) ;

}


void wizard_displayClock( bool hoursvisible , bool minutesvisible , bool secondsvisible ) {

    if( hoursvisible ) {

        display_printNumberPad( rtc_now.hour( )  , 6 , 2 ) ;

    }

    if( minutesvisible ) {

        display_printNumberPad( rtc_now.minute( )  , 3 , 2 ) ;

    }

    if( secondsvisible ) {

        display_printNumberPad( rtc_now.second( )  , 0 , 2 ) ;

    }


}


void wizard_displayBrightness( ) {

    display_printNumberPad( display_brightness , 0 , 2 ) ;
}



void wizard_displayBrightnessInc( ) {
    if(display_brightness==15){
        display_brightness=0;
    }else{
        display_brightness++;
    }


    display_brightnessUpdate();
}


void wizard_displayBrightnessDec( ) {
    if(display_brightness==0){
        display_brightness=15;
    }else{
        display_brightness--;
    }
    display_brightnessUpdate();
}


void wizard_displayCommsSwitchValue( ) {

    if( comms_enabled ) {

        display_printByte( B0000000 , 5 ) ;
        display_printByte( B0011101 , 6 ) ;
        display_printByte( B0010101 , 7 ) ;

    } else {

        display_printByte( B0011101 , 5 ) ;
        display_printByte( B1000111 , 6 ) ;
        display_printByte( B1000111 , 7 ) ;

    }

    comms_updateEnabled( ) ;

}


void wizard_displaySleepSwitchValue( ) {

    if( !sleep_shutdown ) {

        display_printByte( B0000000 , 5 ) ;
        display_printByte( B0011101 , 6 ) ;
        display_printByte( B0010101 , 7 ) ;

    } else {

        display_printByte( B0011101 , 5 ) ;
        display_printByte( B1000111 , 6 ) ;
        display_printByte( B1000111 , 7 ) ;

    }

    comms_updateEnabled( ) ;

}



void wizard_displaySoundSwitchValue( ) {

    if( buzzer_enabled ) {

        display_printByte( B0000000 , 5 ) ;
        display_printByte( B0011101 , 6 ) ;
        display_printByte( B0010101 , 7 ) ;

    } else {

        display_printByte( B0011101 , 5 ) ;
        display_printByte( B1000111 , 6 ) ;
        display_printByte( B1000111 , 7 ) ;

    }


    //buzzer_updateEnabled( ) ;

}

void wizard_displayReady( ) {

    display_ready();
    //buzzer_updateEnabled( ) ;

}





void wizard_rtcAdd( int t ) {

    DateTime now = rtc_now ;

    time_t t1 = now.unixtime( ) ;

    //wizard_Log(String(t1));
    t1 += t ;

    //wizard_Log(String(t1));

    DateTime newdt = new DateTime( t1 ) ;

    rtc_rtc.adjust( t1 ) ;

}




bool wizard_buttonsIsPressed( int bid ) {


    if(bid==1){

        if(button1_isPressed_){
            return(true);
        }

    }


    if(bid==2){

        if(button2_isPressed_){
            return(true);
        }

    }


    if(bid==3){

        if(button3_isPressed_){
            return(true);
        }

    }    

    return( false ) ;


}


bool wizard_buttonsNotPressed( ) {

    if(wizard_buttonsIsPressed(1)) return(false);
    if(wizard_buttonsIsPressed(2)) return(false);
    if(wizard_buttonsIsPressed(3)) return(false);

    return(true);


}


bool wizard_buttonsHaveOrBeenPressed( int bid ) {


    if(bid==1){

        if(button1_isPressed_){
            return(true);
        }
        if(button1_wasPressed_){
            button1_wasPressed_ = false ;
            return(true);
        }

    }


    if(bid==2){

        if(button2_isPressed_){
            return(true);
        }
        if(button2_wasPressed_){
            button2_wasPressed_ = false ;
            return(true);
        }

    }
    

    return( false ) ;


}

bool wizard_buttonsWasShortPress( int b ) {

    if(b==3){

        if( button3_wasShortPress_ ) {

            button3_wasShortPress_ = false ;


            return(true);
        }

        return(false);

    }


    if(b==1){

        if( button1_wasShortPress_ ) {

            button1_wasShortPress_ = false ;


            return(true);
        }

        return(false);

    }


    if(b==2){

        if( button2_wasShortPress_ ) {

            button2_wasShortPress_ = false ;


            return(true);
        }

        return(false);

    }
    return(false);

}

bool wizard_buttonsWasLongPress( int b ) {

    if(b==3){

        if( button3_wasLongPress_ ) {

            button3_wasLongPress_ = false ;


            return(true);
        }

        return(false);

    }

    return(false);

}

bool wizard_buttonsWentUpDown( int b ) {

    if( b == 1 ) {

        if( button1_hasChangedUpDown_ ) {

            button1_hasChangedUpDown_ = false ;


            return( true ) ;
        }

        return( false ) ;

    }

    if( b == 2 ) {

        if( button2_hasChangedUpDown_ ) {

            button2_hasChangedUpDown_ = false ;


            return( true ) ;
        }

        return( false ) ;

    }

    return( false ) ;

}


void wizard_buttonsSimulateWasPressed( int b ) {

    //wizard_Log(String(b));

    if( b == 3 ) {

        button3_wasPressed_ = true ;
        return ;

    }


    if( b == 2 ) {

        button2_wasPressed_ = true ;
        return ;

    }

    if( b == 1 ) {

        button1_wasPressed_ = true ;
        return ;

    }


}


void wizard_buttonsInit( ) {

    button1_reset_( ) ;
    button2_reset_( ) ;
    button3_reset_( ) ;
    
}

void wizard_buttonsUpDownReset( ) {

    button1_hasChangedUpDown_ = false ;
    button2_hasChangedUpDown_ = false ;
    button3_hasChangedUpDown_ = false ;

}










unsigned long wizard_countdownDataTime[ 10 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ;
unsigned long wizard_countdownDataWait[ 10 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ;

bool wizard_countdownCompleted[ 10 ] = { true , true , true , true , true , true , true , true , true , true } ;

void wizard_countdownSet( byte c , unsigned long t ) {

    wizard_countdownDataTime[ c ] = clock_getTimeMicros( ) ;
    wizard_countdownDataWait[ c ] = t ;
    wizard_countdownCompleted[ c ] = false ;

}

bool wizard_countdownDone( byte c ) {

    if( wizard_countdownCompleted[ c ] ) {

        return( true ) ;

    }

    if( ( clock_getTimeMicros( ) - wizard_countdownDataTime[ c ] ) < wizard_countdownDataWait[ c ] ) {

        return( false ) ;

    }

    wizard_countdownCompleted[ c ] = true ;

    return( true ) ;     

}




void wizard_playersSet1( int timem , int times  , int timei  ) {

    //wizard_Log(String(timem));

    player1_timeInitialM = timem ;
    player1_timeInitialS = times ;
    player1_timeInitialI = timei ;

}

void wizard_playersSet2( int timem , int times  , int timei  ) {

    //wizard_Log(String(timem));
    player2_timeInitialM = timem ;
    player2_timeInitialS = times ;
    player2_timeInitialI = timei ;

}

void wizard_playersActive( int p ) {

    if( p == 1 ) {

        player1_active = true ;
        player2_active = false ;
        
        return ;

    }

    if( p == 2 ) {

        player1_active = false ;
        player2_active = true ;

        return ;

    }


}

void wizard_playersInit( ) {

    //wizard_Log(String(player1_timeInitialM));
    //wizard_Log(String(player2_timeInitialM));

    //player1_ctime = clock_getTimeMillis( ) ;
    player1_ctime = rtc_millis( ) ;
    player1_totalMoves = 0 ;
    player1_timeRemaining = ( ( player1_timeInitialM * 60L ) + player1_timeInitialS ) * 1000L ;
    
    //player2_ctime = clock_getTimeMillis( ) ;
    player2_ctime = rtc_millis( ) ;
    player2_totalMoves = 0 ;
    player2_timeRemaining = ( ( player2_timeInitialM * 60L ) + player2_timeInitialS ) * 1000L ;


    player1_timeRemainingCurrent = player1_timeRemaining ;
    player2_timeRemainingCurrent = player2_timeRemaining ;

    //wizard_Log(String(player1_timeRemaining));
    //wizard_Log(String(player2_timeRemaining));

}

void wizard_playersUpdateTimeRemaining( ) {


    if( player1_active ) {

//        player1_timeRemainingCurrent = player1_timeRemaining  - ( clock_getTimeMillis( ) - player1_ctime ) ;
        player1_timeRemainingCurrent = player1_timeRemaining - ( rtc_millis( ) - player1_ctime ) ;

        return ;
    }

    if( player2_active ) {

        //player2_timeRemainingCurrent = player2_timeRemaining  - ( clock_getTimeMillis( ) - player2_ctime ) ;
        player2_timeRemainingCurrent = player2_timeRemaining - ( rtc_millis( ) - player2_ctime ) ;

        return ;

    }


    
}

void wizard_playersTimeLock(  ) {

    if( player1_active ) {

        player1_timeRemaining = player1_timeRemainingCurrent + ( player1_timeInitialI * 1000L ) ;
        player1_timeRemainingCurrent  = player1_timeRemaining ;
        
    }

    if( player2_active ) {

        player2_timeRemaining = player2_timeRemainingCurrent + ( player2_timeInitialI * 1000L ) ;
        player2_timeRemainingCurrent  = player2_timeRemaining ;
 
    }


    //wizard_Log("times," + String(player1_timeRemainingCurrent)+","+String(player2_timeRemainingCurrent));

    //player1_ctime = clock_getTimeMillis( ) ;
    //player2_ctime = clock_getTimeMillis( ) ;
    player1_ctime = rtc_millis( ) ;
    player2_ctime = rtc_millis( ) ;


}


bool wizard_playersDetectMoveAndProcess( ) {


    if( player1_active ) {

        //if( wizard_buttonsHaveOrBeenPressed( 1 ) ) {
        if( wizard_buttonsWentUpDown( 1 ) ) {

            wizard_playersTimeLock( ) ;
            wizard_playersActive( 2 ) ;
            wizard_sfxPlay( 300 , 100 ) ; 

            //wizard_buttonsInit( ) ;

            return( true ) ;

        }

        return( false ) ;

    }


    if( player2_active ) {

        //if( wizard_buttonsHaveOrBeenPressed( 2 ) ) {
        if( wizard_buttonsWentUpDown( 2 ) ) {

            wizard_playersTimeLock( ) ;
            wizard_playersActive( 1 ) ;
            wizard_sfxPlay( 300 , 100 ) ; 
            
            //wizard_buttonsInit( ) ;

            return( true ) ;

        }

        return( false ) ;

    }


}

bool wizard_playersDetectEnd( ) {


    if( player1_active ) {

        if( player1_timeRemainingCurrent <= 0 ) {

            //wizard_Log("times," + String(player1_timeRemainingCurrent)+","+String(player2_timeRemainingCurrent));
            return( true ) ;

        }

        return( false ) ;

    }

    if( player2_active ) {
        if( player2_timeRemainingCurrent <= 0 ) {

            //wizard_Log("times," + String(player1_timeRemainingCurrent)+","+String(player2_timeRemainingCurrent));
            return( true ) ;

        }

        return( false ) ;

    }


}

void wizard_playersPauseStart( ) {

    if( player1_active ) {

        player1_timeRemaining = player1_timeRemainingCurrent ;

    }

    if( player2_active ) {

        player2_timeRemaining = player2_timeRemainingCurrent ;

    }

    //player1_ctime = clock_getTimeMillis( ) ;
    //player2_ctime = clock_getTimeMillis( ) ;
    player1_ctime = rtc_millis( ) ;
    player2_ctime = rtc_millis( ) ;


}

void wizard_playersPauseEnd( ) {

    //player1_ctime = clock_getTimeMillis( ) ;
    //player2_ctime = clock_getTimeMillis( ) ;
    player1_ctime = rtc_millis( ) ;
    player2_ctime = rtc_millis( ) ;

}




void wizard_programSetCurrent( int id ) {

    //wizard_Log(String(id));

    wizard_storageStash.program = id ;

    wizard_playersSet1( wizard_storageStash.programs[ id ].player1TimeInitialM , wizard_storageStash.programs[ id ].player1TimeInitialS , wizard_storageStash.programs[ id ].player1TimeInitialI ) ;

    wizard_playersSet2( wizard_storageStash.programs[ id ].player2TimeInitialM , wizard_storageStash.programs[ id ].player2TimeInitialS , wizard_storageStash.programs[ id ].player2TimeInitialI ) ;



}

int wizard_programGetCurrent( ) {

    return( wizard_storageStash.program ) ;


}


void wizard_programNext( ) {

    wizard_storageStash.program++ ;
    if( wizard_storageStash.program == wizard_storageProgramTotal ) {

        wizard_storageStash.program = 0 ;

    }

    wizard_programSetCurrent( wizard_storageStash.program ) ;

}


void wizard_programPrev( ) {

    wizard_storageStash.program-- ;

    if( wizard_storageStash.program < 0 ) {

        wizard_storageStash.program=wizard_storageProgramTotal-1 ;

    }

    wizard_programSetCurrent( wizard_storageStash.program ) ;


}


void wizard_programCurrentPlayer1MinutesAdd( int t ) {

    int a = wizard_storageStash.programs[ wizard_storageStash.program ].player1TimeInitialM ;

    a += t ;

    if( a > 99 ) a = 0 ;
    if( a < 0 ) a = 99 ;

    wizard_storageStash.programs[ wizard_storageStash.program ].player1TimeInitialM = a ;
    wizard_programSetCurrent( wizard_storageStash.program ) ;

}

void wizard_programCurrentPlayer1SecondsAdd( int t ) {

    int a = wizard_storageStash.programs[ wizard_storageStash.program ].player1TimeInitialS ;

    a += t ;

    if( a > 59 ) a = 0 ;
    if( a < 0 ) a = 59 ;

    wizard_storageStash.programs[ wizard_storageStash.program ].player1TimeInitialS = a ;
    wizard_programSetCurrent( wizard_storageStash.program ) ;

}


void wizard_programCurrentPlayer1IncrementAdd( int t ) {

    int a = wizard_storageStash.programs[ wizard_storageStash.program ].player1TimeInitialI ;

    a += t ;

    if( a > 99 ) a = 0 ;
    if( a < 0 ) a = 99 ;

    wizard_storageStash.programs[ wizard_storageStash.program ].player1TimeInitialI = a ;
    wizard_programSetCurrent( wizard_storageStash.program ) ;



}

void wizard_programCurrentPlayer2MinutesAdd( int t ) {

    int a = wizard_storageStash.programs[ wizard_storageStash.program ].player2TimeInitialM ;

    a += t ;

    if( a > 99 ) a = 0 ;
    if( a < 0 ) a = 99 ;

    wizard_storageStash.programs[ wizard_storageStash.program ].player2TimeInitialM = a ;
    wizard_programSetCurrent( wizard_storageStash.program ) ;



}

void wizard_programCurrentPlayer2SecondsAdd( int t ) {

    int a = wizard_storageStash.programs[ wizard_storageStash.program ].player2TimeInitialS ;

    a += t ;

    if( a > 59 ) a = 0 ;
    if( a < 0 ) a = 59 ;

    wizard_storageStash.programs[ wizard_storageStash.program ].player2TimeInitialS = a ;
    wizard_programSetCurrent( wizard_storageStash.program ) ;

}


void wizard_programCurrentPlayer2IncrementAdd( int t ) {

    int a = wizard_storageStash.programs[ wizard_storageStash.program ].player2TimeInitialI ;

    a += t ;

    if( a > 99 ) a = 0 ;
    if( a < 0 ) a = 99 ;

    wizard_storageStash.programs[ wizard_storageStash.program ].player2TimeInitialI = a ;
    wizard_programSetCurrent( wizard_storageStash.program ) ;

}








void wizard_commsSwitchToggle( ) {

    comms_enabled =! comms_enabled ;



}




bool wizard_sleepIsOff( ) {

    return( sleep_shutdown ) ;

}

void wizard_sleepSwitchToggle( ) {


    sleep_shutdown = ! sleep_shutdown ;


}






#define grandmaster_modulename F( "grandmaster_" )


void grandmaster_Log( const __FlashStringHelper * k , const __FlashStringHelper * i ) {


    log_initClientModule( grandmaster_modulename , k ) ;

    log_print( i ) ;

}

void grandmaster_Log( const __FlashStringHelper * k , char * i ) {

    log_initClientModule( grandmaster_modulename , k ) ;
    log_print( i ) ;

}

void grandmaster_Log( const __FlashStringHelper * k , byte i ) {

    log_initClientModule( grandmaster_modulename , k ) ;
    log_print( i ) ;

}

void grandmaster_Log( const __FlashStringHelper * k , int i ) {

    log_initClientModule( grandmaster_modulename , k ) ;
    log_print( i ) ;

}

void grandmaster_Log( const __FlashStringHelper * k , unsigned int i ) {

    log_initClientModule( grandmaster_modulename , k ) ;
    log_print( i ) ;

}

void grandmaster_Log( const __FlashStringHelper * k , long i ) {

    log_initClientModule( grandmaster_modulename , k ) ;
    log_print( i ) ;

}



void grandmaster_Log( const __FlashStringHelper * k , unsigned long i ) {

    log_initClientModule( grandmaster_modulename , k ) ;
    log_print( i ) ;

}


bool grandmaster_Trigger( ) {

    static unsigned long triggerCurrentTime = micros( ) ;

    if( ( clock_getTimeMicros( ) - triggerCurrentTime ) < 1000L ) {

        return( false ) ;

    } else { 

        triggerCurrentTime = clock_getTimeMicros( ) ;

        return( true ) ;

    }

}





void grandmaster_Setups( ) {

    grandmaster_setup( ) ;

}




void grandmaster_Loops( ) {

    if( ! grandmaster_Trigger( ) ) return ;

    grandmaster_loop( ) ;

}




void grandmaster_setup( ) {

    randomSeed( analogRead( 3 ) ) ;

}

void grandmaster_loop( ) {

    fsm_enabled = true ;
    watchdog_reset( ) ;

}



void setup( ) {

power_Setups( ) ;
watchdog_Setups( ) ;
sleep_Setups( ) ;
serial_Setups( ) ;
log_Setups( ) ;
clock_Setups( ) ;
buzzer_Setups( ) ;
display_Setups( ) ;
rtc_Setups( ) ;
button1_Setups( ) ;
button2_Setups( ) ;
button3_Setups( ) ;
player1_Setups( ) ;
player2_Setups( ) ;
comms_Setups( ) ;
fsm_Setups( ) ;
grandmaster_Setups( ) ;

}



void loop( ) {

power_Loops( ) ;
watchdog_Loops( ) ;
sleep_Loops( ) ;
serial_Loops( ) ;
log_Loops( ) ;
clock_Loops( ) ;
buzzer_Loops( ) ;
display_Loops( ) ;
rtc_Loops( ) ;
button1_Loops( ) ;
button2_Loops( ) ;
button3_Loops( ) ;
player1_Loops( ) ;
player2_Loops( ) ;
comms_Loops( ) ;
fsm_Loops( ) ;
grandmaster_Loops( ) ;

}

