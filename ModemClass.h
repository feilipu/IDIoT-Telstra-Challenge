/*
   Class example for communicating the the Telstra LPWAN modem.
*/

#ifndef _MODEMCLASS_H
#define _MODEMCLASS_H

#include <stdlib.h>
#include <stdarg.h>

#include <Arduino.h>

#include "SoftwareSerial.h"
//#include "AltSoftSerial.h"

// To have the Regexp.h line below work, firstly
// 1) Download the file http://gammon.com.au/Arduino/Regexp.zip
// 2) Within the Arduino IDE, use Sketch > Include Library > Add .ZIP Library
//    and select the Regexp.zip file you just downloaded
#include "Regexp.h"

// Remove this define to keep the USB comms quiet
#define DEBUG

#define rx_timeout 8000
#define rx_timeout_fast 2000
#define rx_buffer 192

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#endif

/***************************************************************

   LoRaModem Class

   LoRaModem.checkAT() -> Send AT to the modem, expect AT+OK back.
   LoRaModem.checkID() -> Retrieve the current ID parameters from the modem (updates LoRaModem.DevAddr / DevEui / AppEui)
   LoRaModem.setID()   -> Set the ID parameters in the modem
   LoRaModem.setKeys() -> Set the NW and App ciphering / integrity keys
   LoRaModem.setPort() -> Set the port # for App payloads
   LoRaModem.Msg()     -> Send a message, no ACK (uplink only)
   LoRaModem.MsgHEX()  -> Send a HEX string message, no ACK (uplink only)
   LoRaModem.cMsg()    -> Send a message, wait for an ACK
   LoRaModem.cMsgHEX() -> Send a HEX string message, wait for an ACK
   LoRaModem.cMsgBytes()> Send a Hex message from a bytes buffer, wait for an ACK
   LoRaModem.Reset()   -> Reset the modem. This is useful if it hangs, sometimes happens after sending a few hundred messages
   LoRaModem.lowPower()-> Sleep the modem.  
   LoRaModem.getAscii()-> If a Downlink payload is received, extract it and decode from hex to Ascii (use after Msg or cMsg).

   LoRaModem.modemResp -> char buffer with the last message received on UART

*/

class LoRaModem
{
  public:
    LoRaModem();
    int checkAT();
    int checkID();
    int getDR();
    int disableADR();
    int setDR(String DR);
    int setID(String addr, String dev);
    int setKeys(String NWKey, String AppKey);
    int setPort(String portNum);
    int Msg(String message);
    int MsgHEX(String message);
    int cMsg(String message);
    int cMsgHEX(String message);
    int cMsgBytes(uint8_t * bytes, int16_t length);
    int lowPower();
    int Reset();

    String getAscii();

    char modemResp[rx_buffer];
    char DevAddr[12];
    char DevEui[24];
    char AppEui[24];

  private:
    MatchState _rspMs;
    void _sendSerial(String message);
    int _checkresponse(const char* checkVal, int call_timeout, int _trim);
    void prnt(char *fmt, ... );
    char _DR[1];
};


#endif

