#ifndef _N2kGroupFunctionHandlers_H_
#define _N2kGroupFunctionHandlers_H_

#include "NMEA2000_CompilerDefns.h"
#include "N2kGroupFunction.h"
#include "State.h"

#if !defined(N2K_NO_GROUP_FUNCTION_SUPPORT)

/*******************************************************************/
//
// 65379 is pilot Mode in Raymarine autopilots
//
//  Pilot mode may be 
//      - 0 : Standby
//      - 64 : Auto compass commanded
//      - 256 : Vane, Wind Mode
//      - 384 : Track Mode
//      - 385 : No Drift COG Referenced
//  
//      With a request will send the info in pilot
//      With command will set the mode
//
//      PyPilot will have 64 as Compass, 256 as Wind, 385 as GPS
//      I don't know if PyPilot has track mode (I believe no) But
//      we may set it in the interface without too much hassle

class tN2kGroupFunctionHandlerForPGN65379 : public tN2kGroupFunctionHandler {
  protected:
    virtual bool HandleRequest(const tN2kMsg &N2kMsg, 
                               uint32_t TransmissionInterval, 
                               uint16_t TransmissionIntervalOffset, 
                               uint8_t  NumberOfParameterPairs,
                               int iDev);
    virtual bool HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev);
    tState* state; 

  public:
    tN2kGroupFunctionHandlerForPGN65379(tNMEA2000 *_pNMEA2000, tState *_state) : tN2kGroupFunctionHandler(_pNMEA2000,65379L),state(_state){}
};

/*******************************************************************/
//
// 127250 is vessel heading
//
// 
//  
//      With a request will send the info
//      With command will set the mode
//


class tN2kGroupFunctionHandlerForPGN127250 : public tN2kGroupFunctionHandler {
  protected:
    virtual bool HandleRequest(const tN2kMsg &N2kMsg, 
                               uint32_t TransmissionInterval, 
                               uint16_t TransmissionIntervalOffset, 
                               uint8_t  NumberOfParameterPairs,
                               int iDev);
    
    tState* state; 

  public:
    tN2kGroupFunctionHandlerForPGN127250(tNMEA2000 *_pNMEA2000, tState *_state) : tN2kGroupFunctionHandler(_pNMEA2000,127250),state(_state){}
};

/*******************************************************************/
//
// 127745 is rudder angle
//
// 
//  
//      With a request will send the info
//     
//


class tN2kGroupFunctionHandlerForPGN127245 : public tN2kGroupFunctionHandler {
  protected:
    virtual bool HandleRequest(const tN2kMsg &N2kMsg, 
                               uint32_t TransmissionInterval, 
                               uint16_t TransmissionIntervalOffset, 
                               uint8_t  NumberOfParameterPairs,
                               int iDev);
   
    tState* state; 

  public:
    tN2kGroupFunctionHandlerForPGN127245(tNMEA2000 *_pNMEA2000, tState *_state) : tN2kGroupFunctionHandler(_pNMEA2000,127745),state(_state){}
};

/*******************************************************************/
//
// 65360 is locked heading
//  must do request and command (for setting it)
// 
//  
//      With a request will send the info
//      With command will set the mode
//


class tN2kGroupFunctionHandlerForPGN65360 : public tN2kGroupFunctionHandler {
  protected:
    virtual bool HandleRequest(const tN2kMsg &N2kMsg, 
                               uint32_t TransmissionInterval, 
                               uint16_t TransmissionIntervalOffset, 
                               uint8_t  NumberOfParameterPairs,
                               int iDev);
    virtual bool HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev);
    tState* state; 

  public:
    tN2kGroupFunctionHandlerForPGN65360(tNMEA2000 *_pNMEA2000, tState *_state) : tN2kGroupFunctionHandler(_pNMEA2000,65360),state(_state){}
};
#endif
#endif