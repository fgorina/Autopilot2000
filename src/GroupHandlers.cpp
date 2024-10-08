
#include <string.h>
#include "GroupHandlers.h"
#include "NMEA2000.h"

#define MANUFACTURER_RAYMARINE 1851
#define INDUSTRY_MARINE 4

#define N2kPGN65379_ManufacturerCode_field 1
#define N2kPGN65379_Reserved_field 2
#define N2kPGN65379_IndustryCode_field 3
#define N2kPGN65379_PilotMode_field 4
#define N2kPGN65379_SubMode_field 5
#define N2kPGN65379_PilotModeData_field 6
#define N2kPGN65379_Reserved_field_1 7



#if !defined(N2K_NO_GROUP_FUNCTION_SUPPORT)




//*****************************************************************************
// See document https://web.archive.org/web/20150910070107/http://www.nmea.org/Assets/20140710%20nmea-2000-060928%20iso%20address%20claim%20pgn%20corrigendum.pdf
// For requirements for handling Group function request for PGN 60928
bool tN2kGroupFunctionHandlerForPGN65379::HandleRequest(const tN2kMsg &N2kMsg,
                               uint32_t TransmissionInterval,
                               uint16_t TransmissionIntervalOffset,
                               uint8_t  NumberOfParameterPairs,
                               int iDev) {
  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec=GetRequestGroupFunctionTransmissionOrPriorityErrorCode(TransmissionInterval,TransmissionIntervalOffset);
  bool MatchFilter=true;
  tN2kMsg N2kRMsg;

  // Start to build response
  SetStartAcknowledge(N2kRMsg,N2kMsg.Source,PGN,
                      N2kgfPGNec_Acknowledge,  // Always acknowledge for mandatory PGN
                      pec,
                      NumberOfParameterPairs);
  N2kRMsg.Destination=N2kMsg.Source;

  if ( NumberOfParameterPairs>0 ) { // We need to filter according to fields
    int i;
    int Index;
    uint8_t field;
    tNMEA2000::tDeviceInformation DI=pNMEA2000->GetDeviceInformation(iDev);
    tN2kGroupFunctionParameterErrorCode FieldErrorCode;
    bool FoundInvalidField=false;

    StartParseRequestPairParameters(N2kMsg,Index);
    // Next read new field values. Note that if message is not broadcast, we need to parse all fields always.
    for (i=0; i<NumberOfParameterPairs && (MatchFilter || !tNMEA2000::IsBroadcast(N2kMsg.Destination)); i++) {
      if ( !FoundInvalidField) {
        field=N2kMsg.GetByte(Index);
        switch (field) {
          case N2kPGN65379_ManufacturerCode_field:
            MatchRequestField(N2kMsg.Get2ByteUInt(Index),(uint16_t)MANUFACTURER_RAYMARINE,(uint16_t)0x3fff,MatchFilter,FieldErrorCode);
            FieldErrorCode=N2kgfpec_Acknowledge;
            break;
          case N2kPGN65379_Reserved_field:
            
            break;
          case N2kPGN65379_IndustryCode_field:
            MatchRequestField(N2kMsg.GetByte(Index),(uint8_t)INDUSTRY_MARINE,(uint8_t)0x07,MatchFilter,FieldErrorCode);
            break;

          case N2kPGN65379_PilotMode_field:       
            N2kMsg.Get2ByteUInt(Index);    
            break;

          case N2kPGN65379_SubMode_field:        
            N2kMsg.Get2ByteUInt(Index);  
            
            break;

          case N2kPGN65379_PilotModeData_field:
            N2kMsg.GetByte(Index);
            break;

          case N2kPGN65379_Reserved_field_1:
            N2kMsg.GetByte(Index);
            break;
          
          default:
            FieldErrorCode=N2kgfpec_InvalidRequestOrCommandParameterField;
            MatchFilter=false;
            FoundInvalidField=true;
        }
      } else {
        // If there is any invalid field, we can not parse others, since we do not
        // know right data length. So for rest of the fields we can only send response below.
        FieldErrorCode=N2kgfpec_TemporarilyUnableToComply;
      }
      AddAcknowledgeParameter(N2kRMsg,i,FieldErrorCode);
    }
  }

  bool RequestOK=(MatchFilter && pec==N2kgfTPec_Acknowledge);

  // Send Acknowledge, if request was not broadcast and it did not match
  if ( !RequestOK ) {
    if ( !tNMEA2000::IsBroadcast(N2kMsg.Destination) ) pNMEA2000->SendMsg(N2kRMsg,iDev);
  } else {
    // Send delayed - there was problems with test tool with too fast response.
    // Must Send PGN
    pNMEA2000->SendIsoAddressClaim(0xff,iDev,2);
  }

  return true;
}

/*****************************************************************************/
// Command group function for 60928 can be used to set Device Instance Lower, Device Instance Upper and System Instance
// values on device name. They all are in padded to one byte.
bool tN2kGroupFunctionHandlerForPGN65379::HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev) {
  int i;
  int Index;
  uint8_t field;
  uint8_t DILower=0xff;
  uint8_t DIUpper=0xff;
  uint8_t SI=0xff;
  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec=N2kgfTPec_Acknowledge;
  tN2kGroupFunctionParameterErrorCode PARec;
  tN2kMsg N2kRMsg;

    SetStartAcknowledge(N2kRMsg,N2kMsg.Source,PGN,
                        N2kgfPGNec_Acknowledge,  // What we actually should response as PGN error, if we have invalid field?
                        pec,
                        NumberOfParameterPairs);

    if (PrioritySetting!=8) pec=N2kgfTPec_TransmitIntervalOrPriorityNotSupported;
    StartParseCommandPairParameters(N2kMsg,Index);
    // Next read new field values
    for (i=0; i<NumberOfParameterPairs; i++) {
      field=N2kMsg.GetByte(Index);
      PARec=N2kgfpec_Acknowledge;
      switch (field) {
        case N2kPGN60928_DeviceInstanceLower_field:
          DILower=N2kMsg.GetByte(Index) & 0x7;
          break;
        case N2kPGN60928_DeviceInstanceUpper_field:
          DIUpper=N2kMsg.GetByte(Index) & 0x1f;
          break;
        case N2kPGN60928_SystemInstance_field:
          SI=N2kMsg.GetByte(Index) & 0x0f;
          break;
        default:
          PARec=N2kgfpec_InvalidRequestOrCommandParameterField;
      }

      AddAcknowledgeParameter(N2kRMsg,i,PARec);
    }

    pNMEA2000->SendMsg(N2kRMsg,iDev);
    pNMEA2000->SetDeviceInformationInstances(DILower,DIUpper,SI,iDev);

    return true;
}


#endif
