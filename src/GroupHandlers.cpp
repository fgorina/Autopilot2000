
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
// PGN65379 - Request Mode
bool tN2kGroupFunctionHandlerForPGN65379::HandleRequest(const tN2kMsg &N2kMsg,
                                                        uint32_t TransmissionInterval,
                                                        uint16_t TransmissionIntervalOffset,
                                                        uint8_t NumberOfParameterPairs,
                                                        int iDev)
{
  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = GetRequestGroupFunctionTransmissionOrPriorityErrorCode(TransmissionInterval, TransmissionIntervalOffset);
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;

  Serial.println("Received Group request");
  // Start to build response
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // Always acknowledge for mandatory PGN
                      pec,
                      NumberOfParameterPairs);
  N2kRMsg.Destination = N2kMsg.Source;

  if (NumberOfParameterPairs > 0)
  { // We need to filter according to fields
    int i;
    int Index;
    uint8_t field;
    tNMEA2000::tDeviceInformation DI = pNMEA2000->GetDeviceInformation(iDev);
    tN2kGroupFunctionParameterErrorCode FieldErrorCode;
    bool FoundInvalidField = false;
    Serial.print("Number of pairs ");
    Serial.println(NumberOfParameterPairs);
    StartParseRequestPairParameters(N2kMsg, Index);
    // Next read new field values. Note that if message is not broadcast, we need to parse all fields always.
    for (i = 0; i < NumberOfParameterPairs && (MatchFilter || !tNMEA2000::IsBroadcast(N2kMsg.Destination)); i++)
    {
      if (!FoundInvalidField)
      {
        field = N2kMsg.GetByte(Index);
        switch (field)
        {
        case N2kPGN65379_ManufacturerCode_field:
        {
          MatchRequestField(N2kMsg.Get2ByteUInt(Index), (uint16_t)MANUFACTURER_RAYMARINE, (uint16_t)0x3fff, MatchFilter, FieldErrorCode);
          break;
        }

        case N2kPGN65379_Reserved_field:
        {
          break;
        }

        case N2kPGN65379_IndustryCode_field:
        {
          MatchRequestField(N2kMsg.GetByte(Index), (uint8_t)INDUSTRY_MARINE, (uint8_t)0x07, MatchFilter, FieldErrorCode);
          break;
        }

        case N2kPGN65379_PilotMode_field:
        {
          N2kMsg.Get2ByteUInt(Index);
          break;
        }

        case N2kPGN65379_SubMode_field:
        {
          N2kMsg.Get2ByteUInt(Index);
          break;
        }
        case N2kPGN65379_PilotModeData_field:
        {
          N2kMsg.GetByte(Index);
          break;
        }

        case N2kPGN65379_Reserved_field_1:
        {
          N2kMsg.GetByte(Index);
          break;
        }

        default:
        {
          Serial.print("Field outside specs ");
          Serial.println(field);
          FieldErrorCode = N2kgfpec_InvalidRequestOrCommandParameterField;
          MatchFilter = false;
          FoundInvalidField = true;
        }
        }
      }
      else
      {
        // If there is any invalid field, we can not parse others, since we do not
        // know right data length. So for rest of the fields we can only send response below.
        FieldErrorCode = N2kgfpec_TemporarilyUnableToComply;
      }
      AddAcknowledgeParameter(N2kRMsg, i, FieldErrorCode);
    }
  }

  bool RequestOK = (MatchFilter && pec == N2kgfTPec_Acknowledge);

  // Send Acknowledge, if request was not broadcast and it did not match
  if (!RequestOK)
  {
    Serial.println("Sending error");
    if (!tNMEA2000::IsBroadcast(N2kMsg.Destination))
      pNMEA2000->SendMsg(N2kRMsg, iDev);
  }
  else
  {
    // Send delayed - there was problems with test tool with too fast response.
    // Must Send PGN
    Serial.println("Sending mode");
    pypilot->sendPilotMode(pNMEA2000); // perhaps use iDev as second parameter???
  }

  return true;
}

/*****************************************************************************/
// PGN65379 - Set Mode
bool tN2kGroupFunctionHandlerForPGN65379::HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev)
{
  int i;
  int Index;
  uint8_t field;
  uint16_t raymarineMode = 0;

  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = N2kgfTPec_Acknowledge;
  tN2kGroupFunctionParameterErrorCode PARec;
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;
  Serial.println("Received Group command");
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // What we actually should response as PGN error, if we have invalid field?
                      pec,
                      NumberOfParameterPairs);

  if (PrioritySetting != 8)
    pec = N2kgfTPec_TransmitIntervalOrPriorityNotSupported;
  StartParseCommandPairParameters(N2kMsg, Index);
  // Next read new field values
  for (i = 0; i < NumberOfParameterPairs; i++)
  {
    field = N2kMsg.GetByte(Index);
    PARec = N2kgfpec_Acknowledge;
    switch (field)
    {
    case N2kPGN65379_ManufacturerCode_field:
    {
      MatchRequestField(N2kMsg.Get2ByteUInt(Index), (uint16_t)MANUFACTURER_RAYMARINE, (uint16_t)0x3fff, MatchFilter, PARec);
      break;
    }
    case N2kPGN65379_Reserved_field:

      break;
    case N2kPGN65379_IndustryCode_field:
    {
      MatchRequestField(N2kMsg.GetByte(Index), (uint8_t)INDUSTRY_MARINE, (uint8_t)0x07, MatchFilter, PARec);
      break;
    }

    case N2kPGN65379_PilotMode_field:
    {
      raymarineMode = N2kMsg.Get2ByteUInt(Index);
      Serial.print("Received change mode to ");
      Serial.println(raymarineMode);
      break;
    }

    case N2kPGN65379_SubMode_field:
    {
      N2kMsg.Get2ByteUInt(Index);

      break;
    }

    case N2kPGN65379_PilotModeData_field:
    {
      N2kMsg.GetByte(Index);
      break;

    case N2kPGN65379_Reserved_field_1:
      N2kMsg.GetByte(Index);
      break;
    }
    default:
    {
      PARec = N2kgfpec_InvalidRequestOrCommandParameterField;
    }
    }

    AddAcknowledgeParameter(N2kRMsg, i, PARec);
  }

  pNMEA2000->SendMsg(N2kRMsg, iDev);

  if (PARec == N2kgfpec_Acknowledge)
  {
    pypilot->setRaymarineMode(tRaymarineMode(raymarineMode), tDataOrigin::kNMEA2000);
  }

  return true;
}

//*****************************************************************************
// PGN127250 - Request Vessel Heading

bool tN2kGroupFunctionHandlerForPGN127250::HandleRequest(const tN2kMsg &N2kMsg,
                                                         uint32_t TransmissionInterval,
                                                         uint16_t TransmissionIntervalOffset,
                                                         uint8_t NumberOfParameterPairs,
                                                         int iDev)
{
  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = GetRequestGroupFunctionTransmissionOrPriorityErrorCode(TransmissionInterval, TransmissionIntervalOffset);
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;

  Serial.println("Received Vessel Heading request");
  // Start to build response
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // Always acknowledge for mandatory PGN
                      pec,
                      NumberOfParameterPairs);
  N2kRMsg.Destination = N2kMsg.Source;

  // Send delayed - there was problems with test tool with too fast response.
  // Must Send PGN

  pypilot->sendVesselHeading(pNMEA2000); // perhaps use iDev as second parameter???

  return true;
}

//*****************************************************************************
// PGN127245 - Request for Rudder
bool tN2kGroupFunctionHandlerForPGN127245::HandleRequest(const tN2kMsg &N2kMsg,
                                                         uint32_t TransmissionInterval,
                                                         uint16_t TransmissionIntervalOffset,
                                                         uint8_t NumberOfParameterPairs,
                                                         int iDev)
{
  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = GetRequestGroupFunctionTransmissionOrPriorityErrorCode(TransmissionInterval, TransmissionIntervalOffset);
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;

  Serial.println("Received Rudder  request");
  // Start to build response
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // Always acknowledge for mandatory PGN
                      pec,
                      NumberOfParameterPairs);
  N2kRMsg.Destination = N2kMsg.Source;

  // Send delayed - there was problems with test tool with too fast response.
  // Must Send PGN

  pypilot->sendVesselHeading(pNMEA2000); // perhaps use iDev as second parameter???

  return true;
}

/*******************************************************************/
//
// 65360 is locked heading
//  must do request and command (for setting it)
//
//
//      With a request will send the info
//      With command will set the mode
//
#define N2kPGN65360_ManufacturerCode_field 1
#define N2kPGN65360_Reserved_field 2
#define N2kPGN65360_IndustryCode_field 3
#define N2kPGN65360_SID_field 4
#define N2kPGN65360_TargetHeadingTrue_field 5
#define N2kPGN65360_TargetHeadingMagnetic_field 6
#define N2kPGN65360_Reserved_field_1 7

bool tN2kGroupFunctionHandlerForPGN65360::HandleRequest(const tN2kMsg &N2kMsg,
                                                        uint32_t TransmissionInterval,
                                                        uint16_t TransmissionIntervalOffset,
                                                        uint8_t NumberOfParameterPairs,
                                                        int iDev)
{

  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = GetRequestGroupFunctionTransmissionOrPriorityErrorCode(TransmissionInterval, TransmissionIntervalOffset);
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;

  Serial.println("Received Locked Heading Request");
  // Start to build response
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // Always acknowledge for mandatory PGN
                      pec,
                      NumberOfParameterPairs);
  N2kRMsg.Destination = N2kMsg.Source;

  if (NumberOfParameterPairs > 0)
  { // We need to filter according to fields
    int i;
    int Index;
    uint8_t field;
    tNMEA2000::tDeviceInformation DI = pNMEA2000->GetDeviceInformation(iDev);
    tN2kGroupFunctionParameterErrorCode FieldErrorCode;
    bool FoundInvalidField = false;

    Serial.println(NumberOfParameterPairs);
    StartParseRequestPairParameters(N2kMsg, Index);
    // Next read new field values. Note that if message is not broadcast, we need to parse all fields always.
    for (i = 0; i < NumberOfParameterPairs && (MatchFilter || !tNMEA2000::IsBroadcast(N2kMsg.Destination)); i++)
    {
      if (!FoundInvalidField)
      {
        field = N2kMsg.GetByte(Index);
        switch (field)
        {
        case N2kPGN65360_ManufacturerCode_field:
        {
          MatchRequestField(N2kMsg.Get2ByteUInt(Index), (uint16_t)MANUFACTURER_RAYMARINE, (uint16_t)0x3fff, MatchFilter, FieldErrorCode);
          break;
        }

        case N2kPGN65360_Reserved_field:
        {
          break;
        }

        case N2kPGN65360_IndustryCode_field:
        {
          MatchRequestField(N2kMsg.GetByte(Index), (uint8_t)INDUSTRY_MARINE, (uint8_t)0x07, MatchFilter, FieldErrorCode);
          break;
        }

        case N2kPGN65360_SID_field:
        {
          N2kMsg.GetByte(Index);
          break;
        }

        case N2kPGN65360_TargetHeadingTrue_field:
        {
          N2kMsg.Get2ByteDouble(0.0001, Index);
          break;
        }
        case N2kPGN65360_TargetHeadingMagnetic_field:
        {
          N2kMsg.Get2ByteDouble(0.0001, Index);
          break;
        }

        case N2kPGN65379_Reserved_field_1:
        {
          N2kMsg.GetByte(Index);
          break;
        }

        default:
        {
          Serial.print("Field outside specs ");
          Serial.println(field);
          FieldErrorCode = N2kgfpec_InvalidRequestOrCommandParameterField;
          MatchFilter = false;
          FoundInvalidField = true;
        }
        }
      }
      else
      {
        // If there is any invalid field, we can not parse others, since we do not
        // know right data length. So for rest of the fields we can only send response below.
        FieldErrorCode = N2kgfpec_TemporarilyUnableToComply;
      }
      AddAcknowledgeParameter(N2kRMsg, i, FieldErrorCode);
    }
  }

  bool RequestOK = (MatchFilter && pec == N2kgfTPec_Acknowledge);

  // Send Acknowledge, if request was not broadcast and it did not match
  if (!RequestOK)
  {
    Serial.println("Sending error for Locked Heading Request");
    if (!tNMEA2000::IsBroadcast(N2kMsg.Destination))
      pNMEA2000->SendMsg(N2kRMsg, iDev);
  }
  else
  {
    // Send delayed - there was problems with test tool with too fast response.
    // Must Send PGN

    pypilot->sendLockedHeading(pNMEA2000); // perhaps use iDev as second parameter???
  }
  return true;
}

bool tN2kGroupFunctionHandlerForPGN65360::HandleCommand(const tN2kMsg &N2kMsg, uint8_t PrioritySetting, uint8_t NumberOfParameterPairs, int iDev)
{
  int i;
  int Index;
  uint8_t field;
  double magneticHeading = 0.0;
  double trueHeading = 0.0;

  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = N2kgfTPec_Acknowledge;
  tN2kGroupFunctionParameterErrorCode PARec;
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;
  Serial.println("Received Group command");
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // What we actually should response as PGN error, if we have invalid field?
                      pec,
                      NumberOfParameterPairs);

  if (PrioritySetting != 8)
    pec = N2kgfTPec_TransmitIntervalOrPriorityNotSupported;
  StartParseCommandPairParameters(N2kMsg, Index);
  // Next read new field values
  for (i = 0; i < NumberOfParameterPairs; i++)
  {
    field = N2kMsg.GetByte(Index);
    PARec = N2kgfpec_Acknowledge;
    switch (field)
    {
    case N2kPGN65360_ManufacturerCode_field:
    {
      MatchRequestField(N2kMsg.Get2ByteUInt(Index), (uint16_t)MANUFACTURER_RAYMARINE, (uint16_t)0x3fff, MatchFilter, PARec);
      break;
    }
    case N2kPGN65360_Reserved_field:

      break;
    case N2kPGN65360_IndustryCode_field:
    {
      MatchRequestField(N2kMsg.GetByte(Index), (uint8_t)INDUSTRY_MARINE, (uint8_t)0x07, MatchFilter, PARec);
      break;
    }

    case N2kPGN65360_SID_field:
    {
       N2kMsg.GetByte(Index);

      break;
    }

    case N2kPGN65360_TargetHeadingTrue_field:
    {
      trueHeading = N2kMsg.Get2ByteDouble(0.0001, Index) / 3.141592 * 180.0;
     
      break;
    }

    case N2kPGN65360_TargetHeadingMagnetic_field:
    {
      magneticHeading = N2kMsg.Get2ByteDouble(0.0001, Index) / 3.141592 * 180.0;
      break;
    }

    case N2kPGN65360_Reserved_field_1:
    {
      N2kMsg.GetByte(Index);
      break;
    }

    default:
    {
      PARec = N2kgfpec_InvalidRequestOrCommandParameterField;
    }
    }

    AddAcknowledgeParameter(N2kRMsg, i, PARec);
  }

  pNMEA2000->SendMsg(N2kRMsg, iDev);

  if (PARec == N2kgfpec_Acknowledge)
  {
     Serial.print("Setting true heading to ");
      Serial.println(trueHeading);
      pypilot->setCommandHeadingTrue(trueHeading, tDataOrigin::kNMEA2000);


      Serial.print("Setting magnetic heading to ");
      Serial.println(magneticHeading);
      pypilot->setCommandHeadingMagnetic(magneticHeading, tDataOrigin::kNMEA2000);
  }

  return true;
}






/*******************************************************************/
//
// 65345 is wind datum
//  must do request and command (for setting it)
//
//
//      With a request will send the info
//      With command will set the mode
//
#define N2kPGN65345_ManufacturerCode_field 1
#define N2kPGN65345_Reserved_field 2
#define N2kPGN65345_IndustryCode_field 3
#define N2kPGN65345_WindDatum_field 4
#define N2kPGN65345_RollingAvgWindAngle_field 5
#define N2kPGN65345_Reserved_field_1 6

bool tN2kGroupFunctionHandlerForPGN65345::HandleRequest(const tN2kMsg &N2kMsg,
                                                        uint32_t TransmissionInterval,
                                                        uint16_t TransmissionIntervalOffset,
                                                        uint8_t NumberOfParameterPairs,
                                                        int iDev)
{

  tN2kGroupFunctionTransmissionOrPriorityErrorCode pec = GetRequestGroupFunctionTransmissionOrPriorityErrorCode(TransmissionInterval, TransmissionIntervalOffset);
  bool MatchFilter = true;
  tN2kMsg N2kRMsg;

  Serial.println("Received Locked Heading Request");
  // Start to build response
  SetStartAcknowledge(N2kRMsg, N2kMsg.Source, PGN,
                      N2kgfPGNec_Acknowledge, // Always acknowledge for mandatory PGN
                      pec,
                      NumberOfParameterPairs);
  N2kRMsg.Destination = N2kMsg.Source;

  if (NumberOfParameterPairs > 0)
  { // We need to filter according to fields
    int i;
    int Index;
    uint8_t field;
    tNMEA2000::tDeviceInformation DI = pNMEA2000->GetDeviceInformation(iDev);
    tN2kGroupFunctionParameterErrorCode FieldErrorCode;
    bool FoundInvalidField = false;

    Serial.println(NumberOfParameterPairs);
    StartParseRequestPairParameters(N2kMsg, Index);
    // Next read new field values. Note that if message is not broadcast, we need to parse all fields always.
    for (i = 0; i < NumberOfParameterPairs && (MatchFilter || !tNMEA2000::IsBroadcast(N2kMsg.Destination)); i++)
    {
      if (!FoundInvalidField)
      {
        field = N2kMsg.GetByte(Index);
        switch (field)
        {
        case N2kPGN65345_ManufacturerCode_field:
        {
          MatchRequestField(N2kMsg.Get2ByteUInt(Index), (uint16_t)MANUFACTURER_RAYMARINE, (uint16_t)0x3fff, MatchFilter, FieldErrorCode);
          break;
        }

        case N2kPGN65345_Reserved_field:
        {
          break;
        }

        case N2kPGN65345_IndustryCode_field:
        {
          MatchRequestField(N2kMsg.GetByte(Index), (uint8_t)INDUSTRY_MARINE, (uint8_t)0x07, MatchFilter, FieldErrorCode);
          break;
        }

        case N2kPGN65345_WindDatum_field:
        {
          N2kMsg.Get2ByteDouble(0.0001, Index);
          break;
        }

        case N2kPGN65345_RollingAvgWindAngle_field:
        {
          N2kMsg.Get2ByteDouble(0.0001, Index);
          break;
        }
        case N2kPGN65360_TargetHeadingMagnetic_field:
        {
          N2kMsg.Get2ByteDouble(0.0001, Index);
          break;
        }

        case N2kPGN65379_Reserved_field_1:
        {
          N2kMsg.GetByte(Index);
          break;
        }

        default:
        {
          Serial.print("Field outside specs ");
          Serial.println(field);
          FieldErrorCode = N2kgfpec_InvalidRequestOrCommandParameterField;
          MatchFilter = false;
          FoundInvalidField = true;
        }
        }
      }
      else
      {
        // If there is any invalid field, we can not parse others, since we do not
        // know right data length. So for rest of the fields we can only send response below.
        FieldErrorCode = N2kgfpec_TemporarilyUnableToComply;
      }
      AddAcknowledgeParameter(N2kRMsg, i, FieldErrorCode);
    }
  }

  bool RequestOK = (MatchFilter && pec == N2kgfTPec_Acknowledge);

  // Send Acknowledge, if request was not broadcast and it did not match
  if (!RequestOK)
  {
    Serial.println("Sending error for Locked Heading Request");
    if (!tNMEA2000::IsBroadcast(N2kMsg.Destination))
      pNMEA2000->SendMsg(N2kRMsg, iDev);
  }
  else
  {
    // Send delayed - there was problems with test tool with too fast response.
    // Must Send PGN

    pypilot->sendWindDatum(pNMEA2000); // perhaps use iDev as second parameter???
  }
  return true;
}

#endif
