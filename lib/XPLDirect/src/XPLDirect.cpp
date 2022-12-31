/*
  XPLDirect.cpp
  Created by Michael Gerlicher, September 2020.
*/

#include <arduino.h>
#include "XPLDirect.h"

XPLDirect::XPLDirect(Stream *device)
{
  streamPtr = device;
  streamPtr->setTimeout(XPLDIRECT_RX_TIMEOUT);
}

void XPLDirect::begin(char *devicename)
{
  _deviceName = devicename;
  _connectionStatus = 0;
  _dataRefsCount = 0;
  _commandsCount = 0;
  _allDataRefsRegistered = 0;
  _receiveBuffer[0] = 0;
}

int XPLDirect::xloop(void)
{
  int i;
  int pinStatus;

  _processSerial();

  if (!_allDataRefsRegistered)
  {
    return _connectionStatus;
  }

  // process datarefs
  for (i = 0; i < _dataRefsCount; i++)
  {
    if (_dataRefs[i]->dataRefHandle >= 0 && (_dataRefs[i]->dataRefRWType == XPL_WRITE || _dataRefs[i]->dataRefRWType == XPL_READWRITE))
    {
      switch (_dataRefs[i]->dataRefVARType)
      {
      case XPL_DATATYPE_INT:
      {
        // _dataRefs[i]->latestValue *= _dataRefs[i]->divider

        if (((*(long int *)_dataRefs[i]->latestValue != _dataRefs[i]->lastSentIntValue) && (millis() - _dataRefs[i]->lastUpdateTime > _dataRefs[i]->updateRate)) || _dataRefs[i]->forceUpdate)
        {

          _sendPacketInt(XPLCMD_DATAREFUPDATE, _dataRefs[i]->dataRefHandle, *(long int *)_dataRefs[i]->latestValue);

          _dataRefs[i]->lastSentIntValue = *(long int *)_dataRefs[i]->latestValue;
          _dataRefs[i]->lastUpdateTime = millis();
          _dataRefs[i]->forceUpdate = 0;
        }

        break;
      }

      case XPL_DATATYPE_FLOAT:
      {
        if (_dataRefs[i]->divider)
        {
          *(float *)_dataRefs[i]->latestValue = ((int)(*(float *)_dataRefs[i]->latestValue / _dataRefs[i]->divider) * _dataRefs[i]->divider);
        }

        if ((*(float *)_dataRefs[i]->latestValue != _dataRefs[i]->lastSentFloatValue && (millis() - _dataRefs[i]->lastUpdateTime) > _dataRefs[i]->updateRate) || _dataRefs[i]->forceUpdate)
        {
          _sendPacketFloat(XPLCMD_DATAREFUPDATE, _dataRefs[i]->dataRefHandle, *(float *)_dataRefs[i]->latestValue);
          _dataRefs[i]->lastSentFloatValue = *(float *)_dataRefs[i]->latestValue;
          _dataRefs[i]->lastUpdateTime = millis();
          _dataRefs[i]->forceUpdate = 0;
        }

        break;
      }
      }
    }
  }

  // now process commands
  for (i = 0; i < _commandsCount; i++)
  {
    if (_commands[i]->commandHandle >= 0 && millis() - _commands[i]->lastUpdateTime > XPL_COMMAND_MAX_UPDATE_RATE_MILLIS)
    {
      if (_commands[i]->pin >= 0) // process if assigned pin
      {
        pinStatus = digitalRead(_commands[i]->pin);
        if (pinStatus != _commands[i]->lastSentValue)
        {
          if (pinStatus == true)
          {
            _sendPacketVoid(XPLCMD_COMMANDEND, _commands[i]->commandHandle);
            _commands[i]->lastUpdateTime = millis(); // only for initial press to debounce
          }
          else
          {
            _sendPacketVoid(XPLCMD_COMMANDSTART, _commands[i]->commandHandle);
          }

          _commands[i]->lastSentValue = pinStatus;
        }
      }
      else // process for normal variables
      {

        if (_commands[i]->latestValue != NULL && *(int *)_commands[i]->latestValue != _commands[i]->lastSentValue)
        {

          if (*_commands[i]->latestValue > 0)
            _sendPacketVoid(XPLCMD_COMMANDEND, _commands[i]->commandHandle);
          else
            _sendPacketVoid(XPLCMD_COMMANDSTART, _commands[i]->commandHandle);

          _commands[i]->lastSentValue = *(int *)_commands[i]->latestValue;
          _commands[i]->lastUpdateTime = millis();
        }
      }
    }
  }

  return _connectionStatus;
}

int XPLDirect::commandTrigger(int commandHandle)
{
  return commandTrigger(commandHandle, 1);
}

int XPLDirect::commandTrigger(int commandHandle, int triggerCount)
{
  if (!_commands[commandHandle])
    return -1; // inactive command

  _sendPacketInt(XPLCMD_COMMANDTRIGGER, _commands[commandHandle]->commandHandle, (long int)triggerCount);
  return 0;
}

int XPLDirect::commandStart(int commandHandle)
{
  if (!_commands[commandHandle])
    return -1;
  _sendPacketVoid(XPLCMD_COMMANDSTART, _commands[commandHandle]->commandHandle);
  return 0;
}

int XPLDirect::commandEnd(int commandHandle)
{
  if (!_commands[commandHandle])
    return -1;
  _sendPacketVoid(XPLCMD_COMMANDEND, _commands[commandHandle]->commandHandle);
  return 0;
}

int XPLDirect::connectionStatus()
{
  return _connectionStatus;
}

int XPLDirect::sendDebugMessage(char *msg)
{
  _sendPacketString(XPLCMD_PRINTDEBUG, msg);
  return 1;
}

int XPLDirect::hasUpdated(int handle)
{
  if (_dataRefs[handle]->updatedFlag)
  {
    _dataRefs[handle]->updatedFlag = false;
    return true;
  }
  return false;
}

int XPLDirect::datarefsUpdated()
{
  if (_datarefsUpdatedFlag)
  {
    _datarefsUpdatedFlag = false;
    return true;
  }
  return false;
}

void XPLDirect::_sendname()
{
  if (_deviceName != NULL)
  {
    _sendPacketString(XPLRESPONSE_NAME, _deviceName);
  }
}

void XPLDirect::_sendVersion()
{
  if (_deviceName != NULL)
  {
    _sendPacketInt(XPLRESPONSE_VERSION, XPLDIRECT_ID, XPLDIRECT_VERSION);
  }
}

void XPLDirect::sendResetRequest()
{

  if (_deviceName != NULL)
  {
    _sendPacketVoid(XPLCMD_RESET, 0);
  }
}

void XPLDirect::_processSerial()
{
  while (streamPtr->available() && _receiveBuffer[0] != XPLDIRECT_PACKETHEADER)
    _receiveBuffer[0] = (char)streamPtr->read();

  if (_receiveBuffer[0] != XPLDIRECT_PACKETHEADER)
    return;

  _receiveBufferBytesReceived = streamPtr->readBytesUntil(XPLDIRECT_PACKETTRAILER, (char *)&_receiveBuffer[1], XPLMAX_PACKETSIZE - 1);

  if (_receiveBufferBytesReceived == 0)
  {
    _receiveBuffer[0] = 0;
    return;
  }

  _receiveBuffer[++_receiveBufferBytesReceived] = XPLDIRECT_PACKETTRAILER;
  _receiveBuffer[++_receiveBufferBytesReceived] = 0; // old habits die hard.

  _processPacket();

  _receiveBuffer[0] = 0;
}

void XPLDirect::_processPacket()
{
  int i;

  switch (_receiveBuffer[1])
  {
  case XPLCMD_RESET:
  {
    _connectionStatus = false;
    break;
  }

  case XPLCMD_SENDNAME:
  {
    _sendname();
    _connectionStatus = true; // not considered active till you know my name

    for (i = 0; i < _dataRefsCount; i++) // also, if name was requested reset active datarefs and commands
    {
      _dataRefs[i]->dataRefHandle = -1; //  invalid again until assigned by Xplane
    }

    for (i = 0; i < _commandsCount; i++)
    {
      _commands[i]->commandHandle = -1;
    }

    break;
  }

  case XPLCMD_SENDVERSION:
  {
    _sendVersion();
    break;
  }

  case XPLRESPONSE_DATAREF:
  {

    for (int i = 0; i < _dataRefsCount; i++)
    {
      if (_dataRefs[i]->dataRefName != NULL)
      {
        if (strncmp((char *)&_receiveBuffer[5], _dataRefs[i]->dataRefName, strlen(_dataRefs[i]->dataRefName)) == 0 && _dataRefs[i]->dataRefHandle == -1)
        {
          _dataRefs[i]->dataRefHandle = _getHandleFromFrame(); // parse the refhandle
          _dataRefs[i]->updatedFlag = true;
          i = _dataRefsCount; // end checking
        }
      }
#ifdef XPL_USE_PROGMEM
      else
      {
        if (strncmp_PF((char *)&_receiveBuffer[5], _dataRefs[i]->FdataRefName, strlen_PF(_dataRefs[i]->FdataRefName)) == 0 && _dataRefs[i]->dataRefHandle == -1)
        {
          _dataRefs[i]->dataRefHandle = _getHandleFromFrame(); // parse the refhandle
          _dataRefs[i]->updatedFlag = true;
          i = _dataRefsCount; // end checking
        }
      }
#endif
    }

    break;
  }

  case XPLRESPONSE_COMMAND:
  {

    for (int i = 0; i < _commandsCount; i++)
    {
      if (_commands[i]->commandName != NULL)
      {
        if (strncmp((char *)&_receiveBuffer[5], _commands[i]->commandName, strlen(_commands[i]->commandName)) == 0 && _commands[i]->commandHandle == -1)
        {
          _commands[i]->commandHandle = _getHandleFromFrame(); // parse the refhandle
          i = _commandsCount;                                  // end checking
        }
      }
#ifdef XPL_USE_PROGMEM
      else
      {
        if (strncmp_PF((char *)&_receiveBuffer[5], _commands[i]->FcommandName, strlen_PF(_commands[i]->FcommandName)) == 0 && _commands[i]->commandHandle == -1)
        {
          _commands[i]->commandHandle = _getHandleFromFrame(); // parse the refhandle
          i = _commandsCount;                                  // end checking
        }
      }
#endif
    }

    break;
  }

  case XPLCMD_SENDREQUEST:
  {
    int packetSent = 0;
    int i = 0;

    while (!packetSent && i < _dataRefsCount && i < XPLDIRECT_MAXDATAREFS_ARDUINO) // send dataref registrations first
    {
      if (_dataRefs[i]->dataRefHandle == -1)
      { // some boards cant do sprintf with floats so this is a workaround

        if (_dataRefs[i]->dataRefName != NULL)
          sprintf(_sendBuffer, "%c%c%1.1i%2.2i%05i.%02i%s%c", XPLDIRECT_PACKETHEADER, XPLREQUEST_REGISTERDATAREF, _dataRefs[i]->dataRefRWType, _dataRefs[i]->arrayIndex,
                  (int)_dataRefs[i]->divider, (int)(_dataRefs[i]->divider * 100) % 100, _dataRefs[i]->dataRefName, XPLDIRECT_PACKETTRAILER);
#ifdef XPL_USE_PROGMEM
        else
          sprintf(_sendBuffer, "%c%c%1.1i%2.2i%05i.%02i%S%c", XPLDIRECT_PACKETHEADER, XPLREQUEST_REGISTERDATAREF, _dataRefs[i]->dataRefRWType, _dataRefs[i]->arrayIndex,
                  (int)_dataRefs[i]->divider, (int)(_dataRefs[i]->divider * 100) % 100, _dataRefs[i]->FdataRefName, XPLDIRECT_PACKETTRAILER);
#endif

        _transmitPacket();
        packetSent = 1;
      }
      i++;
    }

    i = 0;

    while (!packetSent && i < _commandsCount && i < XPLDIRECT_MAXCOMMANDS_ARDUINO) // now send command registrations
    {
      if (_commands[i]->commandHandle == -1)
      {
        if (_commands[i]->commandName != NULL)
          sprintf(_sendBuffer, "%c%c%s%c", XPLDIRECT_PACKETHEADER, XPLREQUEST_REGISTERCOMMAND, _commands[i]->commandName, XPLDIRECT_PACKETTRAILER);
#ifdef XPL_USE_PROGMEM
        else
          sprintf(_sendBuffer, "%c%c%S%c", XPLDIRECT_PACKETHEADER, XPLREQUEST_REGISTERCOMMAND, _commands[i]->FcommandName, XPLDIRECT_PACKETTRAILER);
#endif

        _transmitPacket();
        packetSent = 1;
      }
      i++;
    }

    if (!packetSent)
    {
      _allDataRefsRegistered = true;
      sprintf(_sendBuffer, "%c%c%c", XPLDIRECT_PACKETHEADER, XPLREQUEST_NOREQUESTS, XPLDIRECT_PACKETTRAILER);
      _transmitPacket();
    }

    break;
  }

  case XPLCMD_DATAREFUPDATE:
  {
    int refhandle = _getHandleFromFrame();

    for (int i = 0; i < _dataRefsCount; i++)
    {
      if (_dataRefs[i]->dataRefHandle == refhandle && (_dataRefs[i]->dataRefRWType == XPL_READ || _dataRefs[i]->dataRefRWType == XPL_READWRITE))
      {
        if (_dataRefs[i]->dataRefVARType == XPL_DATATYPE_INT)
        {

          _getPayloadFromFrame((long int *)_dataRefs[i]->latestValue);
          _dataRefs[i]->lastSentIntValue = *(long int *)_dataRefs[i]->latestValue;
          _dataRefs[i]->updatedFlag = true;
          _datarefsUpdatedFlag = true;
        }

        if (_dataRefs[i]->dataRefVARType == XPL_DATATYPE_FLOAT)
        {

          _getPayloadFromFrame((float *)_dataRefs[i]->latestValue);
          _dataRefs[i]->lastSentFloatValue = *(float *)_dataRefs[i]->latestValue;
          _dataRefs[i]->updatedFlag = true;
          _datarefsUpdatedFlag = true;
        }

        if (_dataRefs[i]->dataRefVARType == XPL_DATATYPE_STRING)
        {

          _getPayloadFromFrame((char *)_dataRefs[i]->latestValue);
          _dataRefs[i]->updatedFlag = true;
          _datarefsUpdatedFlag = true;
        }

        i = _dataRefsCount; // skip the rest
      }
    }

    break;
  }

  case XPLREQUEST_REFRESH:
  {

    for (int i = 0; i < _dataRefsCount; i++)
    {
      if (_dataRefs[i]->dataRefRWType == XPL_WRITE || _dataRefs[i]->dataRefRWType == XPL_READWRITE)
      {
        _dataRefs[i]->forceUpdate = 1; // bypass noise and timing filters
      }
    }

    break;
  }

  case XPLCMD_DUMPREGISTRATIONS:
  {
    dumpRegistrations();
    break;
  }

  default:
  {
    // streamPtr->println("invalid command received");
    break;
  }
  }
}

void XPLDirect::_sendPacketInt(int command, int handle, long int value) // for ints
{
  if (handle < 0)
    return;

  sprintf(_sendBuffer, "%c%c%3.3i%ld%c", XPLDIRECT_PACKETHEADER, command, handle, value, XPLDIRECT_PACKETTRAILER);
  _transmitPacket();
}

void XPLDirect::_sendPacketFloat(int command, int handle, float value) // for floats
{
  if (handle < 0)
    return;
  // some boards cant do sprintf with floats so this is a workaround.  What a pain.  Implementing 2 decimals of precision for now.
  sprintf(_sendBuffer, "%c%c%3.3i%i.%i%c", // change to reduce data flow 01/26/2021 MG
          XPLDIRECT_PACKETHEADER,
          command,
          handle,
          abs((int)value),
          abs((int)(value * 100) % 100),
          XPLDIRECT_PACKETTRAILER);

  if (value < 0)
    _sendBuffer[5] = '-';

  _transmitPacket();
}

void XPLDirect::_sendPacketVoid(int command, int handle) // just a command with a handle
{
  if (handle < 0)
    return;

  sprintf(_sendBuffer, "%c%c%3.3i%c", XPLDIRECT_PACKETHEADER, command, handle, XPLDIRECT_PACKETTRAILER);

  _transmitPacket();
}

void XPLDirect::_sendPacketString(int command, char *str) // for a string
{

  sprintf(_sendBuffer, "%c%c%s%c", XPLDIRECT_PACKETHEADER, command, str, XPLDIRECT_PACKETTRAILER);

  _transmitPacket();
}

void XPLDirect::_transmitPacket(void)
{
  streamPtr->write(_sendBuffer);
  if (strlen(_sendBuffer) == 64)
    streamPtr->print(" "); // apparantly a bug on some boards when we transmit exactly 64 bytes
}

int XPLDirect::_getHandleFromFrame() // Assuming receive buffer is holding a good frame
{
  char holdChar;
  int handleRet;

  holdChar = _receiveBuffer[5];
  _receiveBuffer[5] = 0;
  handleRet = atoi((char *)&_receiveBuffer[2]);

  _receiveBuffer[5] = holdChar;
  return handleRet;
}

int XPLDirect::_getPayloadFromFrame(long int *value) // Assuming receive buffer is holding a good frame
{
  char holdChar;

  holdChar = _receiveBuffer[15];
  _receiveBuffer[15] = 0;
  *value = atol((char *)&_receiveBuffer[5]);

  _receiveBuffer[15] = holdChar;

  return 0;
}

int XPLDirect::_getPayloadFromFrame(float *value) // Assuming receive buffer is holding a good frame
{
  char holdChar;

  holdChar = _receiveBuffer[15];
  _receiveBuffer[15] = 0;
  *value = atof((char *)&_receiveBuffer[5]);

  _receiveBuffer[15] = holdChar;
  return 0;
}

int XPLDirect::_getPayloadFromFrame(char *value) // Assuming receive buffer is holding a good frame
{

  memcpy(value, (char *)&_receiveBuffer[5], _receiveBufferBytesReceived - 6);
  value[_receiveBufferBytesReceived - 6] = 0; // erase the packet trailer
  for (int i = 0; i < _receiveBufferBytesReceived - 6; i++)
    if (value[i] == 7)
      value[i] = XPLDIRECT_PACKETTRAILER; //  How I deal with the possibility of the packet trailer being within a string
  return 0;
}

int XPLDirect::allDataRefsRegistered()
{
  return _allDataRefsRegistered;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerDataRef(const __FlashStringHelper *datarefName, int rwmode, unsigned int rate, float divider, long int *value)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1; // Error
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = datarefName; // added for F() macro
  _dataRefs[_dataRefsCount]->dataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_INT;
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentIntValue = 0;
  _dataRefs[_dataRefsCount]->arrayIndex = 0;     // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}
#endif

int XPLDirect::registerDataRef(const char *datarefName, int rwmode, unsigned int rate, float divider, long int *value)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_INT;
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentIntValue = 0;
  _dataRefs[_dataRefsCount]->arrayIndex = 0;     // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerDataRef(const __FlashStringHelper *datarefName, int rwmode, unsigned int rate, float divider, long int *value, int index)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->dataRefName = NULL;
  _dataRefs[_dataRefsCount]->FdataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_INT; // arrays are dealt with on the XPlane plugin side
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentIntValue = 0;
  _dataRefs[_dataRefsCount]->arrayIndex = index; // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}
#endif

int XPLDirect::registerDataRef(const char *datarefName, int rwmode, unsigned int rate, float divider, long int *value, int index)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_INT; // arrays are dealt with on the XPlane plugin side
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentIntValue = 0;
  _dataRefs[_dataRefsCount]->arrayIndex = index; // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerDataRef(const __FlashStringHelper *datarefName, int rwmode, unsigned int rate, float divider, float *value)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;

  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_FLOAT;
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentFloatValue = -1; // force update on first loop
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->arrayIndex = 0;     // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}
#endif

int XPLDirect::registerDataRef(const char *datarefName, int rwmode, unsigned int rate, float divider, float *value)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;

  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_FLOAT;
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentFloatValue = 0;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->arrayIndex = 0;     // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerDataRef(const __FlashStringHelper *datarefName, int rwmode, unsigned int rate, float divider, float *value, int index)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_FLOAT; // arrays are dealt with on the Xplane plugin side
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentFloatValue = 0;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->arrayIndex = index; // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}
#endif

int XPLDirect::registerDataRef(const char *datarefName, int rwmode, unsigned int rate, float divider, float *value, int index)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_FLOAT; // arrays are dealt with on the Xplane plugin side
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentFloatValue = 0;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->divider = divider;
  _dataRefs[_dataRefsCount]->arrayIndex = index; // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerDataRef(const __FlashStringHelper *datarefName, int rwmode, unsigned int rate, char *value)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1; // Error
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = datarefName; // added for F() macro
  _dataRefs[_dataRefsCount]->dataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->divider = 0;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_STRING;
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentIntValue = 0;
  _dataRefs[_dataRefsCount]->arrayIndex = 0;     // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}
#endif

int XPLDirect::registerDataRef(const char *datarefName, int rwmode, unsigned int rate, char *value)
{

  if (_dataRefsCount >= XPLDIRECT_MAXDATAREFS_ARDUINO)
    return -1;
  _dataRefs[_dataRefsCount] = new _dataRefStructure;

  _dataRefs[_dataRefsCount]->FdataRefName = NULL;
  _dataRefs[_dataRefsCount]->dataRefName = datarefName;
  _dataRefs[_dataRefsCount]->dataRefRWType = rwmode;
  _dataRefs[_dataRefsCount]->divider = 0;
  _dataRefs[_dataRefsCount]->updateRate = rate;
  _dataRefs[_dataRefsCount]->dataRefVARType = XPL_DATATYPE_STRING;
  _dataRefs[_dataRefsCount]->latestValue = (void *)value;
  _dataRefs[_dataRefsCount]->lastSentIntValue = 0;
  _dataRefs[_dataRefsCount]->arrayIndex = 0;     // not used unless we are referencing an array
  _dataRefs[_dataRefsCount]->dataRefHandle = -1; // invalid until assigned by xplane

  _dataRefsCount++;
  _allDataRefsRegistered = 0;

  return _dataRefsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerCommand(const __FlashStringHelper *commandName, int *value)
{

  if (_commandsCount >= XPLDIRECT_MAXCOMMANDS_ARDUINO)
    return -1;
  _commands[_commandsCount] = new _commandStructure;

  _commands[_commandsCount]->FcommandName = commandName;
  _commands[_commandsCount]->commandName = NULL;
  _commands[_commandsCount]->latestValue = value;
  _commands[_commandsCount]->lastSentValue = 1;  // assume not pressed for digital switches (0 = pressed)
  _commands[_commandsCount]->commandHandle = -1; // invalid until assigned by xplane
  _commands[_commandsCount]->pin = -1;           // not using pin with this overload

  _commandsCount++;
  _allDataRefsRegistered = 0; // share this flag with the datarefs, true when everything is registered with xplane.

  return _commandsCount - 1;
}

#endif

int XPLDirect::registerCommand(const char *commandName, int *value)
{

  if (_commandsCount >= XPLDIRECT_MAXCOMMANDS_ARDUINO)
    return -1;
  _commands[_commandsCount] = new _commandStructure;

  _commands[_commandsCount]->FcommandName = NULL;
  _commands[_commandsCount]->commandName = commandName;
  _commands[_commandsCount]->latestValue = value;
  _commands[_commandsCount]->lastSentValue = 1;  // assume not pressed for digital switches (0 = pressed)
  _commands[_commandsCount]->commandHandle = -1; // invalid until assigned by xplane
  _commands[_commandsCount]->pin = -1;           // not using pin with this overload

  _commandsCount++;
  _allDataRefsRegistered = 0; // share this flag with the datarefs, true when everything is registered with xplane.

  return _commandsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerCommand(const __FlashStringHelper *commandName, int pin) // associate a digital pin with an xplane command
{

  if (_commandsCount >= XPLDIRECT_MAXCOMMANDS_ARDUINO)
    return -1;
  _commands[_commandsCount] = new _commandStructure;

  _commands[_commandsCount]->FcommandName = commandName;
  _commands[_commandsCount]->commandName = NULL;
  _commands[_commandsCount]->latestValue = NULL; // default to "not pressed" for now
  _commands[_commandsCount]->lastSentValue = 1;  // assume not pressed for digital switches (0 = pressed)
  _commands[_commandsCount]->commandHandle = -1; // invalid until assigned by xplane
  _commands[_commandsCount]->pin = pin;          // pin to use

  pinMode(pin, INPUT_PULLUP); // set pin up for use

  _commandsCount++;
  _allDataRefsRegistered = 0; // share this flag with the datarefs, true when everything is registered with xplane.

  return _commandsCount - 1;
}
#endif

int XPLDirect::registerCommand(const char *commandName, int pin) // associate a digital pin with an xplane command
{

  if (_commandsCount >= XPLDIRECT_MAXCOMMANDS_ARDUINO)
    return -1;
  _commands[_commandsCount] = new _commandStructure;
  _commands[_commandsCount]->FcommandName = NULL;
  _commands[_commandsCount]->commandName = commandName;
  _commands[_commandsCount]->latestValue = NULL; // default to "not pressed" for now
  _commands[_commandsCount]->lastSentValue = 1;  // assume not pressed for digital switches (0 = pressed)
  _commands[_commandsCount]->commandHandle = -1; // invalid until assigned by xplane
  _commands[_commandsCount]->pin = pin;          // pin to use

  pinMode(pin, INPUT_PULLUP); // set pin up for use

  _commandsCount++;
  _allDataRefsRegistered = 0; // share this flag with the datarefs, true when everything is registered with xplane.

  return _commandsCount - 1;
}

#ifdef XPL_USE_PROGMEM
int XPLDirect::registerCommand(const __FlashStringHelper *commandName) // user will trigger commands with commandTrigger
{

  if (_commandsCount >= XPLDIRECT_MAXCOMMANDS_ARDUINO)
    return -1;
  _commands[_commandsCount] = new _commandStructure;

  _commands[_commandsCount]->FcommandName = commandName;
  _commands[_commandsCount]->commandName = NULL;
  _commands[_commandsCount]->latestValue = NULL; // default to "not pressed" for now
  _commands[_commandsCount]->lastSentValue = 1;  // assume not pressed for digital switches (0 = pressed)
  _commands[_commandsCount]->commandHandle = -1; // invalid until assigned by xplane
  _commands[_commandsCount]->pin = -1;           // pin to use

  _commandsCount++;
  _allDataRefsRegistered = 0; // share this flag with the datarefs, true when everything is registered with xplane.

  return _commandsCount - 1;
}
#endif

int XPLDirect::registerCommand(const char *commandName) // user will trigger commands with commandTrigger
{

  if (_commandsCount >= XPLDIRECT_MAXCOMMANDS_ARDUINO)
    return -1;
  _commands[_commandsCount] = new _commandStructure;
  _commands[_commandsCount]->FcommandName = NULL;
  _commands[_commandsCount]->commandName = commandName;
  _commands[_commandsCount]->latestValue = NULL; // default to "not pressed" for now
  _commands[_commandsCount]->lastSentValue = 1;  // assume not pressed for digital switches (0 = pressed)
  _commands[_commandsCount]->commandHandle = -1; // invalid until assigned by xplane
  _commands[_commandsCount]->pin = -1;           // pin to use

  _commandsCount++;
  _allDataRefsRegistered = 0; // share this flag with the datarefs, true when everything is registered with xplane.

  return _commandsCount - 1;
}

void XPLDirect::dumpRegistrations(void) // a debug function.  Disable for production
{
  // not enabled for production
  /*	int i;
    for (i = 0; i < _dataRefsCount; i++)
    {
      streamPtr->println();
      streamPtr->print("handle: ");		streamPtr->print(_dataRefs[i]->dataRefHandle);
      streamPtr->print(" RWtype: ");		streamPtr->print(_dataRefs[i]->dataRefRWType);
      streamPtr->print(" VARtype: ");		streamPtr->print(_dataRefs[i]->dataRefVARType);
      streamPtr->print(" divider: ");		streamPtr->print(_dataRefs[i]->divider);
      streamPtr->print(" updateRate: ");	streamPtr->print(_dataRefs[i]->updateRate);
      streamPtr->print(" dataRefName: "); streamPtr->print(_dataRefs[i]->dataRefName);
      //streamPtr->print(" latestValue: ");	streamPtr->print(_dataRefs[i]->latestValue)

    }

    for (i = 0; i < _commandsCount; i++)
    {
      streamPtr->println();
      streamPtr->print("handle: ");		streamPtr->print(_commands[i]->commandHandle);
      streamPtr->print(" name: ");		streamPtr->print(_commands[i]->commandName);
      streamPtr->print(" latestValue: ");	streamPtr->print(*_commands[i]->latestValue);
      streamPtr->print(" lastSent: ");	streamPtr->print(_commands[i]->lastSentValue);
      streamPtr->print(" pin: ");			streamPtr->print(_commands[i]->pin);

    }
    */
}