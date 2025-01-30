#include "ATCommands.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <map>


ATCommands::ATCommands()
{
  _initialized = false;
  _ok_flag = true;
}

bool ATCommands::begin(HardwareSerial &SerialPort)
{
  _serialPort = &SerialPort;
  _initialized = true;
  return true;
}

void ATCommands::setCommandMap(const std::map<String, CommandInfo> &cmdMap)
{
  if (!_initialized)
  {
    Serial.println("must call begin() first");
  }
  _commandMap = cmdMap;
  _cmds_set = true;
}

void ATCommands::setEventMap(const std::map<EventEnum, String> &evtMap)
{
  _eventMap = evtMap;
  _evt_set = true;
}

/********************************************Command Functions**************************************/

const size_t maxCommandLength = 256; // Define a maximum command length
char inputBuffer[maxCommandLength];
size_t inputPos = 0;

void ATCommands::Command_Read()
{
  static int state = 0;
  while (_serialPort->available() > 0)
  {
    char inChar = _serialPort->read();

    if (inChar == '/0')
    {
      continue;
    }

    if (state == 0)
    {
      if (inChar == 'A')
      {
        state = 1;
      }
    }

    if (state == 1)
    {
      // Check for buffer overflow
      if (inputPos < maxCommandLength - 1)
      {
        inputBuffer[inputPos++] = inChar;

        // Check for end of command
        if (inChar == '\n' | '\r')
        {
          inputBuffer[inputPos] = '\0'; // Null-terminate the string
          String inputCommand = String(inputBuffer);
          state = 0;

          processInputCommand(inputCommand);

          // Reset buffer for the next command
          inputPos = 0;
        }
      }
      else
      {
        // Handle buffer overflow
        Serial.println("Error: Command too long.");
        inputPos = 0; // Reset buffer
      }
    }
  }
}

CommandData ATCommands::parseCommand(const String &input)
{
  CommandData result;
  String trimmedInput = input;
  trimmedInput.trim(); // Remove leading/trailing whitespace and newline

  int equalPos = trimmedInput.indexOf('=');

  if (equalPos == -1)
  {
    result.command = trimmedInput;
    result.parameters = "";
  }
  else
  {
    result.command = trimmedInput.substring(0, equalPos);
    result.command.trim();

    result.parameters = trimmedInput.substring(equalPos + 1);
    result.parameters.trim();
  }

  return result;
}

void ATCommands::processInputCommand(const String &input)
{
  // Parse the command and parameters
  CommandData cmdData = parseCommand(input);

  // Look up the command in the map
  auto it = _commandMap.find(cmdData.command);
  if (it != _commandMap.end())
  {
    // Command found, invoke the callback with parameters

    it->second.callback(cmdData.parameters);
  }
  else
  {
    // Command not found, handle the error
    Send_Reply(false);
    Serial.println("Error: Command not recognized - " + cmdData.command);
  }
}

void ATCommands::Send_Reply(bool ok)
{
  ESP_LOGD("SendEvent", "Sent reply: %s", ok ? "OK" : "ERROR");
  if (ok)
    _serialPort->print("OK\n");
  else
    _serialPort->print("ERROR\n");
}

void ATCommands::AckReceived()
{
  _ok_flag = true;
  ESP_LOGD("SendEvent", "AckReceived");
}

/********************************************Event Functions**************************************/

int ATCommands::CreateEvent(EventEnum event, const String &params)
{
  String eventMessage = "";
  auto it = _eventMap.find(event);

  if (it != _eventMap.end())
  {
    eventMessage = it->second;
    if (params.length() > 0)
    {
      eventMessage += "=" + params;
    }
    eventMessage += "\n";
    return SendEvent(eventMessage);
  }
  return -1;
}

int ATCommands::SendEvent(const String &eventString)
{
  int attempts = 0;
  String localEventString = "";
  localEventString = eventString;

  while (!_ok_flag && attempts < MAX_SEND_ATTEMPTS)
  {
    attempts++;
    delay(100);
  }

  if (attempts >= MAX_SEND_ATTEMPTS)
  {
    _ok_flag = true;
    return -1;
  }

  _serialPort->print(localEventString);
  _ok_flag = false;
  return 0;
}
