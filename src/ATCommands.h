#ifndef _AT_COMMANDS_H
#define _AT_COMMANDS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <map>
#include "../../../include/CommandTypes.h"


#define MAX_SEND_ATTEMPTS 5
#define AT_TAG "ATCommands"

struct CommandInfo {
    CommandEnum cmd;
    std::function<void(const String&)> callback;
};

struct CommandData {
  String command;
  String parameters;
};

class ATCommands
{
public:
    ATCommands();
    bool begin(HardwareSerial &SerialPort);
    void setCommandMap(const std::map<String, CommandInfo>& cmdMap);
    void setEventMap(const std::map<EventEnum, String>& evtMap);

    //Command Functions
    void Command_Read();
    void processInputCommand(const String& input);
    CommandData parseCommand(const String &command);
    void Send_Reply(bool ok);
    void AckReceived();

    //Event Functions 
    int CreateEvent(EventEnum event, const String &params);
    int SendEvent(const String &eventString);

    
private:

    bool _initialized;
    bool _ok_flag;
    bool _cmds_set;
    bool _evt_set;
    HardwareSerial* _serialPort;
    std::map<String, CommandInfo> _commandMap;
    std::map<EventEnum, String> _eventMap;

};

#endif