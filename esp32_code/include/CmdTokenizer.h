// #include <Arduino.h>
#include <datatypes.h>
#include <unordered_map>
/* Example
R:3 :0|CRC
W:3 :128|CRC
S:99:-500|CRC
*/

class LogReporter {
    public:
    LogReporter(LOGLEVEL logLevel, String message) : logLevel(logLevel), message(message) {}

    void report() {
        auto it = logCharMap.find(logLevel);
        Serial.printf("*%c:%s\n",it->second, message.c_str());
    }

    public:
    LOGLEVEL logLevel;
    String message;
    const std::unordered_map<LOGLEVEL, char> logCharMap = {
        {LOGLEVEL::DEBUG, 'D'},
        {LOGLEVEL::INFO, 'I'},
        {LOGLEVEL::WARNING, 'W'},
        {LOGLEVEL::ERROR, 'E'},
        {LOGLEVEL::CRITICAL, 'C'},
    };
};

class CmdTokenizer { 
    public:

    CmdTokenizer(String input) : input(input) {}

    CmdTokenizer& tokenize() {
        if (errMessage == "") {
        char actionChar = input[0];
        if (actionChar == 'R')
            action = ACTIONS::READPIN;
        else if (actionChar == 'W')
            action = ACTIONS::WRITEANALOG;
        else if (actionChar == 'S')
            action = ACTIONS::WRITESERVO;
        else if (actionChar == 'E')
            action = ACTIONS::STEP;
        else if (actionChar == 'L')
            action = ACTIONS::LED;
        else 
            errMessage = "Error deciphering desired action"; 
        try {
            // Serial.println(input.substring(2,4).c_str());
            std::string pinString = input.substring(2,4).c_str();
            pin = std::stoi(pinString);
            
            // Serial.println(input.substring(5,input.indexOf('|')).c_str());
            std::string valueString = input.substring(5,input.indexOf('|')).c_str();
            value = std::stoi(valueString);
        } catch (std::invalid_argument&) {
            errMessage = "Error during pin number or value to int convertion"; 
        }
        }
        return *this;
    }

    CmdTokenizer& validateChecksum(){
        const int CRCindex = input.indexOf('|');
        byte inputCRC = (byte) strtol(input.substring(CRCindex+1, input.length()).c_str(), NULL, 16);
        if (CRCindex != -1) {
        if (computeChecksum(CRCindex) != inputCRC) {
            errMessage = "CRC does not match!";
        }
        
        } else {
        errMessage = "Could not find '|'";
        }

        return *this;
    }

    bool isValid() {
        return (errMessage == "");
    }

    int pin;
    ACTIONS action;
    int value;

    // bool validMessage = true;
    String errMessage = "";
    String input = "";

    private:
    byte computeChecksum(int CRCindex) {
        byte cs = 0;
        for (int i = 0; i <= CRCindex; i++) {
            cs ^= input[i];
        }
        return cs;
    }
};