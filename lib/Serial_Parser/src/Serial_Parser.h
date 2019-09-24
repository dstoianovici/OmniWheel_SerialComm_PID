#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>

class Serial_Parser
{
  public:
    Serial_Parser(char delimiter);
    // int GetNumParams();
    int GetParams(int* nums);


  private:
    // long _baud_rate;
    char _delimiter;

};

#endif
