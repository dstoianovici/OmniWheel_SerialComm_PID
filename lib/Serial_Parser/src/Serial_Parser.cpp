#include <Serial_Parser.h>


///////////Constructor////////////
Serial_Parser::Serial_Parser(char delimiter){
  _delimiter = delimiter;
  Serial.println("parser created");
}

//////////Methods//////////////////
Serial_Parser::GetParams(int* nums){
  String py_data = "/n";
  String sub_string = "/n";
  int index = 0; //Index of the comma
  int i = 0; // itterator for nums array
  int flag = 1; //Flag for end of string object processing

  if (Serial.available() > 0){
    py_data = Serial.readString();

    while(flag == 1){
      index = py_data.indexOf(_delimiter);

      if(index == -1){//No more delimiter
        nums[i] = py_data.toInt();
        i++;
        flag = 0;
      }

      else{
        sub_string = py_data.substring(0,index);
        py_data.remove(0,index+1);
        nums[i] = sub_string.toInt();
        i++;
        flag = 1;
      }
    }
  }
  else{
    return 0; //return 0 if
  }

  return i;
  }
