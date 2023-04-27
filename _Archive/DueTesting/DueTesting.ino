//extern "C"{
  // put any files wanting to import into workspace in here

  // this is the synatx for headers:
  //#include "testheader.h"

  // headers CANNOT be included twice. will break the compilation.
  // if the header is imported in the function, then cannot import 
  // it again separately in the .ino sketch.

  //this is the syntax for regular functions
  //void testfcn();
  
//}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //digitalWrite(LED_BUILTIN,LOW);

  // this calls a function or script. Have not tested this yet, 
  // don't know if this is how it works.
  // void factorial(); //funciton in header
  //void testfcn(); //separate script#
  }
  
void loop() {
  // put your main code here, to run repeatedly:
    //digitalWrite(LED_BUILTIN,LOW);
    Serial.println("Hello, World!");
    //delay(100);
    //digitalWrite(LED_BUILTIN,HIGH);
    delay(200);

}
