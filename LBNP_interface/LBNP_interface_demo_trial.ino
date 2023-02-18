//2022-07-14
#include "LBNP_interface_demo_trial.h"

#include <Wire.h>
//#include "Adafruit_MPRLS.h"
#include <vector>
#include <MsTimer2.h> //introduce timer setting lib on Arduino

LBNPInterface *lbnpInterface;

//int serialLen;

//public variables definition
volatile bool enable;
volatile double reference=0;
volatile double Kp;
volatile double Ki;
volatile double Kd;
volatile double Output=0;
int counter=0;
int j=0;
int x=0;
//only for debug use
volatile int serialLen_debug=0;
volatile std::vector<uint8_t> inputData_debug; 
volatile int len_debug=0;
volatile std::vector<uint8_t> toSend_debug;

void setup() {
 
  Serial.begin(9600);
  Serial.println("I am ready");
  Serial.println("i am running2");
  //MsTimer2::set(6000,Send_to_python); //set timer interupt interval as 1ms;
  MsTimer2::set(4000,report_data); //set timer interupt interval as 1ms;
  // MsTimer2::set(6000,send_trial); //test trial just for simple data send, no more need in main function
  MsTimer2::start();
  for(int i=0; i<100;i++){
      inputData_debug.push_back(0);
  }

/*
     pinMode(13, LED_BUILTIN);
      delay(3000);
      digitalWrite(13, HIGH);
      delay(3000);
      digitalWrite(13, LOW);
*/

}


//test trial just for simple data send, no more need in main function
void send_trial() {
   lbnpInterface->send_trial();
}


void report_data(){
  //report the basic serial received data
 Serial.println("the length of the basic serial received data inputdata_debug from python is:");
 Serial.println(serialLen_debug);
 Serial.println("the basic serial received data inputData_debug from python is"); 
  for(int i=0;i<serialLen_debug;i++){
  Serial.println(inputData_debug[i]); 
}
//report updated data
        Serial.println("the data updated from python is:"); 
        Serial.println("Enable is:");
        Serial.println(enable);
        Serial.println("Reference is:");
        Serial.println(reference);
        Serial.println("Kp is:");
        Serial.println(Kp);
        Serial.println("Ki is:");
        Serial.println(Ki);
        Serial.println("Kd is:");
        Serial.println(Kd);
        Serial.println();
        Serial.println();

 // Send_to_python_test();
 
// send received data back to python
Send_to_python();

//report the basic serial send data
 Serial.println("the length of the basic serial send data toSend_debug to python is:");
 Serial.println(len_debug);
 Serial.println("the basic serial send data toSend_debug to python is");
  for(int i=0;i<len_debug;i++){
  Serial.println(toSend_debug[i]); 
}
        Serial.println();
        Serial.println();
    /*Serial.println("remaining1 is");
    Serial.println(toSend_debug[len_debug]);
    Serial.println("remaining2 is");
    Serial.println(toSend_debug[len_debug+1]);*/
}

//only used to send fixed reference and output to test sending_data_to_python
void Send_to_python_test(){
  reference=3.6;
  Output=5.8;
  lbnpInterface->Senddata(reference,Output);
}

void Send_to_python(){
  lbnpInterface->Senddata(reference,Output);  
}

void loop() {
   //Data send module
   //reference=3.8;
   //Output=3.6;
   //delay(8000);
   //lbnpInterface->Senddata(reference,Output);
   //lbnpInterface->test(8);
   //lbnpInterface->test1();


   //Data receive module
    lbnpInterface->upload_data();
    //lbnpInterface->upload_debug();
}

//above is reserved by 2022-06-25 for the port of python send order
