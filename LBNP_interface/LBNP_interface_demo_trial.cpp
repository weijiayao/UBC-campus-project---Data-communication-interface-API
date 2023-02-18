//2022-07-14
#include "LBNP_interface_demo_trial.h"
#include "LBNP_interface_demo_trial.h"
#include <string.h>
//public variables declaration, in attempt to use them from main file
extern bool enable;
extern double reference;
extern double Kp;
extern double Ki;
extern double Kd;
extern double Output;
//only for debug
extern int serialLen_debug;
extern std::vector<uint8_t> inputData_debug; 
extern int len_debug;
extern std::vector<uint8_t> toSend_debug;

typedef enum {
    status_header1,
    status_header2,
    status_enable,
    //status_command,
    status_data,
    status_checksum1,
    status_checksum2
} FsmStatus;

#define HEADER1     0xAB
#define HEADER2     0xCD
#define PIDENABLE   0xAA
#define PIDDISABLE  0xBB


//only used to observe original data received from serial.reand.
void LBNPInterface::upload_debug(){
  std::vector<uint8_t> inputData;       // use vector to read up all serial buffer
  std::vector<uint8_t> inputData1;       // inputData1 is introduced to filter out previous data from inputData
   //Data receive module
    if (Serial.available() > 0){ // update to read up all serial buffer to follow python logic
      //Serial.println("I get serial");
      while(Serial.available() > 0){
      //Serial.println(Serial.read());
      inputData.push_back(Serial.read());
      }
    int serialLen = inputData.size();

    for(int i=0;i<serialLen;i++){
      inputData1.push_back(inputData[i]);
    }
    
inputData_debug.clear();
for(int i=0;i<serialLen;i++){
      inputData_debug.push_back(inputData[i]);
    }
    serialLen_debug=serialLen;

    
        }
        
}

void LBNPInterface::upload_data(){
  std::vector<uint8_t> inputData;       // use vector to read up all serial buffer
//std::vector<uint8_t> *serialData;  // the pointer is cancelled to follow python logic
//std::vector<uint8_t> inputData1;       // inputData1 is introduced to filter out previous data from inputData
   //non-filter reading: there is no similar issue with python reading as there is no serial.println from python to Arduino
   //Data receive module
    if (Serial.available() > 0){ // update to read up all serial buffer to follow python logic
      //Serial.println("I get serial");
      while(Serial.available() > 0){
      //Serial.println(Serial.read());
      inputData.push_back(Serial.read());
      }
    int serialLen = inputData.size();
    //to show received data length and data
    inputData_debug.clear();

    //report received data length and data contend.
for(int i=0;i<serialLen;i++){
      inputData_debug.push_back(inputData[i]);
    }
    serialLen_debug=serialLen;
    receive(inputData, serialLen);
    

/* //it was used for filtering out the previous data, not is not used anymore
    for(int i=serialLen-37;i<serialLen;i++){
      inputData1.push_back(inputData[i]);
    }
    receive(inputData1, 37);
        }
        */
        
    /*  //check serial length
  for(int i =0; i<32; i++){
  if (Serial.available() > 0) {

      j=1;
      //Serial.println(Serial.read());
      inputData.push_back(Serial.read());
      }
    //inputData = Serial.read();
    //serialData = &inputData;
  }

  if(j==1){
    j=0;
    serialLen = inputData.size();
    Serial.println("the length is:");
    Serial.println(serialLen);
  }
*/

    /* //used to check original receive data from serial, in this test, it is found that the serial read data all carry the previous data, so inputData1 is introduced to filter out useless data 
   Serial.println("inputData is"); 
  for(int i=0;i<serialLen;i++){
  Serial.println(inputData[i]); 
}
    Serial.println("inputData1 is"); 
for(int i=0;i<inputData1.size();i++){
  Serial.println(inputData1[i]); 
}
    Serial.println("the length of inputdata is:");
    Serial.println(serialLen);
    Serial.println("the length of inputdata1 is:");
    Serial.println(inputData1.size());*/
}

}

void LBNPInterface::receive(std::vector<uint8_t> data, int len) {//update it into vector and use data rather than pointer to following python logic

    static FsmStatus s = status_header1; // state machine status
    static AppCommand command = nothing;
    static uint8_t receivedData[100] = { 0 };
    static int receivedDataLen = 0;
    static int dataLen = 0;
    /*static int checkLen = 0;
    checkLen = data.size();
    Serial.println("the checklength is:"); //cbeck length of data serial, it is expected to be header1*1, header2*1, enable*1, reference*8, PID*3*8, sum1*1, sum2*1.
    Serial.println(checkLen);
    Serial.println(len);*/
    
    for (int i = 0; i < len; i++) {

     uint8_t c = data[i];

        switch (s) {
        case status_header1:
            if (c == HEADER1) {
            digitalWrite(13, HIGH);
             delay(3000);
              digitalWrite(13, LOW);
                receivedDataLen = 0;
                receivedData[receivedDataLen++] = c;
                s = status_header2;
            }
            break;
        case status_header2:
            if (c == HEADER2) {
                receivedData[receivedDataLen++] = c;
                s = status_enable;
            }
            break;
        case status_enable:
                receivedData[receivedDataLen++] = c;
                s = status_data;
                dataLen = 4 * 8; // 4 double value: reference+PID
            break;

        case status_data:
            receivedData[receivedDataLen++] = c;
            dataLen--;
            if (dataLen == 0) s = status_checksum1;
            break;
        case status_checksum1:
            receivedData[receivedDataLen++] = c;
            s = status_checksum2;
            break;
        case status_checksum2:

            receivedData[receivedDataLen++] = c;

            // check the checksum:
            if (checkCheckSum(receivedData, receivedDataLen)) {
                decode(receivedData, receivedDataLen);
            }
            s = status_header1;
            break;
        }
    }

}

bool LBNPInterface::checkCheckSum(uint8_t* data, int len) {

    uint8_t sum1 = 0;
    uint8_t sum2 = 0;
    for (int i = 0; i < len - 2; i++) {
        sum1 += data[i];
        sum2 += sum1;
    }
    return (sum1 == data[len - 2]) && (sum2 == data[len - 1]);
}


void LBNPInterface::decode(uint8_t* data, int len) {
    //auto command = (AppCommand)data[2];
Serial.println("receivedata is"); 
for(int i=0;i<38;i++){
  Serial.println(data[i]); 
}
    if (len == 2 /*header*/ + 1 /*enable*/ + 1 * 8 /*1 doubles - reference*/ + 3 * 8 /*1 doubles - PID*/ + 2 /*checksum*/) {
        switch (data[2]) {
        case PIDENABLE:
            enable = 1;
            break;
        case PIDDISABLE:
            enable = 0;
            break;
        }
        
        /*if(data[2]==PIDENABLE){
          enable = 1;
        }
        if(data[2]==PIDDISABLE){
          enable = 0;
        }*/
        Output=Output+1; //simulate
        reference = char2double(data + 3 + 0 * 8); // the reference data starts from the third byte,
        Kp = char2double(data + 11 + 0 * 8); // the reference data starts from the third byte,
        Ki = char2double(data + 19 + 0 * 8); // the reference data starts from the third byte,
        Kd = char2double(data + 27 + 0 * 8); // the reference data starts from the third byte,

    }

    //callback(command);  //weijia questinon: how it link to interfacecallback in info file??
}

typedef union {
    uint8_t bytes[8];
    double d;
} double_tt;

double LBNPInterface::char2double(uint8_t* data) {
    double_tt aa;
    memcpy(aa.bytes, data, sizeof(aa.bytes));
    return aa.d;
}

//above is reserved by 2022-06-25 for the port of python send order
void LBNPInterface::Senddata(double reference, double Output) {
    static uint8_t data[100];
    double2char(reference, &data[0]);//updated from 1 to 0, to start record from 0 position.
    double2char(Output, &data[8]);//the next data record from 8 position.
    /*Serial.println("data after double2char is:");
    for(int i=0;i<16;i++){
    Serial.println(data[i]); 
    }
    Serial.println("data after double2char end.");*/
    packetizeAndSend(data, 16); //update from 1+len to len
    /*pinMode(13, LED_BUILTIN);
    delay(3000);
    digitalWrite(13, HIGH);
    delay(3000);
    digitalWrite(13, LOW);*/
}

void LBNPInterface::double2char(double a, uint8_t* data) {
    double_tt aa;
    aa.d = a;
    memcpy(data, aa.bytes, sizeof(aa.bytes));
    
}

void LBNPInterface::packetizeAndSend(uint8_t* data, int len) {
    static uint8_t toSend[100];
    toSend[0] = HEADER1;
    toSend[1] = HEADER2;
    memcpy(toSend + 2, data, (size_t)len);
    len += 2; // for headers
    addChecksum(toSend, len);
    len += 2; // the added checksums

    
    //report the basic serial send data
    toSend_debug.clear();
for(int i=0;i<len;i++){
      toSend_debug.push_back(toSend[i]);
    }
    len_debug=len;
    
    serial_write(toSend, len);
    //Serial.println("data sent out");  //serial.print is deleted to purify the data sent to python.
    
}


//test trial just for simple data send, no more need in main function
void LBNPInterface::send_trial() {
    static uint8_t toSend[100];
    toSend[0] = HEADER1;
    toSend[1] = HEADER2;
for(int i=2;i<20;i++){
    toSend[i] = HEADER1;
    }
    serial_write(toSend, 30); 
    //Serial.println("datasent");
    
}

void LBNPInterface::addChecksum(uint8_t* data, int len) {
    uint8_t sum1 = 0;
    uint8_t sum2 = 0;
    for (int i = 0; i < len; i++) {
        sum1 += data[i];
        sum2 += sum1;
    }
    data[len] = sum1;
    data[len + 1] = sum2;
}

void LBNPInterface::serial_write(uint8_t *str, int n) {
  Serial.write(str, n);
}

void LBNPInterface::test(int len) {
    static uint8_t toSend[100];
   static uint8_t data[100];
   for(int i=0;i<8;i++){
    data[i]=HEADER1;
   }
    toSend[0] = HEADER1;
    toSend[1] = HEADER2;
    memcpy(toSend + 2, data, (size_t)8);
    len += 2; // for headers
    addChecksum(toSend, len);
    len += 2; // the added checksums
    
    Serial.println("header1 is");
    Serial.println(toSend[0]);
    Serial.println("header2 is");
    Serial.println(toSend[1]);
    for(int i=2;i<10;i++){
    Serial.println("data is");
    Serial.println(toSend[i]); 
    }
    Serial.println("check1 is");
    Serial.println(toSend[10]);
    Serial.println("check2 is");
    Serial.println(toSend[11]);
    Serial.println("remaining1 is");
    Serial.println(toSend[12]);
    Serial.println("remaining2 is");
    Serial.println(toSend[13]);
    serial_write(toSend, len);
    Serial.println("data sent out");
    Serial.println("length is");
    Serial.println(len);
}
void LBNPInterface::test1() {
    static uint8_t toSend[100];
    toSend[0] = HEADER1;
    toSend[1] = HEADER2;

    Serial.println("header1 is");
    Serial.println(toSend[0]);
    Serial.println("header2 is");
    Serial.println(toSend[1]);

    Serial.println("remaining1 is");
    Serial.println(toSend[2]);
    Serial.println("remaining2 is");
    Serial.println(toSend[3]);
    serial_write(toSend, 2);
}
    

    
