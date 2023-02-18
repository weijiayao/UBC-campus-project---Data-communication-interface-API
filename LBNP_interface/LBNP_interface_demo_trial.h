//2022-07-14
#pragma once
#ifndef THE_INTERFACE_H
#define THE_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include <vector>



// received command byte:
typedef enum {
    nothing = 0x00,
    beginBoxSensor = 0x02,
    turnVacuumOn = 0x03,
    turnVacuumOff = 0x04,
    getChan3Voltage = 0x05,
    getChan4Voltage = 0x07
} AppCommand;

// send command byte:
typedef enum {
    sendChan3Voltage = 0x01,
    sendChan4Voltage = 0x06,
    sendBoxPressureSensorStatus = 0x08
} SoftwareCommand;

class LBNPInterface {

public:

    void LBNPInterface::send_trial(); //test trial just for simple data send, no more need in main function
    void LBNPInterface::upload_debug();
    void LBNPInterface::upload_data();
    void Senddata(double reference, double Output);
     void test(int len);
     void test1();

private:
    //    bool boxPressureSensorStatus = false;
    void receive(std::vector<uint8_t> data, int len);
    double voltage3 = 0;
    double voltage4 = 0;
    double dutyCycle = 0;

    // ******** functions ************
    void decode(uint8_t* data, int len);
   
    bool checkCheckSum(uint8_t* data, int len);
    void addChecksum(uint8_t* data, int len); // adds two bytes to the end of data, it should have room for those
    double char2double(uint8_t* data); // little endian
    
    void double2char(double a, uint8_t* data);
    void packetizeAndSend(uint8_t* data, int len);
    void serial_write(uint8_t *str, int n);
   
    
};


#endif //THE_INTERFACE_H
