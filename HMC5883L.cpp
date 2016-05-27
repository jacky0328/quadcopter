/* 
 * File:   HMC5883L.cpp
 * Author: matt
 * 
 * Created on 16 November 2012, 22:44
 */

/*
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout);
static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
*/

#include "HMC5883L.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;

HMC5883LClass HMC5883L;

HMC5883LClass::HMC5883LClass()
{
}

HMC5883LClass::HMC5883LClass(const HMC5883LClass& orig)
{
}

HMC5883LClass::~HMC5883LClass()
{
}

void HMC5883LClass::getField(s_rawData* rawData)
{
    uint8_t buf[6];
    //I2CInterface.readRegister(HMC5883L_ADDRESS, HMC5883L_RA_X_H, buf, 6);
    I2Cdev::readBytes(HMC5883L_ADDRESS, HMC5883L_RA_X_H, 6,buf);
    rawData->mag_x = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    rawData->mag_z = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    rawData->mag_y = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    double len =  sqrt(rawData->mag_x*rawData->mag_x + rawData->mag_z*rawData->mag_z + rawData->mag_y * rawData->mag_y);
    calc_angle(rawData->mag_x/len,rawData->mag_y/len,rawData);
}

void HMC5883LClass::calc_angle(double _x, double _y,s_rawData* rawData )  
{
   rawData->mag_angle = atan2(_y,_x)*180/M_PI;
}

void HMC5883LClass::initialise()
{
    checkCommunication_();
    setConfigA_(0b00011000); //No averaging, 75Hz update rate, no bias
    setConfigB_(0b00100000); //+-1.3 gauss range, 1090 LSB/gauss
    setMode_(0); //Continuous measurement mode
}

bool HMC5883LClass::setConfigA_(uint8_t value) // set data rate
{
    //return I2CInterface.writeRegister(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, &value, 1);
    return I2Cdev::writeBytes(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 1,&value);

}

bool HMC5883LClass::setConfigB_(uint8_t value) // set range
{
    //return I2CInterface.writeRegister(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, &value, 1);
    return I2Cdev::writeBytes(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 1,&value);  //set range
}

bool HMC5883LClass::setMode_(uint8_t value)
{
    //return I2CInterface.writeRegister(HMC5883L_ADDRESS, HMC5883L_RA_MODE, &value, 1);
    return I2Cdev::writeBytes(HMC5883L_ADDRESS, HMC5883L_RA_MODE, 1,&value);
}

bool HMC5883LClass::checkCommunication_()
{
    uint8_t buf[3];
    //I2CInterface.readRegister(HMC5883L_ADDRESS, HMC5883L_RA_ID_A, buf, 3);
    I2Cdev::readBytes(HMC5883L_ADDRESS, HMC5883L_RA_ID_A, 3,buf);
    cout << "buf[0]:" <<buf[0] << endl;
    if ((buf[0] != 'H') || (buf[1] != '4') || (buf[2] != '3'))
    {
	cout << "HMC5883L communication failed, recieved " << buf[0] << ", " << buf[1] << ", " << buf[2] << endl;
	return false;
    }
    return true;
}


