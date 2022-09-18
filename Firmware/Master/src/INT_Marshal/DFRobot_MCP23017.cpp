/*!
 * @file DFRobot_MCP23017.h
 * @brief Define the basic structure of class DFRobot_MCP23017
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-07-16
 * @https://github.com/DFRobot/DFRobot_MCP23017
 */
#include <Arduino.h>
#include <INT_Marshal/DFRobot_MCP23017.h>
#include <string.h>

#define REG_MCP23017_IODIRA   0x00   //< direction register portA, control the direction of the data I/O.
#define REG_MCP23017_IODIRB   0x01   //< direction register portB, control the direction of the data I/O. 
#define REG_MCP23017_IPOLA    0x02   //< Input polarity registerA, 1 = GPIO register bit will reflect the opposite logic state of the input pin;0 = GPIO register bit will reflect the same logic state of the input pin.
#define REG_MCP23017_IPOLB    0x03   //< Input polarity registerB, 1 = GPIO register bit will reflect the opposite logic state of the input pin;0 = GPIO register bit will reflect the same logic state of the input pin.
#define REG_MCP23017_GPINTENA 0x04   //< Interrupt on change control registorA, control the interrupt-on change feature for each pin: 1 = Enable GPIO input pin for interrupt-on-change event; 0 = Disable GPIO input pin for interrupt-on-change event.
#define REG_MCP23017_GPINTENB 0x05   //< Interrupt on change control registorB, control the interrupt-on change feature for each pin: 1 = Enable GPIO input pin for interrupt-on-change event; 0 = Disable GPIO input pin for interrupt-on-change event
#define REG_MCP23017_DEFVALA  0x06   //< Default compare register A for interrupt-on-change.If enabled (via GPINTEN and INTCON) to compare against the DEFVAL register, an opposite value on the associated pin will cause an interrupt to occur.
#define REG_MCP23017_DEFVALB  0x07   //< Default compare register B for interrupt-on-change.If enabled (via GPINTEN and INTCON) to compare against the DEFVAL register, an opposite value on the associated pin will cause an interrupt to occur
#define REG_MCP23017_INTCONA  0x08   //< Interrupt control register A, 1 = Controls how the associated pin value is compared for interrupt-on-change; 0 = Pin value is compared against the previous pin value.
#define REG_MCP23017_INTCONB  0x09   //< Interrupt control register B, 1 = Controls how the associated pin value is compared for interrupt-on-change; 0 = Pin value is compared against the previous pin value.
#define REG_MCP23017_IOCONA   0x0A   //< Configuration register A contains several bits for configuring the device.
#define REG_MCP23017_IOCONB   0x0B   //< Configuration register B contains several bits for configuring the device.
#define REG_MCP23017_GPPUA    0x0C   //< Port A pull-up resistor configuration registerA, control the pull-up resistors for the port pins.
#define REG_MCP23017_GPPUB    0x0D   //< Port B pull-up resistor configuration registerB, control the pull-up resistors for the port pins.
#define REG_MCP23017_INTFA    0x0E   //< Interrupt flag register A: 1 = Pin caused interrupt; 0 = Interrupt not pending.
#define REG_MCP23017_INTFB    0x0F   //< Interrupt flag register B: 1 = Pin caused interrupt; 0 = Interrupt not pending.
#define REG_MCP23017_INTCAPA  0x10   //< Interrupt capture register A : the INTCAP register captures the GPIO port value at the time the interrupt occurred. The register will remain unchanged until the interrupt is cleared via a read of INTCAP or GPIO.
#define REG_MCP23017_INTCAPB  0x11   //< Interrupt capture register B: the INTCAP register captures the GPIO port value at the time the interrupt occurred. The register will remain unchanged until the interrupt is cleared via a read of INTCAP or GPIO
#define REG_MCP23017_GPIOA    0x12   //< Port registerA reflects the value on the port.Reading from this register reads the port. Writing to this register modifies the Output Latch (OLAT) register.
#define REG_MCP23017_GPIOB    0x13   //< Port registerB reflects the value on the port.Reading from this register reads the port. Writing to this register modifies the Output Latch (OLAT) register
#define REG_MCP23017_OLATA    0x14   //< Oupput latch registerA provides access to the output latches. 
#define REG_MCP23017_OLATB    0x15   //< Oupput latch registerB provides access to the output latches.

DFRobot_MCP23017::sPinDescription_t DFRobot_MCP23017::_pinDescriptions[eGPIOTotal]=
{
  {eGPA0,"GPA0"},
  {eGPA1,"GPA1"},
  {eGPA2,"GPA2"},
  {eGPA3,"GPA3"},
  {eGPA4,"GPA4"},
  {eGPA5,"GPA5"},
  {eGPA6,"GPA6"},
  {eGPA7,"GPA7"},
  {eGPB0,"GPB0"},
  {eGPB1,"GPB1"},
  {eGPB2,"GPB2"},
  {eGPB3,"GPB3"},
  {eGPB4,"GPB4"},
  {eGPB5,"GPB5"},
  {eGPB6,"GPB6"},
  {eGPB7,"GPB7"}
};

DFRobot_MCP23017::DFRobot_MCP23017(TwoWire &wire, uint8_t addr)
{
  _addr = addr;
  _pWire = &wire;
  memset(_cbs, 0, sizeof(_cbs));
}

DFRobot_MCP23017::~DFRobot_MCP23017(){

}

int DFRobot_MCP23017::begin(void){
  _pWire->begin();
  if(i2cdetect(_addr) != 0){
      DBG("I2C ADDR ERROR!");
      return ERR_ADDR;
  }
  uint8_t value = 0xff;
  writeReg(REG_MCP23017_IODIRA, &value, 1);
  writeReg(REG_MCP23017_IODIRB, &value, 1);
  value = 0x00;
  for(uint8_t ad = 0x02; ad < 0x16; ad++){
     if((ad > REG_MCP23017_GPPUA) && (ad < REG_MCP23017_GPIOA)){
         continue;
     }
     writeReg(ad, &value, 1);
  }
  return 0;
}

String DFRobot_MCP23017::pinDescription(ePin_t pin)
{
  for(int i=0; i<sizeof(_pinDescriptions)/sizeof(_pinDescriptions[i]); i++){
    if(pin == (uint8_t)_pinDescriptions[i].pin){
      return _pinDescriptions[i].description;
    }
  }
  return "";
}

String DFRobot_MCP23017::pinDescription(int pin){
  for(int i=0; i<sizeof(_pinDescriptions)/sizeof(_pinDescriptions[i]); i++){
    if(pin == (int)_pinDescriptions[i].pin){
      return _pinDescriptions[i].description;
    }
  }
  return "";
}

int DFRobot_MCP23017::pinMode(ePin_t pin, uint8_t mode)
{
  uint8_t _pin = (uint8_t)pin;
  uint8_t reg = 0;
  uint8_t reg1 = 0;
  bool flag = false;
  if(_pin == (uint8_t)eGPA){
      reg = REG_MCP23017_IODIRA;
      reg1 = REG_MCP23017_GPPUA;
  }else if(_pin == (uint8_t)eGPB){
      reg = REG_MCP23017_IODIRB;
      reg1 = REG_MCP23017_GPPUB;
  }else{
      if(_pin >= /*16*/(uint8_t)eGPIOTotal){
          return ERR_PIN;
      }
      if(_pin < (uint8_t)eGPB0){
          reg = REG_MCP23017_IODIRA;
          reg1 = REG_MCP23017_GPPUA;
      }else{
          reg = REG_MCP23017_IODIRB;
          reg1 = REG_MCP23017_GPPUB;
      }
      flag = true;
  }
  switch(mode){
      case INPUT:
          setInput(reg, _pin%8, flag);
          break;
      case INPUT_PULLUP:
          setInput(reg, _pin%8, flag);
          setPullUp(reg1, _pin%8, flag);
          break;
      case OUTPUT:
          setOutput(reg, _pin%8, flag);
          break;
      default:
          setInput(reg, _pin%8, flag);
          setPullUp(reg1, _pin%8, flag);
          break;
  }
  return ERR_OK;
}

int DFRobot_MCP23017::digitalWrite(ePin_t pin, uint8_t level){
  uint8_t _pin = (uint8_t)pin;
  uint8_t reg = 0;
  uint8_t reg1 = 0;
  uint8_t value = level;
  uint8_t flag = true;
  if(_pin == (uint8_t)eGPA){
      reg = REG_MCP23017_GPIOA;
      reg1 = REG_MCP23017_OLATA;
      flag = false;
  }else if(_pin == (uint8_t)eGPB){
      reg = REG_MCP23017_GPIOB;
      reg1 = REG_MCP23017_OLATB;
      flag = false;
  }else{
      if(_pin >= /*15*/(uint8_t)eGPIOTotal){
          DBG("pin ERROR!");
          return ERR_PIN;
      }
      if(_pin < (uint8_t)eGPB0){
          reg = REG_MCP23017_GPIOA;
          reg1 = REG_MCP23017_OLATA;
      }else{
          reg = REG_MCP23017_GPIOB;
          reg1 = REG_MCP23017_OLATB;
      }
  }
  if(flag){
      if(readReg(reg, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, _pin%8, level&0x01);
  }
  writeReg(reg, &value, 1);
  if(readReg(reg1, &value, 1) != 1){
       DBG("I2C READ ERROR!");
       return ERR_DATA_READ;
   }
  return ERR_OK;
}

int DFRobot_MCP23017::digitalRead(ePin_t pin){
  uint8_t _pin = (uint8_t)pin;
  uint8_t value = 0;
  uint8_t reg = 0;
  uint8_t flag = false;
  if(_pin == (uint8_t)eGPA){
      reg = REG_MCP23017_GPIOA;
  }else if(_pin == (uint8_t)eGPB){
      reg = REG_MCP23017_GPIOB;
  }else{
      if(_pin >= /*15*/(uint8_t)eGPIOTotal){
          DBG("pin ERROR!");
          return ERR_PIN;
      }
      if(_pin < (uint8_t)eGPB0) reg = REG_MCP23017_GPIOA;
      else reg = REG_MCP23017_GPIOB;
      flag = true;
  }
  readReg(reg, &value, 1);
  if(flag) value = (value >> (_pin%8))&0x01;
  return value;
}


void DFRobot_MCP23017::pollInterrupts(eGPIOGroup_t group)
{
  uint8_t value;
  if(group & eGPIOA){
    if(readReg(REG_MCP23017_INTFA, &value, 1) != 1){
      DBG("I2C READ ERROR!");
      return;
    }
    DBGI("INTFA=");DBGI(value,HEX);
    if(value != 0){
      uint8_t level;
      readReg(REG_MCP23017_GPIOA, &level, 1);
      for(int i=0; i<8; i++){
        if((value & (1<<i)) && ( _cbs[i].cb!=NULL )){
          uint8_t l = (level>>i)&0x01;
          if(((_cbs[i].mode == eRising) && (!l)) || ((_cbs[i].mode == eFalling) & l)){
            continue;
          }
          _cbs[i].cb(i);
        }
      }
      if(readReg(REG_MCP23017_INTCAPA, &value, 1) != 1){
        DBG("I2C READ ERROR!");
        return;
      }
    }
  }
  if(group & eGPIOB){
    if(readReg(REG_MCP23017_INTFB, &value, 1) != 1){
      DBG("I2C READ ERROR!");
      return ;
    }
    DBGI("INTFB=");DBGI(value,HEX);
    if(value != 0){
      uint8_t level;
      readReg(REG_MCP23017_GPIOB, &level, 1);
      for(int i=0; i<8; i++){
        if((value & (1<<i)) && (_cbs[i+8].cb!=NULL)){
          uint8_t l = (level>>i)&0x01;
          if(((_cbs[i+8].mode == eRising) && (!l)) || ((_cbs[i+8].mode == eFalling) & l)){
            continue;
          }
          _cbs[i+8].cb(i+8);
        }
      }
      if(readReg(REG_MCP23017_INTCAPB, &value, 1) != 1){
        DBG("I2C READ ERROR!");
        return;
      }
    }
  }
}

void DFRobot_MCP23017::pinModeInterrupt(ePin_t pin, eInterruptMode_t mode, MCP23017_INT_CB cb){
  uint8_t _pin = (uint8_t)pin;
  if(_pin >= /*16*/(uint8_t)eGPIOTotal){
      DBGI("PIN ERROR!");
      return ;
  }
  if(cb == NULL){
      DBGI("null pointer!");
      return ;
  }
  pinMode(pin, INPUT_PULLUP);
  interruptConfig();
  _cbs[_pin].cb = cb;
  _cbs[_pin].mode = mode;

  switch(mode){
    case eLowLevel:
      setInterruptModeLowLevel(_pin);
      break;
    case eHighLevel:
      setInterruptModeHighLevel(_pin);
      break;
    default:
      setInterruptModeChangeLevel(_pin);
      break;
  }
}

int DFRobot_MCP23017::i2cdetect(uint8_t addr){
  _pWire->beginTransmission(addr);
  if(_pWire->endTransmission() == 0){
      DBG("Addr ok!");
      return  ERR_OK;
  }
  return ERR_ADDR;
}

int DFRobot_MCP23017::setInput(uint8_t reg, uint8_t index, bool flag){
  uint8_t data = 0xFF;
  if(flag){
      if(readReg(reg, &data, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBG(data,HEX);
      data = updateBit(data, index, 1);
  }
  writeReg(reg, &data, 1);
  return ERR_OK;
}

int DFRobot_MCP23017::setOutput(uint8_t reg, uint8_t index, bool flag){
  uint8_t value = 0;
  if(flag){
      if(readReg(reg, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBG(value,HEX);
      value = updateBit(value, index, 0);
  }
  writeReg(reg, &value, 1);
  return ERR_OK;
}
int DFRobot_MCP23017::setPullUp(uint8_t reg, uint8_t index, bool flag){
  uint8_t value = 0xFF;
  if(flag){
      if(readReg(reg, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
  }
  writeReg(reg, &value, 1);
  return ERR_OK;
}

int DFRobot_MCP23017::setInterruptModeChangeLevel(uint8_t index){
  uint8_t value = 0;
  if(index < (uint8_t)eGPB0){
      if(readReg(REG_MCP23017_INTCONA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 0);
      DBG("INTCONA：");DBG(value,HEX);
      writeReg(REG_MCP23017_INTCONA, &value, 1);
      if(readReg(REG_MCP23017_DEFVALA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 0);
      writeReg(REG_MCP23017_DEFVALA, &value, 1);
      DBG("DEFVALA：");DBG(value,HEX);
      if(readReg(REG_MCP23017_GPINTENA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_GPINTENA, &value, 1);
      DBG("GPINTENA：");DBG(value,HEX);
  }else{
      index -= 8;
      if(readReg(REG_MCP23017_INTCONB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 0);
      writeReg(REG_MCP23017_INTCONB, &value, 1);
      DBG("INTCONB：");DBG(value,HEX);
      if(readReg(REG_MCP23017_DEFVALB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 0);
      writeReg(REG_MCP23017_DEFVALB, &value, 1);
      DBG("DEFVALB：");DBG(value,HEX);
      if(readReg(REG_MCP23017_GPINTENB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_GPINTENB, &value, 1);
      DBG("GPINTENB：");DBG(value,HEX);
  }
  return ERR_OK;
}
int DFRobot_MCP23017::setInterruptModeLowLevel(uint8_t index){
  uint8_t value = 0;
  if(index < (uint8_t)eGPB0){
      if(readReg(REG_MCP23017_INTCONA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_INTCONA, &value, 1);
      DBG("INTCONA：");DBG(value,HEX);
      DBG("DEFVALA：");DBG(value,HEX);
      if(readReg(REG_MCP23017_DEFVALA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_DEFVALA, &value, 1);
      DBG("DEFVALA：");DBG(value,HEX);
      if(readReg(REG_MCP23017_GPINTENA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_GPINTENA, &value, 1);
      DBG("GPINTENA：");DBG(value,HEX);
  }else{
      index -= 8;
      if(readReg(REG_MCP23017_INTCONB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_INTCONB, &value, 1);
      DBG("INTCONA：");DBG(value,HEX);
      if(readReg(REG_MCP23017_DEFVALB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_DEFVALB, &value, 1);
      DBG("DEFVALB：");DBG(value,HEX);
      if(readReg(REG_MCP23017_GPINTENB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_GPINTENB, &value, 1);
      DBG("GPINTENB：");DBG(value,HEX);
  }
  return ERR_OK;
}
int DFRobot_MCP23017::setInterruptModeHighLevel(uint8_t index){
  uint8_t value = 0;
  if(index < (uint8_t)eGPB0){
      if(readReg(REG_MCP23017_INTCONA, &value, 1) != 1){//interrupt control, set pin to interrupt
          DBGI("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBGI("INTCONA=");DBGI(value,HEX);
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_INTCONA, &value, 1);
      if(readReg(REG_MCP23017_INTCONA, &value, 1) != 1){//read interrupt control register 
          DBGI("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBGI("INTCONA=");DBGI(value,HEX);
      if(readReg(REG_MCP23017_DEFVALA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBGI("DEFVALA=");DBGI(value,HEX);
      value = updateBit(value, index, 0);
      writeReg(REG_MCP23017_DEFVALA, &value, 1);
      
      if(readReg(REG_MCP23017_DEFVALA, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBGI("DEFVALA=");DBGI(value,HEX);
      if(readReg(REG_MCP23017_GPINTENA, &value, 1) != 1){
          DBGI("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBGI("GPINTENA=");DBGI(value,HEX);
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_GPINTENA, &value, 1);
      if(readReg(REG_MCP23017_GPINTENA, &value, 1) != 1){
          DBGI("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      DBGI("GPINTENA=");DBGI(value,HEX);
  }else{
      index -= 8;
      if(readReg(REG_MCP23017_INTCONB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_INTCONB, &value, 1);
      DBG("INTCONA：");DBG(value,HEX);
      if(readReg(REG_MCP23017_DEFVALB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 0);
      writeReg(REG_MCP23017_DEFVALB, &value, 1);
      DBG("DEFVALB：");DBG(value,HEX);
      if(readReg(REG_MCP23017_GPINTENB, &value, 1) != 1){
          DBG("I2C READ ERROR!");
          return ERR_DATA_READ;
      }
      value = updateBit(value, index, 1);
      writeReg(REG_MCP23017_GPINTENB, &value, 1);
      DBG("GPINTENB：");DBG(value,HEX);
  }
  return ERR_OK;
}

uint8_t DFRobot_MCP23017::updateBit(uint8_t val, uint8_t pin, uint8_t level){
  uint8_t value = val;
  if(level){
      value |= (1 << pin);
  }else{
      value &= (~(1 << pin));
  }
  return value;
}

void DFRobot_MCP23017::interruptConfig(){
  sIOCON_t iocon={0};
  if(readReg(REG_MCP23017_IOCONA, &iocon, 1) != 1){
      DBG("I2C READ ERROR!");
      return ;
  }
  iocon.INTPOL = 1;
  iocon.ODR = 0;
  iocon.MIRROR = 0;
  DBGI("IOCONA：");DBGI(*((uint8_t*)&iocon), HEX);
  writeReg(REG_MCP23017_IOCONA, &iocon, 1);
  if(readReg(REG_MCP23017_IOCONB, &iocon, 1) != 1){
      DBGI("I2C READ ERROR!");
      return ;
  }
  iocon.INTPOL = 1;
  iocon.ODR = 0;
  iocon.MIRROR = 0;
  writeReg(REG_MCP23017_IOCONA, &iocon, 1);
  DBGI("IOCONB：");DBGI(*((uint8_t*)&iocon), HEX);
}

void DFRobot_MCP23017::writeReg(uint8_t reg, const void* pBuf, size_t size){
  if(pBuf == NULL){
      DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_addr);
  _pWire->write(&reg, 1);

  for(uint16_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

uint8_t DFRobot_MCP23017::readReg(uint8_t reg, void* pBuf, size_t size){
  if(pBuf == NULL){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_addr);
  _pWire->write(&reg, 1);
  if( _pWire->endTransmission() != 0){
      return 0;
  }
  _pWire->requestFrom(_addr, (uint8_t) size);
  for(uint16_t i = 0; i < size; i++){
    _pBuf[i] = _pWire->read();
  }
  _pWire->endTransmission();
  return size;
}