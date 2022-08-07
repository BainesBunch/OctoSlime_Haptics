/*****************************************
This is a library for the MCP23017/MCP23S17/MCP23018/MCP23S18 I/O Port Expander

You'll find some examples including the wiring which should enable you to use the library. 
Furthermore I have added a list of commands.

You are free to use it, change it or build on it. In case you like it, it would be cool 
if you give it a star.

If you find bugs, please inform me!

Written by Wolfgang (Wolle) Ewald
https://wolles-elektronikkiste.de/en/port-expander-mcp23017-2 (English)
https://wolles-elektronikkiste.de/portexpander-mcp23017       (German)

*******************************************/

#include "MCP23017.h"

MCP23017::MCP23017(int addr){
    resetPin = 99;
    useSPI = false; 
#ifndef USE_TINY_WIRE_M_     
    _wire = &Wire;
#endif
    I2C_Address = addr; 
}

MCP23017::MCP23017(int addr, int rp){
    useSPI = false; 
#ifndef USE_TINY_WIRE_M_     
    _wire = &Wire;
#endif
    I2C_Address = addr;
    resetPin = rp;
    pinMode(csPin, HIGH);
}

#ifndef USE_TINY_WIRE_M_
MCP23017::MCP23017(TwoWire *w, int addr){
    resetPin = 99;
    useSPI = false; 
    _wire = w;
    I2C_Address = addr; 
}

MCP23017::MCP23017(TwoWire *w, int addr, int rp){
    useSPI = false; 
    _wire = w;
    I2C_Address = addr;
    resetPin = rp;
}

 MCP23017::MCP23017(SPIClass *s, int cs, int rp, int addr){
    useSPI = true;
    _spi = s;
    csPin = cs;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    resetPin = rp;
    SPI_Address = addr;
}

MCP23017::MCP23017(int cs, int rp, int addr){
    useSPI = true;
    _spi = &SPI;
    csPin = cs;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    resetPin = rp;
   
    SPI_Address = addr;
}
#endif

bool MCP23017::Init(){
    if(resetPin < 99){
        pinMode(resetPin, OUTPUT); 
        digitalWrite(resetPin, HIGH);
        reset();
    }
    else{
        softReset();
    }
    setIntCon(0b10101010, A);
    if(readMCP23017(INTCONA) != 0b10101010){
        return false;
    }
    intConA = 0b00000000;
    setIntCon(intConA, A);
    intConB = 0b00000000;
    ioConA = 0b00000000;
    ioConB = 0b00000000;
    ioDirA = 0b00000000;
    ioDirB = 0b00000000;
    gppuA = 0b00000000;
    gppuB = 0b00000000;
    gpioA = 0b00000000;
    gpioB = 0b00000000;
    gpIntEnA = 0b00000000;
    gpIntEnB = 0b00000000;
    defValA = 0b00000000;
    defValB = 0b00000000;
#ifndef USE_TINY_WIRE_M_ 
    mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE0); 
#endif
    return true;
};

void MCP23017::reset(){
    digitalWrite(resetPin,LOW);
    delay(10);
    digitalWrite(resetPin, HIGH);
    delay(10);
}

void MCP23017::setPinMode(uint8_t pin, MCP_PORT port, uint8_t state){
    if(port==A){
        if(state==ON){
            ioDirA &= ~(1<<pin);
            gppuA &= ~(1<<pin);
        }
        else if(state==OFF){
            ioDirA |= (1<<pin);
            gppuA &= ~(1<<pin);
        }
        else if(state==INPUT_PULLUP){
            ioDirA |= (1<<pin);
            gppuA |= (1<<pin);
        }
        writeMCP23017(GPPUA, gppuA);
        writeMCP23017(IODIRA, ioDirA);  
    }
    else if(port==B){
        if(state==ON){
            ioDirB &= ~(1<<pin);
            gppuB &= ~(1<<pin);
        }
        else if(state==OFF){
            ioDirB |= (1<<pin);
            gppuB &= ~(1<<pin);
        }
        else if(state==INPUT_PULLUP){
            ioDirB |= (1<<pin);
            gppuB |= (1<<pin);
        }
        writeMCP23017(GPPUB, gppuB);
        writeMCP23017(IODIRB, ioDirB);  
    }       
}

void MCP23017::setPortMode(uint8_t val, MCP_PORT port){
    if(port==A){
        ioDirA = ~val;
        gppuA = 0;
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPPUA, gppuA);
    }
    else if(port==B){
        ioDirB = ~val;
        gppuB = 0;
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPPUB, gppuB);
    }
}

void MCP23017::setPortMode(uint8_t val, MCP_PORT port, uint8_t pu){
    if(pu != INPUT_PULLUP){
        return;
    }
    if(port==A){
        ioDirA = ~val;
        gppuA = ~val;
        writeMCP23017(GPPUA, gppuA);
        writeMCP23017(IODIRA, ioDirA);  
    }
    else if(port==B){
        ioDirB = ~val;
        gppuB = ~val;
        writeMCP23017(GPPUB, gppuB);
        writeMCP23017(IODIRB, ioDirB);
    }
}

void MCP23017::setPin(uint8_t pin, MCP_PORT port, uint8_t state){
    if(port==A){
        if(state==ON){
            gpioA |= (1<<pin); 
        }
        else if(state==OFF){
            gpioA &= ~(1<<pin); 
        }
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
    }
    if(port==B){
        if(state==ON){
            gpioB |= (1<<pin); 
        }
        else if(state==OFF){
            gpioB &= ~(1<<pin); 
        }
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
    }
}

void MCP23017::togglePin(uint8_t pin, MCP_PORT port){
    if(port==A){
        if(((gpioA) & (1<<pin))==0){
            gpioA |= (1<<pin); 
        }
        else if(((gpioA) & (1<<pin)) >= 1){
            gpioA &= ~(1<<pin); 
        }
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
    }
    if(port==B){
        if(((gpioB) & (1<<pin))==0){
            gpioB |= (1<<pin); 
        }
        else if(((gpioB) & (1<<pin)) >= 1){
            gpioB &= ~(1<<pin); 
        }
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
    }
}

void MCP23017::setPinX(uint8_t pin, MCP_PORT port, uint8_t ioDir, uint8_t state){
    if(port==A){
        if(ioDir==OUTPUT){
            ioDirA &= ~(1<<pin);
            gppuA &= ~(1<<pin);
        }
        else if(ioDir==INPUT){
            ioDirA |= (1<<pin);
            gppuA &= ~(1<<pin);
        }
        else if(ioDir==INPUT_PULLUP){
            ioDirA |= (1<<pin);
            gppuA |= (1<<pin);
        }
        if(state==ON){
            gpioA |= (1<<pin); 
        }
        else if(state==OFF){
            gpioA &= ~(1<<pin); 
        }
        writeMCP23017(GPPUA, gppuA);
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
    }
    if(port==B){
        if(ioDir==OUTPUT){
            ioDirB &= ~(1<<pin);
            gppuB &= ~(1<<pin);
        }
        else if(ioDir==INPUT){
            ioDirB |= (1<<pin);
            gppuB &= ~(1<<pin);
        }
        else if(ioDir==INPUT_PULLUP){
            ioDirB |= (1<<pin);
            gppuB |= (1<<pin);
        }
        if(state==ON){
            gpioB |= (1<<pin); 
        }
        else if(state==OFF){
            gpioB &= ~(1<<pin); 
        }
        writeMCP23017(GPPUB, gppuB);
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
    }
}

void MCP23017::setAllPins(MCP_PORT port, uint8_t state){
    if(port==A){
        if(state==ON){
            gpioA = 0b11111111;
        }
        else if (state==OFF){
            gpioA = 0b00000000;
        }
        writeMCP23017(GPIOA, gpioA);
    }
    if(port==B){
        if(state==ON){
            gpioB = 0b11111111;
        }
        else if (state==OFF){
            gpioB = 0b00000000;
        }
        writeMCP23017(GPIOB, gpioB);
    }
}

void MCP23017::setPort(uint8_t val, MCP_PORT port){
    if(port==A){
        gpioA = val;
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
    }
    else if(port==B){
        gpioB = val;
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
    }
}

void MCP23017::setPort(uint8_t valA, uint8_t valB){
    gpioA = valA;
    gpioB = valB;
    writeMCP23017(IODIRA, ioDirA, ioDirB);
    writeMCP23017(GPIOA, gpioA, gpioB);
}

void MCP23017::setPortX(uint8_t iodirval, uint8_t gpioval, MCP_PORT port){
    if(port==A){
        ioDirA = ~iodirval;
        gpioA = gpioval;
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
    }
    else if(port==B){
        ioDirB = ~iodirval;
        gpioB = gpioval;
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
    }
}

void MCP23017::setInterruptPinPol(uint8_t state){
    if(state==HIGH){
        ioConA |= (1<<INTPOL);
        ioConB |= (1<<INTPOL);
    }
    if(state==LOW){
        ioConA &= ~(1<<INTPOL);
        ioConB &= ~(1<<INTPOL);
    }
    writeMCP23017(IOCONA, ioConA);
    writeMCP23017(IOCONB, ioConB);
}   

void MCP23017::setIntOdr(uint8_t state){
    if(state==ON){
        ioConA |= (1<<INTODR);
        ioConB |= (1<<INTODR);
    }
    if(state==OFF){
        ioConA &= ~(1<<INTODR);
        ioConB &= ~(1<<INTODR);
    }
    writeMCP23017(IOCONA, ioConA);
    writeMCP23017(IOCONB, ioConB);
}   

void MCP23017::setInterruptOnChangePin(uint8_t pin, MCP_PORT port){
    if(port==A){
        ioDirA |= (1<<pin); 
        gpIntEnA |= (1<<pin);
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
        writeMCP23017(GPINTENA, gpIntEnA);
    }
    else if (port==B){
        ioDirB |= (1<<pin); 
        gpIntEnB |= (1<<pin);
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
        writeMCP23017(GPINTENB, gpIntEnB);
    }
}

void MCP23017::setInterruptOnDefValDevPin(uint8_t pin, MCP_PORT port, uint8_t intState){
    if(port==A){
        ioDirA |= (1<<pin); 
        gpIntEnA |= (1<<pin);
        intConA |= (1<<pin);
        if(intState==ON) defValA |= (1<<pin);
        else if(intState==OFF) defValA &= ~(1<<pin);
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPIOA, gpioA);
        writeMCP23017(GPINTENA, gpIntEnA);
        writeMCP23017(INTCONA, intConA);
        writeMCP23017(DEFVALA, defValA);
    }
    else if (port==B){
        ioDirB |= (1<<pin); 
        gpIntEnB |= (1<<pin);
        intConB |= (1<<pin);
        if(intState==ON) defValB |= (1<<pin);
        else if(intState==OFF) defValB &= ~(1<<pin);
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPIOB, gpioB);
        writeMCP23017(GPINTENB, gpIntEnB);
        writeMCP23017(INTCONB, intConB);
        writeMCP23017(DEFVALB, defValB);
    }
}

void MCP23017::setInterruptOnChangePort(uint8_t val, MCP_PORT port){
    if(port==A){
        ioDirA |= val;
        gpIntEnA = val;
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPINTENA, gpIntEnA);
    }
    else if (port==B){
        ioDirB |= val;
        gpIntEnB = val;
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPINTENB, gpIntEnB);
    }
}

void MCP23017::setInterruptOnDefValDevPort(uint8_t val, MCP_PORT port, uint8_t state){
    if(port==A){
        ioDirA |= val; 
        gpIntEnA |= val;
        intConA |= val;
        defValA = state;
        writeMCP23017(IODIRA, ioDirA);
        writeMCP23017(GPINTENA, gpIntEnA);
        writeMCP23017(INTCONA, intConA);
        writeMCP23017(DEFVALA, defValA);
    }
    else if (port==B){
        ioDirB |= val; 
        gpIntEnB |= val;
        intConB |= val;
        defValB = state;
        writeMCP23017(IODIRB, ioDirB);
        writeMCP23017(GPINTENB, gpIntEnB);
        writeMCP23017(INTCONB, intConB);
        writeMCP23017(DEFVALB, defValB);
    }
}

void MCP23017::deleteAllInterruptsOnPort(MCP_PORT port){
    if(port==A){
        gpIntEnA = 0b00000000;
        writeMCP23017(GPINTENA, gpIntEnA);
    }
    else if (port==B){
        gpIntEnB = 0b00000000;
        writeMCP23017(GPINTENB, gpIntEnB);
    }
}

void MCP23017::setPinPullUp(uint8_t pin, MCP_PORT port, uint8_t state){
    if(port==A){
        if(state==ON){
            gppuA |= (1<<pin);
        }
        else if(state==OFF){
            gppuA &= ~(1<<pin);
        }
        writeMCP23017(GPPUA, gppuA);
    }
    else if(port==B){
        if(state==ON){
            gppuB |= (1<<pin);
        }
        else if(state==OFF){
            gppuB &= ~(1<<pin);
        }
        writeMCP23017(GPPUB, gppuB);
    }
}   
        
void MCP23017::setPortPullUp(uint8_t val, MCP_PORT port){
    if(port==A){
        gppuA = val;
        writeMCP23017(GPPUA, gppuA);
    }
    else if(port==B){
        gppuB = val;
        writeMCP23017(GPPUB, gppuB);
    }
}

void MCP23017::setIntMirror(uint8_t state){
    if(state==ON){
        ioConA |= (1<<MIRROR);
        ioConB |= (1<<MIRROR);
    }
    if(state==OFF){
        ioConA &= ~(1<<MIRROR);
        ioConB &= ~(1<<MIRROR);
    }
    writeMCP23017(IOCONA, ioConA);
    writeMCP23017(IOCONB, ioConB);
}   

uint8_t MCP23017::getIntFlag(MCP_PORT port){
    uint8_t value = 0;
    if(port==A){
        value = readMCP23017(INTFA);
    }
    else if (port==B){ 
        value = readMCP23017(INTFB);
    }
    return value;
}

bool MCP23017::getPin(uint8_t pin, MCP_PORT port){
    uint8_t result = 0;
    if(port==A){
        result = readMCP23017(GPIOA);
    }
    else if(port==B){
        result = readMCP23017(GPIOB);
    }
    result &= (1<<pin);
    if(result) return true;
    else return false;  
}

uint8_t MCP23017::getPort(MCP_PORT port){
    uint8_t value = 0;
    if(port==A){
        value = readMCP23017(GPIOA);
    }
    else if(port==B){
        value = readMCP23017(GPIOB);
    }
    return value;
}

uint8_t MCP23017::getIntCap(MCP_PORT port){
    uint8_t value = 0;
    if(port==A){
        value = readMCP23017(INTCAPA);
    }
    else if (port==B){
        value = readMCP23017(INTCAPB);
    }
    return value;
}

#ifndef USE_TINY_WIRE_M_
void MCP23017::setSPIClockSpeed(unsigned long clock){
    mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}
#endif

void MCP23017::softReset(){
    setPortMode(0, A);
    setPortMode(0, B);
    uint8_t reg = 0x02;
    if(!useSPI){
#ifndef USE_TINY_WIRE_M_
        _wire->beginTransmission(I2C_Address);
        _wire->write(reg);
        for(int8_t i=0; i<18; i++){
            _wire->write(0x00);
        }
        _wire->endTransmission();
#else
        TinyWireM.beginTransmission(I2C_Address);
        TinyWireM.send(reg);
        for(int8_t i=0; i<18; i++){
            TinyWireM.send(0x00);
        }
        TinyWireM.endTransmission();    
#endif
    }
#ifndef USE_TINY_WIRE_M_    
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        uint16_t transBytes = ((SPI_Address<<1) << 8 | reg);
        _spi->transfer16(transBytes); 
        for(int8_t i=0; i<18; i++){
            _spi->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
#endif
}

/* Private Functions */

void MCP23017::setI2C_Address(int addr){
    I2C_Address = addr;
}

void MCP23017::setResetPin(uint8_t rp){
    resetPin = rp;
}

void MCP23017::setIoCon(uint8_t val, MCP_PORT port){
    if(port==A){
        ioConA = val;
        writeMCP23017(IOCONA, ioConA);
    }
    else if (port==B){
        ioConB = val;
        writeMCP23017(IOCONB, ioConB);
    }
}

void MCP23017::setGpIntEn(uint8_t val, MCP_PORT port){
    if(port==A){
        gpIntEnA = val;
        writeMCP23017(GPINTENA, gpIntEnA);
    }
    else if (port==B){
        gpIntEnB = val;
        writeMCP23017(GPINTENB, gpIntEnB);  
    }
}

void MCP23017::setIntCon(uint8_t val, MCP_PORT port){
    if(port==A){
        intConA = val;
        writeMCP23017(INTCONA, intConA);
    }
    else if (port==B){
        ioConB = val;
        writeMCP23017(INTCONB, intConB);
    }
}

void MCP23017::setDefVal(uint8_t val, MCP_PORT port){
    if(port==A){
        defValA = val;
        writeMCP23017(DEFVALA, defValA);
    }
    else if (port==B){
        defValB = val;
        writeMCP23017(DEFVALB, defValB);    
    }
}

void MCP23017::writeMCP23017(uint8_t reg, uint8_t val){
    if(!useSPI){
#ifndef USE_TINY_WIRE_M_
        _wire->beginTransmission(I2C_Address);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
#else
        TinyWireM.beginTransmission(I2C_Address);
        TinyWireM.send(reg);
        TinyWireM.send(val);
        TinyWireM.endTransmission();    
#endif
    }
#ifndef USE_TINY_WIRE_M_    
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        uint16_t transBytes = ((SPI_Address<<1) << 8 | reg);
        _spi->transfer16(transBytes); 
        _spi->transfer(val);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
#endif
}

void MCP23017::writeMCP23017(uint8_t reg, uint8_t valA, uint8_t valB){
    if(!useSPI){
#ifndef USE_TINY_WIRE_M_
        _wire->beginTransmission(I2C_Address);
        _wire->write(reg);
        _wire->write(valA);
        _wire->write(valB);
        _wire->endTransmission();
#else
        TinyWireM.beginTransmission(I2C_Address);
        TinyWireM.send(reg);
        TinyWireM.send(valA);
        TinyWireM.send(valB);
        TinyWireM.endTransmission();    
#endif  
    }
#ifndef USE_TINY_WIRE_M_
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        uint16_t transBytes = ((SPI_Address<<1) << 8 | reg);
        _spi->transfer16(transBytes); 
        _spi->transfer(valA);
        _spi->transfer(valB);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
#endif
}

uint8_t MCP23017::readMCP23017(uint8_t reg){
    uint8_t regVal;
    if(!useSPI){
#ifndef USE_TINY_WIRE_M_
        _wire->beginTransmission(I2C_Address);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(I2C_Address, 1);
        regVal = _wire->read();
#else
        TinyWireM.beginTransmission(I2C_Address);
        TinyWireM.send(reg);
        TinyWireM.endTransmission();
        TinyWireM.requestFrom(I2C_Address, 1);
        regVal = TinyWireM.receive();
#endif
        return regVal;
    }
#ifndef USE_TINY_WIRE_M_
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        uint16_t transBytes = (((SPI_Address<<1) | SPI_READ) << 8 | reg);
        _spi->transfer16(transBytes); 
        regVal = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
        return regVal;
    }
#endif
}

