#include <STM32FreeRTOS.h>
//#include "stm32f1xx_hal.h"
#include "softI2C.h"





void SoftI2C::dwt_init() {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}


void SoftI2C::_delay() {
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < _delay_cycles);
}

void SoftI2C::SDA_input() {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = _pin_sda,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL
    };
    HAL_GPIO_Init(_port_sda, &GPIO_InitStruct);
}
    
void SoftI2C::SDA_output() {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = _pin_sda,
        .Mode = GPIO_MODE_OUTPUT_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(_port_sda, &GPIO_InitStruct);
}

void SoftI2C::SCL_output() {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = _pin_scl,
        .Mode = GPIO_MODE_OUTPUT_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(_port_scl, &GPIO_InitStruct);
}





SoftI2C::SoftI2C(GPIO_TypeDef* port_sda, uint16_t pin_sda, GPIO_TypeDef* port_scl, uint16_t pin_scl, uint32_t speed_khz) {
    _port_sda = port_sda;
    _port_scl = port_scl;
    _pin_sda = pin_sda;
    _pin_scl = pin_scl;
    _speed_khz = speed_khz;
    _delay_cycles = SystemCoreClock / (_speed_khz * 1000) / 2; // two delays per bit.
}

    
void SoftI2C::begin() {
    dwt_init();
    SDA_output();
    SCL_output();
    SCL_high();
    SDA_high();
    // small delay is recommended for the stablization of the bus and the initialization of the slave.
    // The user should adjust this delay according to the speed of the I2C bus.
    // and also select between the busy delay or task delay after the call of init function.
}
   

void SoftI2C::i2c_start() {
    SDA_output();
    SDA_high();
    SCL_high();
    _delay();
    SDA_low();
    _delay();
    SCL_low();
}    

void SoftI2C::i2c_stop() {
    SDA_output();
    SCL_low();
    SDA_low();
    _delay();
    SCL_high();
    _delay();
    SDA_high();
    //_delay();
}

    
void SoftI2C::i2c_write_bit(uint8_t bit) {
    SDA_output();
    SCL_low();
    if (bit) SDA_high(); else SDA_low();
    _delay();
    SCL_high();
    _delay();
    SCL_low();
    //_delay();
}

uint8_t SoftI2C::i2c_read_bit(void) {
    SDA_input();
    SCL_low();
    _delay();
    SCL_high();
    _delay();
    uint8_t bit = SDA_read();
    SCL_low();
    return bit;
}

uint8_t SoftI2C::i2c_write_byte(uint8_t byte) {
    //SDA_output();
    for (int i = 7; i >= 0; i--) {
        i2c_write_bit((byte >> i) & 0x01);
    }
    // ACK bit, confirmed = !ACK
    uint8_t confirmed = !i2c_read_bit(); // 1 = NACK, 0 = ACK
    return confirmed;
}

uint8_t SoftI2C::i2c_read_byte(uint8_t more) {
    uint8_t byte = 0;
    //SDA_input();
    for (int i = 7; i >= 0; i--) {
        byte <<= 1;
        byte |= i2c_read_bit();
    }
    //SDA_output();
    uint8_t ack = !more;
    i2c_write_bit(ack);  // ACK = 0(more), NACK = 1(no_more)
    return byte;
}


// high level functions

uint8_t SoftI2C::i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t val;
    taskENTER_CRITICAL();
    i2c_start();
    i2c_write_byte(dev_addr << 1);   // Write mode
    i2c_write_byte(reg_addr);
    i2c_start();
    i2c_write_byte((dev_addr << 1) | 1); // Read mode
    val = i2c_read_byte(0);          // no_more after last byte
    i2c_stop();
    taskEXIT_CRITICAL();
    return val;
}

bool SoftI2C::i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    taskENTER_CRITICAL();
    i2c_start();
    if (!i2c_write_byte(dev_addr << 1)) goto fail;
    if (!i2c_write_byte(reg_addr)) goto fail;
    if (!i2c_write_byte(data)) goto fail;  
    i2c_stop();
    taskEXIT_CRITICAL();
    return  true;
    
fail:
    i2c_stop();
    taskEXIT_CRITICAL();
    return false;
}

bool SoftI2C::i2c_read_reg_many(uint8_t dev_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length) {
    taskENTER_CRITICAL();
    i2c_start();
    if (!i2c_write_byte(dev_addr << 1)) goto fail;
    if (!i2c_write_byte(reg_addr)) goto fail;
    i2c_start();
    if (!i2c_write_byte((dev_addr << 1) | 1)) goto fail;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t more = (i < length - 1);
        buffer[i] = i2c_read_byte(more);
    }
    i2c_stop();
    taskEXIT_CRITICAL();
    return true;  // Success

fail:
    i2c_stop();
    taskEXIT_CRITICAL();
    return false;  // Failure
}

bool SoftI2C::i2c_write_reg_many(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, uint8_t length) {
    taskENTER_CRITICAL();
    i2c_start();
    if (!i2c_write_byte(dev_addr << 1)) goto fail;      // Write mode
    if (!i2c_write_byte(reg_addr)) goto fail;           // Register address
    for (uint8_t i = 0; i < length; i++) {
        if (!i2c_write_byte(data[i])) goto fail;        // Write data byte
    }
    i2c_stop();
    taskEXIT_CRITICAL();
    return true;  // Success

fail:
    i2c_stop();
    taskEXIT_CRITICAL();
    return false;  // Failure
}

