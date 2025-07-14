#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f1xx_hal.h"




class SoftI2C {
private:
    GPIO_TypeDef* _port_sda;
    GPIO_TypeDef* _port_scl;
    uint16_t _pin_sda;
    uint16_t _pin_scl;
    uint32_t _speed_khz;
    uint32_t _delay_cycles;

    void dwt_init();
    void _delay();
    //void SDA_high() { HAL_GPIO_WritePin(_port_sda, _pin_sda, GPIO_PIN_SET); }
    void SDA_high() { HAL_GPIO_WritePin(_port_sda, _pin_sda, GPIO_PIN_SET); }// release the line 
    void SDA_low() { HAL_GPIO_WritePin(_port_sda, _pin_sda, GPIO_PIN_RESET); }
    void SCL_high() {
        HAL_GPIO_WritePin(_port_scl, _pin_scl, GPIO_PIN_SET); // let it float
        // You can read the pin if SCL is configured as GPIO_MODE_OUTPUT_OD
        while (HAL_GPIO_ReadPin(_port_scl, _pin_scl) == GPIO_PIN_RESET);
    }
    void SCL_low() { HAL_GPIO_WritePin(_port_scl, _pin_scl, GPIO_PIN_RESET); }
    uint8_t SDA_read() { return HAL_GPIO_ReadPin(_port_sda, _pin_sda); }
    void SDA_input();    
    void SDA_output();
    void SCL_output();

public:
    SoftI2C(GPIO_TypeDef* port_sda, uint16_t pin_sda, GPIO_TypeDef* port_scl, uint16_t pin_scl, uint32_t speed_khz);
    void begin();  
    // basic functions
    void i2c_start();  
    void i2c_stop();    
    void i2c_write_bit(uint8_t bit);
    uint8_t i2c_read_bit(void);
    uint8_t i2c_write_byte(uint8_t byte);
    uint8_t i2c_read_byte(uint8_t more);
    // high-level functions
    uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr);
    bool i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
    bool i2c_read_reg_many(uint8_t dev_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length);
    bool i2c_write_reg_many(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, uint8_t length);
};


#endif  // SOFT_I2C_H
