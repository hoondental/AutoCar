#ifndef MPU9250_H
#define MPU9250_H

#define MPU9250_ID 0x71
#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C

#define MPU_INT_EN_REG              0X38        //  Interrupt enable register
#define MPU_USER_CTRL_REG           0X6A        //  User control register
#define MPU_FIFO_EN_REG             0X23        //  FIFO enable register
#define MPU_PWR_MGMT1_REG           0X6B        //  Power management register 2
#define MPU_PWR_MGMT2_REG           0X6C        //  Power management register 2
#define MPU_GYRO_CFG_REG            0X1B        //  Gyroscope configuration register
#define MPU_ACCEL_CFG_REG           0X1C        //  Accelerometer configuration register
#define MPU_CFG_REG                 0X1A        //  Configuration register
#define MPU_SAMPLE_RATE_REG         0X19        //  Sampling frequency divider
#define MPU_INTBP_CFG_REG           0X37        //  Interrupt/bypass setting register
#define MPU_WHO_AM_I_REG	    	0X75

#define MPU_TEMP_OUT_H_REG		0X41	
#define MPU_TEMP_OUT_L_REG		0X42

#define MAG_WHO_AM_I_REG		0x00
#define AK8963_ID				0X48
#define MAG_CNTL1_REG     	  	0X0A    
#define MAG_CNTL2_REG          	0X0B
#define MAG_XOUT_L_REG			0X03	
#define MAG_XOUT_H_REG			0X04
#define MAG_ASA_REG             0x10



#define MPU_SMPLRT_DIV_REG          0x19        //  Sampling frequency divider
#define MPU_I2C_MST_CTRL_REG		0x24
#define MPU_SLV0_ADDR_REG			0x25
#define MPU_SLV0_REG				0x26
#define MPU_SLV0_CTRL_REG			0x27
#define MPU_SLV0_DO_REG				0x63
#define MPU_ACCEL_XOUT_H_REG		0x3B
#define MPU_GYRO_XOUT_H_REG			0x43
#define MPU_EXT_SENS_DATA_00        0x49



#include <Arduino.h>
#include "softI2C.h"


struct AccelData {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct AccelData_f {
    float_t x = 0.0;
    float_t y = 0.0;
    float_t z = 0.0;
};


struct GyroData
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct GyroData_f
{
    float_t x = 0.0;
    float_t y = 0.0;
    float_t z = 0.0;
};


struct MagData
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct MagData_f
{
    float_t x = 0.0;
    float_t y = 0.0;
    float_t z = 0.0;
};


struct MPUData
{
    AccelData accel;
    GyroData gyro;
    MagData mag;
    int16_t temp = 0;
};


struct MPUData_f
{
    AccelData_f accel;
    GyroData_f gyro;
    MagData_f mag;
    float_t temp = 0.0;
};





// ============================ I2C version ========================
class MPU9250I2C
{
private:
    MPUData _data;
    MPUData_f _data_f;
    SoftI2C _i2c;
    GPIO_TypeDef* _port_ad0;
    uint16_t _pin_ad0;

    bool writeByte(uint8_t dev, uint8_t reg, uint8_t val) { return _i2c.i2c_write_reg(dev, reg, val); }
    bool readBytes(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t len) { return _i2c.i2c_read_reg_many(dev, reg, buf, len); }
    
    bool set_sampling_rate(uint16_t rate_Hz);
    bool initAK8963(uint16_t rate_Hz);
    bool i2cBypassEnable();
    float _accScale = 16384.0;   // for ±2g
    float _gyroScale = 131.0;    // for ±250°/s
    float _magScale_x = 0.15f;     // µT/LSB
    float _magScale_y = 0.15f;     // µT/LSB
    float _magScale_z = 0.15f;     // µT/LSB
    uint8_t _asa_x, _asa_y, _asa_z;

    // singleton
    MPU9250I2C() = delete;    
    MPU9250I2C(const MPU9250I2C&) = delete;
    MPU9250I2C(GPIO_TypeDef* port_sda, uint16_t pin_sda, GPIO_TypeDef* port_scl, uint16_t pin_scl, 
               GPIO_TypeDef* port_ad0, uint16_t pin_ad0, uint32_t i2c_speed_khz) 
        : _i2c(port_sda, pin_sda, port_scl, pin_scl, i2c_speed_khz)
    { _port_ad0 = port_ad0; _pin_ad0 = pin_ad0; }
    MPU9250I2C& operator=(const MPU9250I2C&) = delete;

public:
    static MPU9250I2C& getInstance(GPIO_TypeDef* port_sda, uint16_t pin_sda, 
                                   GPIO_TypeDef* port_scl, uint16_t pin_scl, 
                                   GPIO_TypeDef* port_ad0, uint16_t pin_ad0, uint32_t i2c_speed_khz) {
        static MPU9250I2C instance = MPU9250I2C(port_sda, pin_sda, port_scl, pin_scl, port_ad0, pin_ad0, i2c_speed_khz);
        return instance;
    }
    
    bool begin(int gyro_range_dps = 250, int accel_range_g = 2, 
               uint16_t mpu_dataRate_Hz = 200, uint16_t mag_dataRate_Hz = 100);
    bool readAccel();
    bool readGyro();
    bool readMag();
    bool readTemp();
    inline bool read() { return (readAccel() && readGyro() && readMag() && readTemp()); }
    inline const MPUData& data() const { return _data; }
    inline const AccelData& accel() const { return _data.accel; }
    inline const GyroData& gyro() const { return _data.gyro; }
    inline const MagData& mag() const { return _data.mag; }
    inline int16_t temp() { return _data.temp; }
    inline const MPUData_f& data_f() const { return _data_f; }
    inline const AccelData_f& accel_f() const { return _data_f.accel; }
    inline const GyroData_f& gyro_f() const { return _data_f.gyro; }
    inline const MagData_f& mag_f() const { return _data_f.mag; }
    inline float_t temp_f() { return _data_f.temp; }
};






// ============================ SPI version ========================

class MPU9250SPI
{
private:
    MPUData _data;
    SPI_HandleTypeDef _hspi;
    GPIO_TypeDef *_port_sdi, *_port_sdo, *_port_sclk, *_port_cs;
    uint16_t _pin_sdi, _pin_sdo, _pin_sclk, _pin_cs;
    

    int MX_GPIO_Init();
    int MX_SPI2_Init();
    int MPU9250_Init(int gyro_range_dps, int accel_range_g);
    int set_sampling_rate(uint16_t rate_Hz);
    int initAK8963(uint16_t rate_Hz);

    inline void CS_low() { HAL_GPIO_WritePin(_port_cs, _pin_cs, GPIO_PIN_RESET); }
    inline void CS_high() { HAL_GPIO_WritePin(_port_cs, _pin_cs, GPIO_PIN_SET); }

    bool SPI_write(uint8_t reg, uint8_t data, uint32_t timeout_ms = 5);
    uint8_t SPI_read(uint8_t reg, uint32_t timeout_ms = 5);
    bool SPI_read_many(uint8_t reg, uint8_t* buf, uint8_t len, uint32_t timeout_ms = 10);
    


    
    
    float _accScale = 16384.0;   // for ±2g
    float _gyroScale = 131.0;    // for ±250°/s
    const float _magScale = 0.15f;     // µT/LSB

    // singleton
    MPU9250SPI() = delete;    
    MPU9250SPI(const MPU9250SPI&) = delete;
    MPU9250SPI(GPIO_TypeDef* port_sdi, uint16_t pin_sdi, GPIO_TypeDef* port_sdo, uint16_t pin_sdo, 
               GPIO_TypeDef* port_sclk, uint16_t pin_sclk, GPIO_TypeDef* port_cs, uint16_t pin_cs) {
        _port_sdi = port_sdi; _pin_sdi = pin_sdi; 
        _port_sdo = port_sdo; _pin_sdo = pin_sdo; 
        _port_sclk = port_sclk; _pin_sclk = pin_sclk; 
        _port_cs = port_cs; _pin_cs = pin_cs; 
    }
    MPU9250SPI& operator=(const MPU9250SPI&) = delete;

public:
    static MPU9250SPI& getInstance(GPIO_TypeDef* port_sdi, uint16_t pin_sdi, GPIO_TypeDef* port_sdo, uint16_t pin_sdo, 
               GPIO_TypeDef* port_sclk, uint16_t pin_sclk, GPIO_TypeDef* port_cs, uint16_t pin_cs) {
        static MPU9250SPI instance = MPU9250SPI(port_sdi, pin_sdi, port_sdo, pin_sdo, port_sclk, pin_sclk, port_cs, pin_cs);
        return instance;
    }
    
    int begin(int gyro_range_dps = 250, int accel_range_g = 2, 
               uint16_t mpu_dataRate_Hz = 200, uint16_t mag_dataRate_Hz = 100);
    bool readAccel();
    bool readGyro();
    bool readMag();
    bool readTemp();
    inline bool read() { return (readAccel() && readGyro() && readMag() && readTemp()); }
    inline const MPUData& data() const { return _data; }
    inline const AccelData& accel() const { return _data.accel; }
    inline const GyroData& gyro() const { return _data.gyro; }
    inline const MagData& mag() const { return _data.mag; }
    inline int16_t temp() { return _data.temp; }
};


#endif
