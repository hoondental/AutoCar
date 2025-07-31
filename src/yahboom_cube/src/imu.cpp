#include "imu.h"



// ================================ I2C version ================================

bool MPU9250I2C::begin(int gyro_range_dps, int accel_range_g, uint16_t mpu_dataRate_Hz, uint16_t mag_dataRate_Hz) {
    _i2c.begin();

    // pull down AD0 pin of MPU9250
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = _pin_ad0,
        .Mode = GPIO_MODE_OUTPUT_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(_port_ad0, &GPIO_InitStruct);
    HAL_GPIO_WritePin(_port_ad0, _pin_ad0, GPIO_PIN_RESET);

    HAL_Delay(100);

    // reset MPU9250
    if (!writeByte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0x80)) return false; // reset
    HAL_Delay(100);
    if (!writeByte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0x00)) return false; // Wake up MPU
    HAL_Delay(10);

    // Set Gyroscope range
    float_t _degree_to_radian = M_PI / 180.0;
    uint8_t gyroConfig;
    switch (gyro_range_dps) {
        case 250:  gyroConfig = 0x00; _gyroScale = 131.0 / _degree_to_radian; break;
        case 500:  gyroConfig = 0x08; _gyroScale = 65.5 / _degree_to_radian; break;
        case 1000: gyroConfig = 0x10; _gyroScale = 32.8 / _degree_to_radian; break;
        case 2000: gyroConfig = 0x18; _gyroScale = 16.4 / _degree_to_radian; break;
        default:   return false; // Invalid input
    }
    if (!writeByte(MPU_ADDR, MPU_GYRO_CFG_REG, gyroConfig)) return false;

    // Set Accelerometer range
    float_t _g_to_m_s2 = 9.8;
    uint8_t accelConfig;
    switch (accel_range_g) {
        case 2:  accelConfig = 0x00; _accScale = 16384.0 / _g_to_m_s2; break;
        case 4:  accelConfig = 0x08; _accScale = 8192.0 / _g_to_m_s2; break;
        case 8:  accelConfig = 0x10; _accScale = 4096.0 / _g_to_m_s2; break;
        case 16: accelConfig = 0x18; _accScale = 2048.0 / _g_to_m_s2; break;
        default: return false; // Invalid input
    }

    if (!writeByte(MPU_ADDR, MPU_ACCEL_CFG_REG, accelConfig)) return false;

    if (!set_sampling_rate(mpu_dataRate_Hz)) return false;

    if (!i2cBypassEnable()) return false;
    HAL_Delay(10);
    if (!initAK8963(mag_dataRate_Hz)) return false;
    return true;
}


bool MPU9250I2C::set_sampling_rate(uint16_t rate)
{
	uint8_t data;
	if (rate>1000) rate=1000;
	if (rate<4) rate=4;
	data = 1000 / rate - 1;
	data = writeByte(MPU_ADDR, MPU_SAMPLE_RATE_REG, data);	

    // set LPF: low pass filter
    uint8_t lpf = data / 2;
    if (lpf >=  188)data = 1;
	else if (lpf>=98) data = 2;
	else if (lpf>=42) data = 3;
	else if (lpf>=20) data = 4;
	else if (lpf>=10) data = 5;
	else data = 6; 
	return writeByte(MPU_ADDR, MPU_CFG_REG, data);
}

bool MPU9250I2C::i2cBypassEnable(){ 
    // turn off all interrupts
    //writeByte(MPU_ADDR, MPU_INT_EN_REG, 0x00);
    // i2c main mode is off
    writeByte(MPU_ADDR, MPU_USER_CTRL_REG, 0x00);
    // close the FIFO
    //writeByte(MPU_ADDR, MPU_FIFO_EN_REG, 0x00);
    // bypass mode
    return writeByte(MPU_ADDR, MPU_INTBP_CFG_REG, 0x02); }// INT_PIN_CFG: BYPASS_EN

bool MPU9250I2C::initAK8963(uint16_t rate_Hz) {
    // CHECK id
    uint8_t res;
    readBytes(MAG_ADDR, MAG_WHO_AM_I_REG, &res, 1);
    if (res != AK8963_ID) return false;

    // reset
    if (!writeByte(MAG_ADDR, MAG_CNTL2_REG, 0x01)) return false;

    // Power down mag
    writeByte(MAG_ADDR, MAG_CNTL1_REG, 0x00);
    HAL_Delay(10);

    // Enter Fuse ROM access mode to read sensitivity adjustment
    writeByte(MAG_ADDR, MAG_CNTL1_REG, 0x0F);
    HAL_Delay(10);

    // Read ASA calibration values (registers 0x10–0x12)
    uint8_t asa[3];
    readBytes(MAG_ADDR, MAG_ASA_REG, asa, 3);
    _asa_x = asa[0];
    _asa_y = asa[1];
    _asa_z = asa[2];
    float_t _mag_adjust_x = float(_asa_x - 128) / 256.0 + 1.0;
    float_t _mag_adjust_y = float(_asa_y - 128) / 256.0 + 1.0;
    float_t _mag_adjust_z = float(_asa_z - 128) / 256.0 + 1.0;

    _magScale_x *= _mag_adjust_x;
    _magScale_y *= _mag_adjust_y;
    _magScale_z *= _mag_adjust_z;

    // Power down again before setting continuous mode
    writeByte(MAG_ADDR, MAG_CNTL1_REG, 0x00);
    HAL_Delay(10);

    // Set data rate
    uint8_t rateConfig = 0x16; // default to 8 Hz
    switch (rate_Hz) {
        case 8: rateConfig = 0x12; break; // 8 Hz
        case 100: rateConfig = 0x16; break; // 100 Hz
        //case 0: rateConfig = 0x18; break; // continuous mode
        default: return false; // invalid data rate, return false
    }

    // Set 16-bit, continuous mode 1 (selected data rate)
    return writeByte(MAG_ADDR, MAG_CNTL1_REG, rateConfig);
}

bool MPU9250I2C::readAccel() {    
    uint8_t buf[6];
    if (!readBytes(MPU_ADDR, 0x3B, buf, 6)) return false;
    _data.accel.x = (int16_t)((buf[0] << 8) | buf[1]);
    _data.accel.y = (int16_t)((buf[2] << 8) | buf[3]);
    _data.accel.z = (int16_t)((buf[4] << 8) | buf[5]);
    _data_f.accel.x = float(_data.accel.x) / _accScale;
    _data_f.accel.y = float(_data.accel.y) / _accScale;
    _data_f.accel.z = float(_data.accel.z) / _accScale;
    return true;
}

bool MPU9250I2C::readGyro() {
    uint8_t buf[6];
    if (!readBytes(MPU_ADDR, 0x43, buf, 6)) return false;
    _data.gyro.x = (int16_t)((buf[0] << 8) | buf[1]);
    _data.gyro.y = (int16_t)((buf[2] << 8) | buf[3]);
    _data.gyro.z = (int16_t)((buf[4] << 8) | buf[5]);
    _data_f.gyro.x = float(_data.gyro.x) / _gyroScale;
    _data_f.gyro.y = float(_data.gyro.y) / _gyroScale;
    _data_f.gyro.z = float(_data.gyro.z) / _gyroScale;
    return true;
}

bool MPU9250I2C::readMag() {
    uint8_t st1;
    //writeByte(MAG_ADDR, 0x0A, 0x11);
    //delay(10);
    readBytes(MAG_ADDR, 0x02, &st1, 1);
    if (!(st1 & 0x01)) return false;

    uint8_t buf[7];
    //if (!readBytes(MAG_ADDR, 0x03, buf, 7)) return false;
    readBytes(MAG_ADDR, MAG_XOUT_L_REG, buf, 7);
    _data.mag.x = (int16_t)((buf[1] << 8) | buf[0]);
    _data.mag.y = (int16_t)((buf[3] << 8) | buf[2]);
    _data.mag.z = (int16_t)((buf[5] << 8) | buf[4]);
    _data_f.mag.x = float(_data.mag.x) * _magScale_x;
    _data_f.mag.y = float(_data.mag.y) * _magScale_y;
    _data_f.mag.z = float(_data.mag.z) * _magScale_z;
    return true;
}

bool MPU9250I2C::readTemp() {
    uint8_t buf[2];
    if (!readBytes(MPU_ADDR, MPU_TEMP_OUT_H_REG, buf, 2)) return false;
    _data.temp = (int16_t)((buf[0] << 8) | buf[1]);
    _data_f.temp = float(_data.temp) / 333.87 + 21.0;
    return true;
}




// ============================= SPI version =============================
// board 의 PB12 와 MPU9250 의 nCS 핀이 연결되어 있고, 
// pull-down 저항이 없어서 항상 I2C 로 설정되는 것으로 보임
// SPI 에서 인식 안됨. 




int MPU9250SPI::MX_GPIO_Init() {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = _pin_cs;  // CS pin
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, _pin_cs, GPIO_PIN_SET);  // CS high
    return 0;
}


int MPU9250SPI::MX_SPI2_Init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();

    _hspi.Instance = SPI2;
    _hspi.Init.Mode = SPI_MODE_MASTER;
    _hspi.Init.Direction = SPI_DIRECTION_2LINES;
    _hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    _hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    _hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    _hspi.Init.NSS = SPI_NSS_SOFT;
    _hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    _hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    _hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    _hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    _hspi.Init.CRCPolynomial = 10;
    HAL_StatusTypeDef status = HAL_SPI_Init(&_hspi);
    if (status != HAL_OK) return -1;
    return 0;
}




int MPU9250SPI::MPU9250_Init(int gyro_range_dps, int accel_range_g)
{
    HAL_Delay(100);
    SPI_write(MPU_PWR_MGMT1_REG, 0x00); // Wake up device
    HAL_Delay(10);
    SPI_write(MPU_PWR_MGMT1_REG, 0x01); // Auto clock

    HAL_Delay(10);
    uint8_t whoami = SPI_read(MPU_WHO_AM_I_REG);
    if (whoami != MPU9250_ID) return -1;

    // Set Gyroscope range
    uint8_t gyroConfig;
    switch (gyro_range_dps) {
        case 250:  gyroConfig = 0x00; _gyroScale = 131.0; break;
        case 500:  gyroConfig = 0x08; _gyroScale = 65.5; break;
        case 1000: gyroConfig = 0x10; _gyroScale = 32.8; break;
        case 2000: gyroConfig = 0x18; _gyroScale = 16.4; break;
        default:   return false; // Invalid input
    }
    if (!SPI_write(MPU_GYRO_CFG_REG, gyroConfig)) return -1;  

    // Set Accelerometer range
    uint8_t accelConfig;
    switch (accel_range_g) {
        case 2:  accelConfig = 0x00; _accScale = 16384.0; break;
        case 4:  accelConfig = 0x08; _accScale = 8192.0; break;
        case 8:  accelConfig = 0x10; _accScale = 4096.0; break;
        case 16: accelConfig = 0x18; _accScale = 2048.0; break;
        default: return false; // Invalid input
    }
    if  (!SPI_write(MPU_ACCEL_CFG_REG, 0x00)) return -1; 
    return 0;
}


int MPU9250SPI::set_sampling_rate(uint16_t rate)
{
	uint8_t data;
	if (rate>1000) rate=1000;
	if (rate<4) rate=4;
	data = 1000 / rate - 1;
	data = SPI_write(MPU_SMPLRT_DIV_REG, data);	

    // set LPF: low pass filter
    uint8_t lpf = data / 2;
    if (lpf >=  188)data = 1;
	else if (lpf>=98) data = 2;
	else if (lpf>=42) data = 3;
	else if (lpf>=20) data = 4;
	else if (lpf>=10) data = 5;
	else data = 6; 
	if (SPI_write(MPU_CFG_REG, data)) {
        return 0;
    } else {
        return -1;
    };
}


int MPU9250SPI::initAK8963(uint16_t rate_Hz) {
    // Enable I2C master mode
    SPI_write(MPU_USER_CTRL_REG, 0x20);  // enable I2C master
    SPI_write(MPU_I2C_MST_CTRL_REG, 0x0D); // I2C master clock 400kHz
    HAL_Delay(10);

    // Configure magnetometer
    HAL_Delay(10);
    // Step 1: Power down AK8963 first
    SPI_write(MPU_SLV0_ADDR_REG, MAG_ADDR);       // Write to AK8963 (ADDR=0x0C)
    SPI_write(MPU_SLV0_REG, MAG_CNTL1_REG);          // Register 0x0A
    SPI_write(MPU_SLV0_DO_REG, 0x00);                   // Power down
    SPI_write(MPU_SLV0_CTRL_REG, 0x81);                 // Enable write of 1 byte
    HAL_Delay(10);

    // Step 2: Set desired mode (e.g., 100Hz Continuous Mode)
    SPI_write(MPU_SLV0_ADDR_REG, MAG_ADDR);       // Write to AK8963
    SPI_write(MPU_SLV0_REG, MAG_CNTL1_REG);          // CNTL1 register
    SPI_write(MPU_SLV0_DO_REG, 0x16);                   // 0x12=8Hz, 0x16=100Hz
    SPI_write(MPU_SLV0_CTRL_REG, 0x81);                 // Enable write
    HAL_Delay(10);   

    // Request AK8963 WHO_AM_I (0x00)
    SPI_write(MPU_SLV0_ADDR_REG, 0x80 | MAG_ADDR);
    SPI_write(MPU_SLV0_REG, MAG_WHO_AM_I_REG);
    SPI_write(MPU_SLV0_CTRL_REG, 0x81);
    HAL_Delay(10);
    uint8_t who_am_i = SPI_read(MPU_EXT_SENS_DATA_00);  // Should return 0x48
    if (who_am_i != 0x48) return -1;


    // setup AK8963 read
    SPI_write(MPU_SLV0_ADDR_REG, 0x80 | MAG_ADDR); // I2C slave address
    SPI_write(MPU_SLV0_REG, MAG_XOUT_L_REG); // start at data register
    SPI_write(MPU_SLV0_CTRL_REG, 0x87); // read 7 bytes
    // this causes MPU9250 to automatically read 7 bytes from AK8963 and save it to EXT_SENS_DATA_00 ~ 06
    return 0;
}

int MPU9250SPI::begin(int gyro_range_dps, int accel_range_g, uint16_t mpu_dataRate_Hz, uint16_t mag_dataRate_Hz) {
    if (MX_GPIO_Init() != 0) return -1; 
    if (MX_SPI2_Init() != 0) return -2;
    if (MPU9250_Init(gyro_range_dps, accel_range_g) != 0) return -3; //MPU9250_Init(gyro_range_dps, accel_range_g);
    if (initAK8963(mag_dataRate_Hz) != 0) return -4;
    if (set_sampling_rate(mpu_dataRate_Hz) != 0) return -5;

    HAL_Delay(10);
    if (initAK8963(mag_dataRate_Hz) != 0) return -6;
    return 0;
}


bool MPU9250SPI::SPI_write(uint8_t reg, uint8_t data, uint32_t timeout_ms) {
    uint8_t tx[2] = {reg & 0x7F, data};
    CS_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&_hspi, tx, 2, timeout_ms);
    CS_high();
    return status == HAL_OK;
}

uint8_t MPU9250SPI::SPI_read(uint8_t reg, uint32_t timeout_ms) {
    uint8_t tx = reg | 0x80;
    uint8_t rx;
    HAL_StatusTypeDef status;
    CS_low();
    status = HAL_SPI_Transmit(&_hspi, &tx, 1, timeout_ms);
    if (status == HAL_OK) status = HAL_SPI_Receive(&_hspi, &rx, 1, timeout_ms);
    CS_high();
    return rx;
}
 
bool MPU9250SPI::SPI_read_many(uint8_t reg, uint8_t* buf, uint8_t len, uint32_t timeout_ms) {
    uint8_t tx = reg | 0x80;
    HAL_StatusTypeDef status;
    CS_low();
    status = HAL_SPI_Transmit(&_hspi, &tx, 1, timeout_ms);
    if (status == HAL_OK) status = HAL_SPI_Receive(&_hspi, buf, len, timeout_ms);
    CS_high();
    return status == HAL_OK;
};


bool MPU9250SPI::readAccel() {    
    uint8_t buf[6];
    if (SPI_read_many(MPU_ACCEL_XOUT_H_REG, buf, 6)) {
        _data.accel.x = (int16_t)((buf[0] << 8) | buf[1]);
        _data.accel.y = (int16_t)((buf[2] << 8) | buf[3]);
        _data.accel.z = (int16_t)((buf[4] << 8) | buf[5]);
        return true;
    } else {
        return false;
    }
}

bool MPU9250SPI::readGyro() {
    uint8_t buf[6];
    if (SPI_read_many(MPU_GYRO_XOUT_H_REG, buf, 6)) {
        _data.gyro.x = (int16_t)((buf[0] << 8) | buf[1]);
        _data.gyro.y = (int16_t)((buf[2] << 8) | buf[3]);
        _data.gyro.z = (int16_t)((buf[4] << 8) | buf[5]);
        return true;
    }
    else {
        return false;
    }
}

bool MPU9250SPI::readMag() {
    uint8_t buf[7];
    if (SPI_read_many(MPU_EXT_SENS_DATA_00, buf, 7)) {
        _data.mag.x = (int16_t)((buf[1] << 8) | buf[0]);
        _data.mag.y = (int16_t)((buf[3] << 8) | buf[2]);
        _data.mag.z = (int16_t)((buf[5] << 8) | buf[4]);
        return true;
    } else {
        return false;   
    }
}

bool MPU9250SPI::readTemp() {
    uint8_t buf[2];
    if (SPI_read_many(MPU_TEMP_OUT_H_REG, buf, 2)) {
        _data.temp = (int16_t)((buf[0] << 8) | buf[1]);
        return true;
    } else {
        return false;   
    }
}
