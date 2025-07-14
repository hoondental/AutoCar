#include <Arduino.h>
#include <sbus.h>

const bool SBUS_SIGNAL_INVERTED = false;
const bool USE_D16_MODE = true;



class RC {
private:
    bfs::SbusRx _sbus;
    bfs::SbusData _sbus_data;
    uint32_t _last_read_ms = 0;
    uint32_t _lost_signal_time_ms = 0;
    uint32_t _last_non_failsafe_ms = 0;
    uint32_t _failsafe_time_ms = 0;

    inline uint16_t sbus_to_pwm_us(int16_t sbus_val) {
        return (uint16_t)(((sbus_val - 172) * 1000) / (1811 - 172) + 1000);
    }
    inline int16_t sbus_to_switch_pos(int16_t sbus_val, uint16_t num_positions) {
        if (num_positions == 1) return 0;
        int16_t half_interval = 820 / (num_positions - 1); // 820 = (1811 - 172) / 2
        return (int16_t)(map(sbus_val, 172 - half_interval, 1811 - half_interval, 0, num_positions - 1));
    }

    // singleton 
    RC() = delete;
    RC(const RC&) = delete;
    RC(HardwareSerial* serial, bool inverted, bool fast)
        : _sbus(serial, inverted, fast)
    {
        //serial->begin(100000, SERIAL_8E2);
        //*_sbus = bfs::SbusRx(serial, true, false);        
        //_sbus.Begin(); 
    }
    RC& operator=(const RC&) = delete;    

public:
    static RC& getInstance(HardwareSerial* serial, bool inverted, bool fast = false) {
        static RC instance = RC(serial, inverted, fast);
        return instance;
    }

    void begin();
    bool read();
    inline const bfs::SbusData& data() const { return _sbus_data; }
    inline bool lost_frame() { return _sbus_data.lost_frame; }
    inline bool failsafe() { return _sbus_data.failsafe; }
    inline const int16_t* channels() const { return _sbus_data.ch; }
    inline int16_t channel(uint8_t channel) { return _sbus_data.ch[channel]; }
    inline uint32_t lost_time_ms() { return max(_lost_signal_time_ms, _failsafe_time_ms); }

    
};




