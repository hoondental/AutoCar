#include <Arduino.h>
#include <sbus.h>

// RC command
#define RC_CHANNEL_MODE 8
#define RC_CHANNEL_LOGNITUDINAL 1
#define RC_CHANNEL_LATERAL 2
#define RC_CHANNEL_YAW 3


const bool SBUS_SIGNAL_INVERTED = false;
const bool USE_D16_MODE = true;



class RC {
private:
    bfs::SbusRx _sbus;
    bfs::SbusData _sbus_data;
    float_t _channels_f[16];
    float_t _channels_uf[16];
    uint32_t _last_read_ms = 0;
    uint32_t _lost_signal_time_ms = 0;
    uint32_t _last_non_failsafe_ms = 0;
    uint32_t _failsafe_time_ms = 0;
    static const uint32_t _sbus_min = 172;
    static const uint32_t _sbus_max = 1811;
    static const uint32_t _sbus_zero = 992;
    static const uint32_t _sbus_range = 1639;
    static const uint32_t _sbus_half_range = 820;
    static const uint32_t _sbus_one_third = 718;
    static const uint32_t _sbus_two_third = 1265;

    /*
    inline uint16_t sbus_to_pwm_us(int16_t sbus_val) {
        return (uint16_t)(((sbus_val - 172) * 1000) / (1811 - 172) + 1000);
    }
    inline int16_t sbus_to_switch_pos(int16_t sbus_val, uint16_t num_positions) {
        if (num_positions == 1) return 0;
        int16_t half_interval = 820 / (num_positions - 1); // 820 = (1811 - 172) / 2
        return (int16_t)(map(sbus_val, 172 - half_interval, 1811 - half_interval, 0, num_positions - 1));
    }
    */

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
    inline int16_t channel(size_t channel) { return _sbus_data.ch[channel]; }
    inline uint32_t lost_time_ms() { return max(_lost_signal_time_ms, _failsafe_time_ms); }
    // -1.0 ~ 1.0
    inline const float_t* channels_f() const { return _channels_f; }
    inline float_t channel_f(size_t channel) { return _channels_f[channel]; }
    // 0.0 ~ 1.0
    inline const float_t* channels_uf() const { return _channels_uf; }
    inline const float_t channel_uf(size_t channel) const { return _channels_uf[channel]; }

    // 2-step switch
    inline uint32_t switch_2(size_t channel) {
        if (_sbus_data.ch[channel] < _sbus_half_range) {
            return 0;
        } else {
            return 1;
        }
    }

    // 3-step switch
    inline uint32_t switch_3(size_t channel) {
        if (_sbus_data.ch[channel] < _sbus_one_third) {
            return 0;
        } else if (_sbus_data.ch[channel] < _sbus_two_third) {
            return 1;
        } else {
            return 2;
        }
    }
    
};


enum RC_MODE {
    RC_RAW_PWM, 
    RC_RAW_SPEED,
    RC_NONE
};






