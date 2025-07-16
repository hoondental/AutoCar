#include "radio_control.h"


void RC::begin() { 
    _sbus.Begin(); 
}


bool RC::read() { 
    uint32_t now = millis();
    if (_sbus.Read()){
        _sbus_data = _sbus.data();    
        for (int i=0; i<16; i++) {
            _channels_f[i] = (float(_sbus_data.ch[i]) - _sbus_zero) / _sbus_half_range;
            _channels_uf[i] = (float(_sbus_data.ch[i]) - _sbus_min) / _sbus_range;
        }
        _last_read_ms = now;
        if (_sbus_data.failsafe) {
            _failsafe_time_ms = now - _last_non_failsafe_ms;
        } else {
            _last_non_failsafe_ms = now;
            _failsafe_time_ms = 0;
        }           
        _lost_signal_time_ms = 0;                 
        return true;
    } else {
        _lost_signal_time_ms = now - _last_read_ms;
        if (_sbus_data.failsafe) _failsafe_time_ms = now - _last_non_failsafe_ms;
        return false;
    }
}