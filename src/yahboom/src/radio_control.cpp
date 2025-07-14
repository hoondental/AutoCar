#include "radio_control.h"


void RC::begin() { 
    _sbus.Begin(); 
}


bool RC::read() { 
    uint32_t now = millis();
    if (_sbus.Read()){
        _sbus_data = _sbus.data();    
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