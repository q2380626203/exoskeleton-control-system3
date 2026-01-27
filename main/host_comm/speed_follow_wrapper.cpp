#include "speed_follow_wrapper.h"
#include "speed_follow_mode.h"

extern "C" {

float speed_follow_adjust_torque(void* speed_follow_ptr, bool increase) {
    if (speed_follow_ptr == nullptr) {
        return 0.0f;
    }
    SpeedFollowMode* sf = static_cast<SpeedFollowMode*>(speed_follow_ptr);
    return sf->adjustTorque(increase);
}

float speed_follow_adjust_kd(void* speed_follow_ptr, bool increase) {
    if (speed_follow_ptr == nullptr) {
        return 0.0f;
    }
    SpeedFollowMode* sf = static_cast<SpeedFollowMode*>(speed_follow_ptr);
    return sf->adjustKd(increase);
}

float speed_follow_adjust_phase2_torque(void* speed_follow_ptr, bool increase) {
    if (speed_follow_ptr == nullptr) {
        return 0.0f;
    }
    SpeedFollowMode* sf = static_cast<SpeedFollowMode*>(speed_follow_ptr);
    return sf->adjustPhase2Torque(increase);
}

float speed_follow_get_current_torque(void* speed_follow_ptr) {
    if (speed_follow_ptr == nullptr) {
        return 0.0f;
    }
    SpeedFollowMode* sf = static_cast<SpeedFollowMode*>(speed_follow_ptr);
    return sf->getCurrentTorque();
}

float speed_follow_get_current_kd(void* speed_follow_ptr) {
    if (speed_follow_ptr == nullptr) {
        return 0.0f;
    }
    SpeedFollowMode* sf = static_cast<SpeedFollowMode*>(speed_follow_ptr);
    return sf->getCurrentKd();
}

float speed_follow_get_current_phase2_torque(void* speed_follow_ptr) {
    if (speed_follow_ptr == nullptr) {
        return 0.0f;
    }
    SpeedFollowMode* sf = static_cast<SpeedFollowMode*>(speed_follow_ptr);
    return sf->getCurrentPhase2Torque();
}

} // extern "C"
