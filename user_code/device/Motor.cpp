#include "motor.h"

#include "can_receive.h"

void m3508_motor_c::init(motor_measure_t *motor_measure_) {
    motor_measure = motor_measure_;
}
