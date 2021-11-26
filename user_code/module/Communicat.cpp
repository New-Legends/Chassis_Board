#include "communicat.h"

remote_control_c remote_control;
can_receive_c can_receive;

void communicat_c::init() {
    remote_control.init();
    can_receive.init();

}