#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "remote_control.h"
#include ""

extern remote_control_c remote_control;
extern can_receive_c can_receive;

class communicat
{
public:
    void init();


};





#endif