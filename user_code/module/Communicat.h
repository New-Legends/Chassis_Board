#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"



class Communicat
{
public:
    void init();


};

extern Remote_control remote_control;
extern Can_receive can_receive;

extern Communicat communicat;

#endif