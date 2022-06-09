#ifndef COMMUNICATe_H
#define COMMUNICATe_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"

#include "Config.h"

class Communicate
{
public:
    void init();

    void run();
};

extern Remote_control remote_control;
extern Communicate communicate;

#endif