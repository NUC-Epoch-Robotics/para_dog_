//
// Created by SlimeTommy on 25-10-13.
//

#ifndef POSTRUE_CONTROL_H
#define POSTRUE_CONTROL_H
#include "dog.h"
void Dog_ParaInit(Dog *dog);
void leg_Act_Bezier_init(Leg *leg,leg_state state);
void leg_Act_Bezier(Leg *leg);
void motor_Ctrl(Leg (*leg_prt)[4]);


void dogTaskCtrl(Dog *dog);
#endif //POSTRUE_CONTROL_H

