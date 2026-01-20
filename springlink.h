//
// Created by Tom Favereau on 03/01/2026.
//

#ifndef SOLVER_SPRINGLINK_H
#define SOLVER_SPRINGLINK_H

struct SpringLink
{
    int groupId    = -1;
    int aNode      = -1;
    int bNode      = -1;
    float restLength = 0.f;
    float stiffness  = 0.9f;
};

#endif //SOLVER_SPRINGLINK_H
