//
// Created by Tom Favereau on 03/01/2026.
//

#ifndef TD9_SPRINGLINK_H
#define TD9_SPRINGLINK_H

struct SpringLink
{
    int groupId    = -1;
    int aNode      = -1;
    int bNode      = -1;
    float restLength = 0.f;
    float stiffness  = 0.9f;
};

#endif //TD9_SPRINGLINK_H
