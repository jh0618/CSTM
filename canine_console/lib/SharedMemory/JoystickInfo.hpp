//
// Created by jaehoon on 23. 7. 19.
//

#ifndef CAMEL_RAISIM_PROJECTS_JOYSTICKINFO_HPP
#define CAMEL_RAISIM_PROJECTS_JOYSTICKINFO_HPP

typedef struct _AXIS_
{
    double LeftStickX;
    double LeftStickY;
    int LeftTrigger;
    double RightStickX;
    double RightStickY;
    int RightTrigger;
    int DpadX;
    int DpadY;
} AXIS, * pAXIS;

typedef struct _BUTTON_
{
    bool A;
    bool B;
    bool X;
    bool Y;
    bool LB;
    bool RB;
    bool Back;
    bool Start;
    bool Guide;
    bool LeftStick;
    bool RightStick;
} BUTTON, * pBUTTON;

typedef struct _GUI_BUTTON_
{
    int GUIButton;
} GUI_BUTTON, * pGUI_BUTTON;

enum GUI_BUTTON_TYPE
{
    GUI_NO_ACT,
    GUI_START,
    GUI_RESTART,
    GUI_RESET,
    GUI_HOME_UP,
    GUI_HOME_DOWN,
    GUI_TROT_OVERLAP,
    GUI_TROT_SLOW,
    GUI_TROT_FAST,
    GUI_TROT_STOP,
    GUI_E_STOP
};


#endif //CAMEL_RAISIM_PROJECTS_JOYSTICKINFO_HPP
