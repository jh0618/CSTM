//
// Created by jaehoon on 23. 7. 19.
//

#ifndef CAMEL_RAISIM_PROJECTS_JOYSTICKINFO_HPP
#define CAMEL_RAISIM_PROJECTS_JOYSTICKINFO_HPP

typedef struct _AXIS_
{
    int LeftStickX;
    int LeftStickY;
    int LeftTrigger;
    int RightStickX;
    int RightStickY;
    int RightTrigger;
    int DpadX;
    int DpadY;
} AXIS, * pAXIS;

typedef struct _BUTTON_
{
    int A;
    int B;
    int X;
    int Y;
    int LB;
    int RB;
    int Back;
    int Start;
    int Guide;
    int LeftStick;
    int RightStick;
} BUTTON, * pBUTTON;

typedef struct _GUI_BUTTON_
{
    int GUIButton;
} GUI_BUTTON, * pGUI_BUTTON;

enum GUI_BUTTON_TYPE
{
    GUI_NO_ACT,
    GUI_START,
    GUI_RECOVERY,
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
