//
// Created by hs on 23. 2. 9.
//
#include <ControlMain/HighControlMain.hpp>


HighControlMain::HighControlMain()
    : mStand(MPC_HORIZON, int(0.1 / HIGH_CONTROL_dT), Vec4<int>(int(0.1 / HIGH_CONTROL_dT), int(0.1 / HIGH_CONTROL_dT), int(0.1 / HIGH_CONTROL_dT), int(0.1 / HIGH_CONTROL_dT)), Vec4<int>(int(0.1 / HIGH_CONTROL_dT), int(0.1 / HIGH_CONTROL_dT), int(0.1 / HIGH_CONTROL_dT), int(0.1 / HIGH_CONTROL_dT)))
    , mTrotSlow(MPC_HORIZON, int(0.6 / HIGH_CONTROL_dT), Vec4<int>(int(0.3 / HIGH_CONTROL_dT), 0, 0, int(0.3 / HIGH_CONTROL_dT)), Vec4<int>(int(0.34 / HIGH_CONTROL_dT), int(0.34 / HIGH_CONTROL_dT), int(0.34 / HIGH_CONTROL_dT), int(0.34 / HIGH_CONTROL_dT)))
    , mTrotFast(MPC_HORIZON, int(0.4 / HIGH_CONTROL_dT), Vec4<int>(int(0.2 / HIGH_CONTROL_dT), 0, 0, int(0.2 / HIGH_CONTROL_dT)), Vec4<int>(int(0.2 / HIGH_CONTROL_dT), int(0.2 / HIGH_CONTROL_dT), int(0.2 / HIGH_CONTROL_dT), int(0.2 / HIGH_CONTROL_dT)))
    , mOverlapTrotFast(MPC_HORIZON, int(0.4 / HIGH_CONTROL_dT), Vec4<int>(int(0.2 / HIGH_CONTROL_dT), 0, 0, int(0.2 / HIGH_CONTROL_dT)), Vec4<int>(int(0.22 / HIGH_CONTROL_dT), int(0.22 / HIGH_CONTROL_dT), int(0.22 / HIGH_CONTROL_dT), int(0.22 / HIGH_CONTROL_dT)))
    , mTrotStair(MPC_HORIZON, 80, Vec4<int>(0, 40, 40, 0), Vec4<int>(65, 65, 65, 65))
    , mbFirstLateLanding{ false, false, false, false }
{
    sharedMemory = SharedMemory::getInstance();
    mStumbleLeg = 6;
}

void HighControlMain::ControllerFunction()
{
    startGaitSelectPanel();
}

void HighControlMain::startGaitSelectPanel()
{
    switch (sharedMemory->gaitState)
    {
    case STAND:
    {
        if (sharedMemory->gaitChangeFlag)
        {
            startGaitChangePanel(&mStand);
        }
        else
        {
            mStand.GetGaitTable();
        }
        for (int leg = 0; leg < 4; leg++)
        {
            sharedMemory->bGaitState[leg] = sharedMemory->gaitTable[leg];
        }

        if(sharedMemory->stumble.recoveryStage == STUMBLE::FOOT_RECOVERY)
        {
            std::cout<<"foot recovery"<<std::endl;
            if(!mbStumbleSwing)
            {
                mStumbleRecoverySwingTime = sharedMemory->localTime + 0.26;
                mbStumbleSwing = true;
                mStumbleLeg = 6;
                for(int leg = 0 ; leg<4; leg++)
                {
                    if(sharedMemory->stumble.bLeg[leg])
                    {
                        mStumbleLeg = leg;
                    }
                }
            }
            else
            {
                if(sharedMemory->localTime < mStumbleRecoverySwingTime)
                {
                    sharedMemory->gaitTable[mStumbleLeg] = 0;
                }
                else
                {
                    mbStumbleSwing = false;
                    if(sharedMemory->stumble.num == 0)
                    {
                        sharedMemory->stumble.bRecovery = false;
                        sharedMemory->stumble.recoveryStage = STUMBLE::NO_ACT;
                    }
                }
            }



        }
        break;
    }
    case TROT_SLOW:
    {
        if (sharedMemory->gaitChangeFlag)
        {
            startGaitChangePanel(&mTrotSlow);
            for (int leg = 0; leg < 4; leg++)
            {
                sharedMemory->bGaitState[leg] = sharedMemory->gaitTable[leg];
            }
        }
        else
        {
            mTrotSlow.GetGaitTable();
            /// stumble
            if(!sharedMemory->stumble.bRecovery)
            {
                if(sharedMemory->stumble.bStumble)
                {
                    sharedMemory->stumble.bRecovery = true;
                }
            }

            if (sharedMemory->stumble.recoveryStage == STUMBLE::STOPPING)
            {
                int standing_leg_count = 0;
                for(int leg=0; leg < 4; leg++)
                {
                    if(sharedMemory->stumble.bLeg[leg])
                    {
                        sharedMemory->gaitTable[leg] = 1;
                    }
                    if(sharedMemory->gaitTable[leg] == 1)
                    {
                        standing_leg_count++;
                    }
                }
                if(standing_leg_count == 4)
                {
                    sharedMemory->gaitState = STAND;
                    sharedMemory->gaitChangeFlag = false;
                }
            }
            for (int leg = 0; leg < 4; leg++)
            {
                sharedMemory->bGaitState[leg] = sharedMemory->gaitTable[leg]; /// bGaitState not used
            }
        }
        break;
    }
    case TROT_FAST:
    {
        if (sharedMemory->gaitChangeFlag)
        {
            startGaitChangePanel(&mTrotFast);
        }
        else
        {
            mTrotFast.GetGaitTable();
        }
        for (int leg = 0; leg < 4; leg++)
        {
            sharedMemory->bGaitState[leg] = sharedMemory->gaitTable[leg];
        }
        break;
    }
    case OVERLAP_TROT_FAST:
    {
        if (sharedMemory->gaitChangeFlag)
        {
            startGaitChangePanel(&mOverlapTrotFast);
        }
        else
        {
            mOverlapTrotFast.GetGaitTable();
        }
        for (int leg = 0; leg < 4; leg++)
        {
            sharedMemory->bGaitState[leg] = sharedMemory->gaitTable[leg];
        }
        break;
    }
    default:
    {
        break;
    }
    }
    mIteration++;
}

void HighControlMain::startGaitChangePanel(OffsetGait* currentGait)
{
    switch (sharedMemory->command.gaitCommand)
    {
    case DESIRED_GAIT_STAND:
    {
        if (currentGait->GetGaitTableTrans())
        {
            sharedMemory->gaitChangeFlag = false;
            sharedMemory->gaitState = STAND;
            sharedMemory->FSMState = FSM_STAND;
            sharedMemory->gaitPeriod = 0.1;
            sharedMemory->swingPeriod = 0.0;
            sharedMemory->standPeriod = 0.1;
        }
        break;
    }
    case DESIRED_GAIT_TROT_SLOW:
    {
        if (currentGait->GetGaitTableTrans())
        {
            sharedMemory->gaitChangeFlag = false;
            sharedMemory->gaitState = TROT_SLOW;
            sharedMemory->FSMState = FSM_TROT_SLOW;
            sharedMemory->gaitPeriod = 0.6;
            sharedMemory->swingPeriod = 0.26;
            sharedMemory->standPeriod = 0.34;
        }
        break;
    }
    case DESIRED_GAIT_TROT_FAST:
    {
        if (currentGait->GetGaitTableTrans())
        {
            sharedMemory->gaitChangeFlag = false;
            sharedMemory->gaitState = TROT_FAST;
            sharedMemory->FSMState = FSM_TROT_FAST;
            sharedMemory->gaitPeriod = 0.4;
            sharedMemory->swingPeriod = 0.2;
            sharedMemory->standPeriod = 0.2;
        }
        break;
    }
    case DESIRED_GAIT_OVERLAP_TROT_FAST:
    {
        if (currentGait->GetGaitTableTrans())
        {
            sharedMemory->gaitChangeFlag = false;
            sharedMemory->gaitState = OVERLAP_TROT_FAST;
            sharedMemory->FSMState = FSM_OVERLAP_TROT_FAST;
            sharedMemory->gaitPeriod = 0.4;
            sharedMemory->swingPeriod = 0.18;
            sharedMemory->standPeriod = 0.20;
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

void HighControlMain::updateGaitState()
{
    for (int leg = 0; leg < 4; leg++)
    {
        // clear landing state when gait table changes from stand(1) to swing(0)
        if ((sharedMemory->gaitTable[leg] == 1) && (sharedMemory->gaitTable[leg + 4] == 0))
        {
            sharedMemory->bSwingLiftDown[leg] = false;
            sharedMemory->earlyLanding[leg] = false;
            sharedMemory->lateLanding[leg] = false;
            mbFirstLateLanding[leg] = false;
        }

        if ((sharedMemory->gaitTable[leg] == 1) && !sharedMemory->contactState[leg] && !sharedMemory->earlyLanding[leg])
        {
            if (!mbFirstLateLanding[leg])
            {
                sharedMemory->lateLanding[leg] = true;
            }
            mbFirstLateLanding[leg] = true;
        }

        // clear late landing state when contact is detected
        if (sharedMemory->earlyLanding[leg])
        {
            sharedMemory->lateLanding[leg] = false;
        }

        if (sharedMemory->bSwingLiftDown[leg] && sharedMemory->contactState[leg])
        {
            sharedMemory->lateLanding[leg] = false;
        }

        // for visualize
        sharedMemory->solvedGRF[leg][1] = sharedMemory->earlyLanding[leg];
        sharedMemory->solvedGRF[leg][0] = sharedMemory->lateLanding[leg];

        // update gait table when early landing
        if (sharedMemory->earlyLanding[leg])
        {
            sharedMemory->bGaitState[leg] = true;
        }
        else if (sharedMemory->lateLanding[leg])
        {
            sharedMemory->bGaitState[leg] = false;
        }
        else
        {
            sharedMemory->bGaitState[leg] = sharedMemory->gaitTable[leg];
        }
    }
}