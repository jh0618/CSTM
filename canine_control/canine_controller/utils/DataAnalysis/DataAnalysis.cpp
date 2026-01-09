#include "DataAnalysis.hpp"

DataAnalysis::DataAnalysis()
        : mSaveLocation(LOG_DIR)
        , mExtension(".csv")
{
    MakeFile();
    sharedMemory = SharedMemory::getInstance();
}

DataAnalysis::~DataAnalysis()
{
    std::cout << "[INFO] "<< mFileName << " is saved" << std::endl;
    mLogger.close();
}

void DataAnalysis::MakeFile()
{
    time_t timer;
    struct tm* t;
    timer = time(NULL);
    t = localtime(&timer);

    mDirectoryLocation = mSaveLocation
        + std::to_string(t->tm_year+1900) + "_"
        + std::to_string(t->tm_mon+1) + "_"
        + std::to_string(t->tm_mday);
    std::filesystem::create_directories(mDirectoryLocation);
    std::cout<<"dir loc : "<<mDirectoryLocation<<std::endl;

    mFileName = mDirectoryLocation + "/" + std::string("BLUE_log_") + std::to_string(t->tm_hour) + "_" + std::to_string(t->tm_min) + mExtension;
    std::cout<<"file name : "<<mFileName<<std::endl;
    mLogger.open(mFileName);
    mLogger << "t" << "\t"
            << "bodyPosX" << "\t" << "bodyPosY" << "\t" << "bodyPosZ" << "\t"
            << "bodyVelX" << "\t" << "bodyVelY" << "\t" << "bodyVelZ" << "\t"
            << "bodyEulerR" << "\t" << "bodyEulerP" << "\t" << "bodyEulerY" << "\t"
            << "bodyQuat0" << "\t" << "bodyQuat1" << "\t" << "bodyQuat2" << "\t" << "bodyQuat3" << "\t"
            << "bodyAngVelX" << "\t" << "bodyAngVelY" << "\t" << "bodyAngVelZ" << "\t"
            << "mtPos0" << "\t" << "mtPos1" << "\t" << "mtPos2" << "\t"
            << "mtPos3" << "\t" << "mtPos4" << "\t" << "mtPos5" << "\t"
            << "mtPos6" << "\t" << "mtPos7" << "\t" << "mtPos8" << "\t"
            << "mtPos9" << "\t" << "mtPos10" << "\t" << "mtPos11" << "\t"
            << "mtVel0" << "\t" << "mtVel1" << "\t" << "mtVel2" << "\t"
            << "mtVel3" << "\t" << "mtVel4" << "\t" << "mtVel5" << "\t"
            << "mtVel6" << "\t" << "mtVel7" << "\t" << "mtVel8" << "\t"
            << "mtVel9" << "\t" << "mtVel10" << "\t" << "mtVel11" << "\t"
            << "FL_footPosX" << "\t" << "FL_footPosY" << "\t" << "FL_footPosZ" << "\t"
            << "FR_footPosX" << "\t" << "FR_footPosY" << "\t" << "FR_footPosZ" << "\t"
            << "RL_footPosX" << "\t" << "RL_footPosY" << "\t" << "RL_footPosZ" << "\t"
            << "RR_footPosX" << "\t" << "RR_footPosY" << "\t" << "RR_footPosZ" << "\t"
            << "FL_footVelX" << "\t" << "FL_footVelY" << "\t" << "FL_footVelZ" << "\t"
            << "FR_footVelX" << "\t" << "FR_footVelY" << "\t" << "FR_footVelZ" << "\t"
            << "RL_footVelX" << "\t" << "RL_footVelY" << "\t" << "RL_footVelZ" << "\t"
            << "RR_footVelX" << "\t" << "RR_footVelY" << "\t" << "RR_footVelZ" << "\t"
            << "Ref_bodyPosX" << "\t" << "Ref_bodyPosY" << "\t" << "Ref_bodyPosZ" << "\t"
            << "Ref_bodyVelX" << "\t" << "Ref_bodyVelY" << "\t" << "Ref_bodyVelZ" << "\t"
            << "Ref_bodyEulerR" << "\t" << "Ref_bodyEulerP" << "\t" << "Ref_bodyEulerY" << "\t"
            << "Ref_bodyQuat0" << "\t" << "Ref_bodyQuat1" << "\t" << "Ref_bodyQuat2" << "\t" << "Ref_bodyQuat3" << "\t"
            << "Ref_bodyAngVelX" << "\t" << "Ref_bodyAngVelY" << "\t" << "Ref_bodyAngVelZ" << "\t"
            << "Ref_mtPos0" << "\t" << "Ref_mtPos1" << "\t" << "Ref_mtPos2" << "\t"
            << "Ref_mtPos3" << "\t" << "Ref_mtPos4" << "\t" << "Ref_mtPos5" << "\t"
            << "Ref_mtPos6" << "\t" << "Ref_mtPos7" << "\t" << "Ref_mtPos8" << "\t"
            << "Ref_mtPos9" << "\t" << "Ref_mtPos10" << "\t" << "Ref_mtPos11" << "\t"
            << "Ref_mtVel0" << "\t" << "Ref_mtVel1" << "\t" << "Ref_mtVel2" << "\t"
            << "Ref_mtVel3" << "\t" << "Ref_mtVel4" << "\t" << "Ref_mtVel5" << "\t"
            << "Ref_mtVel6" << "\t" << "Ref_mtVel7" << "\t" << "Ref_mtVel8" << "\t"
            << "Ref_mtVel9" << "\t" << "Ref_mtVel10" << "\t" << "Ref_mtVel11" << "\t"
            << "Ref_FL_footPosX" << "\t" << "Ref_FL_footPosY" << "\t" << "Ref_FL_footPosZ" << "\t"
            << "Ref_FR_footPosX" << "\t" << "Ref_FR_footPosY" << "\t" << "Ref_FR_footPosZ" << "\t"
            << "Ref_RL_footPosX" << "\t" << "Ref_RL_footPosY" << "\t" << "Ref_RL_footPosZ" << "\t"
            << "Ref_RR_footPosX" << "\t" << "Ref_RR_footPosY" << "\t" << "Ref_RR_footPosZ" << "\t"
            << "Ref_FL_footVelX" << "\t" << "Ref_FL_footVelY" << "\t" << "Ref_FL_footVelZ" << "\t"
            << "Ref_FR_footVelX" << "\t" << "Ref_FR_footVelY" << "\t" << "Ref_FR_footVelZ" << "\t"
            << "Ref_RL_footVelX" << "\t" << "Ref_RL_footVelY" << "\t" << "Ref_RL_footVelZ" << "\t"
            << "Ref_RR_footVelX" << "\t" << "Ref_RR_footVelY" << "\t" << "Ref_RR_footVelZ" << "\t"
            << "gaitFL" << "\t" << "gaitFR" << "\t" << "gaitRL" << "\t" << "gaitRR" << "\t"
            << "mtTau0" << "\t" << "mtTau1" << "\t" << "mtTau2" << "\t"
            << "mtTau3" << "\t" << "mtTau4" << "\t" << "mtTau5" << "\t"
            << "mtTau6" << "\t" << "mtTau7" << "\t" << "mtTau8" << "\t"
            << "mtTau9" << "\t" << "mtTau10" << "\t" << "mtTau11" << "\t"
            << "Ref_mtTau0" << "\t" << "Ref_mtTau1" << "\t" << "Ref_mtTau2" << "\t"
            << "Ref_mtTau3" << "\t" << "Ref_mtTau4" << "\t" << "Ref_mtTau5" << "\t"
            << "Ref_mtTau6" << "\t" << "Ref_mtTau7" << "\t" << "Ref_mtTau8" << "\t"
            << "Ref_mtTau9" << "\t" << "Ref_mtTau10" << "\t" << "Ref_mtTau11" << "\t"
            << "contactStateFL" << "\t" << "contactStateFR" << "\t" << "contactStateRL" << "\t" << "contactStateRR" << "\t"
            << "contactResTauFL" << "\t" << "contactResTauFR" << "\t" << "contactResTauRL" << "\t" << "contactResTauRR" << "\t"
            << "FL_mpcGRFX" << "\t" << "FL_mpcGRFY" << "\t" << "FL_mpcGRFZ" << "\t"
            << "FR_mpcGRFX" << "\t" << "FR_mpcGRFY" << "\t" << "FR_mpcGRFZ" << "\t"
            << "RL_mpcGRFX" << "\t" << "RL_mpcGRFY" << "\t" << "RL_mpcGRFZ" << "\t"
            << "RR_mpcGRFX" << "\t" << "RR_mpcGRFY" << "\t" << "RR_mpcGRFZ" << "\t"
            << "localSlopeR" << "\t" << "localSlopeP" << "\t"
            << "SPI2CAN_failed_num" << "\t" << "SPI2CAN_failed_ID" << "\n";
}

void DataAnalysis::SaveRobotState()
{
    if (mLogger.is_open())
    {
        mLogger << sharedMemory->localTime << "\t"
            << sharedMemory->globalBasePosition[0] << "\t" << sharedMemory->globalBasePosition[1] << "\t" << sharedMemory->globalBasePosition[2] << "\t"
            << sharedMemory->globalBaseVelocity[0] << "\t" << sharedMemory->globalBaseVelocity[1] << "\t" << sharedMemory->globalBaseVelocity[2] << "\t"
            << sharedMemory->globalBaseEulerAngle[0] << "\t" << sharedMemory->globalBaseEulerAngle[1] << "\t" << sharedMemory->globalBaseEulerAngle[2] << "\t"
            << sharedMemory->globalBaseQuaternion[0] << "\t" << sharedMemory->globalBaseQuaternion[1] << "\t" << sharedMemory->globalBaseQuaternion[2] << "\t" << sharedMemory->globalBaseQuaternion[3] << "\t"
            << sharedMemory->bodyBaseAngularVelocity[0] << "\t" << sharedMemory->bodyBaseAngularVelocity[1] << "\t" << sharedMemory->bodyBaseAngularVelocity[2] << "\t"
            << sharedMemory->motorPosition[0] << "\t" << sharedMemory->motorPosition[1] << "\t" << sharedMemory->motorPosition[2] << "\t"
            << sharedMemory->motorPosition[3] << "\t" << sharedMemory->motorPosition[4] << "\t" << sharedMemory->motorPosition[5] << "\t"
            << sharedMemory->motorPosition[6] << "\t" << sharedMemory->motorPosition[7] << "\t" << sharedMemory->motorPosition[8] << "\t"
            << sharedMemory->motorPosition[9] << "\t" << sharedMemory->motorPosition[10] << "\t" << sharedMemory->motorPosition[11] << "\t"
            << sharedMemory->motorVelocity[0] << "\t" << sharedMemory->motorVelocity[1] << "\t" << sharedMemory->motorVelocity[2] << "\t"
            << sharedMemory->motorVelocity[3] << "\t" << sharedMemory->motorVelocity[4] << "\t" << sharedMemory->motorVelocity[5] << "\t"
            << sharedMemory->motorVelocity[6] << "\t" << sharedMemory->motorVelocity[7] << "\t" << sharedMemory->motorVelocity[8] << "\t"
            << sharedMemory->motorVelocity[9] << "\t" << sharedMemory->motorVelocity[10] << "\t" << sharedMemory->motorVelocity[11] << "\t"
            << sharedMemory->bodyBase2FootPosition[0][0] << "\t" << sharedMemory->bodyBase2FootPosition[0][1] << "\t" << sharedMemory->bodyBase2FootPosition[0][2] << "\t"
            << sharedMemory->bodyBase2FootPosition[1][0] << "\t" << sharedMemory->bodyBase2FootPosition[1][1] << "\t" << sharedMemory->bodyBase2FootPosition[1][2] << "\t"
            << sharedMemory->bodyBase2FootPosition[2][0] << "\t" << sharedMemory->bodyBase2FootPosition[2][1] << "\t" << sharedMemory->bodyBase2FootPosition[2][2] << "\t"
            << sharedMemory->bodyBase2FootPosition[3][0] << "\t" << sharedMemory->bodyBase2FootPosition[3][1] << "\t" << sharedMemory->bodyBase2FootPosition[3][2] << "\t"
            << sharedMemory->bodyBase2FootVelocity[0][0] << "\t" << sharedMemory->bodyBase2FootVelocity[0][1] << "\t" << sharedMemory->bodyBase2FootVelocity[0][2] << "\t"
            << sharedMemory->bodyBase2FootVelocity[1][0] << "\t" << sharedMemory->bodyBase2FootVelocity[1][1] << "\t" << sharedMemory->bodyBase2FootVelocity[1][2] << "\t"
            << sharedMemory->bodyBase2FootVelocity[2][0] << "\t" << sharedMemory->bodyBase2FootVelocity[2][1] << "\t" << sharedMemory->bodyBase2FootVelocity[2][2] << "\t"
            << sharedMemory->bodyBase2FootVelocity[3][0] << "\t" << sharedMemory->bodyBase2FootVelocity[3][1] << "\t" << sharedMemory->bodyBase2FootVelocity[3][2] << "\t"
            << sharedMemory->globalBaseDesiredPosition[0] << "\t" << sharedMemory->globalBaseDesiredPosition[1] << "\t" << sharedMemory->globalBaseDesiredPosition[2] << "\t"
            << sharedMemory->globalBaseDesiredVelocity[0] << "\t" << sharedMemory->globalBaseDesiredVelocity[1] << "\t" << sharedMemory->globalBaseDesiredVelocity[2] << "\t"
            << sharedMemory->globalBaseDesiredEulerAngle[0] << "\t" << sharedMemory->globalBaseDesiredEulerAngle[1] << "\t" << sharedMemory->globalBaseDesiredEulerAngle[2] << "\t"
            << sharedMemory->globalBaseDesiredQuaternion[0] << "\t" << sharedMemory->globalBaseDesiredQuaternion[1] << "\t" << sharedMemory->globalBaseDesiredQuaternion[2] << "\t" << sharedMemory->globalBaseDesiredQuaternion[3] << "\t"
            << sharedMemory->bodyBaseDesiredAngularVelocity[0] << "\t" << sharedMemory->bodyBaseDesiredAngularVelocity[1] << "\t" << sharedMemory->bodyBaseDesiredAngularVelocity[2] << "\t"
            << sharedMemory->motorDesiredPosition[0] << "\t" << sharedMemory->motorDesiredPosition[1] << "\t" << sharedMemory->motorDesiredPosition[2] << "\t"
            << sharedMemory->motorDesiredPosition[3] << "\t" << sharedMemory->motorDesiredPosition[4] << "\t" << sharedMemory->motorDesiredPosition[5] << "\t"
            << sharedMemory->motorDesiredPosition[6] << "\t" << sharedMemory->motorDesiredPosition[7] << "\t" << sharedMemory->motorDesiredPosition[8] << "\t"
            << sharedMemory->motorDesiredPosition[9] << "\t" << sharedMemory->motorDesiredPosition[10] << "\t" << sharedMemory->motorDesiredPosition[11] << "\t"
            << sharedMemory->motorDesiredVelocity[0] << "\t" << sharedMemory->motorDesiredVelocity[1] << "\t" << sharedMemory->motorDesiredVelocity[2] << "\t"
            << sharedMemory->motorDesiredVelocity[3] << "\t" << sharedMemory->motorDesiredVelocity[4] << "\t" << sharedMemory->motorDesiredVelocity[5] << "\t"
            << sharedMemory->motorDesiredVelocity[6] << "\t" << sharedMemory->motorDesiredVelocity[7] << "\t" << sharedMemory->motorDesiredVelocity[8] << "\t"
            << sharedMemory->motorDesiredVelocity[9] << "\t" << sharedMemory->motorDesiredVelocity[10] << "\t" << sharedMemory->motorDesiredVelocity[11] << "\t"
            << sharedMemory->bodyBase2FootDesiredPosition[0][0] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[0][1] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[0][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredPosition[1][0] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[1][1] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[1][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredPosition[2][0] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[2][1] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[2][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredPosition[3][0] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[3][1] << "\t" << sharedMemory->bodyBase2FootDesiredPosition[3][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredVelocity[0][0] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[0][1] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[0][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredVelocity[1][0] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[1][1] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[1][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredVelocity[2][0] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[2][1] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[2][2] << "\t"
            << sharedMemory->bodyBase2FootDesiredVelocity[3][0] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[3][1] << "\t" << sharedMemory->bodyBase2FootDesiredVelocity[3][2] << "\t"
            << sharedMemory->gaitTable[0] << "\t" << sharedMemory->gaitTable[1] << "\t" << sharedMemory->gaitTable[2] << "\t" << sharedMemory->gaitTable[3] << "\t"
            << sharedMemory->motorTorque[0] << "\t" << sharedMemory->motorTorque[1] << "\t" << sharedMemory->motorTorque[2] << "\t"
            << sharedMemory->motorTorque[3] << "\t" << sharedMemory->motorTorque[4] << "\t" << sharedMemory->motorTorque[5] << "\t"
            << sharedMemory->motorTorque[6] << "\t" << sharedMemory->motorTorque[7] << "\t" << sharedMemory->motorTorque[8] << "\t"
            << sharedMemory->motorTorque[9] << "\t" << sharedMemory->motorTorque[10] << "\t" << sharedMemory->motorTorque[11] << "\t"
            << sharedMemory->motorDesiredTorque[0] << "\t" << sharedMemory->motorDesiredTorque[1] << "\t" << sharedMemory->motorDesiredTorque[2] << "\t"
            << sharedMemory->motorDesiredTorque[3] << "\t" << sharedMemory->motorDesiredTorque[4] << "\t" << sharedMemory->motorDesiredTorque[5] << "\t"
            << sharedMemory->motorDesiredTorque[6] << "\t" << sharedMemory->motorDesiredTorque[7] << "\t" << sharedMemory->motorDesiredTorque[8] << "\t"
            << sharedMemory->motorDesiredTorque[9] << "\t" << sharedMemory->motorDesiredTorque[10] << "\t" << sharedMemory->motorDesiredTorque[11] << "\t"
            << sharedMemory->contactState[0] << "\t" << sharedMemory->contactState[1] << "\t" << sharedMemory->contactState[2] << "\t" << sharedMemory->contactState[3] << "\t"
            << sharedMemory->contactResidualTorque[0] << "\t" << sharedMemory->contactResidualTorque[1] << "\t" << sharedMemory->contactResidualTorque[2] << "\t" << sharedMemory->contactResidualTorque[3] << "\t"
            << sharedMemory->solvedGRF[0][0] << "\t" << sharedMemory->solvedGRF[0][1] << "\t" << sharedMemory->solvedGRF[0][2] << "\t"
            << sharedMemory->solvedGRF[1][0] << "\t" << sharedMemory->solvedGRF[1][1] << "\t" << sharedMemory->solvedGRF[1][2] << "\t"
            << sharedMemory->solvedGRF[2][0] << "\t" << sharedMemory->solvedGRF[2][1] << "\t" << sharedMemory->solvedGRF[2][2] << "\t"
            << sharedMemory->solvedGRF[3][0] << "\t" << sharedMemory->solvedGRF[3][1] << "\t" << sharedMemory->solvedGRF[3][2] << "\t"
            << sharedMemory->estimatedLocalSlopeRPY[0] << "\t" << sharedMemory->estimatedLocalSlopeRPY[1] << "\t"
            << sharedMemory->SPI2CANConnectionFailedNum << "\t" << sharedMemory->SPI2CANConnectionFailedID << "\n";
    }
    if(sharedMemory->SPI2CANConnectionFailedNum != 0)
    {
        sharedMemory->SPI2CANConnectionFailedNum = 0;
        sharedMemory->SPI2CANConnectionFailedID = -1;
    }
}
