#include "SimulVisualizer/SimulVisualizer.hpp"

raisim::World world;
raisim::RaisimServer server(&world);
extern std::string modelFile;

SimulVisualizer::SimulVisualizer()
    : mMap(MAP_UNEVEN_TERRAIN_SLOPE)
{
    mWorld = &world;
    mServer = &server;
    mRobot = mWorld->addArticulatedSystem(modelFile);
    mWorld->setGravity({ 0.0, 0.0, -9.81 });
    mWorld->setTimeStep(VISUAL_dT);
    mRobot->setName("BLUE");
    HWData = _HWD_::getInstance();
    sharedMemory = SharedMemory::getInstance();
    switch (mMap)
    {
    case MAP_DEFAULT:
    {
        auto ground = mWorld->addGround();
        ground->setAppearance("simple");
        break;
    }
    case MAP_STAIR:
    {
        double tread = 24;      // 24 == 0.3 m in real world.
        double riser = 0.06;    // real value
        mRaisimHeightMap = mWorld->addHeightMap(std::string(RAISIM_RSC_DIR) + "stair.png", -1.5, 0, tread, tread, riser, -riser);
        break;
    }
    case MAP_HILL:
    {
        mRaisimHeightMap = mWorld->addHeightMap(std::string(RAISIM_RSC_DIR) + "hill1.png", 0, 0, 504, 504, 38.0 / (37312 - 32482), -270.3750, "hm");
    }
    case MAP_SLOPE:
    {
        mRaisimHeightMap = mWorld->addHeightMap(std::string(RAISIM_RSC_DIR) + "slope.txt", 11, 0, "hm");
    }
    case MAP_ROLL_SLOPE:
    {
        mRaisimHeightMap = mWorld->addHeightMap(std::string(RAISIM_RSC_DIR) + "roll_slope.txt", 11, 0, "hm");
        break;
    }
    case MAP_UNEVEN_TERRAIN:
    {
        mRaisimHeightMap = mWorld->addHeightMap(std::string(RAISIM_RSC_DIR) + "uneven_slope.txt", 11, 0, "hm");
        break;
    }
    case MAP_UNEVEN_TERRAIN_SLOPE:
    {
        mRaisimHeightMap = mWorld->addHeightMap(std::string(RAISIM_RSC_DIR) + "uneven_terrain_slope.txt", 7, 0, "hm");
        auto ground = mWorld->addGround();
        ground->setAppearance("simple");

        /// Map generation code for txt height map
        /*for(int i = 0; i < 20 ; i++)
        {
            for(int j = 0; j < 20 ; j++)
            {
                std::cout<<(double)(rand()%100)/1000 + j * 0.1<<" ";
            }
            std::cout<<std::endl;
        }*/

        break;
    }
    default:
        break;
    }
    mServer->focusOn(mRobot);

    for(int idx=0; idx<25; idx++)
    {
        mSlopeBox[0][idx] = server.addVisualBox(std::string("FL_")+std::to_string(idx), 0.02, 0.02, 0.01, 0.5, 0, 0);
        mSlopeBox[1][idx] = server.addVisualBox(std::string("FR_")+std::to_string(idx), 0.02, 0.02, 0.01, 0, 0.5, 0);
        mSlopeBox[2][idx] = server.addVisualBox(std::string("HL_")+std::to_string(idx), 0.02, 0.02, 0.01, 0.5, 0, 0.5);
        mSlopeBox[3][idx] = server.addVisualBox(std::string("HR_")+std::to_string(idx), 0.02, 0.02, 0.01, 0, 0.5, 0.5);
    }

    mShoulderPosition[FL_IDX] << SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[FR_IDX] << SHOULD_X_POS, -SHOULD_Y_POS, 0;
    mShoulderPosition[HL_IDX] << -SHOULD_X_POS, SHOULD_Y_POS, 0;
    mShoulderPosition[HR_IDX] << -SHOULD_X_POS, -SHOULD_Y_POS, 0;

    mSlopeBaseBox = server.addVisualBox("s_base", 0.02, 0.02, 0.01, 0.5, 0.5, 0.5);

    mSlopeContactFoot[0] = server.addVisualBox("s_contact_foot_0", 0.03, 0.03, 0.02, 0.1, 0.5, 0.5);
    mSlopeContactFoot[1] = server.addVisualBox("s_contact_foot_1", 0.03, 0.03, 0.02, 0.5, 0.1, 0.5);
    mSlopeContactFoot[2] = server.addVisualBox("s_contact_foot_2", 0.03, 0.03, 0.02, 0.5, 0.5, 0.1);
    mSlopeContactFoot[3] = server.addVisualBox("s_contact_foot_3", 0.03, 0.03, 0.02, 0.1, 0.1, 0.5);

    mSlopeShoulderPosition[0] = server.addVisualBox("s_shoulder_0", 0.03, 0.03, 0.02, 0.1, 0.5, 0.5);
    mSlopeShoulderPosition[1] = server.addVisualBox("s_shoulder_1", 0.03, 0.03, 0.02, 0.5, 0.1, 0.5);
    mSlopeShoulderPosition[2] = server.addVisualBox("s_shoulder_2", 0.03, 0.03, 0.02, 0.5, 0.5, 0.1);
    mSlopeShoulderPosition[3] = server.addVisualBox("s_shoulder_3", 0.03, 0.03, 0.02, 0.1, 0.1, 0.5);

    mSlopeOrientation[0] = server.addVisualArrow("s_x", 0.08, 0.1);
    mSlopeOrientation[0]->setColor(1.0, 0, 0, 1);
    mSlopeOrientation[1] = server.addVisualArrow("s_y", 0.08, 0.1);
    mSlopeOrientation[1]->setColor(0, 1.0, 0, 1);
    mSlopeOrientation[2] = server.addVisualArrow("s_z", 0.08, 0.1);
    mSlopeOrientation[2]->setColor(0, 0, 1.0, 1);

    mSlopeDesiredOrientation[0] = server.addVisualArrow("s_d_x", 0.08, 0.1);
    mSlopeDesiredOrientation[0]->setColor(1.0, 0, 0, 1);
    mSlopeDesiredOrientation[1] = server.addVisualArrow("s_d_y", 0.08, 0.1);
    mSlopeDesiredOrientation[1]->setColor(0, 1.0, 0, 1);
    mSlopeDesiredOrientation[2] = server.addVisualArrow("s_d_z", 0.08, 0.1);
    mSlopeDesiredOrientation[2]->setColor(0, 0, 1.0, 1);

    initRobotPose();
    openRaisimServer();
}

SimulVisualizer::~SimulVisualizer()
{
    std::cout<<"[SIMULATOR] is EXITING\n";

    /// kill server
    mServer->killServer();

    /// kill raisimUnity
    try {
        std::string pid = getProcessPID("pidof raisimUnity.x86_64");
        if (!pid.empty()) {
            std::cout << "PID of raisimUnity.x86_64: " << pid << std::endl;

            std::string command = "kill -9 " + pid;
            int result = system(command.c_str());

            if (result == -1) {
                std::cerr << "Failed to execute kill command" << std::endl;
            } else {
                std::cout << "Program with PID " << pid << " terminated successfully" << std::endl;
            }

        } else {
            std::cout << "No process found for raisimUnity.x86_64" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

std::string SimulVisualizer::getProcessPID(const std::string& command) {
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        // fail the popen
        throw std::runtime_error("Failed to open pipe for command: " + command);
    }

    char buffer[128];
    std::string result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    int returnCode = pclose(pipe); // check the process kill code
    if (returnCode != 0) {
        throw std::runtime_error("Command failed with return code: " + std::to_string(returnCode));
    }

    return result;
}

void SimulVisualizer::openSimulation()
{
    std::string path = std::string(RAISIM_SIM_DIR) + "raisimUnity.x86_64";
    std::filesystem::path exePath = path;
    if (std::filesystem::exists(exePath))
    {
        std::cout << "[SIMULATOR] File found\n";
        std::string command = exePath.string();
        system(command.c_str());
    }
    else
    {
        std::cerr << "[SIMULATOR] Failed to find simulation program. Need to open manually\n";
    }
}

void SimulVisualizer::openRaisimServer()
{
    mServer->launchServer(8080);
}

void SimulVisualizer::initRobotPose()
{
    Eigen::VectorXd initialJointPosition(mRobot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(mRobot->getDOF());
    initialJointPosition.setZero();
    initialJointVelocity.setZero();

    initialJointPosition[2] = 0.06 + 0.015;
    initialJointPosition[3] = 1;

    for (int idx = 0; idx < 4; idx++)
    {
        initialJointPosition[idx * 3 + 7] = pow(-1, idx) * 4.5 * D2R;//hip roll
        initialJointPosition[idx * 3 + 8] = 126 * D2R;//hip pitch
        initialJointPosition[idx * 3 + 9] = -159 * D2R;//knee pitch
    }

    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        HWData->sensor.motor[idx].pos = initialJointPosition[idx + 7];
        HWData->sensor.motor[idx].vel = initialJointVelocity[idx + 6];
    }

    mRobot->setGeneralizedCoordinate(initialJointPosition);
    mRobot->setGeneralizedVelocity(initialJointVelocity);
    mRobot->setGeneralizedForce(Eigen::VectorXd::Zero(mRobot->getDOF()));
    sharedMemory->FLMotorStatus = true;
    sharedMemory->FRMotorStatus = true;
    sharedMemory->HLMotorStatus = true;
    sharedMemory->HRMotorStatus = true;
    HWData->motorALLStatus = true;
    sharedMemory->isRobotControl = true;

    auto box0 = mWorld->addBox(0.1, 1.0, 0.15, 100000);
    box0->setPosition(-2.0, 0.0, 0.3);

    auto box1 = mWorld->addBox(0.1, 1.0, 0.125, 100000);
    box1->setPosition(-2.0, 1, 0.3);

    auto box2 = mWorld->addBox(0.1, 1.0, 0.1, 100000);
    box2->setPosition(-2.0, 2, 0.3);

    auto box3 = mWorld->addBox(0.1, 1.0, 0.075, 100000);
    box3->setPosition(-2.0, 3, 0.3);

    auto box4 = mWorld->addBox(0.1, 1.0, 0.05, 100000);
    box4->setPosition(-2.0, 4, 0.3);

    auto box5 = mWorld->addBox(0.1, 1.0, 0.025, 100000);
    box5->setPosition(-2.0, 5, 0.3);
}

void SimulVisualizer::updateVisual()
{
    Eigen::VectorXd position = mRobot->getGeneralizedCoordinate().e();
    Eigen::Vector3d basePostion = Eigen::Vector3d(position[0], position[1], position[2]);
    Eigen::Matrix3d rotWB = RobotMath::GetBaseRotationMat(sharedMemory->globalBaseQuaternion);
    // Eigen::Matrix3d rotBS = RobotMath::GetBaseRotationMat(sharedMemory->estimatedLocalSlopeQuat);
    Eigen::Matrix3d rotBS = RobotMath::GetSlopeRotationMat(sharedMemory->estimatedLocalSlopeRPY);
    Eigen::Matrix3d rotWS = rotWB * rotBS;

    Eigen::Vector3d groundShoulder[4];
    int cnt=0;
    for(int leg=0; leg<4; leg++)
    {
        if(sharedMemory->gaitTable[leg] == 1)
        {
            for(int x=-2; x<3; x++)
            {
                for(int y=-2; y<3; y++)
                {
                    mSlopeBox[leg][cnt]->setPosition(basePostion + rotWB * sharedMemory->bodyBase2FootPosition[leg] + rotWS * Eigen::Vector3d(x*0.1, y*0.1,0));
                    cnt++;
                }
            }
        }
        cnt=0;

        mSlopeContactFoot[leg]->setPosition(sharedMemory->groundContactFootPos[leg]);
        // mSlopeContactFoot[leg]->setPosition(sharedMemory->groundContactFootPos[leg][0], sharedMemory->groundContactFootPos[leg][1], sharedMemory->bodyBasePosition[2]);
        // mSlopeShoulderPosition[leg]->setPosition(sharedMemory->bodyBasePosition + rotBS.transpose() * mShoulderPosition[leg]);
        // mSlopeShoulderPosition[leg]->setPosition(sharedMemory->bodyBasePosition + mShoulderPosition[leg] - rotBS.transpose() * sharedMemory->bodyBasePosition.dot(rotWS.transpose()* Eigen::Vector3d(0, 0, 1)) * rotWS.transpose()* Eigen::Vector3d(0, 0, 1));
        Eigen::Vector3d worldZ = rotWS.transpose()* Eigen::Vector3d(0, 0, 1);
        double l_gravity = sharedMemory->bodyBasePosition[2]/worldZ[2];
        groundShoulder[leg] = sharedMemory->bodyBasePosition - rotWS.transpose()*Eigen::Vector3d(0, 0, l_gravity);
        groundShoulder[leg][2] = 0;
        mSlopeShoulderPosition[0]->setPosition(groundShoulder[leg]);
        mSlopeShoulderPosition[1]->setPosition((sharedMemory->groundContactFootPos[0] + sharedMemory->groundContactFootPos[1] + sharedMemory->groundContactFootPos[2] + sharedMemory->groundContactFootPos[3])/4);
        mSlopeShoulderPosition[2]->setPosition(groundShoulder[leg]);
        Eigen::Vector3d groundContactCenter = rotBS.transpose() * (sharedMemory->bodyBase2FootPosition[0] + sharedMemory->bodyBase2FootPosition[1] + sharedMemory->bodyBase2FootPosition[2] + sharedMemory->bodyBase2FootPosition[3])/4;
        groundContactCenter[2] = 0;
        mSlopeShoulderPosition[3]->setPosition(groundContactCenter);

        if(sharedMemory->stumble.bStumble)
        {
//            sharedMemory->stumble.bRecovery = true;
        }
    }

    Eigen::Vector3d heading;
    if(sharedMemory->stumble.bRecovery)
    {
        for(int leg=0; leg<4; leg++)
        {
            if(sharedMemory->stumble.bLeg[leg])
            {
//                sharedMemory->stumble.bHeadingLeg[leg] = true;
            }

            if(sharedMemory->stumble.bHeadingLeg[leg])
            {
                heading = groundShoulder[leg] - sharedMemory->groundContactFootPos[leg];

                mSlopeDesiredOrientation[0]->setPosition(sharedMemory->groundContactFootPos[leg]);
                mSlopeDesiredOrientation[1]->setPosition(sharedMemory->groundContactFootPos[leg]);
                mSlopeDesiredOrientation[2]->setPosition(sharedMemory->groundContactFootPos[leg]);

                double yaw;
                if(heading.norm() > 0.07)
                {
                    yaw = atan2(heading[1], heading[0]);
                }
                else
                {
                    yaw = 0;
                }
                Eigen::Vector3d desiredRPY = Eigen::Vector3d (0, 0, yaw);
                mSlopeDesiredOrientation[0]->setOrientation(RobotMath::rpy2QUAT(desiredRPY + Eigen::Vector3d (0, PI/2, 0)));
                mSlopeDesiredOrientation[1]->setOrientation(RobotMath::rpy2QUAT(desiredRPY + Eigen::Vector3d (-PI/2, 0, 0)));
                mSlopeDesiredOrientation[2]->setOrientation(RobotMath::rpy2QUAT(desiredRPY));
            }
        }
    }
    else
    {
        mSlopeDesiredOrientation[0]->setPosition(0, 0, 0);
        mSlopeDesiredOrientation[1]->setPosition(0, 0, 0);
        mSlopeDesiredOrientation[2]->setPosition(0, 0, 0);

        mSlopeDesiredOrientation[0]->setOrientation(1, 0, 0, 0);
        mSlopeDesiredOrientation[1]->setOrientation(1, 0, 0, 0);
        mSlopeDesiredOrientation[2]->setOrientation(1, 0, 0, 0);
    }

//    if(heading.norm() < 0.01)
//    {
//        sharedMemory->stumble.bRecovery = false;
//        for(int leg=0; leg<4; leg++)
//        {
//            sharedMemory->stumble.bHeadingLeg[leg] = false;
//        }
//    }

    mSlopeOrientation[0]->setPosition(sharedMemory->bodyBasePosition);
    mSlopeOrientation[1]->setPosition(sharedMemory->bodyBasePosition);
    mSlopeOrientation[2]->setPosition(sharedMemory->bodyBasePosition);

    Eigen::Vector3d s_bodyRPY = RobotMath::rot2RPY(RobotMath::GetBaseRotationMatInverse(sharedMemory->estimatedLocalSlopeQuat));
    Eigen::Vector4d s_qaut_x = RobotMath::rpy2QUAT(s_bodyRPY + Eigen::Vector3d(0, PI/2, 0));
    Eigen::Vector4d s_qaut_y = RobotMath::rpy2QUAT(-sharedMemory->estimatedLocalSlopeRPY + Eigen::Vector3d(-PI/2, 0, 0));
    Eigen::Vector4d s_qaut_z = RobotMath::rpy2QUAT(s_bodyRPY);
    mSlopeOrientation[0]->setOrientation(s_qaut_x);
    mSlopeOrientation[1]->setOrientation(s_qaut_y);
    mSlopeOrientation[2]->setOrientation(s_qaut_z);

    mSlopeBaseBox->setPosition(sharedMemory->bodyBasePosition);

    if(sharedMemory->FSMState == FSM_EMERGENCY_STOP)
    {
        sharedMemory->FLMotorStatus = false;
        sharedMemory->FRMotorStatus = false;
        sharedMemory->HLMotorStatus = false;
        sharedMemory->HRMotorStatus = false;
    }
    Eigen::VectorXd JointTorque(mRobot->getDOF());
    JointTorque.setZero();
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        JointTorque[idx + 6] = HWData->motorDesiredTorque[idx];

    }
    mRobot->setGeneralizedForce(JointTorque);
    mServer->integrateWorldThreadSafe();
}

void SimulVisualizer::updateCurrent()
{
    Eigen::VectorXd position = mRobot->getGeneralizedCoordinate().e();
    Eigen::VectorXd velocity = mRobot->getGeneralizedVelocity().e();
    Eigen::VectorXd force = mRobot->getGeneralizedForce().e();
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        HWData->sensor.motor[idx].pos = position[idx + 7];
        HWData->sensor.motor[idx].vel = velocity[idx + 6];
        HWData->sensor.motor[idx].torque = force[idx + 6];
    }
    HWData->sensor.imu.quat = position.block<4, 1>(3, 0);

    RobotMath::TransformQuat2Euler(HWData->sensor.imu.quat, mRawEulerAngle);

    if (tempV[2] > 3.0 && mRawEulerAngle[2] < 0)
    {
        mMulti++;
    }
    if (tempV[2] < -3.0 && mRawEulerAngle[2] > 0)
    {
        mMulti--;
    }
    tempV[2] = mRawEulerAngle[2];

    HWData->sensor.imu.rpy[0] = mRawEulerAngle[0];
    HWData->sensor.imu.rpy[1] = mRawEulerAngle[1];
    HWData->sensor.imu.rpy[2] = mRawEulerAngle[2] + mMulti * 2.0 * 3.141592;
    HWData->sensor.imu.gyro = RobotMath::GetBaseRotationMatInverse(HWData->sensor.imu.quat) * velocity.block<3, 1>(3, 0);
    HWData->sensor.imu.acc = mRobot->getGeneralizedAcceleration().e().block<3, 1>(0, 0);

}
