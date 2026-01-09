//
// Created by camel on 23. 1. 23.
//

#ifndef RAISIM_DATAANALYSIS_HPP
#define RAISIM_DATAANALYSIS_HPP

#include <fstream>
#include <iostream>
#include <ctime>
#include <filesystem>


#include "SharedMemory.hpp"

class DataAnalysis{
public:
    DataAnalysis();
    ~DataAnalysis();

    void MakeFile();
    void SaveRobotState();

private:
    SharedMemory* sharedMemory;

    std::string mSaveLocation;
    std::string mExtension;
    std::string mFileName;
    std::string mDirectoryLocation;
    std::ofstream mLogger;
};

#endif //RAISIM_DATAANALYSIS_HPP
