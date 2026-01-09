//
// Created by hs on 24. 2. 28.
//

#include "YamlReader.hpp"

YamlReader::YamlReader()
{
    std::string modelFile = std::string(YAML_RSC_DIR) + "RobotParameters.yaml";
    mConfig = YAML::LoadFile(modelFile);
}

YamlReader& YamlReader::GetInstance()
{
    static YamlReader instance;
    return instance;
}

YAML::Node YamlReader::GetConfig()
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mConfig;
}

void YamlReader::SetConfig(const std::string& parentKey, const std::string& childKey, std::string value)
{
    std::lock_guard<std::mutex> lock(mMutex);
    YAML::Node Node(value);
    mConfig[parentKey][childKey] = value;
}

void YamlReader::SaveConfig()
{
    time_t timer;
    struct tm* t;
    timer = time(NULL);
    t = localtime(&timer);
    std::string currentTime = std::to_string(t->tm_year+1900) + "_"
                              + std::to_string(t->tm_mon+1) + "_"
                              + std::to_string(t->tm_mday) + "_"
                              + std::to_string(t->tm_hour) + "_"
                              + std::to_string(t->tm_min) + "_"
                              + std::to_string(t->tm_sec);
    std::string saveDir = "log/" + currentTime + "_config.yaml";
    std::cout << saveDir << "\n";
    YAML::Emitter emitter;
    emitter << mConfig;
    std::ofstream fout(saveDir);
    fout << emitter.c_str();
}

YAML::Node YamlReader::ReloadConfig()
{
    std::lock_guard<std::mutex> lock(mMutex);
    mConfig = YAML::LoadFile("RobotParameters.yaml");
    return mConfig;
}
