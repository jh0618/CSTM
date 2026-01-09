//
// Created by hs on 24. 2. 28.
//

#ifndef CAMEL_CANINE_YAMLREADER_HPP
#define CAMEL_CANINE_YAMLREADER_HPP

#include <mutex>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>

class YamlReader
{
public:
    template<typename T>
    std::vector<T> ParseStringToArray(const std::string& input);

    template<typename T>
    std::string ParseArrayToString(const std::vector<T>& input);

    static YamlReader& GetInstance();
    YAML::Node GetConfig();
    void SetConfig(const std::string& parentKey, const std::string& childKey, std::string value);
    void SaveConfig();
    YAML::Node ReloadConfig();
private:
    YamlReader();

    ~YamlReader()
    {
    };

    YAML::Node mConfig;
    std::mutex mMutex;
};

template<typename T>
std::vector<T> YamlReader::ParseStringToArray(const std::string& input)
{
    std::vector<T> result;
    std::stringstream ss(input);
    char ch; // 배열의 괄호와 콤마를 무시하기 위한 임시 변수
    T value;

    // 배열의 시작 괄호 '[' 무시
    ss >> ch;
    while (ss >> value)
    {
        result.push_back(value);
        ss >> ch; // 콤마 ',' 또는 배열의 끝 괄호 ']' 무시
    }

    return result;
}

template<typename T>
std::string YamlReader::ParseArrayToString(const std::vector<T>& input)
{
    std::stringstream ss;
    ss << "[";

    for (size_t i = 0; i < input.size(); ++i)
    {
        if constexpr (std::is_same<T, std::string>::value)
        {
            ss << input[i]; // T가 std::string인 경우
        }
        else if constexpr (std::is_same<T, double>::value)
        {
            // T가 double인 경우, 소수점 한 자리까지 포맷
            ss << std::fixed << std::setprecision(1) << input[i];
        }
        else
        {
            // T가 double이 아닌 숫자 타입인 경우
            ss << std::to_string(input[i]);
        }

        if (i < input.size() - 1)
        {
            ss << ", "; // 요소 사이에 콤마와 공백 추가
        }
    }
    ss << "]";

    return ss.str();
}

#endif //CAMEL_CANINE_YAMLREADER_HPP
