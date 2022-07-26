//
// Created by sleepingmachine on 22-7-20.
//

#ifndef ABYSSAL_CV_2022_ENGINEERING_CONTROL_READ_CONFIG_HPP
#define ABYSSAL_CV_2022_ENGINEERING_CONTROL_READ_CONFIG_HPP

#include <string>
#include <map>
#include <fstream>
#include <stdlib.h>

namespace rr
{
    class RrConfig
    {
    public:
        RrConfig()
        {
        }
        ~RrConfig()
        {
        }
        bool ReadConfig(const std::string & filename);
        std::string ReadString(const char* section, const char* item, const char* default_value);
        int ReadInt(const char* section, const char* item, const int& default_value);
        float ReadFloat(const char* section, const char* item, const float& default_value);
    private:
        bool IsSpace(char c);
        bool IsCommentChar(char c);
        void Trim(std::string & str);
        bool AnalyseLine(const std::string & line, std::string& section, std::string & key, std::string & value);

    private:
        //std::map<std::string, std::string> settings_;
        std::map<std::string, std::map<std::string, std::string> >settings_;
    };
}

#endif //ABYSSAL_CV_2022_ENGINEERING_CONTROL_READ_CONFIG_HPP
