#ifndef MOCAP2MAV_SAFETYPARSER_H
#define MOCAP2MAV_SAFETYPARSER_H

#define strtk_no_tr1_or_boost
#include "common/strtk.hpp"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <list>
#include "common/common.h"

class SafetyParser
{

private:

    double getValue(std::string str);
    bool parseParameter(int pos);
    bool insertValue(std::string a, float value);
    std::vector<std::vector<std::string>> _tokens;
    std::map<std::string, float> _paramsMapParsed;
    int _num_params;

public:

    SafetyParser();

    bool parse();
    bool loadFile(std::string file);
    std::map<std::string, float> getParamsMapParsed();

};

#endif //MOCAP2MAV_SAFETYPARSER_H
