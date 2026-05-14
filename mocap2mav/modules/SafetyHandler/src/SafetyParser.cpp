#include "SafetyParser.h"


SafetyParser::SafetyParser(): _num_params(0) {}

bool SafetyParser::loadFile(std::string file)
{

    std::ifstream myfile (file);
    if (myfile.is_open())
    {
        //Clear token vector
        _tokens.clear();
        _tokens.shrink_to_fit();

        //Build tokenized vector line by line
        std::string line;
        while ( getline (myfile,line))
        {

            if(line == "#") break;
            std::vector<std::string> str_vector;
            strtk::parse(line,"=",str_vector);

            _tokens.push_back(str_vector);

        }

        myfile.close();
        return true;
    }
    else
    {

        std::cout << "Unable to open file" << std::endl;
        return false;

    }

}

bool SafetyParser::parse()
{
    //Start Parsing
    if(_tokens.size() > 0)
    {

        for (int j = 0; j < _tokens.size(); j++)
        {
            //Parse single parameters
            if(!parseParameter(j)) return false;
        }
        if(_num_params>28)
        {
          std::cout << "There are some parameters not matching, please check" << std::endl;
          return false;
        }
        else if(_num_params<28)
        {
          std::cout << "Some safety parameters are missing, please check" << std::endl;
          return false;
        }
    }
    else
    {
        std::cout << "Invalid tokens count" << std::endl;
        return false;
    }
    return true;

}

double SafetyParser::getValue(std::string str)  {
    char *cstr = new char[str.length() + 1];
    strcpy(cstr, str.c_str());
    double value = atof(cstr);
    delete (cstr);
    return value;
}

bool SafetyParser::parseParameter(int pos)
{

    //Looking for the parameter
    std::string a = _tokens[pos][0];
    float value = getValue(_tokens[pos][1]);
    //Select the right parameter and parse it
    if(a.compare("battery_threshold1") == 0 || a.compare("emergland_threshold1") == 0 ||
      a.compare("emergland_threshold2") == 0 || a.compare("emergland_gain") == 0 ||
      a.compare("gpsgone_threshold1") == 0 || a.compare("gpsgone_threshold2") == 0 ||
      a.compare("cameragone_threshold") == 0 || a.compare("safeharbor_lat") == 0 ||
      a.compare("safeharbor_lon") == 0 || a.compare("safeharbor_alt") == 0 ||
      a.compare("cameratosafe_threshold") == 0 || a.compare("safeharbor_threshold") == 0 ||
      a.compare("cameragone_altitude") == 0 || a.compare("movetosafeharbor_altitude") == 0 ||
      a.compare("move_alpha") == 0 || a.compare("cameraonline_threshold") == 0 ||
      a.compare("home_longitude") == 0 || a.compare("home_latitude") == 0 ||
      a.compare("home_altitude") == 0 || a.compare("rotation_threshold") == 0 ||
      a.compare("battery_threshold2") == 0 || a.compare("target_dispersed") == 0 ||
      a.compare("reachplatform_threshold") == 0 || a.compare("reachplatfrom_altitude") == 0 ||
      a.compare("platform_rdv_lon") == 0 || a.compare("platform_rdv_lat") == 0 ||
      a.compare("platform_rdv_alt") == 0 || a.compare("platform") == 0)
    {
      return insertValue(a,value);
    }

    std::cout << "Unrecognised type at position: "<< pos << std::endl;
    return false;
}


bool SafetyParser::insertValue(std::string a, float value)
{
  if(std::isfinite(value))
  {
    // inserting it in parameters Map
    _paramsMapParsed.insert({a, value});
    // counting it
    _num_params++;
    // and returning success
    return  true;
  }
  // else returning fail
  std::cout << "value is not finite" << std::endl;
  return false;
}


std::map<std::string, float> SafetyParser::getParamsMapParsed()
{

    return _paramsMapParsed;

}
