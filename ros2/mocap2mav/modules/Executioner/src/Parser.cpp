//
// Created by andreanisti on 27/10/16.
//

#include "Parser.h"
#define PI 3.141592653589



Parser::Parser() {}

bool Parser::loadFile(std::string file) {

    std::ifstream myfile (file);
    if (myfile.is_open()) {
        //Clear token vector
        _tokens.clear();
        // Why not?
        _tokens.shrink_to_fit();

        //Build tokenized vector line by line
        std::string line;
        while ( getline (myfile,line)){

            if(line == "#") break;
            std::vector<std::string> str_vector;
            strtk::parse(line,"=",str_vector);

            _tokens.push_back(str_vector);

        }

        myfile.close();
        return true;
    }
    else{

        std::cout << "Unable to open file" << std::endl;
        return false;

    }

}

bool Parser::parse() {

    //Save type position
    std::vector<int> type_pos;

    for (int i = 0; i < _tokens.size(); ++i) {

        if(_tokens[i][0] == "type") type_pos.push_back(i);


    }
    for (int k = 0; k < type_pos.size(); ++k) {
        std::cout << type_pos[k] << std::endl;
    }

    //Start Parsing
    if(type_pos.size() > 0) {


        for (int j = 0; j < type_pos.size(); j++) {

            //Store the position in the token vector of the value "type"
            int pos = type_pos[j];

            //Parse single actions
            if(!parseAction(pos)) return false;

        }

    } else {

        std::cout << "Invalid tokens count" << std::endl;
        return false;

    }
    return true;

}

double Parser::getValue(std::string str)  {
    char *cstr = new char[str.length() + 1];
    strcpy(cstr, str.c_str());
    double value = atof(cstr);
    delete (cstr);
    return value;
}

bool Parser::parseAction(int pos) {

    //Look for the action
    std::string a = _tokens[pos][1];
    //create task
    exec::task task;

    //Select the right action and parse it
    bool res = false;
    if(a == "takeoff"){

        res = parseTakeOff(task,pos);
        return  res;
    }
    else if(a == "move"){

        res = parseMove(task,pos);
        return  res;
    }
    else if(a == "search"){

        res = parseSearch(task,pos);
        return  res;
    }
    else if (a == "rotate") {

        res = parseRotate(task,pos);
        return res;
    }
    else if (a == "land") {

        res = parseLand(task, pos);
        return res;
    }
    else if (a == "idle"){

        res = parseIdle(task);
        return res;
    }else {
        std::cout << "Unrecognised type at position: "<< pos << std::endl;
        return false;
    }

}

std::deque<exec::task> Parser::getTaskListParsed() {

    return _taskListParsed;

}

bool Parser::parseTakeOff(exec::task& task,int pos) {

    std::cout << "Takeoff found" << std::endl;
    const unsigned char zFound = 0b01; //
    const unsigned char trFound = 0b10; //
    unsigned char mask = 0b00;
    task.action = common::actions::TAKE_OFF;

    for (int i = pos+1; i < _tokens.size() && _tokens[i][0] != "type" ; ++i) {

        std::string field = _tokens[i][0];
        std::string value_str = _tokens[i][1];

        if(field == "z"){

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if(std::isfinite(value)){
                mask |= zFound;
                task.params[0] = value;
                std::cout << "  Z: " << value << std::endl;

            }else{
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if(field == "id"){

            double value = getValue(value_str);
            task.id = (int)value;

        }
        else if(field == "threshold")
        {
          // if the found value is a threshold
          double value = getValue(value_str);
          // we check its validity
          if(std::isfinite(value) && value >= 0)
          {
            mask |= trFound;
            // and we assign it to task's parameters
            task.params[1] = value;
            std::cout << "  threshold: " <<value<< " found for action: " << pos << std::endl;
          }
          else
          {
            // otherwhise we signal it and we stop the execution
            std::cout << "value threshold is not valid" << std::endl;
            return false;
          }
        }
        else{

            std::cout << "unrecognized field" << std::endl;
            return false;

        }

    }
    if(mask & (zFound | trFound) == (zFound | trFound)) _taskListParsed.push_back(task);
    else
    {
        std::cout << "field missing in action: " << pos << std::endl;
        return false;
    }
    return true;
}

bool Parser::parseMove(exec::task& task,int pos) {
    std::cout << "Move found" << std::endl;
    const unsigned char xFound = 0b00000001; //
    const unsigned char yFound = 0b00000010; //
    const unsigned char zFound = 0b00000100; //
    const unsigned char aFound = 0b00001000; //
    const unsigned char tFound = 0b00010000; //
    const unsigned char hlonFound = 0b00100000; //
    const unsigned char hlatFound = 0b01000000; //
    const unsigned char haltFound = 0b10000000; //
    unsigned char mask = 0b00000000;

    bool trFound = false;

    task.action = common::actions::MOVE;

    for (int i = pos+1; i < _tokens.size() && _tokens[i][0] != "type" ; ++i) {

        std::string field = _tokens[i][0];
        std::string value_str = _tokens[i][1];

        if(field == "target")
        {
          if (value_str == "rescue" || value_str == "platform")
          {
            mask |= tFound;
            task.target=value_str;
          }
          else
          {
            std::cout << "unknown target, please fix" << std::endl;
            return false;
          }

        }
        else if(field == "x") {

            //atof Helper, cast string into double

            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= xFound;
                task.x = value;
                std::cout << "  X: " <<value<< " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if(field == "id"){

            double value = getValue(value_str);
            task.id = (int)value;

        }
        else if(field == "y"){

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= yFound;
                task.y = value;
                std::cout << "  Y: " <<value<< " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if(field == "z"){

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= zFound;
                task.z = value;
                std::cout << "  Z: " <<value<< " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if(field == "home_lon")
        {
          //atof Helper, cast string into double
          double value = getValue(value_str);

          if (std::isfinite(value)) {
              mask |= hlonFound;
              task.params[2] = value;
              std::cout << "  Home_lon: " <<value<< " found for action: " << pos << std::endl;

          } else {
              std::cout << "value is not finite" << std::endl;
              return false;
          }
        }
        else if(field == "home_lat")
        {
          //atof Helper, cast string into double
          double value = getValue(value_str);

          if (std::isfinite(value)) {
              mask |= hlatFound;
              task.params[3] = value;
              std::cout << "  Home_lat: " <<value<< " found for action: " << pos << std::endl;

          } else {
              std::cout << "value is not finite" << std::endl;
              return false;
          }
        }
        else if(field == "home_alt")
        {
          //atof Helper, cast string into double
          double value = getValue(value_str);

          if (std::isfinite(value)) {
              mask |= haltFound;
              task.params[4] = value;
              std::cout << "  Home_alt: " <<value<< " found for action: " << pos << std::endl;

          } else {
              std::cout << "value is not finite" << std::endl;
              return false;
          }
        }
        else if(field == "alpha"){

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= aFound;
                task.params[0] = value;
                std::cout << "  Alpha: " <<value<< " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if(field == "threshold")
        {
          // if the found value is a threshold
          double value = getValue(value_str);
          // we check its validity
          if(std::isfinite(value) && value >= 0)
          {
            trFound = true;
            // and we assign it to task's parameters
            task.params[1] = value;
            std::cout << "  threshold: " <<value<< " found for action: " << pos << std::endl;
          }
          else
          {
            // otherwhise we signal it and we stop the execution
            std::cout << "value threshold is not valid" << std::endl;
            return false;
          }
        }
        else{

            std::cout << "unrecognized field" << std::endl;

        }

    }
    if((mask & (xFound | yFound | zFound | aFound | tFound | hlonFound | hlatFound | haltFound) == (xFound | yFound | zFound | aFound | tFound | hlonFound | hlatFound | haltFound)) && trFound) _taskListParsed.push_back(task);
    else
    {
        std::cout << "field missing in action: " << pos << std::endl;
        return false;
    }
    return true;
}

bool Parser::parseSearch(exec::task& task, int pos) {

    std::cout << "Search found" << std::endl;
    const unsigned char rFound = 0b0001; // hex for 0000 0001
    const unsigned char vFound = 0b0010; // hex for 0000 0010
    const unsigned char zFound = 0b0100; // hex for 0000 0100
    const unsigned char pFound = 0b1000; // hex for 0000 1000
    unsigned char mask = 0b0000;
    task.action = common::actions::SEARCH;

    for (int i = pos+1; i < _tokens.size() && _tokens[i][0] != "type" ; ++i)
    {

        std::string field = _tokens[i][0];
        std::string value_str = _tokens[i][1];

        if (field == "radius")
        {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value))
            {
                mask |= rFound;
                task.params[0] = value;
                std::cout << "Radius: " << value << " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if (field == "velocity")
        {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= vFound;
                task.params[1] = value;
                std::cout << "Velocity: " << value << " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if (field == "plat_dist")
        {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= pFound;
                task.params[2] = value;
                std::cout << "Platform Distancee Threshold: " << value << " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if (field == "id")
        {

            double value = getValue(value_str);
            task.id = (int) value;

        }
        else if(field == "z")
        {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= zFound;
                task.z = value;
                std::cout << "  Z: " <<value<< " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else
        {

            std::cout << "unrecognized field" << std::endl;

        }
    }
    if((mask & (rFound | vFound | zFound | pFound) == (rFound | vFound | zFound | pFound))) _taskListParsed.push_back(task);
    else
    {
        std::cout << "field missing in action: " << pos << std::endl;
        return false;
    }
    return true;
}

bool Parser::parseRotate(exec::task& task, int pos) {

    std::cout << "Rotate found" << std::endl;
    const unsigned char xFound = 0b00001; // hex for 0000 0001
    const unsigned char yFound = 0b00010; // hex for 0000 0010
    const unsigned char aFound = 0b00100; // hex for 0000 0100
    const unsigned char yawFound = 0b01000; // hex for 0000 1000
    const unsigned char trFound = 0b10000; // hex for 0001 0000
    unsigned char mask = 0b00000;
    task.action = common::actions::ROTATE;

    for (int i = pos+1; i < _tokens.size() && _tokens[i][0] != "type" ; ++i) {

        std::string field = _tokens[i][0];
        std::string value_str = _tokens[i][1];

        if (field == "x") {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= xFound;
                task.x = value;
                std::cout << "X: " << value << " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        } else if (field == "y") {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= yFound;
                task.y = value;
                std::cout << "Y: " << value << " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        } else if (field == "id") {

            double value = getValue(value_str);
            task.id = (int) value;

        } else if (field == "yaw") {

            //atof Helper, cast string into double
            double value = getValue(value_str);

            if (std::isfinite(value)) {
                mask |= yawFound;
                task.yaw = value * PI / 180;
                std::cout << "Yaw: " << value << " found for action: " << pos << std::endl;

            } else {
                std::cout << "value is not finite" << std::endl;
                return false;
            }

        }
        else if(field == "threshold")
        {
          // if the found value is a threshold
          double value = getValue(value_str);
          // we check its validity
          if(std::isfinite(value) && value >= 0)
          {
            mask |= trFound;
            // and we assign it to task's parameters
            task.params[1] = value * PI / 180;
            std::cout << "  threshold: " <<value<< " found for action: " << pos << std::endl;
          }
          else
          {
            // otherwhise we signal it and we stop the execution
            std::cout << "value threshold is not valid" << std::endl;
            return false;
          }
        }
        else{

            std::cout << "unrecognized field" << std::endl;

        }
    }
    if((mask & (xFound | yFound | aFound | yawFound | trFound) == (xFound | yFound | aFound | yawFound | trFound))) _taskListParsed.push_back(task);
    else
    {
        std::cout << "field missing in action: " << pos << std::endl;
        return false;
    }
    return true;
    _taskListParsed.push_back(task);
    return true;
}

bool Parser::parseLand(exec::task& task, int pos)
{
    std::cout << "Land found" << std::endl;
    const unsigned char pFound = 0b1; // hex for 0000 0010
    unsigned char mask = 0b0;
    task.action = common::actions::LAND;
    task.z = 0; //Assume that landing is normal, not on a platform. We'll see later
    task.params[0] = 0;

    for (int i = pos+1;  i < _tokens.size() && _tokens[i][0] != "type" ; i++) {
        std::string field = _tokens[i][0];
        std::string value_str;

        if(field == "id"){
            value_str = _tokens[i][1];
            double value = getValue(value_str);
            task.id = (int)value;

        }
        else if(field == "z"){
            value_str = _tokens[i][1];
            double value = getValue(value_str);
            task.z = value;

        }
        else if(field == "platform"){

            value_str = _tokens[i][1];
            mask |= pFound;
            task.params[0] = getValue(value_str);
            std::cout << "  platform: 1 found for action: " << pos << std::endl;

        }
        else{

            std::cout << "unrecognized field" << std::endl;
            return false;

        }

    }
    if((mask & (pFound) == (pFound))) _taskListParsed.push_back(task);
    else
    {
        std::cout << "field missing in action: " << pos << std::endl;
        return false;
    }
    return true;
}

bool Parser::parseIdle(exec::task &task) {
    std::cout << "Idle found" << std::endl;
    task.action = common::actions::IDLE;
    _taskListParsed.push_back(task);
    return true;
}
