//
// Created by andreanistico on 24/10/17.
//


#include "Lander/Parameters.h"

Parameters::Parameters() {

}

void Parameters::loadConfigFile(const char *config_file) {

    loadParamFile(config_file);
    }

void Parameters::loadParamFile(const char *config_file) {

    
    std::ifstream cFile (config_file);
    if( cFile.is_open() ){

        std::string line;
        
        while( getline(cFile, line) ){
            line.erase( std::remove_if(line.begin(),line.end(), isspace), line.end() );

            if( line[0] == '#' || line.empty() ) continue;

            auto delimiterPos = line.find("=");
            std::string param = line.substr(0, delimiterPos);
            double value = std::stod( line.substr(delimiterPos+1) );


			if ( !param.compare("SEARCH") )
                search = value;

            if ( !param.compare("INSPECTION_RADIUS") )
                inspeRadius = value;

            else if( !param.compare("VELOCITY_INSPECTION") )
                inspeLinVel = value;
        
            else if( !param.compare("ROLL_THRESHOLD") )
                RollThreshold = value;

            else if( !param.compare("PITCH_THRESHOLD") )
                PitchThreshold = value;

            else if( !param.compare("FRAMES_FOR_HOLDING") )
                NFramesHold = value;

            else if( !param.compare("FRAMES_FOR_LOST") )
                NFramesLost = value;

            else if( !param.compare("HOLD_THRESHOLD") )
                hold_threshold = value;

            else if( !param.compare("COMP_THRESHOLD") )
                comp_threshold = value;

            else if( !param.compare("LOST_THRESHOLD") )
                lost_threshold = value;

            else if( !param.compare("LAND_THRESHOLD") )
                land_threshold = value;

            else if( !param.compare("FRAMES_FOR_COMP") )
                NFramesComp = value;

            else if( !param.compare("PLATFORM_LENGHT") )
                platformLenght = value;

            else if( !param.compare("MAX_ALTITUDE") )
                zMax = value;

            else if( !param.compare("MIN_ALTITUDE") )
                zMin = value;

            else if( !param.compare("KP_HOLD_VEL") )
                KpHoldV = value;

            else if( !param.compare("KP_HOLD") )
                KpHold = value;

            else if( !param.compare("KD_HOLD") )
                KdHold = value;

            else if( !param.compare("KI_HOLD") )
                KiHold = value;

            else if( !param.compare("KP_COMP_VEL") )
                KPCompV = value;

            else if( !param.compare("MAX_INT_VALUE") )
                maxIntValue = value;
                
            else if( !param.compare("MIN_INT_VALUE") )
                minIntValue = value;                    

            else if( !param.compare("MAX_OUTPUT") )
                maxOutput = value;
            
        }
		cFile.close();



    }
    else {
        std::cerr<<"Not able to open the configuration file"<<std::endl;
    }



    }


