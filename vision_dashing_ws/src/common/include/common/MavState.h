

#ifndef MOCAP2MAV_MAVSTATE_H
#define MOCAP2MAV_MAVSTATE_H

#include <cmath>
#include <cstdint>
#include <math.h>
#include <Eigen/Eigen>


/** @ingroup Automatic
 * @brief It is a class which allows to set: position, velocity, orientation and bool variable to control drones.
 */

class MavState {



public:
    //MavState(const MavState&) = default;

    MavState();

    /**
     *  It allows to diversificate between a velocity command or a position command.
     */

    enum type{

        POSITION,
        VELOCITY

    };
	
	/**
	 * @brief Such method sets the position
	 * 
	 * @param x X position 
	 * @param y Y position 
	 * @param z Z position 
	 */

	void setPosition(double x, double y, double z);
    
	/**
	 * @brief Such method sets the X position
	 * 
	 * 
	 * @param x X position 
	 */

	void setX(double x);
    
	/**
	 * @brief Such method sets the Y position
	 * 
	 * 
	 * @param y Y position 
	 */

    void setY(double y);
    
    /**
     * @brief Such method sets the Z position
     * 
     * 
     * @param z Z position 
     */

    void setZ(double z);
    
    /**
     * @brief Such method returns the X position 
     * 
     * @return X position
     */

    double getX() const;
    
    /**
     * @brief Such method returns the Y position 
     * 
     * @return Y position
     */

    double getY() const;

    /**
     * @brief Such method returns the Z position 
     * 
     * @return Z position
     */

    double getZ() const;

    /**
     * @brief Such method allows to set the velocity vector
     * 
     * @param vx Velocity on x axis
     * @param vy Velocity on y axis
     * @param vz Velocity on z axis
     */

    void setV(double vx, double vy, double vz);
    
    /**
     * @brief Such method allows to set the X velocity 
     * 
     * @param v Velocity on X axis
     */

    void setVx(double v);
    
    /**
     * @brief Such method allows to set the Y velocity 
     * 
     * @param v Velocity on Y axis
     */

    void setVy(double v);

    /**
     * @brief Such method allows to set the Z velocity
     * 
     * @param v Velocity on Z axis
     */

    void setVz(double v);
    
    /**
     * @brief Such method returns the X velocity vector
     * @return Velocity on X axis
     */

    double getVx() const;
    
    /**
     * @brief Such method returns the Y velocity vector
     * @return Velocity on Y axis
     */

    double getVy() const;
    
    /**
     * @brief Such method returns the Z velocity vector
     * @return Velocity on Z axis
     */

    double getVz() const;

	/**
     * @brief Such method allows to set roll angle
     * @param yaw Roll angle of the drone.
     */

	void setRoll(double roll);

	/**
     * @brief Such method allows to set pitch angle
     * @param pitch Pitch angle of the drone.
     */

	void setPitch(double pitch);

    /**
     * @brief Such method allows to set yaw angle
     * @param yaw Yaw angle of the drone.
     */

    void setYaw(double yaw);

	/**
     * @brief Such method returns the roll angle 
     * @return The roll angle
     */

	double getRoll() const;

	/**
     * @brief Such method returns the pitch angle 
     * @return The pitch angle
     */

	double getPitch() const;
    /**
     * @brief Such method returns the yaw angle 
     * @return The yaw angle
     */

    double getYaw() const;

    /**
     * @brief Such method returns the yaw from the quaternion previously set
     * @return The yaw angle
     */

    double getYawFromQuat() const;

    /**
     * @brief Such method sets the orientation using the Eigen quaternion
     * @param quat Eigen quaternion
     */
   
    void setOrientation(Eigen::Quaterniond quat);
    
    /**
     * @brief Such method sets the quaternion using four parameters 
     * @param qw Scalar part of the quaternion
     * @param qx Quaternion part on X axis
     * @param qy Quaternion part on Y axis
     * @param qz Quaternion part on Z axis
     */

    void setOrientation(double qw, double qx, double qy, double qz);

    /**
     * @brief Such method returns the position vector  
     * @param x Position on X axis
     * @param y Position on Y axis
     * @param z Position on Z axis
     */

    void getPosition(double& x, double& y, double& z) const;

    /**
     * @brief Such method returns Eigen quaternion
     * @return Eigen quaternion
     */

    Eigen::Quaterniond getOrientation() const;

    /**
     * @brief Such method allows to set roll, pitch and yaw from Eigen quaternion
     * @param roll Angle on X axis
     * @param pitch Angle on Y axis
     * @param yaw Angle on Z axis
     */
    
    void getOrientationRPY(double& roll, double& pitch, double& yaw) const;

    /**
     * @brief Such method sets the type of command: velocity or position command.
     * @param t Type of command
     */

    void setType(type t);

    /**
     * @brief Such method returns the type of command
     * @return Type of command
     */

    type getType() const;

    /**
     * @brief Equality operator 
     * @param m MavState object
     */

    void operator=(const MavState &m){

        this->setPosition(m.getX(),m.getY(),m.getZ());

        this->setOrientation(m.getOrientation());

        this->setV(m.getVx(),m.getVy(),m.getVz());

        this->setYaw(m.getYaw());

		this->setRoll(m.getRoll());

		this->setRoll(m.getPitch());

        this->setType(m.getType());

	this->VisionDataUpdated = m.VisionDataUpdated;
    	
	this->IsValid = m.IsValid;
    
	this->UltrasonicDataUpdated = m.UltrasonicDataUpdated; 	
	
	}

	/**
	 *Actual time
	 */

    long int timestamp;
	
    /**
     *Bool variable to check if vision data are updated.
     */

	bool VisionDataUpdated;

	/**
	 *Bool variable to check if the information are valid. 
	 *In this case, it is used for ultrasonic sensor.
	 */

	bool IsValid;

	/**
	 *Bool variable to check if the ultrasonic data are updated.
	 */

	bool UltrasonicDataUpdated;			

/**
 * State of the drone: position, velocity, orientation.
 */
private:

    double _x;
    double _y;
    double _z;
    double _vx;
    double _vy;
    double _vz;
	double _roll;
	double _pitch;
    double _yaw;
    type   _type;
    Eigen::Quaterniond _orientation;

};


#endif //MOCAP2MAV_MAVSTATE_H
