//
// Created by andreanistico on 31/01/17.
//

#ifndef LANDINGPLAT_WAVEGEN_H
#define LANDINGPLAT_WAVEGEN_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <random>

class WaveGen {

private:
    int _n;     //number of sampled frequencies
    double _maxW;
    double _minW;
    double deltaW;
    double _U; //wind speed
    double S(double U , double w);
    double eps(double w);

public:
    void info();
    //sampled frequencies
    std::vector < double > _w_vect;
    //random phases
    std::vector <double> _phi_vect;
    void init(int n, double maxw, double minw, double U);
    double generateWaveHeightAtPoint(double x, double y, double t, double beta = 0);

};


#endif //LANDINGPLAT_WAVEGEN_H
