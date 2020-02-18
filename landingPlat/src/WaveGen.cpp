//
// Created by andreanistico on 31/01/17.
//

#include "WaveGen.h"

void WaveGen::init(int n, double maxw, double minw, double U) {

    _n = n;
    _minW = minw;
    _maxW = maxw;
    _U = U;

    //Start random engine
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_real_distribution<> distr(0, 2 * M_PI); // define the range

    if (_n >0) deltaW = (_maxW - _minW) / _n;
    else return;

    //Fill frequency vector
    for (int i = 0; i < _n; ++i) {
        double t = _minW + i * deltaW;
        _w_vect.push_back(t);
        std::cout << "omegas: "<<_w_vect[i] << std::endl;
    }
    //Fill phi vector
    for(int i=0; i<_n; ++i) {
        _phi_vect.push_back((float) distr(eng));
        std::cout << "phis: "<<_phi_vect[i] << std::endl;
    }



}

double WaveGen::S(double U, double w) {

    double A = (0.78 / pow(w,5));
    //if (U  == 0) return A;

    double t = A * (exp(-(0.74 * pow(9.81,4)) / (pow(U * w,4))));

    return t;

}

double WaveGen::eps(double w) {

    double t;

    t = sqrt(2 * S(_U,w) * deltaW);

    return t;
}

double WaveGen::generateWaveHeightAtPoint(double x, double y, double t, double beta) {

    double h = 0;
    //fetch values

    for (int i = 0; i < _n; ++i) {

        double wi  = _w_vect[i];
        double phi = _phi_vect[i];
        double ki  = pow(wi,2) / 9.81;
        double A   = eps(wi);

        h = h + A * cos(ki * x * cos(beta) + ki * y * sin(beta) - wi * t + phi);

    }

    return h;
}

void WaveGen::info() {
    std::cout <<"N: " <<_n << std::endl;
    std::cout <<"deltaw: " <<deltaW << std::endl;
    std::cout <<"_maxW: " <<_maxW << std::endl;
    std::cout <<"_minW: " <<_minW << std::endl;
    std::cout <<"_U: "    <<_U << std::endl;

}
























