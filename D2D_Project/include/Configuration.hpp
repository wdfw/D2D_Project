#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "Utils.hpp"

struct GAConfiguration ;
struct SAConfiguration ;

std::ostream& operator<<(std::ostream& os, const GAConfiguration& config) ;
std::ostream& operator<<(std::ostream& os, const SAConfiguration& config) ;

extern GAConfiguration GlobalGAConfig ; 
extern SAConfiguration GloablSAConfig ; 


extern double GlobalResolution ;
extern double GlobalEpsilonX ;
extern double GlobalEpsilonY ;
extern double GlobalEpsilonR ;
extern int GlobalPositionResolution ;
extern int GlobalThreadNum ;
extern Timer GlobalTimer ; 
extern RandomTool GlobalRandomTool ; 

#define ELAPSE_TIME(execution) GlobalTimer.set_clock() ; execution ;  std::cout << "Elapsed Time in construcion: " << GlobalTimer.get_duration_milliseconds() << " ms\n"  ;
// #define PARALLEL_MODE

struct GAConfiguration {
    int populationSize = 1 ;
    double crossoverRate = 0.9 ;
    double mutationRate = 1;
    int generations = 200 ; 
    int tournamentSize = 8 ;  
    double alpha = 1.0 ;
    double beta = 100000.0 ;
    double gamma = 100000.0 ;
    double delta = 1.5 ;
}  ;

struct SAConfiguration {
    double tempature = 10.0 ;
    double decayRatio = 0.1 ;
    double rejectRatio = 0.95 ;
    double uphillLimit = -1 ;
    double moveInEachTempature = 1000.0 ; 
    double timeLimite = 10*1000 ;
} ;
