#pragma once
#include "Configuration.hpp"


GAConfiguration GlobalGAConfig ; 
SAConfiguration GloablSAConfig ; 

double GlobalResolution  = 100000 ;
double GlobalEpsilonX = 1e-3 ;
double GlobalEpsilonY = 1e-3 ;
double GlobalEpsilonR = sqrt(pow(GlobalEpsilonX,2) + pow(GlobalEpsilonY,2)) ;
int GlobalPositionResolution = 10 ; 
int GlobalThreadNum = 4 ;
Timer GlobalTimer ; 
RandomTool GlobalRandomTool(42) ; 

std::ostream& operator<<(std::ostream& os, const GAConfiguration& config){
     int width = 40 ; 
     int precision = 6 ; 
     int frameWidth =  width + 3;
     
     std::vector<std::string> titles = {
          "Popultation Size:",
          "Number of Generations:",
          "Crossover Rate:",
          "Mutation Rate:",
          "Tournament Size:",
          "Alpha:",
          "Beta:",
          "Gamma:",
          "Delta:"
     } ;

     std::vector<std::string> values = {
          std::to_string(config.populationSize),
          std::to_string(config.crossoverRate),
          std::to_string(config.mutationRate),
          std::to_string(config.generations),
          std::to_string(config.tournamentSize),
          std::to_string(config.alpha),
          std::to_string(config.beta),
          std::to_string(config.gamma),
          std::to_string(config.delta)
     } ;

     os << std::string(frameWidth, '-') << "\n";
     for(int i=0; i<values.size(); ++i){
          os << titles[i] << std::setw(width-titles[i].size()) << values[i] << "\n" ;
     }
     os << std::string(frameWidth, '-') << "\n";
     
     return os ; 
}

