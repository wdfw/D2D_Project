#pragma once

#include <iostream>
#include <sstream>
#include <cmath>
#include <vector>
#include <fstream>
#include <map>
#include <tuple>

#include "DesignStructure.hpp"
#include "DesignRule.hpp"
#include "Utils.hpp"

// for translation between enum and string
BumpType str_to_bump_type(const std::string& str) ; 
DieType str_to_die_type(const std::string& str) ; 

std::string die_type_to_str(const DieType& die) ;
std::string bump_type_to_str(const BumpType& type) ;

// for D2D Algorithm 
void parse_design_bump(const std::string &inputPath, std::vector<Bump>& bumps, box_xy& boundary) ; 
void parse_design_rule(const std::string &inputPath, DesignRule &designRule) ; 

// for D2D GUI
void parse_label(const std::string &inputPath, std::vector<Label>& labels) ; 
void parse_point(const std::string &inputPath, std::vector<Point>& points) ; 
void parse_edge(const std::string &inputPath, std::vector<Edge>& edges) ; 

void parse_bump(const std::string &inputPath, std::vector<Bump>& bumps) ; 
void parse_offset_via(const std::string &inputPath, std::vector<OffsetVia>& offsetVias) ;
void parse_net(const std::string &inputPath, std::vector<Net>& nets) ; 
void parse_teardrop(const std::string &inputPath, std::vector<Teardrop>& teardrops) ;









