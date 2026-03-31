#pragma once
#include <iostream>
#include <vector>
#include <iomanip>

#include "Geometry.hpp"

struct DesignRule ;
std::ostream& operator<<(std::ostream& os, const DesignRule& rule) ;

struct DesignRule {
    double viaRadius; // Bump不含Padding的半徑(不含Padding)
    double viaPadRadius; // Bump含Padding的半徑(含Padding)
    double minimumViaPadSpacing; // Bumps間的最小距離(含Padding)
    double minimumOffsetViaSpacing; // Offset Bumps間的最小距離(不含Padding)
    double minimumLineWidth; // 線寬
    double minimumLineSpacing;  // 線與線之間的距離
    double minimumLineViaPadSpacing; // 線與Bump之間的距離(含黃色)
    double minimumTeardropDist; // teardrop 外面的點, 到 teardrop 圓心的距離
}; 

struct GeometricStrategy {
    bgsb::distance_symmetric<double> viaStrategy = bgsb::distance_symmetric<double>(0.0) ; // Real
    bgsb::distance_symmetric<double> viaPadStrategy = bgsb::distance_symmetric<double>(0.0) ; // Real
    bgsb::distance_symmetric<double> lineWidthStrategy = bgsb::distance_symmetric<double>(0.0) ; // Real
    bgsb::join_round joinStrategy = bgsb::join_round(0) ; // Real
    bgsb::end_round endStrategy = bgsb::end_round(0) ; // Real
    bgsb::side_straight sideStrategy = bgsb::side_straight() ; // Real
    bgsb::point_circle circleStrategy = bgsb::point_circle(36) ; // Real

    void set_strategy(const DesignRule& design_rule) ;

    void buffer_via(const point_xy& pt, multi_polygon_xy& poly) const ;
    void buffer_viapad(const point_xy& pt, multi_polygon_xy& poly) const ;
    void buffer_line(const linestring_xy& line, multi_polygon_xy& poly) const ;

} ;
