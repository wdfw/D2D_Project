#include "DesignRule.hpp"

std::ostream& operator<<(std::ostream& os, const DesignRule& rule) {
    int width = 40 ; 
    int precision = 6 ; 
    int frameWidth =  width + 3;

    std::vector<std::string> titles = {
        "Via Radius:",
        "Via Pad Radius:",
        "Minimum Via Spacing:",
        "Minimum Via Pad Spacing:",
        "Minimum Line Width:",
        "Minimum Line Spacing:",
        "Minimum Line Via Pad Spacing:",
        "Minimum Teardrop Distance:"
    } ;

    std::vector<double> values = {
        rule.viaRadius, 
        rule.viaPadRadius, 
        rule.minimumOffsetViaSpacing, 
        rule.minimumViaPadSpacing, 
        rule.minimumLineWidth, 
        rule.minimumLineSpacing, 
        rule.minimumLineViaPadSpacing, 
        rule.minimumTeardropDist
    } ;

    os << std::string(frameWidth, '-') << "\n";
    for(int i=0; i<values.size(); ++i){
        os << titles[i] << std::setw(width-titles[i].size()) << std::setprecision(precision) << values[i] << "\n" ;
    }
    os << std::string(frameWidth, '-') << "\n";

    return os ; 
}

void GeometricStrategy::set_strategy(const DesignRule& design_rule) {
    const int points_per_circle = 36 ;  
    viaStrategy = bgsb::distance_symmetric<double>(design_rule.viaRadius) ; 
    viaPadStrategy = bgsb::distance_symmetric<double>(design_rule.viaPadRadius) ;
    lineWidthStrategy = bgsb::distance_symmetric<double>(design_rule.minimumLineWidth/2) ; 
    joinStrategy = bgsb::join_round(points_per_circle) ; 
    endStrategy = bgsb::end_round(points_per_circle) ;
    sideStrategy = bgsb::side_straight() ; 
    circleStrategy = bgsb::point_circle() ; 
} ;

void GeometricStrategy::buffer_via(const point_xy& pt, multi_polygon_xy& poly) const {
    boost::geometry::buffer(pt, poly, viaStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
}

void GeometricStrategy::buffer_viapad(const point_xy& pt, multi_polygon_xy& poly) const {
    boost::geometry::buffer(pt, poly, viaPadStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
}
void GeometricStrategy::buffer_line(const linestring_xy& line, multi_polygon_xy& poly) const {
    boost::geometry::buffer(line, poly, lineWidthStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
}
