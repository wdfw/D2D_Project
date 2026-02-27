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