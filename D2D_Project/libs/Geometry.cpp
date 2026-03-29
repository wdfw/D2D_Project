#include "Geometry.hpp"

std::ostream& operator<<(std::ostream& os, const point_xy& point){
    os << point.x() << "," << point.y() ;
    return os ; 
}

std::ostream& operator<<(std::ostream& os, const box_xy& box){
    os << box.min_corner() << " " << box.max_corner() ;
    return os ; 
}

std::ostream& operator<<(std::ostream& os, const segment_xy& segment){
    os << segment.first << " " << segment.second ;
    return os ; 
}

std::ostream& operator<<(std::ostream& os, const polygon_xy& polygon){
    os << bg::wkt(polygon) ;
    return os ; 
}
std::ostream& operator<<(std::ostream& os, const linestring_xy& linestring){
    for(int i=0; i<linestring.size(); ++i){
        os << linestring[i] << " " ;
    }
    return os ; 
}


