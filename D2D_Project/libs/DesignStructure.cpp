#include "DesignStructure.hpp"

bool BI_Key::operator==(const BI_Key& key) const {
    return type==key.type && id==key.id ;
}

bool BI_Key::operator!=(const BI_Key& key) const {
    return !(*this==key) ;
}

bool DBI_Key::operator==(const DBI_Key& key) const {
    return die==key.die && type==key.type && id==key.id ;
}

bool DBI_Key::operator!=(const DBI_Key& key) const {
    return !(*this==key) ;
}

std::ostream& operator<<(std::ostream& os, const BumpType& type) {
    switch (type){
        case DUMMY : 
            os << "Dummy" ;
            break;
        case SIGNAL :  
            os << "Signal" ;
            break ;
        case VDD :  
            os << "Vdd" ;
            break ;
        case VSS  :  
            os << "Vss" ;
            break ;
        default :
            throw std::invalid_argument("Unknown BumpType") ;
            break;
    }
    return os ;
}

std::ostream& operator<<(std::ostream& os, const DieType& die) {
    switch (die){
        case DIE1 : 
            os << "Die1" ;
            break;
        case DIE2 :  
            os << "Die2" ;
            break ;
        case DIE12 :  
            os << "Die12" ;
            break ;
        default :
            throw std::invalid_argument("Unknown DieType") ;
            break;
    }
    return os ;
}

std::ostream& operator<<(std::ostream& os, const BI_Key& key) {
    os << "(" << key.type << "," << key.id << ")" ; 
    return os ;
}

std::ostream& operator<<(std::ostream& os, const DBI_Key& key) {
    os << "(" << key.die << "," << key.type << "," << key.id << ")" ; 
    return os ;
}

std::ostream& operator<<(std::ostream& os, const Label& label) {
    os << label.labelStr << " " << static_cast<const point_xy&>(label) ; 
    return os ;
}

std::ostream& operator<<(std::ostream& os, const Point& point) {
    os << " " << static_cast<const point_xy&>(point) ; 
    return os ;
}

std::ostream& operator<<(std::ostream& os, const Edge& edge) {
    os << " " << edge.first << " -> " << edge.second ; 
    return os ;
}

std::ostream& operator<<(std::ostream& os, const Bump& bump) {
    os << static_cast<const DBI_Key&>(bump) << " " << static_cast<const point_xy&>(bump) ;
    return os ;
}

std::ostream& operator<<(std::ostream& os, const OffsetVia& offsetVia) {
    os << " " << offsetVia.first << " -> " << offsetVia.second ; 
    return os ;
}
std::ostream& operator<<(std::ostream& os, const Net& net) {
    for(int i=0; i<net.size(); ++i){
        os << net[i] ;
        if(i!=net.size()-1) os << "\n" ;
    }
    return os ; 
}

std::ostream& operator<<(std::ostream& os, const Teardrop& teardrop) {
    os << " " << teardrop.first << " -> " << teardrop.second ; 
    return os ;
}

std::string to_formatted_string(const ColorCode& colorCode) {
    // std::string str = "0X";
    
    // for(int i=0, c=colorCode, r; i<6; ++i){
    //     r = colorCode % 16 ; c /= 16 ; 
    //     if(r<10) str.push_back('0'+r) ; 
    //     else str.push_back('A'+r-10) ; 
    // }
    return colorCode ; 
}

std::string to_formatted_string(const BumpType& bumpType) {
    switch (bumpType){
        case DUMMY : return "DUMMY" ;
        case SIGNAL : return "SIGNAL" ;
        case VDD : return "VDD" ;
        case VSS  : return "VSS" ;
        default : throw std::invalid_argument("Unknown BumpType") ;
    }
}

std::string to_formatted_string(const DieType& dieType) {
    switch (dieType){
        case DIE1 : return "DIE1" ;
        case DIE2 : return "DIE2" ;
        case DIE12 : return "DIE12" ;
        default : throw std::invalid_argument("Unknown DieType") ;
    }
}

std::string to_formatted_string(const Label& label) {
    std::string str =    label.labelStr + " " + std::to_string(label.x()-label.fontSize/2) + " " + std::to_string(label.y()-label.fontSize/2) + " " +
                    std::to_string(label.fontSize) + to_formatted_string(label.color) ; 
    return str ;
}

std::string to_formatted_string(const Point& point) {
    std::string str =    std::to_string(point.x()) + " " + std::to_string(point.y()) + " " +
                    std::to_string(point.radius) + to_formatted_string(point.color) ; 
    return str ;
}

std::string to_formatted_string(const Edge& edge) {
    std::string str =   std::to_string(edge.first.x()) + " " + std::to_string(edge.first.y()) + " " +
                        std::to_string(edge.second.x()) + " " + std::to_string(edge.second.y()) + " " +
                        std::to_string(edge.width) + to_formatted_string(edge.color) ; 
    return str ;
}

std::string to_formatted_string(const Bump& bump) {
    std::string str =   to_formatted_string(bump.die) + " " +
                        to_formatted_string(bump.type) + " " +
                        std::to_string(bump.id) + " " +
                        std::to_string(bump.x()) + " " +
                        std::to_string(bump.y()) ;
    return str ;
}
std::string to_formatted_string(const OffsetVia& offsetVia) {
    std::string str =   to_formatted_string(offsetVia.die) + " " +
                        to_formatted_string(offsetVia.type) + " " +
                        std::to_string(offsetVia.id) + " " +
                        std::to_string(offsetVia.first.x()) + " " +
                        std::to_string(offsetVia.first.y()) + " " +
                        std::to_string(offsetVia.second.x()) + " " +
                        std::to_string(offsetVia.second.y()) ;
    return str ;
} 

std::string to_formatted_string(const Net& net) {
    std::string str, temp ;
    
    for(auto& segment : net){
        temp =  to_formatted_string(net.type) + " " +
                std::to_string(net.id) + " " +
                std::to_string(segment.first.x()) + " " +
                std::to_string(segment.first.y()) + " " +
                std::to_string(segment.second.x()) + " " +
                std::to_string(segment.second.y()) ;
        str += temp + "\n" ;
    }

    return str ; 
}
// std::string to_formatted_string(const Teardrop& teardrop) ; 