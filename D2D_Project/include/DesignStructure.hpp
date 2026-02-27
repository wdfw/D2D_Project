#pragma once

#include <iostream>
#include <vector>
#include <tuple>
#include <utility>

#include "Geometry.hpp"

enum BumpType {
    DUMMY, // 擴充點
    SIGNAL, // 信號點
    VDD, // 電源點
    VSS, // 接地點
} ;

enum DieType {
    DIE1, // 左邊晶片
    DIE2, // 右邊晶片
    DIE12 // 兩邊晶片
} ;

using ColorCode = std::string ;

class BI_Key ; // 具有Bump Type, ID的鍵值
class DBI_Key ; // 具有Die Type, Bump Type, ID的鍵值

class Label ; 
class Point ; 
class Edge ; 

class Bump ; 
class OffsetVia ; 
class Net ; 
class Teardrop ; 

std::ostream& operator<<(std::ostream& os, const BumpType& type) ;
std::ostream& operator<<(std::ostream& os, const DieType& type) ;

std::ostream& operator<<(std::ostream& os, const BI_Key& key) ;
std::ostream& operator<<(std::ostream& os, const DBI_Key& key) ;

std::ostream& operator<<(std::ostream& os, const Label& label) ;
std::ostream& operator<<(std::ostream& os, const Point& point) ;
std::ostream& operator<<(std::ostream& os, const Edge& edge) ;

std::ostream& operator<<(std::ostream& os, const Bump& bump) ;
std::ostream& operator<<(std::ostream& os, const OffsetVia& offsetVia) ;
std::ostream& operator<<(std::ostream& os, const Net& net) ;
std::ostream& operator<<(std::ostream& os, const Teardrop& teardrop) ;

std::string to_formatted_string(const Label& label) ; 
std::string to_formatted_string(const Point& point) ; 
std::string to_formatted_string(const Edge& edge) ; 
std::string to_formatted_string(const Bump& bump) ; 
std::string to_formatted_string(const OffsetVia& offsetVia) ; 
std::string to_formatted_string(const Net& net) ; 
std::string to_formatted_string(const Teardrop& teardrop) ; 
std::string to_formatted_string(const ColorCode& colorCode) ; 
std::string to_formatted_string(const BumpType& bumpType) ; 
std::string to_formatted_string(const DieType& dieType) ; 

class BI_Key {
public:
    BumpType type ;
    int id ;
    BI_Key(BumpType type, int id) : type(type), id(id) {}
    bool operator==(const BI_Key& key) const ;  
    bool operator!=(const BI_Key& key) const ;  
} ;

class DBI_Key {
public:
    DieType die ; 
    BumpType type ;
    int id ;
    DBI_Key(DieType die, BumpType type, int id) : die(die), type(type), id(id) {}
    bool operator==(const DBI_Key& key) const ;  
    bool operator!=(const DBI_Key& key) const ;  
} ;

class Label : public point_xy {
public:
    std::string labelStr ; 
    unsigned int fontSize ;
    ColorCode color ;
    Label(const std::string& labelStr, const point_xy& point, unsigned int fontSize = 14, ColorCode color = "#FFFFFF") 
        : labelStr(labelStr), point_xy(point), fontSize(fontSize), color(color) {}
} ; 

class Point : public point_xy {
public:
    double radius ;
    ColorCode color ;
    Point(const point_xy& point,double radius, ColorCode color = "#FFFFFF") 
        :  point_xy(point), radius(radius), color(color) {}
} ; 

class Edge : public segment_xy {
public:
    double width ;
    ColorCode color ;
    Edge(const segment_xy& segment, double width, ColorCode color = "#FFFFFF") 
        : segment_xy(segment), width(width), color(color) {}
} ; 

class Bump : public DBI_Key, public point_xy {
public:
    bool isOffsetVia ;
    bool isRouting ;
    Bump(DieType die=DIE12, BumpType type=DUMMY, int id=-1, const point_xy& point = {0,0}, bool isOffsetVia = false, bool isRouting = false) 
        : DBI_Key(die, type, id), point_xy(point), isOffsetVia(isOffsetVia), isRouting(isRouting) {} 
    bool operator<(const Bump& rhs) const ; 
} ;

class OffsetVia : public DBI_Key, public std::pair<Bump, Bump> {
public:
    OffsetVia(const Bump& bump1, const Bump& bump2) : DBI_Key(bump1.die, bump1.type, bump1.id), std::pair<Bump, Bump>(bump1, bump2) {
        if(bump1!=bump2) throw std::logic_error("Via and Offset via should have same keys.") ; 
        second.isOffsetVia = true ;
    };
} ;

class Net : public BI_Key, public std::vector<segment_xy> {
public:
    Net(BumpType type, int id, const std::vector<segment_xy>& sequence=std::vector<segment_xy>()) 
        : BI_Key(type, id), std::vector<segment_xy>(sequence) {} ;
} ;

class Teardrop : public DBI_Key, public segment_xy {
public:
    Teardrop(DieType die, BumpType type, int id, const segment_xy& segment = {{0, 0}, {0, 0}})  
        : DBI_Key(die, type, id), segment_xy(segment) {}
} ;
//  error: invalid use of non-static data member ‘DBI_Key::die’