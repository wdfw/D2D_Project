#include "Parser.hpp"

BumpType str_to_bump_type(const std::string& str) {
    if(str=="Dummy" || str=="DUMMY") return DUMMY ;
    else if(str=="Signal" || str=="SIGNAL") return SIGNAL ;
    else if(str=="Vdd" || str=="VDD") return VDD ;
    else if(str=="Vss" || str=="VSS") return VSS ;
    throw std::invalid_argument("str_to_bump_type: Unkonwn bump type " + str + ".\n") ;
}

DieType str_to_die_type(const std::string& str) {
    if(str=="DIE1") return DIE1 ;
    else if(str=="DIE2") return DIE2 ; 
    else if(str=="DIE12") return DIE12 ;
    throw std::invalid_argument("str_to_die_type: Unkonwn die type " + str + ".\n") ;
}

std::string die_type_to_str(const DieType& die) {
    switch (die){
        case DIE1 : return "DIE1" ;
        case DIE2 : return "DIE2" ;
        case DIE12 : return "DIE12" ;
        default : throw std::invalid_argument("die_type_to_str: Unknown die type: " + std::to_string(die) + ".\n") ;
    }
    return "UNKNOWN" ;
}

std::string bump_type_to_str(const BumpType& type) {
    switch (type){
        case DUMMY : return "Dummy" ;
        case SIGNAL :  return "SIGNAL" ;
        case VDD :  return "VDD" ;
        case VSS :  return "VSS" ;
        default : throw std::invalid_argument("bump_type_to_str: Unknown bump type: " + std::to_string(type) + ".\n") ;
    }
}

void parse_design_bump(const std::string &inputPath, std::vector<Bump>& bumps, box_xy& boundary) {
    std::ifstream file(inputPath);
    if(!file.is_open()) throw std::runtime_error("[ParseDesignBump] Failed to open file: " + inputPath);

    std::string line, die, type ;
    int id ;
    double x, y, v1, v2 ; ;

    bumps.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);
        if(lineNumber<=2 && (iss >> v1 >> v2)){
            if(lineNumber==1) boundary.min_corner() = {v1, v2} ; 
            else boundary.max_corner() = {v1, v2} ; 
        }else{
            std::istringstream iss(line);
            if (!(iss >> die >> type >> id >> x >> y )) throw std::runtime_error("[ParseDesignBump] Error parsing in line #" + std::to_string(lineNumber)) ;
            bumps.push_back(Bump(str_to_die_type(die), str_to_bump_type(type), id, {x, y}));
        }
    }
}

void parse_design_rule(const std::string &inputPath, DesignRule &designRule){
    
    std::ifstream file(inputPath);
    if(!file.is_open()) throw std::runtime_error("[ParseDesignRule] Failed to open file: " + inputPath);

    std::string line, key;
    
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);

        if (!(iss >> key)) throw std::runtime_error("[ParseDesignRule] Error parsing in line #" + std::to_string(lineNumber)) ;
        if(key == "via_opening_diameter"){
            iss >> designRule.viaRadius ; designRule.viaRadius /= 2 ; 
        }else if(key == "via_pad_diameter"){
            iss >> designRule.viaPadRadius ; designRule.viaPadRadius /= 2 ; 
        }else if(key == "minimum_via_pad_spacing"){
            iss >> designRule.minimumViaPadSpacing ;
        }else if(key == "minimum_via_spacing"){
            iss >> designRule.minimumOffsetViaSpacing ; 
        }else if(key == "minimum_line_width"){
            iss >> designRule.minimumLineWidth ;
        }else if(key == "minimum_line_spacing"){
            iss >> designRule.minimumLineSpacing ;
        }else if(key == "minimum_line_via_spacing"){
            iss >> designRule.minimumLineViaPadSpacing ;
        }else if(key == "minimum_teardropDist"){
            iss >> designRule.minimumTeardropDist ;
        }else{
            std::cerr << "Unknown Rule:" << key << std::endl;
        }
    }
    
    if(designRule.viaRadius>designRule.viaPadRadius){
        throw std::runtime_error("[ParseDesignRule] Invalid design rule : via opening diameter larger than via pad diameter");
    }
    file.close();
}

void parse_label(const std::string &inputPath, std::vector<Label>& labels) {
    std::ifstream file(inputPath) ;
    if(!file.is_open()) throw std::runtime_error("[ParseLabel] Failed to open file: " + inputPath);

    std::string line, labelStr ; 
    double x, y ;
    unsigned int fontSize ;
    ColorCode color ;

    labels.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);

        if (!(iss >> labelStr >> x >> y >> fontSize >> color )) throw std::runtime_error("[ParseLabel] Error parsing in line #" + std::to_string(lineNumber)) ;
        labels.push_back(Label(labelStr, {x, y}, fontSize, color)) ;
    }
}

void parse_point(const std::string &inputPath, std::vector<Point>& points) {
    std::ifstream file(inputPath) ;
    if(!file.is_open()) throw std::runtime_error("[ParsePoint] Failed to open file: " + inputPath);

    std::string line ; 
    double x, y, radius ;
    ColorCode color ;

    points.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);

        if (!(iss >> x >> y >> radius >> color )) throw std::runtime_error("[ParsePoint] Error parsing in line #" + std::to_string(lineNumber)) ;
        points.push_back(Point({x, y}, radius, color)) ;
    }
}

void parse_edge(const std::string &inputPath, std::vector<Edge>& edges) {
    std::ifstream file(inputPath) ;
    if(!file.is_open()) throw std::runtime_error("[ParseEdge] Failed to open file: " + inputPath);

    std::string line ; 
    double x1, y1, x2, y2, width ;
    ColorCode color ;

    edges.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);

        if (!(iss >> x1 >> y1 >> x2 >> y2 >> width >> color )) throw std::runtime_error("[ParseEdge] Error parsing in line #" + std::to_string(lineNumber)) ;
        edges.push_back(Edge({{x1, y1}, {x2, y2}}, width, color)) ;
    }
}

void parse_bump(const std::string &inputPath, std::vector<Bump>& bumps) {
    std::ifstream file(inputPath);
    if(!file.is_open()) throw std::runtime_error("[ParseBump] Failed to open file: " + inputPath);

    std::string line, die, type ;
    int id ;
    double x, y ;

    bumps.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);
        if (!(iss >> die >> type >> id >> x >> y )) throw std::runtime_error("[ParseBump] Error parsing in line #" + std::to_string(lineNumber)) ;
        bumps.push_back(Bump(str_to_die_type(die), str_to_bump_type(type), id, {x, y}));
    }
}
void parse_offset_via(const std::string &inputPath, std::vector<OffsetVia>& offsetVias) {
    std::ifstream file(inputPath);
    if(!file.is_open()) throw std::runtime_error("[ParseOffsetVia] Failed to open file: " + inputPath);

    std::string line, die, type ;
    int id ;
    double x1, y1, x2, y2 ;

    offsetVias.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);
        if (!(iss >> die >> type >> id >> x1 >> y1 >> x2 >> y2 )) throw std::runtime_error("[ParseOffsetVia] Error parsing in line #" + std::to_string(lineNumber)) ;
        offsetVias.push_back(OffsetVia({
            {str_to_die_type(die), str_to_bump_type(type), id, {x1, y1}}, 
            {str_to_die_type(die), str_to_bump_type(type), id, {x2, y2}}
        })) ;
    }
}

void parse_net(const std::string &inputPath, std::vector<Net>& nets) {
    std::ifstream file(inputPath);
    if(!file.is_open()) throw std::runtime_error("[ParseNet] Failed to open file: " + inputPath);

    std::string line, type ;
    int id ;
    double x1, y1, x2, y2 ;
    
    std::map<std::pair<BumpType, int>, std::vector<segment_xy>> mappingSequences ; 
    
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);
        if (!(iss >> type >> id >> x1 >> y1 >> x2 >> y2)) throw std::runtime_error("[ParseNet] Error parsing in line #" + std::to_string(lineNumber)) ;
        
        segment_xy segment({x1, y1}, {x2, y2}) ;
        mappingSequences[{str_to_bump_type(type), id}].push_back(segment) ; 
        // nets.push_back(Net(str_to_bump_type(type), id, {segment})) ;
    }

    nets.clear() ; 
    for(auto& [key, sequence] : mappingSequences){
        nets.push_back(Net(key.first, key.second, sequence)) ; 
    }
}

void parse_teardrop(const std::string &inputPath, std::vector<Teardrop>& teardrops) {
    std::ifstream file(inputPath);
    if(!file.is_open()) throw std::runtime_error("[ParseTeardrop] Failed to open file: " + inputPath);

    std::string line, die, type ;
    int id ;
    double x1, y1, x2, y2 ;

    teardrops.clear() ; 
    for(int lineNumber=1; getline(file, line); ++lineNumber){
        if((line = strip(line)).empty()) continue ;
        std::istringstream iss(line);
        if (!(iss >> die >> type >> id >> x1 >> y1 >> x2 >> y2 )) throw std::runtime_error("[ParseTeardrop] Error parsing in line #" + std::to_string(lineNumber)) ;
        teardrops.push_back(Teardrop(str_to_die_type(die), str_to_bump_type(type), id, {{x1, y1}, {x2, y2}})) ;

        
    }
}

