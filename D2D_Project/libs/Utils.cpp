#include "Utils.hpp"

Timer::Timer() {
    set_clock();
}

void Timer::set_clock() {
    _clock_time1 = std::chrono::steady_clock::now(); 
    _clock_time2 = clock() ; 
}

std::clock_t Timer::get_duration_seconds() const {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _clock_time1).count() ;
    // return double(clock()-_clock_time2)/CLOCKS_PER_SEC ;
}

std::clock_t Timer::get_duration_milliseconds() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _clock_time1).count() ;
    // return double(clock()-_clock_time2)/CLOCKS_PER_SEC*1000.0 ;
}

std::string strip(const std::string& str){
    int l=0, r=str.size(), f=0 ; 
    for(int i=0; i<str.size(); i++){
        if(isspace(str[i])){
            if(!f){
                l = i+1 ;  
            }
        }else{
            if(!f) f = 1 ;
            else r = i+1 ;
        }
    }
    return str.substr(l, r-l) ;
}

std::string get_string_before(const std::string& str, char sp){
    return str.substr(0,str.find_first_of(sp)) ; 
}

std::string get_trailing_number(const std::string& str){
    std::string res ;
    for(auto c=str.crbegin(); c!=str.crend(); ++c){
        if(isdigit(*c))  res.push_back(*c) ;
        else break ;
    }
    std::reverse(res.begin(), res.end()) ;
    return res ;
}

void RandomTool::srand(int seed){
    randEngine = std::default_random_engine(seed) ;
}

int RandomTool::rand(int l, int u){
    return std::uniform_int_distribution<>(l,u)(randEngine) ;
}
    
bool RandomTool::Bernoulli_trial(double pr){
    return std::bernoulli_distribution(pr)(randEngine) ;
}
