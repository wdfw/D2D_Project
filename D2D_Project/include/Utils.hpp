#pragma once

#include <iostream>
#include <chrono>
#include <algorithm>
#include <random>
#include <ctime>

class Timer ; // 計時工具
class RandomTool ; // 統一用的亂數工具

std::string strip(const std::string& str) ; // 截斷字串左右兩邊的空白符(\n, ' ', '\t')
std::string get_trailing_number(const std::string& str) ; // 取得字串尾端的數字
std::string get_string_before(const std::string& str, char sp) ; // 取得sp後的子字串


class Timer {
private:
    std::chrono::steady_clock::time_point _clock_time1 ;
    std::clock_t _clock_time2 ;
public:
    Timer() ;
    void set_clock() ;
    std::clock_t get_duration_seconds() const ;
    std::clock_t get_duration_milliseconds() const ;
};

class RandomTool {
public :
    std::default_random_engine randEngine ; 

    RandomTool(int seed = 0) : randEngine(seed) {}
    int rand(int l=0, int u=1073741824) ;
    bool Bernoulli_trial(double pr) ; 
    void srand(int seed) ;
}  ;




