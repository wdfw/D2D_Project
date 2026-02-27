#pragma once
#include <iostream>
#include <vector>
#include <iomanip>


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