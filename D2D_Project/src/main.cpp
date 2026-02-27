#include <iostream>
#include <vector>
#include <unistd.h>

#include "Utils.hpp"
#include "Geometry.hpp"
#include "DesignStructure.hpp"
#include "Configuration.hpp"
#include "Verifier.hpp"
#include "Parser.hpp"

using namespace std ; 

int main(int argc, char *argv[]) {

    if (argc<4) {
        cerr << "Usage: " << argv[0] << " <input_file> <design_rule_file> <output_dir>" << "\n";
        return 1 ;
    }

    string designBumpPath = argv[1] ; 
    string designRulePath = argv[2] ;
    string outputDirectories = argv[3] ;
    box_xy chipBoundary ;
    vector<Bump> designBumps ; 
    
    DesignRule desigRule ; 
    int seed = 42 ; 
    for(int i=4; i<argc; ++i){
        if(string(argv[i])=="-g") GlobalGAConfig.generations = stoi(argv[++i]) ;
        else if(string(argv[i])=="-c") GlobalGAConfig.crossoverRate = stod(argv[++i]) ;
        else if(string(argv[i])=="-m") GlobalGAConfig.mutationRate = stod(argv[++i]) ;
        else if(string(argv[i])=="-p") GlobalGAConfig.populationSize = stoi(argv[++i]) ;
        else if(string(argv[i])=="-k") GlobalGAConfig.tournamentSize = stoi(argv[++i]) ;
        else if(string(argv[i])=="-s") seed = stoi(argv[++i]) ;
        else if(string(argv[i])=="-t") GlobalThreadNum = stoi(argv[++i]) ;
        else if(string(argv[i])=="-r") GlobalPositionResolution = stoi(argv[++i]) ;
    }
    GlobalRandomTool.srand(seed) ; 

    parse_design_bump(designBumpPath, designBumps, chipBoundary) ;
    parse_design_rule(designRulePath, desigRule) ;
    
    Verifier router ; 

    router.solve(designBumps, desigRule, GlobalGAConfig, chipBoundary, outputDirectories) ;



    return 0 ; 
}

// ./bin/D2D testcase/standard_44/bumps.loc testcase/standard_44/rule.txt ./result -p 300 -g 200 -m 1 -c 1 -ctr 30

//  圖層顯示設計:
//      顯示用:
//          xxx.label (7)
//          xxx.point (2)
//          xxx.edge (1)
//      設計用:
//          xxx.bump (3) 
//          xxx.offset_via (4)
//          xxx.net (5)
//          xxx.teardrop (6)
//  變數調整設計:
//      全流程變數:
//          seed: int 
//          debug_control: int 
//          routing_method: GAAS (GA+A*-search), AS (A*-search only)
//          routing_bump_distribute: int or list[Die, id] (分配每一層要繞線的Bumps)
//      全局繞線變數:
//          reserved_space_ratio: double, >= 1.0
//          number_of_generations: int, >= 0
//          crossover_rate: double, 0.0 - 1.0
//          mutation_rate: double, 0.0 - 1.0
//          population_size: int, >= 1 
//          tournament_size: int, >= 1
//      細節繞線變數:
//          angle_penalty_multiplier: double, >= 0.0
//  UI功能設計:
//      1. Rerun (重新執行程式並重新載入結果)
//      2. Reload (重新載入結果)
//      3. Modify variables in process (修改演算法用到的參數)
//      4. Show result and mouse position (顯示結果與鼠標所在位置, 方便進行幾何上的debug)
//      5. 圖層顯示器(layer + showed file in each layer)
//  UI用法:
//      ./D2D_Gui [result_folder] [design_rule] -b [d2d_bin_file]
//      result_folder: 放置結果的資料夾
//      design_rule: 設計規則
//      d2d_bin_file: Optional, 若沒有則Rerun功能失效






