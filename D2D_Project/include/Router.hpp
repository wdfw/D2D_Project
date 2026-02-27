#pragma once

#include <filesystem>
#include <fstream>
#include <vector>
#include <limits>
#include <cmath>

#include <deque>

#ifdef PARALLEL_MODE
    #include <thread>
    // #include <omp.h> // OpenMp
#endif

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;
typedef K::Point_2 CGAL_Point2 ;

#include "Geometry.hpp"
#include "DesignRule.hpp"
#include "DesignStructure.hpp"
#include "RoutingStructure.hpp"
#include "Utils.hpp"
#include "Configuration.hpp"

#define DEBUGGER

using namespace std ; 

// extern vector<vector<double>> debugEdges ; 
// extern vector<Bump> debugBumps ; 
// extern vector<Net> debugNets ; 
// extern vector<tuple<string, double, double>> debugLabels ; 

struct CostStruct ;
class FlowChecker ;
class Router ;

struct CostStruct {
    double wireLength ;
    double excessiveCapacity ;
    double conflictCount ;
    double differentialLength ;
} ; 

ostream& operator<<(ostream& os, const CostStruct& costStruct) ; 


class FlowChecker {
protected:
    DesignRule designRule ; 
public:
    FlowChecker() = default ;
    FlowChecker(const DesignRule& designRule) : designRule(designRule) {} ;

    void ckeck_bump_and_offset_via(const RoutingInfo& routingInfo) const ;
    void ckeck_via_nodes(const RoutingInfo& routingInfo) const ;
    void ckeck_tile_nodes(const RoutingInfo& routingInfo) const ;
    void ckeck_edge_bodes(const RoutingInfo& routingInfo) const ;
    void ckeck_position_nodes(const RoutingInfo& routingInfo) const ;
    void ckeck_v2v_edges(const RoutingInfo& routingInfo) const ;
    void ckeck_t2t_edges(const RoutingInfo& routingInfo) const ;
    void ckeck_e2e_edges(const RoutingInfo& routingInfo) const ;
    void ckeck_p2p_edges(const RoutingInfo& routingInfo) const ;
} ;

class Router {
private:
    FlowChecker checker ;
protected:
    DesignRule designRule ; 
    box_xy boundary ;
    vector<clock_t> routingTimes ;
    string outputDirectory ;
    GAConfiguration GAConfig ;

    using ConnectionMap = map<Bump, set<Bump>> ; 

    void clear_debug_buffer(RoutingInfo& routingInfo){
        routingInfo.debugLabelMapping.clear() ; 
        routingInfo.debugPointMapping.clear() ; 
        routingInfo.debugEdgeMapping.clear() ; 
        routingInfo.debugBumpMapping.clear() ;
    }
    void wait_until_input(const RoutingInfo& routingInfo){
        #ifdef DEBUGGER
            generate_routing_result(routingInfo, outputDirectory) ;
            string foo ; 
            cin >> foo ;
        #endif
    }


    void initialize_routing_information(RoutingInfo& routingInfo) ;
    void select_routing_bump(const vector<Bump>& bumps, RoutingInfo& routingInfo, unsigned int selecNumber = numeric_limits<int>::max()) ; 
    void adjust_offset_vias(RoutingInfo& routingInfo, int layer) ; 
    void add_dummy_bumps(RoutingInfo& routingInfo) ; 
    void rouding_bumps(RoutingInfo& routingInfo) ;

    void step1_check(const RoutingInfo& routingInfo) const ; // 驗證routing bumps與offset via為空集合 以及offset via的合法性

    void construct_routing_graph(RoutingInfo& routingInfo) ; 
        void triangulation(const RoutingInfo& routingInfo, const vector<Bump>& bumps, ConnectionMap& connections) ;
        void combine_connections(const ConnectionMap& connections1, const ConnectionMap& connections2, ConnectionMap& combinedConnections) ; 
        void create_node_and_edges(const ConnectionMap& combinedConnections, RoutingInfo& routingInfo, int k) ; 
            void create_viaMatrix(RoutingInfo& routingInfo) ;

            void create_vias_and_v2v_edges(const ConnectionMap& combinedConnections, RoutingInfo& routingInfo) ; 
            void create_edges_and_via_crossing(RoutingInfo& routingInfo) ; 
            void create_positions_and_viaExtendedPositions_and_edgeExtendedPositions(RoutingInfo& routingInfo) ; 
            void create_tiles_and_tile_surround_vias(RoutingInfo& routingInfo, int k) ;

            void create_e2e_edges(RoutingInfo& routingInfo) ; 
            void create_p2p_edges(RoutingInfo& routingInfo) ; 
            void create_tiles(RoutingInfo& routingInfo) ; 
            void create_t2t_edges_and_tile_crossing_edges(RoutingInfo& routingInfo) ; 
            void create_tileSurroundVias(RoutingInfo& routingInfo) ; 
            void create_tileSurroundEdges_and_tileSurroundPositions(RoutingInfo& routingInfo) ; 
        void set_capacity(RoutingInfo& routingInfo) ; // 設定每一邊的預估容量
        void set_edge_type(RoutingInfo& routingInfo) ; // 設定edge node的種類

    void step2_check(const RoutingInfo& routingInfo) const ; // 驗證繞線圖的合法性

    virtual double global_route(RoutingInfo& routingInfo, CostStruct& costStruct) = 0 ; 
    void step3_check(const RoutingInfo& routingInfo) const ; // 驗證繞線圖的合法性

    
    virtual double detailed_route(RoutingInfo& routingInfo) = 0 ; 
    void step4_check(const RoutingInfo& routingInfo) const ; // 驗證繞線圖的合法性
    
    virtual double design_rule_check(RoutingInfo& routingInfo) = 0 ;

    void renew_routing_bumps(RoutingInfo& routingInfo) ; // 修正原本被當作Offset Via但被成功繞出的點

    void generate_routing_result(const RoutingInfo& routingInfo, const string& directory) ; 

public:
    void solve(const vector<Bump>& bumps, const DesignRule& designRule,  const GAConfiguration& GAConfig,
                const box_xy& boundary, const string& outputDirectory) ;
} ;

