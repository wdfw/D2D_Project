#include "GlobalRoute.hpp"

class DetailedRoute3 : public GlobalRoute {
protected:
    bgsb::distance_symmetric<double> routingDistStrategy = bgsb::distance_symmetric<double>(0.0) ;
    bgsb::side_straight routingSideStrategy = bgsb::side_straight() ;
    bgsb::join_miter routingJoinStrategy = bgsb::join_miter() ;
    bgsb::end_flat routingEndStrategy = bgsb::end_flat() ;  // 讓末端變成平的四邊形
    bgsb::point_circle routingCircleStrategy = bgsb::point_circle() ;
    bgi::rtree<pair<point_xy, PositionNodePtr>, bgi::quadratic<500>> rtree ;

    bgsb::join_round joinStrategy = bgsb::join_round(0) ;
    bgsb::end_round endStrategy = bgsb::end_round(0) ;
    bgsb::point_circle circleStrategy = bgsb::point_circle(0) ; 
    bgsb::side_straight sideStrategy = bgsb::side_straight() ;

    // double global_route(RoutingInfo& routingInfo, CostStruct& costStruct) = 0 ; 

    double detailed_route(RoutingInfo& routingInfo) override ; 
        void set_routing_geometry_strategy(RoutingInfo& routingInfo) ;
        void greedy_detailed_route(RoutingInfo& routingInfo, vector<DetailedNet>& channelDetailedNets) ;
        void surrounding_detailed_route(RoutingInfo& routingInfo, vector<DetailedNet>& channelDetailedNets, vector<DetailedNet>& resultDetailedNets, DetailedNet& combinedDetailedNet) ; 
        void ground_source_detailed_route(RoutingInfo& routingInfo, DetailedNet& combinedDetailedNet,  vector<DetailedNet>& result) ;
        void combine_detailed_net_groups(RoutingInfo& routingInfo, DetailedNet& combinedDetailedNet, vector<DetailedNet>& groupingDetailedNets, vector<DetailedNet>& groundSourceDetailedNets, vector<DetailedNet>& result) ;

        void A_star_route(RoutingInfo& routingInfo, set<PositionNodePtr>& startNodes, set<PositionNodePtr>& targetNodes, set<PositionNodePtr>& forbiddenNodes, const DetailedNet& routedNet, DetailedNet& detailedNet) ;
            double A_star_cost(RoutingInfo& routingInfo, PositionNodePtr prevNode, PositionNodePtr currentNode, PositionNodePtr nextNode, BumpType netType, const DetailedNet& routedNet) ; 
                // double caclute_conflict_count(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode, BumpType netType, const DetailedNet& routedNet) ;
                double caclute_conflict_count(RoutingInfo& routingInfo, PositionNodePtr& currentNode, PositionNodePtr& nextNode, BumpType netType, const DetailedNet& routedNet) ;
                double caclute_teardrop_extension_penalty(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode) ; 
                double caclute_angle(RoutingGraph& graph, PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) ;
                double caclute_connected_count(PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) ;
    
        
                
        void update_detailed_net(DetailedNet& detailedNet) ; // 更新某條線造成的覆蓋狀態
        void remove_detailed_net(DetailedNet& detailedNet) ; // 清空某條線造成的覆蓋狀態
        void clear_all_detailed_nets(RoutingInfo& routingInfo) ; // 清空所有覆蓋狀態
        

        void generate_design_net(vector<DetailedNet>& detailedNets, vector<Net>& designNets) ;
        void generate_design_teardrop(RoutingInfo& routingInfo, vector<DetailedNet>& detailedNets, vector<Teardrop>& teardrops) ;

public:
    using GlobalRoute::GlobalRoute ; 
} ;