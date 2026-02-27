#include "GlobalRoute.hpp"


class DetailedRoute : public GlobalRoute {
protected:
    bgsb::distance_symmetric<double> routingDistStrategy = bgsb::distance_symmetric<double>(0.0) ;
    bgsb::side_straight routingSideStrategy = bgsb::side_straight() ;
    bgsb::join_miter routingJoinStrategy = bgsb::join_miter() ;
    bgsb::end_flat routingEndStrategy = bgsb::end_flat() ;  // 讓末端變成平的四邊形
    bgsb::point_circle routingCircleStrategy = bgsb::point_circle() ;
    bgi::rtree<pair<point_xy, BumpType>, bgi::quadratic<500>> rtree ;

    // double global_route(RoutingInfo& routingInfo, CostStruct& costStruct) = 0 ; 

    double detailed_route(RoutingInfo& routingInfo) override ; 
    void set_routing_geometry_strategy() ;
    void greedy_detailed_route(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets) ;
    void surrounding_detailed_route2(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets) ; 

    void surrounding_detailed_route(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets) ; 
        void A_star_route(RoutingInfo& routingInfo, set<PositionNodePtr>& startNodes, set<PositionNodePtr>& targetNodes, set<PositionNodePtr>& forbiddenNodes, const DetailedNet& routedNet, DetailedNet& detailedNet) ;
            double A_star_cost(RoutingInfo& routingInfo, PositionNodePtr prevNode, PositionNodePtr currentNode, PositionNodePtr nextNode, BumpType netType, const DetailedNet& routedNet) ; 
            
                double caclute_conflict_count(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode, BumpType netType, const DetailedNet& routedNet) ;
                double caclute_teardrop_extension_penalty(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode) ; 
                double caclute_angle(RoutingGraph& graph, PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) ;

                double caclute_connected_count(PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) ;

        // void update_detailed_net(DetailedNet& detailedNet, DetailedNet& newDetailedNet) ;
    void ground_source_detailed_route(RoutingInfo& routingInfo, vector<DetailedNet>& routedDetailedNets,  vector<DetailedNet>& finalDetailedNet) ;
    void combine_detailed_nets(vector<DetailedNet>& detailedNets, vector<DetailedNet>& result) ; // 將相同BumpType & ID的線路組合起來
    void generate_design_net(vector<DetailedNet>& detailedNets, vector<Net>& designNets) ;


    void update_detailed_net(DetailedNet& detailedNet) ;
    void generate_channel_line_groups(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets) ;
        void generate_power_line_groups(RoutingInfo& routingInfo, vector<DetailedNet>& routedDetailedNets, vector<DetailedNet>& generatedDetailedNets) ;
        void generate_detailed_net_groups(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets) ;

public:
    using GlobalRoute::GlobalRoute ; 
} ;