#include "Verifier.hpp"

void Verifier::set_geometry_strategy(const DesignRule& designRule) {
    this->designRule = designRule ;

    // 設定緩衝區策略
    const double via_radius_distance = designRule.viaRadius ;
    const double via_buffer_distance = designRule.viaPadRadius - designRule.viaRadius  ; 
    const double track_buffer_distance = designRule.minimumLineWidth / 2 + designRule.minimumLineSpacing ;
    const int points_per_circle = 36;

    viaStrategy = bgsb::distance_symmetric<double>(via_radius_distance);
    viaPadStrategy = bgsb::distance_symmetric<double>(via_buffer_distance);

    lineWidthStrategy = bgsb::distance_symmetric<double>(track_buffer_distance);
    joinStrategy = bgsb::join_round(points_per_circle);
    endStrategy = bgsb::end_round(points_per_circle);
    circleStrategy = bgsb::point_circle(points_per_circle);
    sideStrategy = bgsb::side_straight();
}



bool Verifier::check_line_to_line_spacing(RoutingInfo& routingInfo) {
    vector<DetailedNet>& detailedNets = routingInfo.detailedNets  ; 
    vector<vector<pair<PositionNodePtr, PositionNodePtr>>> segments(detailedNets.size()) ;
    vector<vector<multi_polygon_xy>> segmentBuffers(detailedNets.size()) ;

    for(int i=0; i<detailedNets.size(); ++i){
        auto& detailedNet = detailedNets[i] ; 

        for(auto& [node1, connectedNodes] : detailedNet){
            for(auto& node2 : connectedNodes){
                linestring_xy line ;
                multi_polygon_xy buffer ;
                
                bg::append(line, point_xy(node1->x() , node1->y())) ;
                bg::append(line, point_xy(node2->x() , node2->y())) ;
                boost::geometry::buffer(line, buffer, lineWidthStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
                
                segments[i].push_back({node1, node2}) ; 
                segmentBuffers[i].push_back(buffer) ; 
            }
        }
    }

    for(int i=0; i<segmentBuffers.size(); ++i){
        for(int j=i+1; j<segmentBuffers.size(); ++j){
            for(int k=0; k<segmentBuffers[i].size(); ++k){
                for(int l=0; l<segmentBuffers[j].size(); ++l){
                    bool isContacted = bg::intersects(segmentBuffers[i][k], segmentBuffers[j][l]);
                    if(isContacted){
                        auto node1 = segments[i][k].first, node2 = segments[i][k].second ; 
                        string color =  "#b117d3" ;
                        Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
                        routingInfo.debugEdgeMapping["ERROR_NET"].push_back(newEdge) ; 
                    }
                }
            }
        }
    }
    
    return true ; 
}

double Verifier::design_rule_check(RoutingInfo& routingInfo) {
    set_geometry_strategy(designRule) ; 
    check_line_to_line_spacing(routingInfo) ; 
    return 0.0 ;
}
