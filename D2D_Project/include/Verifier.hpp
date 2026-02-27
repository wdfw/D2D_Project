#include "DetailedRoute.hpp"

class Verifier : public DetailedRoute {
// protected:
public:

    bgsb::distance_symmetric<double> viaStrategy = bgsb::distance_symmetric<double>(0.0) ;
    bgsb::distance_symmetric<double> viaPadStrategy = bgsb::distance_symmetric<double>(0.0) ;
    bgsb::distance_symmetric<double> lineWidthStrategy = bgsb::distance_symmetric<double>(0.0) ;
    bgsb::join_round joinStrategy = bgsb::join_round(0) ;
    bgsb::end_round endStrategy = bgsb::end_round(0) ;
    bgsb::point_circle circleStrategy = bgsb::point_circle(0) ;
    bgsb::side_straight sideStrategy = bgsb::side_straight() ;
    DesignRule designRule ; 

    void set_geometry_strategy(const DesignRule& designRule) ;

    // double viaRadius; // Bump不含Padding的半徑(不含黃色區塊)
    // double viaPadRadius; // Bump含Padding的半徑(含黃色區塊)
    // double minimumViaPadSpacing; // Bumps間的最小距離(含黃色)
    // double minimumOffsetViaSpacing; // Bumps間的最小距離(不含黃色)
    // double minimumLineWidth; // 線寬
    // double minimumLineSpacing;  // 線與線之間的距離
    // double minimumLineViaPadSpacing; // 線與Bump之間的距離(含黃色)
    // double minimumTeardropDist; // teardrop 外面的點, 到 teardrop 圓心的距離

    bool check_via_pad_to_via_pad_spacing(RoutingInfo& routingInfo) ; 
    bool check_line_to_line_spacing(RoutingInfo& routingInfo) ; 
    bool check_via_pad_to_line_spacing(RoutingInfo& routingInfo) ; 
    bool check_line_to_teardrop_spacing(RoutingInfo& routingInfo) ; 

    // write_result(directory, "Layer", "bump", routingInfo.routingBumps) ;
    // write_result(directory, "Layer", "offset_via", routingInfo.offsetVias) ;
    // write_result(directory, "Layer", "net", routingInfo.designNets) ;
    using DetailedRoute::DetailedRoute ; 

    double design_rule_check(RoutingInfo& routingInfo) override ;

} ; 