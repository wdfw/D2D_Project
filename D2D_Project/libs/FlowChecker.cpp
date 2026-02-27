#include "Router.hpp"

void FlowChecker::ckeck_bump_and_offset_via(const RoutingInfo& routingInfo) const {
    const vector<Bump>& routingBumps = routingInfo.routingBumps ;
    const vector<OffsetVia>& offsetVias = routingInfo.offsetVias ;

    for(auto& bump : routingBumps){
        if(bump.die!=DIE1 && bump.die!=DIE2){
            throw runtime_error("[Bump & Offset Via Check] DIE12 should not included in the stage.") ; 
        }
        for(auto& offsetVia : offsetVias){
            if(bump==offsetVia.first || bump==offsetVia.second){
                throw runtime_error("[Bump & Offset Via Check] Unconsistent of Routing Bump and Offset Via.") ; 
            }
        }
        for(auto& bump2 : routingBumps){
            if(&bump!=&bump2 && bump==bump2){
                throw runtime_error("[Bump & Offset Via Check] Multiple bump with same key." ) ; 
            }
        }
    }

    // for(auto& offsetVia : offsetVias){
    //     if (distance_sc(offsetVia.first, offsetVia.second) < (designRule.minimumViaPadSpacing + 2*designRule.viaPadRadius)){
    //         throw runtime_error("[Bump & Offset Via Check] Minimum via pad spacing violated.") ; 
    //     }else if(offsetVia.first.isOffsetVia || !offsetVia.second.isOffsetVia){
    //         throw runtime_error("[Bump & Offset Via Check] Invalid setting of isOffsetVia.") ; 
    //     }
    // }
}

void FlowChecker::ckeck_via_nodes(const RoutingInfo& routingInfo) const {
    set<pair<double, double>> occupyPosition ; 
    for(auto& via : routingInfo.graph.vias){
        if(occupyPosition.contains({via->x(), via->y()})){
            throw runtime_error("[Via Node Check] Via Node Occupy Same Position.") ; 
        }

        for(auto& via2 : routingInfo.graph.vias){
            if(via!=via2 && static_cast<Bump&>(*via)==static_cast<Bump&>(*via2)){
                throw runtime_error("[Via Node Check] Multiple via with same key in bump class." ) ; 
            }
        }
    }
}

// void FlowChecker::ckeck_tile_nodes(const RoutingInfo& routingInfo) const ;
// void FlowChecker::ckeck_edge_bodes(const RoutingInfo& routingInfo) const ;
// void FlowChecker::ckeck_position_nodes(const RoutingInfo& routingInfo) const ;

void FlowChecker::ckeck_v2v_edges(const RoutingInfo& routingInfo) const {
    for(auto& v2vEdge : routingInfo.graph.v2vEdges){
        if((v2vEdge->node1==v2vEdge->node2) || static_cast<const Bump&>(*v2vEdge->node1)==static_cast<const Bump&>(*v2vEdge->node2)){
                throw runtime_error("[V2V Edge Check] Self connected edge." ) ; 
        }

        for(auto& v2vEdge2 : routingInfo.graph.v2vEdges){
            // if(v2vEdge!=v2vEdge2 && )
        }
    }
}
// void FlowChecker::ckeck_t2t_edges(const RoutingInfo& routingInfo) const ;
// void FlowChecker::ckeck_e2e_edges(const RoutingInfo& routingInfo) const ;
// void FlowChecker::ckeck_p2p_edges(const RoutingInfo& routingInfo) const ;