#include "DetailedRoute.hpp"
#define M_COST 1000000 
#define DE2
#ifdef DE2 

int bufferInUpdate = 0 ;
int bufferInSearch = 0 ;

double angle(const PositionNodePtr& position, const PositionNodePtr& center){
    double dx = position->x() - center->x() ; 
    double dy = position->y() - center->y() ; 
    return atan2(dy, dx) + M_PI ; // [0, 2*pi]
}

void DetailedRoute::set_routing_geometry_strategy() {
    routingDistStrategy = bgsb::distance_symmetric<double>(designRule.minimumLineWidth) ;
    routingSideStrategy = bgsb::side_straight() ;
    routingJoinStrategy = bgsb::join_miter() ;
    routingEndStrategy = bgsb::end_flat() ;  // 讓末端變成平的四邊形
    routingCircleStrategy = bgsb::point_circle() ;
}


double DetailedRoute::caclute_connected_count(PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) {
    double connectedCount = 0.0 ; 

    if(routedNet.contains(currentNode)){
        connectedCount = routedNet.at(currentNode).size() ; 
        if(prevNode && !routedNet.at(currentNode).contains(prevNode)) ++ connectedCount ; 
        if(nextNode && !routedNet.at(currentNode).contains(nextNode)) ++ connectedCount ; 
    }else{
        if(prevNode) ++ connectedCount ; 
        if(nextNode) ++ connectedCount ; 
    }

    return connectedCount ; 
}


double DetailedRoute::caclute_teardrop_extension_penalty(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode){
    double minimumDistance = 0.0 ;
    const double extensionProduct = 1.5 ; 
    if(graph.viaExtendedPositions.contains(currentNode)) minimumDistance += designRule.minimumTeardropDist*extensionProduct ;
    if(graph.viaExtendedPositions.contains(nextNode)) minimumDistance += designRule.minimumTeardropDist*extensionProduct ;

    return (distance_sc(*currentNode, *nextNode)<minimumDistance) ? 1.0 : 0.0 ; 
}


double DetailedRoute::caclute_angle(RoutingGraph& graph, PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) {

    vector<PositionNodePtr> surroundingPositions ; 
    vector<double> diffs ; 

    // if(graph.viaExtendedPositions.contains(currentNode) && 
    //     (*graph.viaExtendedPositions.at(currentNode).begin())->type !=DUMMY) return 0.0 ; 

    if(routedNet.contains(currentNode)){
        for(auto& position : routedNet.at(currentNode)){
            surroundingPositions.push_back(position) ; 
        }
    }

    if(prevNode) surroundingPositions.push_back(prevNode) ; 
    if(nextNode) surroundingPositions.push_back(nextNode) ; 

    sort(surroundingPositions.begin(), surroundingPositions.end(), 
        [&currentNode](const PositionNodePtr& p1, const PositionNodePtr& p2){
            return angle(p1, currentNode) < angle(p2, currentNode) ;
        }
    ) ;
    
    double diffSUM = 0.0, diffAVG, diffVAR = 0.0 ;
    for(int i=0; i<surroundingPositions.size(); ++i){
        double diff = angle(surroundingPositions[(i+1)%surroundingPositions.size()], currentNode) - angle(surroundingPositions[i], currentNode) ;
        if(diff<0.0) diff += 2*M_PI ; 
        if(diff!=0.0){
            diffSUM += diff ;
            diffs.push_back(diff) ; 
        }
    }

    if(!diffs.size()) return 0.0 ;
    diffAVG = diffSUM/diffs.size() ; 
    for(auto& diff : diffs){
        diffVAR += pow(diff-diffAVG, 2) ; 
    }
    diffVAR = sqrt(diffVAR/diffs.size()) ; 


    return diffVAR ;                 
}


double DetailedRoute::caclute_conflict_count(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode, BumpType netType, const DetailedNet& routedNet) {
    double crossingCount = 0.0 ;
++bufferInSearch ; 
// linestring_xy line ;
// multi_polygon_xy buffer ;
// bg::append(line, point_xy(currentNode->x() , currentNode->y())) ;
// bg::append(line, point_xy(nextNode->x() , nextNode->y())) ;
// boost::geometry::buffer(line, buffer, routingDistStrategy, routingSideStrategy, routingJoinStrategy, routingEndStrategy, routingCircleStrategy) ;
// vector<pair<point_xy, BumpType>> results ; 
// rtree.query(bgi::within(buffer), std::back_inserter(results));
// for(auto& result : results){
//     if(result.second!=netType) ++crossingCount ;
// }
// return crossingCount ;

    for(auto& tile1 : graph.tileSurroundPositions.at(currentNode)){
        for(auto& tile2 : graph.tileSurroundPositions.at(nextNode)){
            if(tile1 != tile2) continue ; 
            for(auto& position1 : graph.tileSurroundPositions.at(tile1)){
                if(!routedNet.contains(position1)) continue ; 
                for(auto& position2 : routedNet.at(position1)){
                    if(position1>=position2) continue ;
                    
                    bool isCrossed =  is_crossed_sc(*position1, *position2, *currentNode, *nextNode) ;
                    bool isTouched =  distance_sc(*position1, *currentNode) < GlobalEpsilonR ||  
                                      distance_sc(*position1, *nextNode) < GlobalEpsilonR ||  
                                      distance_sc(*position2, *currentNode) < GlobalEpsilonR ||  
                                      distance_sc(*position2, *nextNode) < GlobalEpsilonR ;

                    if(position1->occupyType==DUMMY || position1->occupyType==netType){
                        if(isCrossed && !isTouched) ++ crossingCount ;
                    }else{
                        if(isCrossed || isTouched) ++ crossingCount ;
                    }
                }
            }
        }
    }
    return crossingCount ; 
}

double DetailedRoute::A_star_cost(RoutingInfo& routingInfo, PositionNodePtr prevNode, PositionNodePtr currentNode, PositionNodePtr nextNode, BumpType netType, const DetailedNet& routedNet) {
    double wireLength = nextNode ? distance_sc(*currentNode, *nextNode) : 0 ;
    double crossingCount = nextNode ? caclute_conflict_count(routingInfo.graph, currentNode, nextNode, netType, routedNet) : 0 ; 
    double anglePenalty = caclute_angle(routingInfo.graph, prevNode, currentNode, nextNode, routedNet) ; 
    double teardropExtensionPenalty = nextNode ? caclute_teardrop_extension_penalty(routingInfo.graph, currentNode, nextNode) : 0 ;
    double connectedCountPenalty = (caclute_connected_count(prevNode, currentNode, nextNode, routedNet)>=4) ? 1.0 : 0.0 ; 

    // if(currentNode->id==5686 && nextNode->id==5701){
    //     cout << "\t\t\t\t" ;
    //     if(prevNode) cout << *prevNode << " " ; 
    //     else cout << "NULL " ;
    //     cout << *currentNode << " " << *nextNode << " | " << wireLength << " " << crossingCount << " " << anglePenalty << " " << teardropExtensionPenalty << " " << connectedCountPenalty << "\n" ;
    // }
    return wireLength + anglePenalty*routingInfo.bumpDistance + connectedCountPenalty*M_COST + crossingCount*M_COST + teardropExtensionPenalty*M_COST ;
}

void DetailedRoute::update_detailed_net(DetailedNet& detailedNet) {
    for(auto& [node1, connectedNodes] : detailedNet){
        for(auto& node2 : connectedNodes){
            node1->occupyType = node2->occupyType = detailedNet.type ; 
            ++bufferInUpdate ; 
            // linestring_xy line ;
            // multi_polygon_xy buffer ;
            // bg::append(line, point_xy(node1->x() , node1->y())) ;
            // bg::append(line, point_xy(node2->x() , node2->y())) ;
            // boost::geometry::buffer(line, buffer, routingDistStrategy, routingSideStrategy, routingJoinStrategy, routingEndStrategy, routingCircleStrategy) ;
            // for(auto& poly : buffer){
            //     for(auto &pt : poly.outer()) rtree.insert({pt, newDetailedNet.type});
            // }

        }
    }
}



void DetailedRoute::greedy_detailed_route(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets) {
    RoutingGraph& graph = routingInfo.graph ; 
    map<EdgeNodePtr, tuple<int, int>> offsetMapping ;
    map<EdgeNodePtr, vector<PositionNodePtr>> sortedPositionMapping ; 

    for(auto& edge : graph.edges){
        offsetMapping[edge] = {0, graph.edgeExtendedPositions.at(edge).size()-1} ;
        for(auto& position : graph.edgeExtendedPositions.at(edge)) sortedPositionMapping[edge].push_back(position) ; 
        sort(sortedPositionMapping[edge].begin(), sortedPositionMapping[edge].end(),
            [&edge](const PositionNodePtr& p1, const PositionNodePtr& p2){
                if(edge->edgeType==BaseEdge) return p1->x() < p2->x() ; 
                return p1->y() < p2->y() ; 
            }
        ) ;
    }

    channelDetailedNets.clear() ; 
    for(auto& globalNet : routingInfo.globalNets){
        ChannelDetailedNet newDetailedNet(globalNet.startVia, globalNet.targetVia) ; 
        BumpType netType = (globalNet.startVia) ? SIGNAL : VSS ; 
        EdgeType prevType = LegEdge ; 
        double prevY ;
        int *positionIndexPtr, sign ;

        if(netType == SIGNAL) newDetailedNet.push_back(*graph.viaExtendedPositions.at(globalNet.startVia).begin()) ;

        for(auto& edge : globalNet){
            const vector<PositionNodePtr>& sortedPositions = sortedPositionMapping[edge] ; 
            PositionNodePtr probedPosition, position ;
            if(get<0>(offsetMapping[edge])<=get<1>(offsetMapping[edge])){
                if(edge->edgeType==BaseEdge && prevY < edge->y()){ 
                    positionIndexPtr = &get<1>(offsetMapping[edge]) ; sign = -1 ; 
                }else{
                    positionIndexPtr = &get<0>(offsetMapping[edge]) ; sign = +1 ;
                }

                probedPosition = sortedPositions[*positionIndexPtr] ;

                if(netType==SIGNAL && probedPosition->occupyType!=DUMMY) (*positionIndexPtr) += sign ;
                else if(netType==VSS && probedPosition->occupyType!=VSS && probedPosition->occupyType!=DUMMY) (*positionIndexPtr) += sign ;

                if(get<0>(offsetMapping[edge])<=get<1>(offsetMapping[edge])){
                    position = sortedPositions[*positionIndexPtr] ;
                    newDetailedNet.push_back(position) ; 
                }else{
                    goto DanglingNet ;
                }
            }else{
                goto DanglingNet ;
            }
            
            prevType = edge->edgeType ; 
            prevY = edge->y() ; 
        }

        if(netType == SIGNAL) newDetailedNet.push_back(*graph.viaExtendedPositions.at(globalNet.targetVia).begin()) ;
        
DanglingNet:;
        for(auto& position: newDetailedNet) position->occupyType = netType ; 
        channelDetailedNets.push_back(newDetailedNet) ; 
    }
}

void DetailedRoute::generate_channel_line_groups(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets) {
    RoutingGraph& graph = routingInfo.graph ; 
    DetailedNet allConnectionNet(DUMMY, -1) ; 
    
    for(int i=0; i<channelDetailedNets.size(); ++i){ // 把Signal Net加入考量
        if(channelDetailedNets[i].startVia){
            
            set<PositionNodePtr> startNodes, targetNodes, forbiddenNodes ; 
            DetailedNet signalDetailedNet = channelDetailedNets[i].to_detailed_net(graph) ;
            allConnectionNet.combine(signalDetailedNet) ; 
            update_detailed_net(signalDetailedNet) ; 
                         
            DetailedNet groundDetailedNet = channelDetailedNets[i-1].to_detailed_net(graph) ; groundDetailedNet.type = VSS ; 
            allConnectionNet.combine(groundDetailedNet) ; 
            update_detailed_net(groundDetailedNet) ; 

            for(auto& [position, _] : groundDetailedNet) targetNodes.insert(position) ;
            for(auto& startNode : {channelDetailedNets[i+1].front(), channelDetailedNets[i+1].back()}){
                DetailedNet surroundingDetailedNet(VSS, 0) ;

                startNodes = {startNode} ; 
                A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, allConnectionNet, surroundingDetailedNet) ; 
                allConnectionNet.combine(surroundingDetailedNet) ; 
                groundDetailedNet.combine(surroundingDetailedNet) ;

                update_detailed_net(surroundingDetailedNet) ; 
            }

            DetailedNet lowerGroundDetailedNet = channelDetailedNets[i+1].to_detailed_net(graph) ;
            allConnectionNet.combine(lowerGroundDetailedNet) ; 
            groundDetailedNet.combine(lowerGroundDetailedNet) ;
            update_detailed_net(lowerGroundDetailedNet) ; 

            detailedNets.push_back(signalDetailedNet) ; 
            detailedNets.push_back(groundDetailedNet) ; 
        }
    }
}

void DetailedRoute::generate_power_line_groups(RoutingInfo& routingInfo, vector<DetailedNet>& routedDetailedNets, vector<DetailedNet>& generatedDetailedNets) {
    RoutingGraph& graph = routingInfo.graph ; 
    DetailedNet allConnectionNet(DUMMY, -1) ; 
    double centralX = (routingInfo.chipBoundary.max_corner().x() + routingInfo.chipBoundary.min_corner().x()) / 2 ;
    

    for(auto& detailedNet : routedDetailedNets) allConnectionNet.combine(detailedNet) ;


    generatedDetailedNets.clear() ;
    for(auto& type : {VDD, VSS}){
        for(int i=0; i<graph.viaMatrix.size(); ++i){
            bool singleGroupFlag = false ; 
            vector<PositionNodePtr> routingPositions ; 
            
            for(int j=0; j<graph.viaMatrix[i].size(); ++j){
                // cout << *graph.viaMatrix[i][j] << "\n" ;
                // if(!graph.viaExtendedPositions.contains(graph.viaMatrix[i][j])) continue ;
                if(graph.viaMatrix[i][j]->type!=DUMMY && graph.viaMatrix[i][j]->type!=type){
                    singleGroupFlag = true ; 
                }else if(!graph.viaMatrix[i][j]->isOffsetVia && graph.viaMatrix[i][j]->type==type){
                    routingPositions.push_back(*graph.viaExtendedPositions.at(graph.viaMatrix[i][j]).begin()) ;
                }
            }

            if(singleGroupFlag){
                for(auto& position : routingPositions){
                    DetailedNet newDetailedNet(type, 0) ; 
                    newDetailedNet[position] = {} ; 
                    generatedDetailedNets.push_back(newDetailedNet) ;
                }
            }else{
                DetailedNet newDetailedNet(type, 0) ; 
                DetailedNet coibinedDetailedNet(type, 0) ; 
                set<PositionNodePtr> startSet, targetSet, forbiddenSet ;

                for(auto& via : graph.vias){
                    if(via->type==type){ // 不同屬性的線路已經不會連在一起, 所以不用考慮
                        if(via->isOffsetVia) forbiddenSet.insert(*graph.viaExtendedPositions.at(via).begin()) ; 
                    }
                }
                
                sort(routingPositions.begin(), routingPositions.end(), 
                    [&centralX](const PositionNodePtr& node1, const PositionNodePtr& node2){
                        return  node1->x() < node2->x() ; 
                    }
                ) ;
                
                for(int i=0; i+1<routingPositions.size(); ++i){
                    startSet = {routingPositions[i]} ; targetSet = {routingPositions[i+1]} ;

                    A_star_route(routingInfo, startSet, targetSet, forbiddenSet, allConnectionNet, newDetailedNet) ; 
                    allConnectionNet.combine(newDetailedNet) ; 
                    coibinedDetailedNet.combine(newDetailedNet) ; 
                    update_detailed_net(newDetailedNet) ; 
                }

                generatedDetailedNets.push_back(coibinedDetailedNet) ;
            }
            cout << "\n" ;
        }
        
// for(auto& net : {coibinedDetailedNet}){
//     for(auto& [node1, connectedNodes] : net){
//         for(auto& node2 : connectedNodes){
//             string color = net.type==VSS ? "#06ea4e" : "#0008ff"  ;
//             Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
//             routingInfo.debugEdgeMapping["GROUND_SOURCE"].push_back(newEdge) ; 
//         }
//     }
// }
        
    }
}

void DetailedRoute::generate_detailed_net_groups(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets) {
    detailedNets.clear() ;

    vector<DetailedNet> channelGroup, powerLineGroup ; 
    generate_channel_line_groups(routingInfo, channelDetailedNets, channelGroup) ;
    generate_power_line_groups(routingInfo, channelGroup, powerLineGroup) ;

    detailedNets = channelGroup ; detailedNets.insert(detailedNets.end(), powerLineGroup.begin(), powerLineGroup.end()) ;
}


void DetailedRoute::combine_detailed_nets(vector<DetailedNet>& detailedNets, vector<DetailedNet>& result) {
    map<pair<BumpType, int>, DetailedNet> mappingDetailedNet ;
    result.clear() ; 

    for(auto& detailNet : detailedNets){
        pair<BumpType, int> key = {detailNet.type, detailNet.id} ;
        if(!mappingDetailedNet.contains(key)){
            mappingDetailedNet[key] = detailNet ; 
        }else{
            mappingDetailedNet[key].combine(detailNet) ;
        }
    } 

    for(auto& [key, detailedNet] : mappingDetailedNet){
        result.push_back(detailedNet) ; 
    }
}

void DetailedRoute::generate_design_net(vector<DetailedNet>& detailedNets, vector<Net>& designNets) {
    designNets.clear() ; 
    for(auto& detailedNet : detailedNets){
        designNets.push_back(detailedNet.to_design_net()) ;
    }
}


void DetailedRoute::A_star_route(RoutingInfo& routingInfo, set<PositionNodePtr>& startNodes, set<PositionNodePtr>& targetNodes, set<PositionNodePtr>& forbiddenNodes, const DetailedNet& routedNet, DetailedNet& detailedNet) {
    RoutingGraph& graph = routingInfo.graph ; 

    for(auto& node1 : startNodes){
        if(targetNodes.contains(node1)){
            detailedNet.clear() ;
            return ; 
        }
    }

    vector<pair<double, PositionNodePtr>> costWithPrevNodes(graph.positions.size(), {numeric_limits<double>::max(), nullptr}) ; 
    priority_queue<pair<double, PositionNodePtr>> pq ; // -cost, positionNode
    PositionNodePtr firstTargetNode = nullptr ; 
    BumpType netType = detailedNet.type ; 
    PositionNodePtr node0 ; 

    // 1. Search Step
    for(auto& startNode : startNodes){
        costWithPrevNodes[startNode->k_id].first = 0.0 ;
        pq.push({-costWithPrevNodes[startNode->k_id].first, startNode}) ; 
    }

    int cid = (*startNodes.begin())->id ; 
    for(int i=0; pq.size(); ++i){
        double currentCost = -pq.top().first ; 
        double nextCost ; 
        PositionNodePtr node1 = pq.top().second ; pq.pop() ; 
        node0 = costWithPrevNodes[node1->k_id].second ; 


        if(costWithPrevNodes[node1->k_id].first!=currentCost) continue ;


        if(currentCost>=M_COST){
            cout << "FAIL\n" ;
            return  ; // 代表繞線失敗
        }

        if(targetNodes.contains(node1) ){
            firstTargetNode = node1 ;
            break ;
        }
        
        for(auto& [node2, p2pEdge] : graph.connectedPositions[node1]){
            if(forbiddenNodes.contains(node2)) continue ; 
            if( // Offset via不可繞線, 其餘位置若occupyType相同或沒有, 就都可以繞
                // ((graph.viaExtendedPositions.contains(node2) && !(*graph.viaExtendedPositions.at(node2).begin())->isOffsetVia)) &&
                (node2->occupyType==netType || node2->occupyType==DUMMY || targetNodes.contains(node2))
            ){
                nextCost = A_star_cost(routingInfo, node0, node1, node2, netType, routedNet) + currentCost ;
                if(targetNodes.contains(node2)) nextCost += A_star_cost(routingInfo, node1, node2, nullptr, netType, routedNet) ;
                
                if(nextCost < costWithPrevNodes[node2->k_id].first){
                    costWithPrevNodes[node2->k_id] = {nextCost, node1} ; 
                    pq.push({-nextCost, node2}) ;
                }

                
            }
        }
    }
    

    PositionNodePtr currentNode = firstTargetNode, prevNode ; 
    detailedNet.clear() ;

    // 2. Backtracking Step
    for(int i=0; startNodes.find(currentNode)==startNodes.end(); ++i){
        prevNode = currentNode ;
        currentNode = costWithPrevNodes[currentNode->k_id].second ; 
        detailedNet[currentNode].insert(prevNode) ; detailedNet[prevNode].insert(currentNode) ; 
    }   
}

double DetailedRoute::detailed_route(RoutingInfo& routingInfo) {
    vector<ChannelDetailedNet> channelDetailedNets ;
    vector<DetailedNet> detailedNets, result ;
    Timer timer ; timer.set_clock() ; 

    routingInfo.detailedNets.clear() ; 
    rtree.clear() ;  
    set_routing_geometry_strategy() ;
    greedy_detailed_route(routingInfo, channelDetailedNets) ;
    generate_detailed_net_groups(routingInfo, channelDetailedNets, detailedNets) ;

    combine_detailed_nets(detailedNets, routingInfo.detailedNets) ;
    generate_design_net(routingInfo.detailedNets, routingInfo.designNets) ;

    cout << channelDetailedNets.size() << " " <<  routingInfo.detailedNets.size() << "\n" ;
    cout << "Detailed A* Route Elapsed Time: " << timer.get_duration_milliseconds() << " ms\n"  ;
    cout << "Update times: " << bufferInUpdate << "\n" ;
    cout << "Search times: " << bufferInSearch << "\n" ;

    for(auto& net : channelDetailedNets){
        for(int i=0; i<int(net.size())-1; ++i){
            auto node1 = net[i] ; 
            auto node2 = net[i+1] ; 
            string color = net.startVia ? "#f60541" : "#06ea4e" ;
            Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
            routingInfo.debugEdgeMapping["GREEDY_NET"].push_back(newEdge) ; 
        }
    }

    return 0.0 ;
} 
#endif