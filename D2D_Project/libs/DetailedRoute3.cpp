#include "DetailedRoute.hpp"
#define M_COST 1000000 


double DetailedRoute3::caclute_connected_count(PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) {
    double connectedCount = 0.0 ; 

    if(routedNet.contains(currentNode)){
        connectedCount = routedNet.at(currentNode).size() ; 
        if(prevNode && !routedNet.at(currentNode).contains(prevNode)) ++ connectedCount ; 
        if(nextNode && !routedNet.at(currentNode).contains(nextNode)) ++ connectedCount ; 
    }else{
        if(prevNode) ++ connectedCount ; 
        if(nextNode) ++ connectedCount ; 
    }

// if(currentNode->id==5686 && nextNode->id==5701){
// cout << "\t\t\t" << *prevNode << " -> " << *currentNode << " -> " << *nextNode << " | " <<  connectedCount << "\n" ;
// }
    return connectedCount ; 
}

void DetailedRoute3::greedy_detailed_route(RoutingInfo& routingInfo, vector<DetailedNet>& detailedNets) {
    RoutingGraph& graph = routingInfo.graph ; 
    map<EdgeNodePtr, vector<PositionNodePtr>> sortedPositionMapping ; 
    map<EdgeNodePtr, pair<int, int>> offsetMapping ; // left, right

    for(auto& edge : graph.edges){
        offsetMapping[edge] = {0, int(graph.edgeExtendedPositions.at(edge).size())-1} ;

        for(auto& position : graph.edgeExtendedPositions.at(edge)) sortedPositionMapping[edge].push_back(position) ; 

        sort(sortedPositionMapping[edge].begin(), sortedPositionMapping[edge].end(),
            [&edge](const PositionNodePtr& p1, const PositionNodePtr& p2){
                if(edge->edgeType==BaseEdge) return p1->x() < p2->x() ; 
                return p1->y() < p2->y() ; 
            }
        ) ;
    }

    detailedNets.clear() ; 
    for(auto& globalNet : routingInfo.globalNets){

        if((globalNet.startVia && !globalNet.targetVia) || (!globalNet.startVia && globalNet.targetVia)) throw logic_error("Incompleted global net.\n") ;

        // Step 1
        BumpType netType = (globalNet.startVia) ? SIGNAL : VSS ; 
        vector<PositionNodePtr> positionSequence ; 
        PositionNodePtr probedPosition ;

        if(netType==SIGNAL){
            PositionNodePtr startPosition = *graph.viaExtendedPositions.at(globalNet.startVia).begin() ;
            positionSequence.push_back(startPosition) ;
        }

        // Step 2

        for(auto& edge : globalNet){
            EdgeType prevType = LegEdge ; 
            double prevY ;
            int *positionIndexPtr, sign ;

            if(offsetMapping[edge].first<=offsetMapping[edge].second){

                if(edge->edgeType==BaseEdge && prevY < edge->y()){ positionIndexPtr = &offsetMapping[edge].second ; sign = -1 ; }
                else{ positionIndexPtr = &offsetMapping[edge].first ; sign = +1 ;}

                
                while(offsetMapping[edge].first <= offsetMapping[edge].second){
                    probedPosition = sortedPositionMapping[edge][*positionIndexPtr] ;
                    if(!probedPosition->is_compitable(netType)){
                        (*positionIndexPtr) += sign ;
                    }else{
                        break ;
                    }
                    // cout << offsetMapping[edge].first << " " << offsetMapping[edge].second << " " << (*positionIndexPtr) << "\n" ;
                }

                if(offsetMapping[edge].first <= offsetMapping[edge].second){
                    positionSequence.push_back( sortedPositionMapping[edge][*positionIndexPtr] ) ; 
                    prevType = edge->edgeType ; 
                    prevY = edge->y() ; 
                }else{
                    goto DanglingNet ;
                }
            }else{
                goto DanglingNet ;
            }
        }

        // Step 3
        if(globalNet.targetVia){
            PositionNodePtr targetPosition = *graph.viaExtendedPositions.at(globalNet.targetVia).begin() ;
            positionSequence.push_back(targetPosition) ;
        }
DanglingNet:;

        DetailedNet newDetailedNet(netType, 0) ; 
        if(netType==SIGNAL) newDetailedNet.id = globalNet.startVia->id ;

        for(int i=0; i+1<positionSequence.size(); ++i){
            newDetailedNet.add_connection(positionSequence[i], positionSequence[i+1]) ;

            for(auto& tile1 : graph.tileSurroundPositions.at(positionSequence[i])){
                for(auto& tile2 : graph.tileSurroundPositions.at(positionSequence[i+1])){
                    if(tile1==tile2){
                        routingInfo.netSequence[tile1].push_back({positionSequence[i], positionSequence[i+1]}) ;
                    }
                }
            }
        }

        detailedNets.push_back(newDetailedNet) ; 
        update_detailed_net(newDetailedNet) ; 

// if(netType==SIGNAL && globalNet.startVia->id==37){
//     cout << netType << " " <<  positionSequence[0]->id << "\n" ;
//     PositionNodePtr n1, n2 ; 

//     for(auto& [node, _] : newDetailedNet){
//     if(node->id==10966) n1 = node ; 
//     cout << *node << "|" << node->coverCount  << "\n" ;
//     }

//     cout << "Prev\n" ;
//     for(auto& [node, _] : detailedNets[detailedNets.size()-2]){
//     if(node->id==10965) n2 = node ; 
//     cout << *node << "|" << node->coverCount  << "\n" ;
//     }

// // (Die2,Dummy,10966) 1954.62,348.536|0
// // (Die2,Dummy,10965) 1952.88,345.505|1
//     cout << distance_sc(*n1, *n2) << "\n" ;

//     linestring_xy line ;
//     multi_polygon_xy buffer ;


//     bg::append(line, point_xy(n1->x() , n1->y())) ;
//     // bg::append(line, point_xy(n2->x() , n2->y())) ;
//     // bg::append(line, point_xy(node2->x() , node2->y())) ;
//     boost::geometry::buffer(line, buffer, routingDistStrategy, routingSideStrategy, routingJoinStrategy, routingEndStrategy, routingCircleStrategy) ;

//     vector<pair<point_xy, PositionNodePtr>> results ; 
//     rtree.query(bgi::covered_by(buffer), std::back_inserter(results)) ;

//     for(auto& result : results){
//         auto& node = result.second ;
//         if(node==n1 || node==n2){
//             cout << "$ " <<  " " << *node << " | " << distance_sc(*n1, *node) << "\n" ;
//         }else{
//             cout <<  *node << " | " << distance_sc(*n1, *node) << "\n" ;
//         }
//     }
// }

// 990.375,287.91
// 992.125,284.883
    }

    // for(auto& node : routingInfo.graph.positions){
    //     routingInfo.debugBumpMapping["DB_Positions"].push_back( static_cast<Bump>(*node) ) ; 
    //     routingInfo.debugBumpMapping["DB_Positions"].back().type = node->occupyType ;
    // }
}


// double DetailedRoute3::caclute_conflict_count(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode, BumpType netType, const DetailedNet& routedNet) {
//     double crossingCount = 0.0 ;
//     for(auto& tile1 : graph.tileSurroundPositions.at(currentNode)){
//         for(auto& position1 : graph.tileSurroundPositions.at(tile1)){
//             if(!routedNet.contains(position1)) continue ; 
//             for(auto& position2 : routedNet.at(position1)){
//                 if(position1>=position2) continue ;
                
//                 bool isCrossed =  is_crossed_sc(*position1, *position2, *currentNode, *nextNode) ;
//                 bool isTouched =  distance_sc(*position1, *currentNode) < GlobalEpsilonR ||  
//                                     distance_sc(*position1, *nextNode) < GlobalEpsilonR ||  
//                                     distance_sc(*position2, *currentNode) < GlobalEpsilonR ||  
//                                     distance_sc(*position2, *nextNode) < GlobalEpsilonR ;

//                 if(position1->occupyType==DUMMY || position1->occupyType==netType){
//                     if(isCrossed && !isTouched) ++ crossingCount ;
//                 }else{
//                     if(isCrossed || isTouched) ++ crossingCount ;
//                 }
//             }
//         }
//     }
//     return crossingCount ; 
// }

void DetailedRoute3::generate_design_net(vector<DetailedNet>& detailedNets, vector<Net>& designNets) {
    designNets.clear() ; 
    for(auto& detailedNet : detailedNets){
        designNets.push_back(detailedNet.to_design_net()) ;
    }
}


void DetailedRoute3::update_detailed_net(DetailedNet& detailedNet) {
   vector<pair<PositionNodePtr, PositionNodePtr>> segements ; 
   detailedNet.to_segements(segements) ; 

   for(auto& segement : segements){
        linestring_xy line, line1, line2 ;
        multi_polygon_xy buffer, buffer1, buffer2 ;

        vector<pair<point_xy, PositionNodePtr>> results1 ; 
        vector<pair<point_xy, PositionNodePtr>> results2 ; 
        vector<pair<point_xy, PositionNodePtr>> results ; 
        
        auto& node1 = segement.first ;
        auto& node2 = segement.second ;

        bg::append(line, point_xy(node1->x() , node1->y())) ;
        bg::append(line, point_xy(node2->x() , node2->y())) ;
 
        boost::geometry::buffer(line, buffer, routingDistStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
        // boost::geometry::buffer(line1, buffer1, routingDistStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
        // boost::geometry::buffer(line2, buffer2, routingDistStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
        
        rtree.query(bgi::covered_by(buffer), std::back_inserter(results)) ;
        // rtree.query(bgi::covered_by(buffer1), std::back_inserter(results1)) ;
        // rtree.query(bgi::covered_by(buffer2), std::back_inserter(results2)) ;
    
        
        for(auto& result : results){
            auto& node = result.second ;
            if(node==node1 || node==node2) node->add_occupy(detailedNet.type) ;
            else node->add_cover() ;
        }
        // bg::append(line, point_xy(node1->x() , node1->y())) ;
        // bg::append(line, point_xy(node2->x() , node2->y())) ;
        // boost::geometry::buffer(line, buffer, routingDistStrategy, routingSideStrategy, routingJoinStrategy, routingEndStrategy, routingCircleStrategy) ;

        // vector<pair<point_xy, PositionNodePtr>> results ; 
        // rtree.query(bgi::covered_by(buffer), std::back_inserter(results)) ;

        // for(auto& result : results1){
        //     auto& node = result.second ;
        //     // if(node1->id==10966) cout << "& " << *node <<  " | " << distance_sc(*node1, *node) << "\n" ;

        //     if(node==node1) node->add_occupy(detailedNet.type) ;
        //     else node->add_cover() ;
        // }

        // for(auto& result : results2){
        //     auto& node = result.second ;
        //     // if(node2->id==10966) cout << "& " << *node <<  " | " << distance_sc(*node2, *node) << "\n" ;

        //     if(node==node2) node->add_occupy(detailedNet.type) ;
        //     else node->add_cover() ;
        // }

    }
}

void DetailedRoute3::remove_detailed_net(DetailedNet& detailedNet){
   vector<pair<PositionNodePtr, PositionNodePtr>> segements ; 
   detailedNet.to_segements(segements) ; 
   for(auto& segement : segements){
        linestring_xy line ;
        multi_polygon_xy buffer ;

        auto& node1 = segement.first ;
        auto& node2 = segement.second ;

        bg::append(line, point_xy(node1->x() , node1->y())) ;
        bg::append(line, point_xy(node2->x() , node2->y())) ;
        boost::geometry::buffer(line, buffer, routingDistStrategy, routingSideStrategy, routingJoinStrategy, routingEndStrategy, routingCircleStrategy) ;

        vector<pair<point_xy, PositionNodePtr>> results ; 
        rtree.query(bgi::covered_by(buffer), std::back_inserter(results)) ;
        
        for(auto& result : results){
            auto& node = result.second ;
            if(node==node1 || node==node2){
                node->reduce_occupy() ;
            }else{
                node->reduce_cover() ;
            }
        }
    }
}

void DetailedRoute3::set_routing_geometry_strategy(RoutingInfo& routingInfo) {

    // cout << "Distance: " << designRule.minimumLineWidth+designRule.minimumLineSpacing << "\n" ;
    routingDistStrategy = bgsb::distance_symmetric<double>((designRule.minimumLineWidth+designRule.minimumLineSpacing)*GlobalExpansionRation) ;
    routingSideStrategy = bgsb::side_straight() ;
    routingJoinStrategy = bgsb::join_miter() ;
    routingEndStrategy = bgsb::end_flat() ;  // 讓末端變成平的四邊形

    int points_per_circle = 36 ; 
    joinStrategy = bgsb::join_round(points_per_circle);
    endStrategy = bgsb::end_round(points_per_circle);
    circleStrategy = bgsb::point_circle(points_per_circle);
    sideStrategy = bgsb::side_straight();

    routingCircleStrategy = bgsb::point_circle() ;

    rtree.clear() ;  
    for(auto& position : routingInfo.graph.positions){
        point_xy pt = *position ; 
        rtree.insert({pt, position});
    }
}


double DetailedRoute3::caclute_angle(RoutingGraph& graph, PositionNodePtr& prevNode, PositionNodePtr& currentNode, PositionNodePtr& nextNode, const DetailedNet& routedNet) {

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
            return angle_sc(p1, currentNode) < angle_sc(p2, currentNode) ;
        }
    ) ;
    
    double diffSUM = 0.0, diffAVG, diffVAR = 0.0 ;
    for(int i=0; i<surroundingPositions.size(); ++i){
        double diff = angle_sc(surroundingPositions[(i+1)%surroundingPositions.size()], currentNode) - angle_sc(surroundingPositions[i], currentNode) ;
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

double DetailedRoute3::caclute_teardrop_extension_penalty(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode){
    double minimumDistance = 0.0 ;
    const double extensionProduct = 1.5 ; 
    if(graph.viaExtendedPositions.contains(currentNode)) minimumDistance += designRule.minimumTeardropDist*extensionProduct ;
    if(graph.viaExtendedPositions.contains(nextNode)) minimumDistance += designRule.minimumTeardropDist*extensionProduct ;

    return (distance_sc(*currentNode, *nextNode)<minimumDistance) ? 1.0 : 0.0 ; 
}


double DetailedRoute3::caclute_conflict_count(RoutingInfo& routingInfo, PositionNodePtr& currentNode, PositionNodePtr& nextNode, BumpType netType, const DetailedNet& routedNet) {
    RoutingGraph& graph = routingInfo.graph ;  
    double crossingCount = 0.0 ;

    for(auto& tile1 : graph.tileSurroundPositions.at(currentNode)){
        for(auto& tile2 : graph.tileSurroundPositions.at(nextNode)){
            if(tile1!=tile2) continue ;

            for(auto& netPair : routingInfo.netSequence[tile1]){
                auto& position1 = netPair.first ; 
                auto& position2 = netPair.second ;

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

    // for(auto& tile1 : graph.tileSurroundPositions.at(currentNode)){
    //     for(auto& position1 : graph.tileSurroundPositions.at(tile1)){
    //         if(!routedNet.contains(position1)) continue ; 
    //         for(auto& position2 : routedNet.at(position1)){
    //             if(position1>=position2) continue ;
                
    //             bool isCrossed =  is_crossed_sc(*position1, *position2, *currentNode, *nextNode) ;
    //             bool isTouched =  distance_sc(*position1, *currentNode) < GlobalEpsilonR ||  
    //                                 distance_sc(*position1, *nextNode) < GlobalEpsilonR ||  
    //                                 distance_sc(*position2, *currentNode) < GlobalEpsilonR ||  
    //                                 distance_sc(*position2, *nextNode) < GlobalEpsilonR ;

    //             if(position1->occupyType==DUMMY || position1->occupyType==netType){
    //                 if(isCrossed && !isTouched) ++ crossingCount ;
    //             }else{
    //                 if(isCrossed || isTouched) ++ crossingCount ;
    //             }
    //         }
    //     }
    // }
    return crossingCount ; 
}

double DetailedRoute3::A_star_cost(RoutingInfo& routingInfo, PositionNodePtr prevNode, PositionNodePtr currentNode, PositionNodePtr nextNode, BumpType netType, const DetailedNet& routedNet) {
    double wireLength = nextNode ? distance_sc(*currentNode, *nextNode) : 0 ;
    double crossingCount = nextNode ? caclute_conflict_count(routingInfo, currentNode, nextNode, netType, routedNet) : 0 ; 
    double anglePenalty = caclute_angle(routingInfo.graph, prevNode, currentNode, nextNode, routedNet) ; 
    double teardropExtensionPenalty = nextNode ? caclute_teardrop_extension_penalty(routingInfo.graph, currentNode, nextNode) : 0 ;
    double connectedCountPenalty = (caclute_connected_count(prevNode, currentNode, nextNode, routedNet)>=4) ? 1.0 : 0.0 ; 

    return wireLength + anglePenalty*routingInfo.bumpDistance + connectedCountPenalty*M_COST + crossingCount*M_COST + teardropExtensionPenalty*M_COST ;
}


void DetailedRoute3::A_star_route(RoutingInfo& routingInfo, set<PositionNodePtr>& startNodes, set<PositionNodePtr>& targetNodes, set<PositionNodePtr>& forbiddenNodes, const DetailedNet& routedNet, DetailedNet& detailedNet) {
    RoutingGraph& graph = routingInfo.graph ; 
    detailedNet.clear() ;

    for(auto& node : startNodes){
        if(targetNodes.contains(node)) return ; 
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

    for(int i=0; pq.size(); ++i){
        double currentCost = -pq.top().first ; 
        double nextCost ; 
        PositionNodePtr node1 = pq.top().second ; pq.pop() ; 
        node0 = costWithPrevNodes[node1->k_id].second ; 


        if(costWithPrevNodes[node1->k_id].first!=currentCost) continue ; // 捨棄不一致的
        if(currentCost>=M_COST){
            cout << "NET: " << detailedNet.type << " " << detailedNet.id << " FAIL\n" ;
            return  ; // 代表繞線失敗
        }
        if(targetNodes.contains(node1) ){
            firstTargetNode = node1 ;
            break ;
        }
        
        for(auto& [node2, p2pEdge] : graph.connectedPositions[node1]){
            if(forbiddenNodes.contains(node2)) continue ; 
            if( node2->is_compitable(netType) ){
                nextCost = A_star_cost(routingInfo, node0, node1, node2, netType, routedNet) + currentCost ;
                if(targetNodes.contains(node2)) nextCost += A_star_cost(routingInfo, node1, node2, nullptr, netType, routedNet) ; // Edgecase

                if(nextCost < costWithPrevNodes[node2->k_id].first){
                    costWithPrevNodes[node2->k_id] = {nextCost, node1} ; 
                    pq.push({-nextCost, node2}) ; // min pq
                }
            }else if(targetNodes.contains(node2)){
                throw logic_error("Target node includes incompitable node.") ;
            }
        }
    }
    
    PositionNodePtr currentNode = firstTargetNode, prevNode ; 

    // 2. Backtracking Step
    for(int i=0; startNodes.find(currentNode)==startNodes.end(); ++i){
        prevNode = currentNode ;
        currentNode = costWithPrevNodes[currentNode->k_id].second ; 
        // detailedNet[currentNode].insert(prevNode) ; detailedNet[prevNode].insert(currentNode) ; 

        
        detailedNet.add_connection(currentNode, prevNode) ; 

        for(auto& tile1 : graph.tileSurroundPositions.at(currentNode)){
            for(auto& tile2 : graph.tileSurroundPositions.at(prevNode)){
                if(tile1==tile2){
                    routingInfo.netSequence[tile1].push_back({currentNode, prevNode}) ;
                }
            }
        }
    }   
}
void DetailedRoute3::clear_all_detailed_nets(RoutingInfo& routingInfo){
    RoutingGraph& graph = routingInfo.graph ; 

    for(auto& position : graph.positions){
        position->clear() ; 
    }
}

void DetailedRoute3::surrounding_detailed_route(RoutingInfo& routingInfo, vector<DetailedNet>& channelDetailedNets, vector<DetailedNet>& resultDetailedNets, DetailedNet& combinedDetailedNet){
    RoutingGraph& graph = routingInfo.graph ; 

    combinedDetailedNet = DetailedNet(MIXED, -1) ; 
    resultDetailedNets.clear() ; 
    clear_all_detailed_nets(routingInfo) ;

    for(int i=0; i<channelDetailedNets.size(); ++i){ 
        update_detailed_net(channelDetailedNets[i]) ;
        combinedDetailedNet.combine(channelDetailedNets[i]) ; 
    }

    for(int i=0; i<channelDetailedNets.size(); ++i){ // 把Signal Net加入考量
        if(channelDetailedNets[i].type==SIGNAL){
            set<PositionNodePtr> startNodes, targetNodes, forbiddenNodes ; 

            DetailedNet signalDetailedNet = channelDetailedNets[i] ; 
            
            DetailedNet upperGroundDetailedNet = channelDetailedNets[i-1] ; 
            DetailedNet lowerGroundDetailedNet = channelDetailedNets[i+1] ;

            DetailedNet leftGroundCoveringDetailedNet(VSS, 0) ; 
            DetailedNet rightGroundCoveringDetailedNet(VSS, 0) ; 
            
            vector<PositionNodePtr> end_points ; lowerGroundDetailedNet.get_end_points(end_points) ; sort(end_points.begin(), end_points.end(), [](PositionNodePtr& p1, PositionNodePtr& p2){return p1->x() < p2->x();}) ;
            if(end_points.size()!=2){
                // throw logic_error("Invalid end_points in channel net. | end_points size: " + to_string(end_points.size())) ; 
                // cout << 
                continue ;
            }
 
            for(auto& [position, _] : upperGroundDetailedNet) targetNodes.insert(position) ;

            startNodes = {end_points.front()} ; 
            A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, combinedDetailedNet, leftGroundCoveringDetailedNet) ; 
            combinedDetailedNet.combine(leftGroundCoveringDetailedNet) ; update_detailed_net(leftGroundCoveringDetailedNet) ; 
            
            startNodes = {end_points.back()} ; 
            A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, combinedDetailedNet, rightGroundCoveringDetailedNet) ; 
            combinedDetailedNet.combine(rightGroundCoveringDetailedNet) ; update_detailed_net(leftGroundCoveringDetailedNet) ; 
            

            DetailedNet groundShielding(VSS, 0) ;
            groundShielding.combine(upperGroundDetailedNet) ;
            groundShielding.combine(lowerGroundDetailedNet) ;
            groundShielding.combine(leftGroundCoveringDetailedNet) ;
            groundShielding.combine(rightGroundCoveringDetailedNet) ;

            resultDetailedNets.push_back(signalDetailedNet) ; 
            resultDetailedNets.push_back(groundShielding) ; 
        }
    }

}

void DetailedRoute3::ground_source_detailed_route(RoutingInfo& routingInfo, DetailedNet& combinedDetailedNet, vector<DetailedNet>& result) {
    RoutingGraph& graph = routingInfo.graph ; 
    double centralX = (routingInfo.chipBoundary.max_corner().x() + routingInfo.chipBoundary.min_corner().x()) / 2 ;
    set<PositionNodePtr> startSet, targetSet, forbiddenSet ;
        
    for(auto& via : graph.vias){
        if(via->isOffsetVia) forbiddenSet.insert(*graph.viaExtendedPositions.at(via).begin()) ; 
    }

    result.clear() ; 
    for(auto& type : {VDD, VSS}){
        vector<ViaNodePtr> routingVias ;
        vector<ViaNodePtr> unroutingVias ; 

        vector<PositionNodePtr> routingPositions ; 
        DetailedNet newDetailedNet(type, 0) ;
        DetailedNet groupingDetailedNet(type, 0) ; 

        // 找出power/ground rows 
        for(auto& viaMatrix : {graph.die1ViaMatrix, graph.die2ViaMatrix}){
            for(int i=0; i<viaMatrix.size(); ++i){
                bool isTargetRow = true ; 
                for(int j=0; j<viaMatrix[i].size() && isTargetRow; ++j){
                    if(viaMatrix[i][j]->type!=DUMMY && viaMatrix[i][j]->type!=type) isTargetRow = false ;
                }            

                if(isTargetRow){
                    for(int j=0; j<viaMatrix[i].size(); ++j){
                        if(viaMatrix[i][j]->type!=DUMMY && !viaMatrix[i][j]->isOffsetVia) routingVias.push_back(viaMatrix[i][j]) ;
                    }            
                }else{
                    for(int j=0; j<viaMatrix[i].size(); ++j){
                        if(!viaMatrix[i][j]->isOffsetVia && viaMatrix[i][j]->type==type) unroutingVias.push_back(viaMatrix[i][j]) ;
                    } 
                }
            }
        }

        for(auto& via : routingVias) routingPositions.push_back(*graph.viaExtendedPositions.at(via).begin()) ;


        sort(routingPositions.begin(), routingPositions.end(), 
            [&centralX](const PositionNodePtr& node1, const PositionNodePtr& node2){
                return  node1->x() < node2->x() ; 
            }
        ) ;


        startSet = {routingPositions.front()} ; targetSet = {routingPositions.back()} ;
        A_star_route(routingInfo, startSet, targetSet, forbiddenSet, combinedDetailedNet, newDetailedNet) ; 

        combinedDetailedNet.combine(newDetailedNet) ; groupingDetailedNet.combine(newDetailedNet) ; 
        update_detailed_net(newDetailedNet) ; 

        startSet.clear() ; for(auto& [position, _] : newDetailedNet) startSet.insert(position) ;

        for(int i=1; i<routingPositions.size()-1; ++i){
            targetSet = {routingPositions[i]} ;

            if(startSet.size()){
                A_star_route(routingInfo, startSet, targetSet, forbiddenSet, combinedDetailedNet, newDetailedNet) ; 
                combinedDetailedNet.combine(newDetailedNet) ; groupingDetailedNet.combine(newDetailedNet) ; 
                update_detailed_net(newDetailedNet) ; 
                for(auto& [position, _] : newDetailedNet) startSet.insert(position) ;
            }else{
                for(auto& postion : targetSet) startSet.insert(postion) ;
            }
        }
        result.push_back(groupingDetailedNet) ;

        for(auto& via : unroutingVias){
            DetailedNet singalDetailedNet(type, 0) ; 
            PositionNodePtr position = *graph.viaExtendedPositions.at(via).begin() ;
            singalDetailedNet[position] = set<PositionNodePtr>() ;
            result.push_back(singalDetailedNet) ;
        }
    }
}
void DetailedRoute3::combine_detailed_net_groups(RoutingInfo& routingInfo, DetailedNet& combinedDetailedNet, vector<DetailedNet>& groupingDetailedNets, vector<DetailedNet>& groundSourceDetailedNets, vector<DetailedNet>& result) {
    result.clear() ; 
    clear_all_detailed_nets(routingInfo) ;
    for(int i=0; i<groupingDetailedNets.size(); ++i) update_detailed_net(groupingDetailedNets[i]) ;
    for(int i=0; i<groundSourceDetailedNets.size(); ++i) update_detailed_net(groundSourceDetailedNets[i]) ;

    DetailedNet VSSMainGroup ; 
    DetailedNet VDDMainGroup ; 

    vector<DetailedNet> unroutedGroundSourceDetailedNets ;
    for(int i=0; i<groundSourceDetailedNets.size(); ++i){
        if(groundSourceDetailedNets[i].size()<=1){
            unroutedGroundSourceDetailedNets.push_back(groundSourceDetailedNets[i]) ; 
            continue;
        }
        if(groundSourceDetailedNets[i].type==VSS) VSSMainGroup = groundSourceDetailedNets[i] ; 
        else if(groundSourceDetailedNets[i].type==VDD) VDDMainGroup = groundSourceDetailedNets[i] ; 
    }

    for(auto& w : {0, 1}){
        vector<DetailedNet>& routingDetailedNets = !w ? groupingDetailedNets : unroutedGroundSourceDetailedNets ;
        for(int i=0; i<routingDetailedNets.size(); ++i){
            set<PositionNodePtr> startNodes, targetNodes, forbiddenNodes ; 
            BumpType type = routingDetailedNets[i].type ; 
            DetailedNet routingNet(type, 0) ; 

            for(auto& [pos, _] : routingDetailedNets[i]) startNodes.insert(pos) ;

            if(type==VSS){
                for(auto& [pos, _] : VSSMainGroup) targetNodes.insert(pos) ;
            }else if(type==VDD){
                for(auto& [pos, _] : VDDMainGroup) targetNodes.insert(pos) ;
            }else{
                result.push_back(routingDetailedNets[i]) ; 
                continue ;
            }   

            A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, combinedDetailedNet, routingNet) ; 
            
            if(!routingNet.size()) routingNet = routingDetailedNets[i] ;
            if(routingNet.size()){
                combinedDetailedNet.combine(routingNet) ; 
                if(type==VSS){
                    VSSMainGroup.combine(routingDetailedNets[i]) ; 
                    VSSMainGroup.combine(routingNet) ; 
                }else if(type==VDD){
                    VDDMainGroup.combine(routingDetailedNets[i]) ; 
                    VDDMainGroup.combine(routingNet) ; 
                }
                update_detailed_net(routingNet) ; 
            }
        }
    }
    
    result.push_back(VSSMainGroup) ; 
    result.push_back(VDDMainGroup) ; 
}

void DetailedRoute3::generate_design_teardrop(RoutingInfo& routingInfo, vector<DetailedNet>& detailedNets, vector<Teardrop>& teardrops) {
    RoutingGraph& graph = routingInfo.graph ; 

    teardrops.clear() ;
    for(auto& detailedNet : detailedNets){
        for(auto& [position1, positions] : detailedNet){
            for(auto& position2 : positions){
                if(position1 >= position2) continue ;
                if(graph.viaExtendedPositions.contains(position1)){
                    ViaNodePtr via = *graph.viaExtendedPositions.at(position1).begin() ;
                    double dx = position2->x() - via->x() ;
                    double dy = position2->y() - via->y() ;
                    double theta = atan2(dy, dx) ;  
                    double rx = designRule.minimumTeardropDist*cos(theta) ;
                    double ry = designRule.minimumTeardropDist*sin(theta) ;
                    // designRule.minimumTeardropDist
                    segment_xy segement = {{via->x(), via->y()}, {via->x()+rx, via->y()+ry}} ; 

                    teardrops.push_back( Teardrop(via->die, via->type, via->id, segement) ) ;
                }
            }
        }
    }
}

double DetailedRoute3::detailed_route(RoutingInfo& routingInfo) {
    vector<DetailedNet> channelDetailedNets, groupingDetailedNets  ;
    vector<DetailedNet> groundSourceDetailedNets  ;
    vector<DetailedNet> finalDetailedNets  ;

    DetailedNet combinedDetailedNet ; 

    Timer timer; timer.set_clock() ;
    set_routing_geometry_strategy(routingInfo) ;

    greedy_detailed_route(routingInfo, channelDetailedNets) ; 
    
    surrounding_detailed_route(routingInfo, channelDetailedNets, groupingDetailedNets, combinedDetailedNet) ;
    ground_source_detailed_route(routingInfo, combinedDetailedNet, groundSourceDetailedNets) ;

    // for(auto& detailedNet : groundSourceDetailedNets) groupingDetailedNets.push_back(detailedNet) ; 
    combine_detailed_net_groups(routingInfo, combinedDetailedNet, groupingDetailedNets, groundSourceDetailedNets, routingInfo.detailedNets) ;

    // generate_design_net(groupingDetailedNets, routingInfo.designNets) ;
    generate_design_net(routingInfo.detailedNets, routingInfo.designNets) ;
    generate_design_teardrop(routingInfo, routingInfo.detailedNets, routingInfo.teardrops) ;


    cout << "Elapsed Time in A* Routing: " << timer.get_duration_milliseconds() << " ms\n" ;

    return 0.0 ;
} 
