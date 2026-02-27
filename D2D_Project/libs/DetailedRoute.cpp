#include "DetailedRoute.hpp"
#define M_COST 1000000 

// #define DE
#ifdef DE 
int debugFlag = 0 ;

int bufferInUpdate = 0 ;
int bufferInSearch = 0 ;


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
double DetailedRoute::caclute_teardrop_extension_penalty(RoutingGraph& graph, PositionNodePtr& currentNode, PositionNodePtr& nextNode){
    double minimumDistance = 0.0 ;
    const double extensionProduct = 1.5 ; 
    if(graph.viaExtendedPositions.contains(currentNode)) minimumDistance += designRule.minimumTeardropDist*extensionProduct ;
    if(graph.viaExtendedPositions.contains(nextNode)) minimumDistance += designRule.minimumTeardropDist*extensionProduct ;

    return (distance_sc(*currentNode, *nextNode)<minimumDistance) ? 1.0 : 0.0 ; 
}

double angle(const PositionNodePtr& position, const PositionNodePtr& center){
    double dx = position->x() - center->x() ; 
    double dy = position->y() - center->y() ; 
    return atan2(dy, dx) + M_PI ; // [0, 2*pi]
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

// if(currentNode->id==5686 && nextNode->id==5701){
// cout << "\t\t\t" << *prevNode << " -> " << *currentNode << " -> " << *nextNode << " | " <<  connectedCount << "\n" ;
// }
    return connectedCount ; 
}


vector<Timer> timers(10) ; 
vector<double> elapsedTimes(10, 0.0) ; 

double DetailedRoute::A_star_cost(RoutingInfo& routingInfo, PositionNodePtr prevNode, PositionNodePtr currentNode, PositionNodePtr nextNode, BumpType netType, const DetailedNet& routedNet) {
    double wireLength = nextNode ? distance_sc(*currentNode, *nextNode) : 0 ;
    double crossingCount = 0 ; // nextNode ? caclute_conflict_count(routingInfo.graph, currentNode, nextNode, netType, routedNet) : 0 ; 
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


// if(globalNet.startVia && globalNet.startVia->id==40){
//     for(auto& edge : globalNet){
//         cout << *edge << "\n" ; 
//     }
// }
        ChannelDetailedNet newDetailedNet(globalNet.startVia, globalNet.targetVia) ; 

        if(globalNet.startVia){
            newDetailedNet.push_back(*graph.viaExtendedPositions.at(globalNet.startVia).begin()) ;
        }

        EdgeType prevType = LegEdge ; 
        BumpType netType = (globalNet.startVia) ? SIGNAL : VSS ; 
        double prevY ;
        int *positionIndexPtr, sign ;

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

        if(globalNet.targetVia){
            newDetailedNet.push_back(*graph.viaExtendedPositions.at(globalNet.targetVia).begin()) ;
        }
DanglingNet:;
        for(auto& position: newDetailedNet){
            position->occupyType = netType ; 
        }
        channelDetailedNets.push_back(newDetailedNet) ; 
    }
}

void DetailedRoute::A_star_route(RoutingInfo& routingInfo, set<PositionNodePtr>& startNodes, set<PositionNodePtr>& targetNodes, set<PositionNodePtr>& forbiddenNodes, const DetailedNet& routedNet, DetailedNet& detailedNet) {
    RoutingGraph& graph = routingInfo.graph ; 

    // cout << "START:\n" ;
    // for(auto& node1 : startNodes){
    //     cout << *node1 << " " ;
    // }
    // cout << "\nTARGET:\n" ;
    // for(auto& node1 : targetNodes){
    //     cout << *node1 << " " ;
    // }
    // cout << "\n" ;

    for(auto& node1 : startNodes){
        if(targetNodes.contains(node1)){
            detailedNet.clear() ;
            // cout << "CONTAIN\n" ;

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

// if(cid==5674){
// cout << "\t" << *node1 << " " << currentCost << "\n" ;
// }

        if(currentCost>=M_COST){
            cout << "FAIL\n" ;
            return  ; // 代表繞線失敗
        }
// cout << *node1 << " " ;
// if(costWithPrevNodes[node1->k_id].second) cout << *costWithPrevNodes[node1->k_id].second << " | " << currentCost << "\n" ;
// else cout << " | " << currentCost << "\n" ; 
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

                // if(node1->die==DIE2 && node1->type==DUMMY && (node1->id==5686)){
                //     cout << "\t\t" << *node1 << " -> " << *node2 << " | " << currentCost << " " << nextCost << "\n" ;
                // }
                
                if(nextCost < costWithPrevNodes[node2->k_id].first){
                    // if(node2->die==DIE1 && node2->type==DUMMY && node2->id==2837){
                        // cout << "\t" << *node1 << " -> " << *node2 << " | " << costWithPrevNodes[node2->k_id].first << " " << nextCost << "\n" ;
                    // }
                    costWithPrevNodes[node2->k_id] = {nextCost, node1} ; 
                    pq.push({-nextCost, node2}) ;
                }

                
            }
        }
    }
    

    PositionNodePtr currentNode = firstTargetNode, prevNode ; 
    currentNode->occupyType = netType ; 
    detailedNet.clear() ;
    // cout << "Clear\n" ;
    // cout << *currentNode << "|\n" ;
    // 2. Backtracking Step
    for(int i=0; startNodes.find(currentNode)==startNodes.end(); ++i){
        prevNode = currentNode ;
        currentNode = costWithPrevNodes[currentNode->k_id].second ; 
        // cout << *currentNode << "|\n" ;
        detailedNet[currentNode].insert(prevNode) ; detailedNet[prevNode].insert(currentNode) ; 
        // bool currentIsViaNode = graph.viaExtendedPositions.contains(currentNode) ;
        // bool prevIsViaNode = graph.edgeExtendedPositions.contains(prevNode) ;
        // bool currentAndPrevOnSameLine = graph.edgeExtendedPositions.contains(currentNode) && 
        //                                 graph.edgeExtendedPositions.contains(prevNode) &&
        //                                 graph.edgeExtendedPositions[currentNode]==graph.edgeExtendedPositions[prevNode] ;
        
        // if(currentIsViaNode && prevIsViaNode){
        //     shared_ptr<ViaNode> currentViaNode = graph.positionNodeToViaNodeMapping[currentNode] ; 
        //     shared_ptr<ViaNode> prevViaNode = graph.positionNodeToViaNodeMapping[prevNode] ; 
        //     shared_ptr<EdgeNode> intermediateEdgeNode = graph.viaToViaEdgeMappings[{currentViaNode, prevViaNode}].intermediateEdgeNode ; 
        //     vector<shared_ptr<PositionNode>>& positionNodes = graph.edgeNodeToPositionNodeMapping[intermediateEdgeNode] ;
            
        //     int startIndex = ((pow(positionNodes.front()->x - prevViaNode->x,2)+pow(positionNodes.front()->y - prevViaNode->y,2)) > 
        //                     (pow(positionNodes.back()->x - prevViaNode->x,2)+pow(positionNodes.back()->y - prevViaNode->y,2))) ? 0 : positionNodes.size()-1 ;
                                
        //     int targetIndex = (startIndex==0) ? positionNodes.size()-1 : 0 ; 
        //     int sign = (startIndex==0) ? 1 : -1 ; 
        //     detailedNet[positionNodes[startIndex]].insert(prevNode) ; 
        //     detailedNet[prevNode].insert(positionNodes[startIndex]) ; 

        //     detailedNet[positionNodes[targetIndex]].insert(currentNode) ; 
        //     detailedNet[currentNode].insert(positionNodes[targetIndex]) ; 
            
        //     for(int i=startIndex; i+sign!=targetIndex; i+=sign){
        //         detailedNet[positionNodes[i]].insert(positionNodes[i+sign]) ; 
        //         detailedNet[positionNodes[i+sign]].insert(positionNodes[i]) ; 
        //     }
        // }else if(currentAndPrevOnSameLine){ 
        //     vector<PositionNodePtr>& positionNodes = graph.sortedEdgeExtendedPositions[*graph.edgeExtendedPositions[currentNode].begin()] ;
        //     int currentIndex = find(positionNodes.begin(), positionNodes.end(), currentNode)-positionNodes.begin() ; 
        //     int prevIndex = find(positionNodes.begin(), positionNodes.end(), prevNode)-positionNodes.begin() ; 
        //     for(int sign = (currentIndex>prevIndex) ? 1 : -1, i = prevIndex; i!=currentIndex; i+=sign){
        //         detailedNet[positionNodes[i]].insert(positionNodes[i+sign]) ; detailedNet[positionNodes[i+sign]].insert(positionNodes[i]) ; 
        //     }
        // }else{
            // detailedNet[currentNode].insert(prevNode) ; detailedNet[prevNode].insert(currentNode) ; 
        // }
    }   
}

void DetailedRoute::update_detailed_net(DetailedNet& detailedNet, DetailedNet& newDetailedNet) {

    for(auto& [node1, connectedNodes] : newDetailedNet){
        for(auto& node2 : connectedNodes){
            node1->occupyType = node2->occupyType = newDetailedNet.type ; 
            detailedNet[node1].insert(node2) ; detailedNet[node2].insert(node1) ; 

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

void DetailedRoute::surrounding_detailed_route2(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets){
    RoutingGraph& graph = routingInfo.graph ; 
    
    DetailedNet newDetailedNet(DUMMY, 0), groundDetailedNet(VSS, 0) ; 
    DetailedNet allConnectionNets(DUMMY, -1) ; 

    for(int i=0; i<channelDetailedNets.size(); ++i){ // 把Signal Net加入考量
        if(channelDetailedNets[i].startVia){
            set<PositionNodePtr> startNodes, targetNodes, forbiddenNodes ; 

            newDetailedNet = channelDetailedNets[i].to_detailed_net(graph) ; newDetailedNet.type = SIGNAL ;
            update_detailed_net(allConnectionNets, newDetailedNet) ; 
                         
            groundDetailedNet.clear() ; 
            newDetailedNet = channelDetailedNets[i-1].to_detailed_net(graph) ; newDetailedNet.type = VSS ;
            update_detailed_net(allConnectionNets, newDetailedNet) ; 
            update_detailed_net(groundDetailedNet, newDetailedNet) ; 
            for(auto& [position, _] : groundDetailedNet) targetNodes.insert(position) ;

            startNodes = {channelDetailedNets[i+1].front()} ; 
            A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, allConnectionNets, newDetailedNet) ; 
            update_detailed_net(allConnectionNets, newDetailedNet) ; 
            update_detailed_net(groundDetailedNet, newDetailedNet) ; 


            startNodes = {channelDetailedNets[i+1].back()} ; 
            A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, allConnectionNets, newDetailedNet) ; 
            update_detailed_net(allConnectionNets, newDetailedNet) ; 
            update_detailed_net(groundDetailedNet, newDetailedNet) ; 

            newDetailedNet = channelDetailedNets[i+1].to_detailed_net(graph) ; newDetailedNet.type = VSS ;
            update_detailed_net(allConnectionNets, newDetailedNet) ; 
            update_detailed_net(groundDetailedNet, newDetailedNet) ; 
            
            detailedNets.push_back(groundDetailedNet) ;
            detailedNets.push_back(channelDetailedNets[i].to_detailed_net(graph)) ;

// for(auto& net : {groundDetailedNet}){
// for(auto& [node1, connectedNodes] : net){
// for(auto& node2 : connectedNodes){
// string color = net.type==SIGNAL ? "#f60541" : "#06ea4e" ;
// Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
// routingInfo.debugEdgeMapping["A_START_NET_LEFT_" + to_string(i)].push_back(newEdge) ; 
// }
// }
// }
        }
    }

    if(groundDetailedNet.size()) detailedNets.push_back(groundDetailedNet) ;
}


void DetailedRoute::surrounding_detailed_route(RoutingInfo& routingInfo, vector<ChannelDetailedNet>& channelDetailedNets, vector<DetailedNet>& detailedNets){
    RoutingGraph& graph = routingInfo.graph ; 
    set<PositionNodePtr> startNodes, targetNodes, forbiddenNodes ; 
    DetailedNet newDetailedNet(DUMMY, 0), groundDetailedNet(VSS, 0) ; 
    DetailedNet allConnectionNets(DUMMY, -1) ; 

    for(int i=0; i<channelDetailedNets.size(); ++i){ // 把Signal Net加入考量
        if(!channelDetailedNets[i].startVia) continue ;
        newDetailedNet = channelDetailedNets[i].to_detailed_net(graph) ; newDetailedNet.type = SIGNAL ;
        update_detailed_net(allConnectionNets, newDetailedNet) ; 
        detailedNets.push_back(newDetailedNet) ;
    }

    for(int i=0; i<channelDetailedNets.size(); ++i){ 
        //代表是Signal Net
        // if(i>=4) break; 

        if(channelDetailedNets[i].startVia) continue ;
        if(groundDetailedNet.size()){
                startNodes = {channelDetailedNets[i].front()} ; 
            // for(auto& node : startNodes){
            //     routingInfo.debugBumpMapping["A_START_NET_" + to_string(i)].push_back(static_cast<Bump>(*node) ) ; 
            //     routingInfo.debugBumpMapping["A_START_NET_" + to_string(i)].back().type = SIGNAL ; 
            // }
                newDetailedNet.type = VSS ;
                A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, allConnectionNets, newDetailedNet) ; 
                update_detailed_net(allConnectionNets, newDetailedNet) ; update_detailed_net(groundDetailedNet, newDetailedNet) ; 
                
// cout << newDetailedNet.size() << "\n" ;
// for(auto& net : {newDetailedNet}){
// for(auto& [node1, connectedNodes] : net){
// for(auto& node2 : connectedNodes){
// string color = net.type==SIGNAL ? "#f60541" : "#06ea4e" ;
// Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
// routingInfo.debugEdgeMapping["A_START_NET_LEFT_" + to_string(i)].push_back(newEdge) ; 
// }
// }
// }
// routingInfo.debugBumpMapping["A_START_NET_LEFT_" + to_string(i)].push_back(*channelDetailedNets[i].front()) ; 
// routingInfo.debugBumpMapping["A_START_NET_LEFT_" + to_string(i)].back().type = VSS ; 
// for(auto& node : targetNodes){
//     routingInfo.debugBumpMapping["A_START_NET_LEFT_" + to_string(i)].push_back(static_cast<Bump>(*node) ) ; 
// }
// cout << "--------------------------------\n" ;
                newDetailedNet.type = VSS ;
                startNodes = {channelDetailedNets[i].back()} ; 
            // for(auto& node : startNodes){
            //     routingInfo.debugBumpMapping["A_START_NET_" + to_string(i)].push_back(static_cast<Bump>(*node) ) ; 
            //     routingInfo.debugBumpMapping["A_START_NET_" + to_string(i)].back().type = SIGNAL ; 
            // }
                A_star_route(routingInfo, startNodes, targetNodes, forbiddenNodes, allConnectionNets, newDetailedNet) ; 
                update_detailed_net(allConnectionNets, newDetailedNet) ; 
                update_detailed_net(groundDetailedNet, newDetailedNet) ; 

// cout << newDetailedNet.size() << "\n" ;
// for(auto& net : {newDetailedNet}){
// for(auto& [node1, connectedNodes] : net){
// for(auto& node2 : connectedNodes){
// string color = net.type==SIGNAL ? "#f60541" : "#06ea4e" ;
// Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
// routingInfo.debugEdgeMapping["A_START_NET_RIGHT_" + to_string(i)].push_back(newEdge) ; 
// }
// }
// }
// routingInfo.debugBumpMapping["A_START_NET_RIGHT_" + to_string(i)].push_back(*channelDetailedNets[i].back()) ; 
// routingInfo.debugBumpMapping["A_START_NET_RIGHT_" + to_string(i)].back().type = VSS ; 

// for(auto& node : targetNodes){
    // routingInfo.debugBumpMapping["A_START_NET_RIGHT_" + to_string(i)].push_back(static_cast<Bump>(*node) ) ; 
// }

        }

        newDetailedNet = channelDetailedNets[i].to_detailed_net(graph) ;
        update_detailed_net(allConnectionNets, newDetailedNet) ; update_detailed_net(groundDetailedNet, newDetailedNet) ; 
        for(auto& [position, _] : groundDetailedNet) targetNodes.insert(position) ;
    }

    if(groundDetailedNet.size()) detailedNets.push_back(groundDetailedNet) ;
}



void DetailedRoute::ground_source_detailed_route(RoutingInfo& routingInfo, vector<DetailedNet>& routedDetailedNets,  vector<DetailedNet>& finalDetailedNet) {
    RoutingGraph& graph = routingInfo.graph ; 
    DetailedNet allConnectionNets ; 
    double centralX = (routingInfo.chipBoundary.max_corner().x() + routingInfo.chipBoundary.min_corner().x()) / 2 ;
    finalDetailedNet = routedDetailedNets ; 

    for(auto& detailedNet : routedDetailedNets){
        update_detailed_net(allConnectionNets, detailedNet) ; 
    }

    for(auto& type : {VDD, VSS}){
        vector<PositionNodePtr> routingPositions ; 
        set<PositionNodePtr> startSet, targetSet, forbiddenSet ;
        DetailedNet newDetailedNet(type, 0), coibinedDetailedNet(type, 0) ; 

        for(auto& via : graph.vias){
            if(via->type==type){
                if(via->isOffsetVia) forbiddenSet.insert(*graph.viaExtendedPositions.at(via).begin()) ; 
                else routingPositions.push_back(*graph.viaExtendedPositions.at(via).begin()) ;
            }
        }

     

        sort(routingPositions.begin(), routingPositions.end(), 
            [&centralX](const PositionNodePtr& node1, const PositionNodePtr& node2){
                return  node1->x() < node2->x() ; 
            }
        ) ;

        for(auto& [position, _] : allConnectionNets){
            if(position->occupyType==type) startSet.insert(position) ;
        }

        startSet = {routingPositions[1]} ;
        targetSet = {routingPositions.back()} ;
        A_star_route(routingInfo, startSet, targetSet, forbiddenSet, allConnectionNets, newDetailedNet) ; 
        update_detailed_net(allConnectionNets, newDetailedNet) ; 
        update_detailed_net(coibinedDetailedNet, newDetailedNet) ; 

        startSet.clear() ;
        for(auto& [position, _] : newDetailedNet) startSet.insert(position) ;
        for(auto& detailedNet : routedDetailedNets){
            if(detailedNet.type!=type) continue ;
            for(auto& [position, _] : detailedNet){
                startSet.insert(position) ; 
            }
        }


        for(int i=2; i<routingPositions.size()-1; ++i){
            // if(i==2) i=routingPositions.size()-1;
            targetSet = {routingPositions[i]} ;

            if(startSet.size()){
                A_star_route(routingInfo, startSet, targetSet, forbiddenSet, allConnectionNets, newDetailedNet) ; 
                update_detailed_net(allConnectionNets, newDetailedNet) ; 
                update_detailed_net(coibinedDetailedNet, newDetailedNet) ; 
                for(auto& [position, _] : newDetailedNet) startSet.insert(position) ;
            }else{
                for(auto& postion : targetSet) startSet.insert(postion) ;
            }
        }
        finalDetailedNet.push_back(coibinedDetailedNet) ;

for(auto& net : {coibinedDetailedNet}){
    for(auto& [node1, connectedNodes] : net){
        for(auto& node2 : connectedNodes){
            string color = net.type==VSS ? "#06ea4e" : "#0008ff"  ;
            Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
            routingInfo.debugEdgeMapping["GROUND_SOURCE"].push_back(newEdge) ; 
        }
    }
}
        if(coibinedDetailedNet.type==VSS){
            set<PositionNodePtr> startSet, targetSet, forbiddenSet ;
            DetailedNet newDetailedNet(VSS, 0) ;
            
            for(auto& groundShildingDetailedNet : routedDetailedNets){
                if(groundShildingDetailedNet.type!=VSS) continue ;
                for(auto& [position, _] : groundShildingDetailedNet) startSet.insert(position) ; 
            }

            for(auto& [position, _] : coibinedDetailedNet){
                targetSet.insert(position) ; 
            }

            A_star_route(routingInfo, startSet, targetSet, forbiddenSet, allConnectionNets, newDetailedNet) ; 
            finalDetailedNet.push_back(newDetailedNet) ;
        }
    }
}


void DetailedRoute::generate_design_net(vector<DetailedNet>& detailedNets, vector<Net>& designNets) {
    designNets.clear() ; 
    for(auto& detailedNet : detailedNets){
        designNets.push_back(detailedNet.to_design_net()) ;
    }
}

void DetailedRoute::combine_detailed_nets(vector<DetailedNet>& detailedNets, vector<DetailedNet>& result) {
    map<pair<BumpType, int>, DetailedNet> mappingDetailedNet ;
    result.clear() ; 

    for(auto& detailNet : detailedNets){
        pair<BumpType, int> key = {detailNet.type, detailNet.id} ;
        if(!mappingDetailedNet.contains(key)){
            mappingDetailedNet[key] = detailNet ; 
        }else{
            update_detailed_net(mappingDetailedNet[key], detailNet) ;
        }
    } 

    for(auto& [key, detailedNet] : mappingDetailedNet){
        result.push_back(detailedNet) ; 
    }
}

void DetailedRoute::set_routing_geometry_strategy() {
    routingDistStrategy = bgsb::distance_symmetric<double>(designRule.minimumLineWidth) ;
    routingSideStrategy = bgsb::side_straight() ;
    routingJoinStrategy = bgsb::join_miter() ;
    routingEndStrategy = bgsb::end_flat() ;  // 讓末端變成平的四邊形
    routingCircleStrategy = bgsb::point_circle() ;
}

double DetailedRoute::detailed_route(RoutingInfo& routingInfo) {
    vector<ChannelDetailedNet> channelDetailedNets ;
    vector<DetailedNet> detailedNets, result ;

    Timer timer; timer.set_clock() ;

    rtree.clear() ;  
    set_routing_geometry_strategy() ;

    greedy_detailed_route(routingInfo, channelDetailedNets) ; 
    surrounding_detailed_route2(routingInfo, channelDetailedNets, detailedNets) ; 
    // routingInfo.detailedNets = detailedNets ;
    ground_source_detailed_route(routingInfo, detailedNets, result) ;
    combine_detailed_nets(result, routingInfo.detailedNets) ;
    generate_design_net(routingInfo.detailedNets, routingInfo.designNets) ;

    cout << "Elapsed Time in A* Routing: " << timer.get_duration_milliseconds() << " ms\n" ;

    for(auto& net : channelDetailedNets){
        for(int i=0; i<int(net.size())-1; ++i){
            auto node1 = net[i] ; 
            auto node2 = net[i+1] ; 
            string color = net.startVia ? "#f60541" : "#06ea4e" ;
            Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 2, color) ; 
            routingInfo.debugEdgeMapping["GREEDY_NET"].push_back(newEdge) ; 
        }
    }
    cout << "Update times: " << bufferInUpdate << "\n" ;
    cout << "Search times: " << bufferInSearch << "\n" ;
// 805084
// 1855754
// 3027549
// 3027549
    // for(auto& node : routingInfo.graph.positions){
    //     routingInfo.debugBumpMapping["DB_Positions"].push_back( static_cast<Bump>(*node) ) ;
    //     routingInfo.debugBumpMapping["DB_Positions"].back().type = node->occupyType ; 
    // }


    // power_rail_detailed_route() ;
    return 0.0 ;
} 

#endif