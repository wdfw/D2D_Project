#include "Router.hpp"

template<typename T> void write_result(const string& directory, const string& mainFileName, const string& , const vector<T>& elements) ;
template<typename T> void write_result(const string& directory, const string& mainFileName, const string& extension, const box_xy& boundary, const vector<T>& elements) ;
Net DetailedNet::to_design_net() {
        
    Net designNet(type, id) ;
    
    for(auto& [node1, connectedNodes] : *this){
        for(auto& node2 : connectedNodes){
            designNet.push_back(segment_xy(static_cast<const point_xy&>(*node1), static_cast<const point_xy&>(*node2))) ; 
        }
    }
    return designNet ; 
}

void DetailedNet::combine(DetailedNet& newDetailedNet) {
  for(auto& [node1, connectedNodes] : newDetailedNet){
        for(auto& node2 : connectedNodes){
            (*this)[node1].insert(node2) ; (*this)[node2].insert(node1) ; 
        }
    }
}
DetailedNet ChannelDetailedNet::to_detailed_net(RoutingGraph& graph) {
    DetailedNet detailedNet ; 
    if(startVia){
        detailedNet.type = startVia->type ;
        detailedNet.id = startVia->id ;
        detailedNet[*graph.viaExtendedPositions.at(startVia).begin()].insert(at(0)) ;
        detailedNet[at(0)].insert(*graph.viaExtendedPositions.at(startVia).begin()) ;
    }

    
    for(int i=0; i<size()-1; ++i){
        detailedNet[at(i)].insert(at(i+1)) ; 
        detailedNet[at(i+1)].insert(at(i)) ; 
    }

    if(targetVia){
        detailedNet[*graph.viaExtendedPositions.at(targetVia).begin()].insert(back()) ;
        detailedNet[back()].insert(*graph.viaExtendedPositions.at(targetVia).begin()) ;
    }
    return detailedNet ;
}

void Router::renew_routing_bumps(RoutingInfo& routingInfo){
    map<ViaNodePtr, bool> powerPlaneMapping ; 
    map<ViaNodePtr, int> nodeRow ; 

    vector<Bump> newRoutingBumps = routingInfo.routingBumps ; 
    vector<OffsetVia> newOffsetVias ;
    RoutingGraph& graph = routingInfo.graph ; 
    
    for(auto& viaMatrix : {graph.die1ViaMatrix, graph.die2ViaMatrix}){
        for(auto& rowVias : viaMatrix){
            for(auto& via : rowVias){
                if(via->type!=DUMMY && via->type==SIGNAL){
                    for(auto& nonPowerPlaneVia : rowVias){
                        powerPlaneMapping[nonPowerPlaneVia] = false ; 
                    }
                    break ; 
                }else{
                    powerPlaneMapping[via] = true ; 
                }
            }
        }
    }
// for(auto& detailedNet : routingInfo.detailedNets){
//     for(auto& [node1, nodes] : detailedNet){
//         cout << *node1 << " | " ;
//         for(auto& node2 : nodes){
//             cout << *node2 << " " ;
//         }
//         cout << "\n" ;
//     }
// }

    for(int i=0; i<routingInfo.offsetVias.size(); ++i){
        Bump& via = routingInfo.offsetVias[i].first ; 
        Bump& offsetVia = routingInfo.offsetVias[i].second ; 

        ViaNodePtr viaNode = routingInfo.bump2ViaNodeMapping[via] ;
        ViaNodePtr offsetViaNode = routingInfo.bump2ViaNodeMapping[offsetVia] ;
        
        
        PositionNodePtr viaPosition = *graph.viaExtendedPositions.at(viaNode).begin() ; 
        PositionNodePtr offsetViaPosition = *graph.viaExtendedPositions.at(offsetViaNode).begin() ; 

        for(auto& detailedNet : routingInfo.detailedNets){
            if(detailedNet.contains(viaPosition) && powerPlaneMapping[viaNode]==false){ // 該Via有被繞出 且 不是在上下邊的電源點
                newRoutingBumps.push_back(via) ;
                // cout << *viaNode << " " << "Routing\n" ;
                goto ViaIsRouted ; 
            }
        }

        // cout << *viaNode << " " << *viaPosition << " " << "Unrouting\n" ;
        newOffsetVias.push_back(routingInfo.offsetVias[i]) ; 
ViaIsRouted:;
    }

    routingInfo.routingBumps = newRoutingBumps ;
    routingInfo.offsetVias = newOffsetVias ;
}
void Router::solve(const vector<Bump>& bumps, const DesignRule& designRule, const GAConfiguration& GAConfig,
                     const box_xy& boundary, const string& outputDirectory) {

    this->GAConfig = GAConfig ;
    this->designRule = designRule ; 
    this->boundary = boundary ; 
    this->routingTimes.clear() ; 

    checker = FlowChecker(designRule) ;

    Timer timer;
    double bumpDistance ;
    CostStruct costStruct ;
    string directory ; 

    vector<Bump> unroutedBumps = bumps, routingBumps ;
    box_xy layerBoundary = boundary ; 

    bumpDistance = minimum_distance_between_elements(unroutedBumps) ;

    for(int layer = 1;  unroutedBumps.size() ; ++layer){
        this->outputDirectory = outputDirectory + "/RDL_" + to_string(layer) ; 

        RoutingInfo routingInfo ;

        routingInfo.chipBoundary = layerBoundary ; routingInfo.bumpDistance = bumpDistance ;

timer.set_clock() ;

        select_routing_bump(unroutedBumps, routingInfo, 8) ;
        adjust_offset_vias(routingInfo, layer-1) ;
        add_dummy_bumps(routingInfo) ;

        rouding_bumps(routingInfo) ;

        step1_check(routingInfo) ; 

        construct_routing_graph(routingInfo) ; 
        step2_check(routingInfo) ; 

        global_route(routingInfo, costStruct) ;
        // step3_check(routingInfo) ; 

        detailed_route(routingInfo) ; 

        // design_rule_check(routingInfo) ; 
        // cout << routingInfo.debugEdgeMapping["ERROR_NET"].size() << "\n" ;

        renew_routing_bumps(routingInfo) ;

        cout << "Elapsed Time: " << timer.get_duration_milliseconds() << " ms\n"  ;

        generate_routing_result(routingInfo, outputDirectory + "/RDL_" + to_string(layer)) ;
        unroutedBumps.clear() ; 
        // ./bin/D2D testcase/standard_44/bumps.loc testcase/standard_44/rule.txt ./result -p 1 -g 0 -m 1 -c 0.9 -s 51

        for(auto& offsetVia: routingInfo.offsetVias){
            Bump& referenceBump = offsetVia.second ; 
            unroutedBumps.push_back(Bump(referenceBump.die, referenceBump.type, referenceBump.id, {referenceBump.x(), referenceBump.y()})) ; 
        }
break;
    }
}

void Router::initialize_routing_information(RoutingInfo& routingInfo){
    routingInfo.chipBoundary = this->boundary ; 
}

void Router::select_routing_bump(const vector<Bump>& bumps, RoutingInfo& routingInfo, unsigned int selecNumber){
    //越靠近中線的越先繞
    routingInfo.routingBumps.clear() ; 
    routingInfo.offsetVias.clear() ; 

    int leftSignalCount = 0 ;
    set<int> selectedID ; 
    vector<int> signalBumpIndexes ;

    for(int i=0; i<bumps.size(); ++i){
        if(bumps[i].type==SIGNAL && bumps[i].die==DIE1) signalBumpIndexes.push_back(i) ; 
    }

    sort(signalBumpIndexes.begin(), signalBumpIndexes.end(), [&bumps](int i, int j) { return bumps[i].x()>bumps[j].x();} ) ; // Die1中越靠近右方的Signal bumps越先選
    for(int i=0; i<min(selecNumber, (unsigned int)(signalBumpIndexes.size())); ++i) selectedID.insert(bumps[signalBumpIndexes[i]].id) ;

// selectedID = set<int>{36, 40, 37, 41, 38, 42, 39, 43} ; 
// selectedID = set<int>{54} ; 

    for(int i=0; i<bumps.size(); i++){
        Bump bump = bumps[i] ; 
        Bump offsetVia = bumps[i] ; offsetVia.isOffsetVia = true ; 
        if(bumps[i].type==SIGNAL){
            if(selectedID.contains(bumps[i].id)){
                routingInfo.routingBumps.push_back(bump) ;
            }else{
                routingInfo.offsetVias.push_back({bump, offsetVia}) ;
            }
        }
    }


    if(routingInfo.offsetVias.size()){  //代表還有信號點沒繞成功
        for(int i=0; i<bumps.size(); i++){
            Bump bump = bumps[i] ; 
            Bump offsetVia = bumps[i] ; offsetVia.isOffsetVia = true ; 
            if(bumps[i].type==VSS || bumps[i].type==VDD){
                routingInfo.offsetVias.push_back({bump, offsetVia}) ;
            }
        }
    }else{
        for(int i=0; i<bumps.size(); i++){
            Bump bump = bumps[i] ; 
            if(bumps[i].type==VSS || bumps[i].type==VDD){
                routingInfo.routingBumps.push_back(bump) ;
            }
        }
    }

    for(auto& bump : routingInfo.routingBumps) bump.isRouting = true ; 
    for(auto& offsetVia : routingInfo.offsetVias) offsetVia.first.isRouting = offsetVia.second.isRouting = false ; 
}

void Router::add_dummy_bumps(RoutingInfo& routingInfo){

    const double cos60 = 1.0/2, sin60 = sqrt(3)/2 ; 
    const double distance = routingInfo.bumpDistance ;
    const double groupedDistance = routingInfo.bumpDistance*0.5 ;
    double centerX = (routingInfo.chipBoundary.max_corner().x()+routingInfo.chipBoundary.min_corner().x())/2 ;
    
    box_xy die1Boundary = routingInfo.chipBoundary ; die1Boundary.max_corner().set<0>( centerX - distance) ;
    box_xy die2Boundary = routingInfo.chipBoundary ; die2Boundary.min_corner().set<0>( centerX + distance) ;

    routingInfo.die1Boundary = die1Boundary ; 
    routingInfo.die2Boundary = die2Boundary ; 
    
    for(auto& dieType : {DIE1, DIE2}){
        vector<Bump> routingBumps, nonOffsetBumps, dummyBumps, bumpStacks ; 
        const box_xy dieBoundary = (dieType==DIE1) ? die1Boundary : die2Boundary ;

        for(auto& bump : routingInfo.routingBumps){
            if(bump.die==dieType) routingBumps.push_back(bump) ; 
        }
        for(auto& offsetVia : routingInfo.offsetVias){
            if(offsetVia.die==dieType) nonOffsetBumps.push_back(offsetVia.first) ; 
        }

        bumpStacks = routingBumps ; copy(nonOffsetBumps.begin(), nonOffsetBumps.end(), std::back_inserter(bumpStacks));
        

        for(int i=0; i<bumpStacks.size(); ++i){

            const Bump currentBump = bumpStacks[i] ;
            double cx = currentBump.x(), cy = currentBump.y() ; 
            
            for(auto [rx, ry] : vector<pair<double,double>>{{1,0},{-1,0},{cos60,sin60},{-cos60,sin60},{cos60,-sin60},{-cos60,-sin60}}){
                double nx = cx+rx*distance, ny = cy+ry*distance ; 
                bool isEdgeBump = true ;

                if(ny<dieBoundary.min_corner().y() || ny>dieBoundary.max_corner().y()) continue ;

                if(nx<dieBoundary.min_corner().x()+groupedDistance) nx = dieBoundary.min_corner().x() ;
                else if(nx>dieBoundary.max_corner().x()-groupedDistance) nx = dieBoundary.max_corner().x() ;
                else isEdgeBump = false ; 

                Bump newDummyBump(currentBump.die, DUMMY, dummyBumps.size(), {nx, ny}) ;
                
                for(auto& sercedBumps : {routingBumps, nonOffsetBumps, dummyBumps}){
                    for(auto& sercedBump : sercedBumps){
                        if(distance_sc(newDummyBump, sercedBump)<groupedDistance){ //Design Dependent
                            goto SearchedExistBump ; 
                        }
                    }
                }
        
                dummyBumps.push_back(newDummyBump) ;
                if(!isEdgeBump) bumpStacks.push_back(newDummyBump) ;
                SearchedExistBump: ;
            }
        }

        for(auto& bump : dummyBumps) bump.isRouting = false ;  
        copy(dummyBumps.begin(), dummyBumps.end(), std::back_inserter(routingInfo.routingBumps));
    }
}

void Router::step1_check(const RoutingInfo& routingInfo) const {
    checker.ckeck_bump_and_offset_via(routingInfo) ; 
}

void Router::adjust_offset_vias(RoutingInfo& routingInfo, int layer){
    for(auto& offsetVia : routingInfo.offsetVias ){
        auto& via = offsetVia.second ;
        double dy = 0 ;
        double dx = - (designRule.minimumOffsetViaSpacing + 2*designRule.viaRadius) ;  //Design Dependent
        if(layer%2) dx = -dx ;
        via.set<0>(via.x()+dx) ; 
        via.set<1>(via.y()+dy) ; 
    }
} 

void Router::create_vias_and_v2v_edges(const ConnectionMap& combinedConnections, RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    map<Bump, ViaNodePtr> bump2ViaNodeMapping ; 

    for(auto& [bump, connectedBumps] : combinedConnections){
        int k_id = graph.vias.size() ; 
        ViaNodePtr newViaNode = make_shared<ViaNode>(ViaNode(bump.die, bump.type, bump.id, k_id, {bump.x(), bump.y()})) ; 
        newViaNode->isOffsetVia = bump.isOffsetVia ; newViaNode->isRouting = bump.isRouting ;

        graph.vias.insert(newViaNode) ;
        bump2ViaNodeMapping[bump] = newViaNode ; 
    }

    for(auto& [bump, connectedBumps] : combinedConnections){
        ViaNodePtr lhs = bump2ViaNodeMapping[bump] ; 
        for(auto& connectedBump : connectedBumps){
            ViaNodePtr rhs = bump2ViaNodeMapping[connectedBump] ; 

            if(lhs<rhs){
                ViaToViaEdgePtr newV2VEdge = make_shared<ViaToViaEdge>(ViaToViaEdge(lhs, rhs, graph.v2vEdges.size())) ; 
                graph.v2vEdges.insert(newV2VEdge) ; 
                graph.connectedVias[lhs][rhs] = newV2VEdge ; 
                graph.connectedVias[rhs][lhs] = newV2VEdge ; 
            }
        }
    }

    routingInfo.bump2ViaNodeMapping = bump2ViaNodeMapping ;
}


void Router::create_edges_and_via_crossing(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    for(auto& v2vEdge : graph.v2vEdges){
        ViaNodePtr via1 = v2vEdge->node1 ; 
        ViaNodePtr via2 = v2vEdge->node2 ; 

        DieType dieType = (via1->die==via2->die) ? via1->die : DIE12 ; 
        BumpType bumpType = DUMMY ; 
        point_xy centerPoint = {(via1->x()+via2->x())/2, (via1->y()+via2->y())/2} ; 
        int id = graph.edges.size() ; 
        int k_id = graph.edges.size() ; 
        EdgeNodePtr newEdgeNode = make_shared<EdgeNode>(EdgeNode(dieType, bumpType, id, k_id, centerPoint)) ; 
        graph.edges.insert(newEdgeNode) ; 
        graph.viaCrossingEdges.insert({v2vEdge, newEdgeNode}) ; 
    }
}

void Router::create_positions_and_viaExtendedPositions_and_edgeExtendedPositions(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    const double sapceWithViaPad = designRule.viaPadRadius + designRule.minimumLineViaPadSpacing + designRule.minimumLineWidth/2 ; //Design Dependent
    const double sapceWithoutViaPad = designRule.minimumLineSpacing/2 + designRule.minimumLineWidth/2 ; //Design Dependent

    for(auto& via : graph.vias){
        DieType dieType = via->die ;
        BumpType bumpType = via->type ; 
        point_xy point = {via->x(), via->y()} ; 
        BumpType occupyType = via->type ;  ; 
        bool isViaNdoe = true ;  
        int id = graph.positions.size() ; 
        int k_id = graph.positions.size() ; 
        PositionNodePtr newPositionNode = make_shared<PositionNode>(PositionNode(dieType, bumpType, id, k_id, point, occupyType, isViaNdoe)) ; 
        graph.positions.insert(newPositionNode) ; 
        graph.viaExtendedPositions.insert({via, newPositionNode}) ; 
    }

    for(auto& v2vEdge : graph.v2vEdges){
        for(auto& edgeNodePtr : graph.viaCrossingEdges.at(v2vEdge)){
            ViaNodePtr via1 = v2vEdge->node1, via2 = v2vEdge->node2;
            DieType dieType = (via1->die==via2->die) ? via1->die : DIE12 ; 
            BumpType bumpType = DUMMY ; 
            BumpType occupyType = DUMMY ; 
            bool isViaNdoe = false ;  
            double dx = via2->x() - via1->x() ; 
            double dy = via2->y() - via1->y() ; 
            double bx = via1->x() ;
            double by = via1->y() ;
            double ox = dx/GlobalPositionResolution * 0.5 ; 
            double oy = dy/GlobalPositionResolution * 0.5 ; 
           
            for(int i=0; i<GlobalPositionResolution; ++i){
                int id = graph.positions.size() ; 
                int k_id = graph.positions.size() ; 
                double cx = bx + dx/GlobalPositionResolution*i + ox ; 
                double cy = by + dy/GlobalPositionResolution*i + oy ; 
                point_xy currentPoint(cx, cy) ; 

                if(via1->type!=DUMMY){
                    if(distance_sc(currentPoint, *via1) < sapceWithViaPad) continue ;
                }else{
                    if(distance_sc(currentPoint, *via1) < sapceWithoutViaPad) continue ;
                }

                if(via2->type!=DUMMY){
                    if(distance_sc(currentPoint, *via2) < sapceWithViaPad) continue ;
                }else{
                    if(distance_sc(currentPoint, *via2) < sapceWithoutViaPad) continue ;
                }
                
                PositionNodePtr newPositionNode = make_shared<PositionNode>(PositionNode(dieType, bumpType, id, k_id, {cx, cy}, occupyType, isViaNdoe)) ; 
                graph.positions.insert(newPositionNode) ; 
                graph.edgeExtendedPositions.insert({edgeNodePtr, newPositionNode}) ; 
            }
        }
    }
}

void create_tiles_DFS(  RoutingGraph& graph, vector<vector<ViaNodePtr>>& results, 
                        vector<ViaNodePtr>& buffer, flat_map<ViaNodePtr, bool>& passed, 
                        bgi::rtree<point_xy, bgi::quadratic<500>>& rtree,
                        ViaNodePtr stratNode, ViaNodePtr currentNode, int k ){
    static vector<point_xy> queryResults ; 
    
    if(buffer.size() >= 3){
        if( graph.connectedVias.contains(currentNode) && 
            graph.connectedVias[stratNode].contains(currentNode)){
            polygon_xy poly, hull ; 

            for(auto& viaNodePtr : buffer) bg::append(poly.outer(), static_cast<const point_xy&>(*viaNodePtr)) ;

            bg::convex_hull(poly, hull) ;
            bg::correct(hull) ; // 自動修正方向與閉合狀態
            for(auto& via1 : buffer){ // 尋找是否有內部連接
                int accessCount = 0 ;
                for(auto& via2 : buffer){
                    if(graph.connectedVias.contains(via1) && graph.connectedVias[via1].contains(via2)) ++accessCount ; 
                    if(accessCount>=3) goto SearchedInnerVia ; 
                }
            }

            // queryResults.clear() ; 

            // rtree.query(bgi::within(hull), std::back_inserter(queryResults));
            
            // if(queryResults.size()){
            //     goto SearchedInnerVia ; 
            // }
            for(auto& viaNodePtr : graph.vias){ // 尋找是否有內部via
                if(bg::within(static_cast<const point_xy&>(*viaNodePtr), hull)) goto SearchedInnerVia ; 
            }
            results.push_back(buffer) ; 
            
            SearchedInnerVia: ;
        }
    }
    
    if(buffer.size() < k){
        for(auto& [connectedNode, v2vEdge] : graph.connectedVias[currentNode]){
            if(!passed[connectedNode]){
                passed[connectedNode] = true ;
                buffer.push_back(connectedNode) ;
                create_tiles_DFS(graph, results, buffer, passed, rtree, stratNode, connectedNode, k) ;
                buffer.pop_back() ;
                passed[connectedNode] = false ;
            }
        }
    }
}


void star_create_tiles_DFS(RoutingGraph& graph, vector<vector<ViaNodePtr>>& results, int k){
    if(k<3) throw logic_error("K less than 3 can't form a hull.") ;
    bgi::rtree<point_xy, bgi::quadratic<500>> rtree ; // 驗證convex hull中是否有其他via
    for(auto &viaNodePtr : graph.vias) rtree.insert(point_xy(*viaNodePtr)) ; 

#ifndef PARALLEL_MODE
    vector<ViaNodePtr> buffer ; 
    flat_map<ViaNodePtr, bool> passed ; 
    for(auto via : graph.vias) rtree.insert(point_xy(*via)) ;
    for(auto via : graph.vias) passed[via] = false ; 

    results.clear() ; 
    for(auto via : graph.vias){
        passed[via] = true ;
        buffer.push_back(via) ; 
        create_tiles_DFS(graph, results, buffer, passed, rtree, via, via, k) ; 
        buffer.pop_back() ; 
        passed[via] = false ;
    }
#else 
    vector<flat_map<ViaNodePtr, bool>> passeds(GlobalThreadNum) ; 
    vector<vector<ViaNodePtr>> buffers(GlobalThreadNum) ; 
    vector< vector<vector<ViaNodePtr>> > subResults(GlobalThreadNum) ; 
    vector<ViaNodePtr> startViaNodes(GlobalThreadNum) ; 
    deque<pair<thread, int>> threads ; 

    for(auto &viaNodePtr : graph.vias) passeds[0][viaNodePtr] = false ; 
    for(int i=1; i<passeds.size(); ++i) passeds[i] = passeds[0] ; 
    for(auto& viaNodePtr : graph.vias){
        t_id = (t_id+1) % GlobalThreadNum ;

        passeds[t_id][viaNodePtr] = true ; 
        startViaNodes[t_id] = viaNodePtr ; 
        buffers[t_id].push_back(viaNodePtr) ; 
        threads.push_back({thread(create_tiles_DFS, ref(graph), ref(subResults[t_id]), 
                    ref(buffers[t_id]), ref(passeds[t_id]), ref(rtree), viaNodePtr, viaNodePtr, k), 
                    t_id}) ; 

        if(threads.size()>=GlobalThreadNum){
            int p_id = threads.front().second ; 
            threads.front().first.join() ; threads.pop_front() ;
            buffers[p_id].pop_back() ; 
            passeds[p_id][startViaNodes[p_id]] = false ;
        }
    }

    for(;threads.size();) threads.front().first.join() ; threads.pop_front() ;

    results.clear() ; 
    for(auto& subResult : subResults){
        results.insert(results.end(), subResult.begin(), subResult.end()) ; 
    }
#endif

}

void Router::create_tiles_and_tile_surround_vias(RoutingInfo& routingInfo, int k) {
    RoutingGraph& graph = routingInfo.graph ; 
    vector<vector<ViaNodePtr>> results ; 
    set<ViaNodePtr> passed ;
    set<vector<ViaNodePtr>> occupyTiles ; 
    
    star_create_tiles_DFS(graph, results, k) ;
    // create_tiles_DFS(graph, results, passed, k) ;
    for(auto& result : results){
        double cx = 0.0, cy = 0.0 ; 
        int dieTypeFlag = -1 ;
        BumpType bumpType = DUMMY ; 
        int id = graph.tiles.size() ; 
        int k_id = graph.tiles.size() ; 
        for(auto& via : result){
            if(dieTypeFlag==-1) dieTypeFlag = via->die ; 
            else if(dieTypeFlag!=via->die) dieTypeFlag = DIE12 ; 
            cx += via->x() ; cy += via->y() ; 
        }

        cx /= result.size() ; cy /= result.size() ; 

        sort(result.begin(), result.end()) ;

        if(!occupyTiles.contains(result)){
            TileNodePtr newTile = make_shared<TileNode>(TileNode(static_cast<DieType>(dieTypeFlag), bumpType, id, k_id, {cx, cy})) ; 
            graph.tiles.insert(newTile) ; 
            for(auto& via1 : result){
                for(auto& via2 : result){
                    if(graph.connectedVias[via1].contains(via2)){
                        graph.tileSurroundVias.insert({newTile, via1}) ; 
                        graph.tileSurroundVias.insert({newTile, via2}) ; 
                    }
                }
            }
            occupyTiles.insert(result) ;
        }
    }
}
void Router::create_t2t_edges_and_tile_crossing_edges(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    map<ViaToViaEdgePtr, set<TileNodePtr>> sharedV2VEdges ; 

    for(auto& tile : graph.tiles){
        for(auto& via1 : graph.tileSurroundVias.at(tile)){
            for(auto& via2 : graph.tileSurroundVias.at(tile)){
                if(graph.connectedVias[via1].contains(via2)){
                    sharedV2VEdges[ graph.connectedVias[via1][via2] ].insert(tile) ;
                }
            }   
        }
    }

    for(auto& [v2vEdge, tiles] : sharedV2VEdges){
        for(auto& tile1 : tiles){
            for(auto& tile2 : tiles){
                if(tile1<tile2){
                    TileToTileEdgePtr newT2TEdge = make_shared<TileToTileEdge>(TileToTileEdge(tile1, tile2, graph.t2tEdges.size(), 0)) ; 
                    graph.t2tEdges.insert(newT2TEdge) ;        
                    graph.connectedTiles[tile1][tile2] = graph.connectedTiles[tile2][tile1] = newT2TEdge ;
                    graph.tileCrossingEdges.insert({newT2TEdge, *graph.viaCrossingEdges.at(v2vEdge).begin()}) ; 
                }
            }
        }
    }
}

void Router::create_tileSurroundEdges_and_tileSurroundPositions(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 

    for(auto& tile : graph.tiles){
        for(auto& via1 : graph.tileSurroundVias.at(tile)){
            for(auto& via2 : graph.tileSurroundVias.at(tile)){
                if(via1!=via2 && graph.connectedVias[via1].contains(via2)){
                    ViaToViaEdgePtr& v2vEdge = graph.connectedVias[via1][via2] ;
                    EdgeNodePtr edge = *graph.viaCrossingEdges.at(v2vEdge).begin() ;
                    graph.tileSurroundEdges.insert({tile, edge}) ;
                }
            }
        }
    }

    for(auto& tile : graph.tiles){
        for(auto& edge : graph.tileSurroundEdges.at(tile)){
            for(auto& position : graph.edgeExtendedPositions.at(edge)){
                graph.tileSurroundPositions.insert({tile, position}) ;
            }
        }
    }

    for(auto& tile : graph.tiles){
        for(auto& via : graph.tileSurroundVias.at(tile)){
            for(auto& position : graph.viaExtendedPositions.at(via)){
                graph.tileSurroundPositions.insert({tile, position}) ;
            }
        }
    }
}

void Router::create_e2e_edges(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    for(auto& tile : graph.tiles){
        for(auto& edge1 : graph.tileSurroundEdges.at(tile)){
            for(auto& edge2 : graph.tileSurroundEdges.at(tile)){
                if(edge1 < edge2){
                    EdgeToEdgeEdgePtr newE2EEdge = make_shared<EdgeToEdgeEdge>(EdgeToEdgeEdge(edge1, edge2, graph.e2eEdges.size())) ; 
                    graph.e2eEdges.insert(newE2EEdge) ; 
                    graph.connectedEdges[edge1][edge2] = newE2EEdge ; 
                    graph.connectedEdges[edge2][edge1] = newE2EEdge ; 
                }
            }
        }
    }
}

void Router::create_p2p_edges(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    for(auto& tile : graph.tiles){
        for(auto& position1 : graph.tileSurroundPositions.at(tile)){
            for(auto& position2 : graph.tileSurroundPositions.at(tile)){
                if(position1 < position2){
                    PositionToPositionEdgePtr newP2PEdge = make_shared<PositionToPositionEdge>(PositionToPositionEdge(position1, position2, graph.p2pEdges.size())) ; 
                    graph.p2pEdges.insert(newP2PEdge) ; 
                    graph.connectedPositions[position1][position2] = newP2PEdge ; 
                    graph.connectedPositions[position2][position1] = newP2PEdge ; 
                }
            }
        }
    }
}

void Router::create_viaMatrix(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ;
    vector<ViaNodePtr> die1ViaNodes, die2ViaNodes ; 
    vector<vector<ViaNodePtr>> die1ViaNodeMatrix, die2ViaNodeMatrix ; 
    for(auto& via : graph.vias){
        if(via->die==DIE1) die1ViaNodes.push_back(via) ;
        else if(via->die==DIE2) die2ViaNodes.push_back(via) ;
    }

    matrixlize(die1ViaNodes, die1ViaNodeMatrix, GlobalEpsilonY) ;
    matrixlize(die2ViaNodes, die2ViaNodeMatrix, GlobalEpsilonY) ;
    
    if(die1ViaNodeMatrix.size()!=die2ViaNodeMatrix.size()){
        throw runtime_error("Invaild # of via node rows on 2 dies (" + to_string(die1ViaNodeMatrix.size()) + "," + to_string(die2ViaNodeMatrix.size()) + ")\n") ; 
    }

    graph.die1ViaMatrix = die1ViaNodeMatrix ;
    graph.die2ViaMatrix = die2ViaNodeMatrix ;

    graph.viaMatrix.clear() ; 
    graph.viaMatrix.resize(die1ViaNodeMatrix.size()) ;


    for(int i=0; i<die1ViaNodeMatrix.size(); ++i){
        for(auto& vias : {die1ViaNodeMatrix[i], die2ViaNodeMatrix[i]}){
            for(auto& via : vias)  graph.viaMatrix[i].push_back(via) ;
        }
    }

}


void Router::create_node_and_edges(const ConnectionMap& combinedConnections, RoutingInfo& routingInfo, int k) {
    
    ELAPSE_TIME(create_vias_and_v2v_edges(combinedConnections, routingInfo) ) ;

    ELAPSE_TIME(create_viaMatrix(routingInfo) ) ;
    ELAPSE_TIME(create_edges_and_via_crossing(routingInfo)) ; 
    
    ELAPSE_TIME(create_positions_and_viaExtendedPositions_and_edgeExtendedPositions(routingInfo)) ; 

    ELAPSE_TIME(create_tiles_and_tile_surround_vias(routingInfo, k)) ; // 重點優化部分

    ELAPSE_TIME(create_t2t_edges_and_tile_crossing_edges(routingInfo)) ; 

    ELAPSE_TIME(create_tileSurroundEdges_and_tileSurroundPositions(routingInfo)) ; 

    ELAPSE_TIME(create_e2e_edges(routingInfo)) ;
    ELAPSE_TIME(create_p2p_edges(routingInfo)) ; 
 
    // vector<TileNodePtr> surTiles ;
    // vector<ViaNodePtr> surVias ; 
    // vector<EdgeNodePtr> surEdges ; 
    // vector<PositionNodePtr> surPoss ; 
    
    // for(auto& tile : routingInfo.graph.tiles){
    //     if( routingInfo.graph.tileSurroundVias.at(tile).size()>=4){
    //         for(auto& via : routingInfo.graph.tileSurroundVias.at(tile)) surVias.push_back(via) ;
    //         for(auto& edge : routingInfo.graph.tileSurroundEdges.at(tile)) surEdges.push_back(edge) ;
    //         for(auto& pos : routingInfo.graph.tileSurroundPositions.at(tile)) surPoss.push_back(pos) ;
    //         surTiles.push_back(tile) ; 
    //     } 
    // }
    // cout << surEdges.size() << " " << surPoss.size() << "\n" ;

    // routingInfo.debugBumpMapping["Sur_Poss"] = routing_node_ptrs_to_bumps(surPoss) ;
    // routingInfo.debugBumpMapping["Sur_Edges"] = routing_node_ptrs_to_bumps(surEdges) ;
    // routingInfo.debugBumpMapping["Sur_Vias"] = routing_node_ptrs_to_bumps(surVias) ;
    // routingInfo.debugBumpMapping["Sur_Tiles"] = routing_node_ptrs_to_bumps(surTiles) ;

    
    
    routingInfo.debugEdgeMapping["DB_T2T_Edges"] = routing_edge_ptrs_to_edge(routingInfo.graph.t2tEdges, "#FF00FF") ;
    routingInfo.debugEdgeMapping["DB_E2E_Edges"] = routing_edge_ptrs_to_edge(routingInfo.graph.e2eEdges, "#00FFFF") ;
    // routingInfo.debugEdgeMapping["DB_P2P_Edges"] = routing_edge_ptrs_to_edge(routingInfo.graph.p2pEdges, "#FFFF00") ;

    routingInfo.debugBumpMapping["DB_Vias"] = routing_node_ptrs_to_bumps(routingInfo.graph.vias) ;
    routingInfo.debugBumpMapping["DB_Edges"] = routing_node_ptrs_to_bumps(routingInfo.graph.edges) ;
    routingInfo.debugBumpMapping["DB_Tiles"] = routing_node_ptrs_to_bumps(routingInfo.graph.tiles) ;
    // routingInfo.debugBumpMapping["DB_Positions"] = routing_node_ptrs_to_bumps(routingInfo.graph.positions) ;
}

void Router::set_edge_type(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 
    for(auto& edge : graph.edges){
        ViaToViaEdgePtr v2vEdge = *graph.viaCrossingEdges.at(edge).begin() ;
        ViaNodePtr via1 = v2vEdge->node1 ; 
        ViaNodePtr via2 = v2vEdge->node2 ; 

        if(fabs(via1->y() - via2->y())<GlobalEpsilonY) edge->edgeType = BaseEdge ;
        else edge->edgeType = LegEdge ;

    }
}

void Router::set_capacity(RoutingInfo& routingInfo) {
    RoutingGraph& graph = routingInfo.graph ; 

    for(auto& v2vEdge : graph.v2vEdges){
        EdgeNodePtr edge = *graph.viaCrossingEdges.at(v2vEdge).begin() ;
        if(!graph.tileCrossingEdges.contains(edge)) continue ; 

        TileToTileEdgePtr t2tEdge = *graph.tileCrossingEdges.at(edge).begin() ;
        ViaNodePtr via1 = v2vEdge->node1 ; 
        ViaNodePtr via2 = v2vEdge->node2 ; 
        
        double viaDistance = distance_sc(*via1, *via2) ;
        int capacity ;

        if(via1->type!=DUMMY) viaDistance -= (designRule.viaPadRadius + designRule.minimumLineViaPadSpacing + designRule.minimumLineWidth/2) ; //Design Dependent
        if(via2->type!=DUMMY) viaDistance -= (designRule.viaPadRadius + designRule.minimumLineViaPadSpacing + designRule.minimumLineWidth/2) ; //Design Dependent
        
        if(static_cast<DBI_Key&>(*via1)==static_cast<DBI_Key&>(*via2)) viaDistance = 0 ; // 同一組Offset Via
        else if(edge->die==DIE12) viaDistance = 0 ; // 逃線區塊不能上下走

        // viaDistance = max(viaDistance, 0.0) ;
        capacity = viaDistance / (designRule.minimumLineSpacing + designRule.minimumLineWidth) ; //Design Dependent
        capacity = min(capacity, int(graph.edgeExtendedPositions.at(edge).size())) ; //Design Dependent
        // capacity 包含signal + Vss Nets
        t2tEdge->capacity = (capacity-1)/2 ;
    }

    

}
void Router::construct_routing_graph(RoutingInfo& routingInfo){
    RoutingGraph die1Graph, die2Graph ; //die1Graph for left part, die2Graph for right part
    vector<Bump> die1Bumps, die2Bumps ; 

    ConnectionMap die1Connections, die2Connections, combinedConnections; 

    box_xy die1Boundary = routingInfo.die1Boundary ; 
    box_xy die2Boundary = routingInfo.die2Boundary ; 
    
    // 假設boundary的中間可以完整切分Die1 & Die2
    for(auto& dieType : {DIE1, DIE2}){
        vector<Bump>& dieBumps = (dieType==DIE1) ? die1Bumps : die2Bumps ; 
        ConnectionMap& dieConnections = (dieType==DIE1) ? die1Connections : die2Connections ; 

        copy_if(routingInfo.routingBumps.begin(), routingInfo.routingBumps.end(), back_inserter(dieBumps), [&dieType](const Bump& bump){return bump.die==dieType; }) ;
        for(auto& offsetVia : routingInfo.offsetVias){
            if(offsetVia.die==dieType){
                dieBumps.push_back(offsetVia.first) ; 
                dieBumps.push_back(offsetVia.second) ; 
            }
        }
        triangulation(routingInfo, dieBumps, dieConnections) ;
    }

    combine_connections(die1Connections, die2Connections, combinedConnections) ; 
    create_node_and_edges(combinedConnections, routingInfo, 4) ; // 最容許到4邊形的tile
    set_capacity(routingInfo) ;
    set_edge_type(routingInfo) ;

    for(auto& t2tEdge : routingInfo.graph.t2tEdges){
        auto edge = *routingInfo.graph.tileCrossingEdges.at(t2tEdge).begin() ; 
        routingInfo.debugLabelMapping["DB_Capacity"].push_back( Label(to_string(t2tEdge->capacity), point_xy{edge->x(), edge->y()}) ) ;
    }
    // cout << routingInfo.debugLabelMapping["DB_Capacity"].size() << "\n" ;
    
    routingInfo.gridEdges = routing_edge_ptrs_to_edge(routingInfo.graph.v2vEdges, "#FFFFFF") ; 
}

void Router::combine_connections(const ConnectionMap& connections1, const ConnectionMap& connections2, ConnectionMap& combinedConnections) {
    combinedConnections.clear() ; 

    vector<Bump> die1Bumps, die2Bumps ; 
    
    vector<vector<Bump>> die1Matrix, die2Matrix ;
    
    for(auto& dieType : {DIE1, DIE2}){
        vector<Bump>& dieBump = (dieType==DIE1) ? die1Bumps : die2Bumps ; 
        const ConnectionMap& connection = (dieType==DIE1) ? connections1 : connections2 ; 
        vector<vector<Bump>>& dieMatrix = (dieType==DIE1) ? die1Matrix : die2Matrix ; 

        for(auto& [bump, _] : connection) dieBump.push_back(bump) ; 
        matrixlize(dieBump, dieMatrix, GlobalEpsilonY) ;


    }    
    // if(die1Matrix.size()!=die2Matrix.size()) {
    //     throw runtime_error("Invaild # of via node rows on 2 dies (" + to_string(die1Matrix.size()) + "," + to_string(die2Matrix.size()) + ")\n") ; 
    // }

    combinedConnections = connections1 ; 
    for(auto& [bump, connectedBumps] : connections2) combinedConnections[bump] = connectedBumps ;

    for(int i=0; i<die1Matrix.size(); ++i){
        combinedConnections[ die1Matrix[i].back() ].insert(die2Matrix[i].front()) ; 
        combinedConnections[ die2Matrix[i].front() ].insert(die1Matrix[i].back()) ; 
    }
}

void Router::triangulation(const RoutingInfo& routingInfo, const vector<Bump>& bumps, ConnectionMap& connections) {
    CDT cdt;
    vector<vector<Bump>> bumpMatrix ; 
    matrixlize(bumps, bumpMatrix, GlobalEpsilonY) ;
    
    vector<CGAL_Point2> allPoints;
    vector<vector<CGAL_Point2>> y_groups; // === Y 分組（允許誤差）===
    map<CGAL_Point2, Bump> point_to_bump ;
    
    for (auto& bump : bumps) {
        connections[bump] = set<Bump>() ; 
        CGAL_Point2 pt(bump.x(), bump.y());
        if(point_to_bump.contains(pt)) throw runtime_error("Same point!!!") ;
        point_to_bump[pt] = bump ;
    }

    for(int i=0; i<bumpMatrix.size()-1; ++i){
        CGAL_Point2 point1(bumpMatrix[i].front().x(), bumpMatrix[i].front().y());
        CGAL_Point2 point2(bumpMatrix[i+1].front().x(), bumpMatrix[i+1].front().y());
        cdt.insert_constraint(point1, point2);
    }

    for(int i=0; i<bumpMatrix.size()-1; ++i){
        CGAL_Point2 point1(bumpMatrix[i].back().x(), bumpMatrix[i].back().y());
        CGAL_Point2 point2(bumpMatrix[i+1].back().x(), bumpMatrix[i+1].back().y());
        cdt.insert_constraint(point1, point2);
    }

    for(int i=0; i<bumpMatrix.size(); ++i){
        for(int j=0; j<bumpMatrix[i].size()-1; ++j){
            CGAL_Point2 point1(bumpMatrix[i][j].x(), bumpMatrix[i][j].y());
            CGAL_Point2 point2(bumpMatrix[i][j+1].x(), bumpMatrix[i][j+1].y());

            cdt.insert_constraint(point1, point2);
        }
    }

    for(auto& bump : bumps){
        CGAL_Point2 point(bump.x(), bump.y());
        cdt.insert(point);
    }

    connections.clear() ; 
    for (auto edge = cdt.finite_edges_begin(); edge != cdt.finite_edges_end(); ++edge) {
        auto segment = cdt.segment(*edge);
        point_xy source(segment.source().x(), segment.source().y()) ;
        point_xy target(segment.target().x(), segment.target().y()) ;

        // if(distance_sc(source, target)<routingInfo.bumpDistance*1.6){ // Design Dependent
        // cout << point_to_bump[segment.source()] << " " << point_to_bump[segment.target()] << "\n" ;
        connections[point_to_bump[segment.source()]].insert(point_to_bump[segment.target()]) ; 
        connections[point_to_bump[segment.target()]].insert(point_to_bump[segment.source()]) ; 

    }
}

void Router::step2_check(const RoutingInfo& routingInfo) const {
    const RoutingGraph& graph = routingInfo.graph ; 

    if( !routingInfo.bump2ViaNodeMapping.size() ||
        !graph.vias.size() ||
        !graph.v2vEdges.size() ){
            throw runtime_error("[Step2 Check] Incomplete construction.") ; 
    }

    for(auto& bump : routingInfo.routingBumps){
        if(!routingInfo.bump2ViaNodeMapping.contains(bump)){
            throw runtime_error("[Step2 Check] Can't find exist bump in bump2ViaNodeMapping.") ; 
        }
    }

    for(auto& offsetVia : routingInfo.offsetVias){
        if(!routingInfo.bump2ViaNodeMapping.contains(offsetVia.first) || !routingInfo.bump2ViaNodeMapping.contains(offsetVia.second)){
            throw runtime_error("[Step2 Check] Can't find exist offset via in bump2ViaNodeMapping.") ; 
        }
    }

    for(auto& [bump, viaNodePtr] : routingInfo.bump2ViaNodeMapping){
        if(!graph.vias.contains(viaNodePtr)){
            throw runtime_error("[Step2 Check] Can't find exist via node via in vias.") ; 
        }
    }

    for(auto& v2vEdge : graph.v2vEdges){
        if(!graph.vias.contains(v2vEdge->node1) || !graph.vias.contains(v2vEdge->node2)){
            throw runtime_error("[Step2 Check] Can't find exist via node via in v2vEdges.") ; 
        }
    }
}

void Router::rouding_bumps(RoutingInfo& routingInfo){
    for(auto& bump : routingInfo.routingBumps){
        bump.set<0>(round(bump.x()*GlobalResolution)/GlobalResolution) ; 
        bump.set<1>(round(bump.y()*GlobalResolution)/GlobalResolution) ; 
    }

    for(auto& bumps : routingInfo.offsetVias){
        bumps.first.set<0>(round(bumps.first.x()*GlobalResolution)/GlobalResolution) ; 
        bumps.first.set<1>(round(bumps.first.y()*GlobalResolution)/GlobalResolution) ; 
        bumps.second.set<0>(round(bumps.second.x()*GlobalResolution)/GlobalResolution) ; 
        bumps.second.set<1>(round(bumps.second.y()*GlobalResolution)/GlobalResolution) ; 
    }
}
template<typename T>
void write_result(const string& directory, const string& mainFileName, const string& extension, const vector<T>& elements) {
    ofstream outputFile ; 
    if(elements.size()){
        outputFile.open(directory + "/" + mainFileName + "." + extension) ;
        for(auto& element : elements) outputFile << to_formatted_string(element) << "\n" ;
        outputFile.close() ; 
    }
}

void write_result(const string& directory, const string& mainFileName, const string& extension, const box_xy& boundary, const vector<Bump>& elements) {
    ofstream outputFile ; 
    if(elements.size()){
        outputFile.open(directory + "/" + mainFileName + "." + extension) ;
        outputFile << boundary.min_corner().x() << " " << boundary.min_corner().y() << "\n" ;
        outputFile << boundary.max_corner().x() << " " << boundary.max_corner().y() << "\n" ;
        write_result(directory, mainFileName, extension, elements) ;
        outputFile.close() ; 
    }
}

void Router::generate_routing_result(const RoutingInfo& routingInfo, const string& directory) {
    string foolProofDirectory = directory + "/" ; 
    if(!filesystem::exists(foolProofDirectory)) filesystem::create_directories(foolProofDirectory) ;

    for(auto& [key, debugLabels] : routingInfo.debugLabelMapping){
        write_result(directory, key, "label", debugLabels) ;
    }
    for(auto& [key, debugPoints] : routingInfo.debugPointMapping){
        write_result(directory, key, "point", debugPoints) ;
    }
    for(auto& [key, debugEdges] : routingInfo.debugEdgeMapping){
        write_result(directory, key, "edge", debugEdges) ;
    }
    for(auto& [key, debugBumps] : routingInfo.debugBumpMapping){
        write_result(directory, key, "bump", debugBumps) ;
    }

    write_result(directory, "Grid", "edge", routingInfo.gridEdges) ;

    write_result(directory, "Layer", "bump", routingInfo.routingBumps) ;
    write_result(directory, "Layer", "offset_via", routingInfo.offsetVias) ;
    write_result(directory, "Layer", "net", routingInfo.designNets) ;

    // for(auto& edge : routingInfo.graph.positions) outputFile << to_formatted_string(static_cast<Bump&>(*edge)) << "\n" ;

    // outputFile.open(resultFileName + ".net") ;
    // outputFile.close() ; 

    // outputFile.open(resultFileName + ".teardrop") ;
    // outputFile.close() ; 
}

bool Bump::operator<(const Bump& rhs) const { // 為了讓Bump可以直接比較, 為了保留offsetVia需要考慮isOffsetVia
    return  tie(die, type, id, isOffsetVia) < tie(rhs.die, rhs.type, rhs.id, rhs.isOffsetVia) ;
} ;

ostream& operator<<(ostream& os, const CostStruct& costStruct) {
    os  << costStruct.wireLength << " " << costStruct.excessiveCapacity << " " 
        << costStruct.conflictCount << " " << costStruct.differentialLength ;
    return os ; 
}