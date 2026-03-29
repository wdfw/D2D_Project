#pragma once
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <utility>
#include <boost/container/flat_set.hpp>

#include "Geometry.hpp"
#include "DesignStructure.hpp"
#include "Configuration.hpp"
#include "DataStructure.hpp"

using namespace std ; 
using boost::container::flat_set ; 
using boost::container::flat_map ; 

enum EdgeType {
    BaseEdge, // 可上下走訪的邊
    LegEdge // 可左右走訪的邊
} ; 

class RoutingNode ;
template <typename T> class RoutingEdge ;

template<typename T> vector<Bump> routing_node_ptrs_to_bumps(T& nodePtrs) ;
template<typename T> vector<Edge> routing_edge_ptrs_to_edge(T& edgePtrs) ;

class ViaNode ; 
class TileNode ; 
class EdgeNode ; 
class PositionNode ; 

class ViaToViaEdge ;
class TileToTileEdge ;
class EdgeToEdgeEdge ;
class PositionToPositionEdge ;

class ChannelGlobalNet ; 
struct ChannelDetailedNet ;
struct DetailedNet ;

class RoutingGraph ; 

struct RoutingInfo ;

using BumpPtr = flat_shared_ptr<Bump> ; 
using ViaNodePtr = flat_shared_ptr<ViaNode> ; 
using TileNodePtr = flat_shared_ptr<TileNode> ; 
using EdgeNodePtr = flat_shared_ptr<EdgeNode> ; 
using PositionNodePtr = flat_shared_ptr<PositionNode> ; 

using ViaToViaEdgePtr = flat_shared_ptr<ViaToViaEdge> ; 
using TileToTileEdgePtr = flat_shared_ptr<TileToTileEdge> ; 
using EdgeToEdgeEdgePtr = flat_shared_ptr<EdgeToEdgeEdge> ; 
using PositionToPositionEdgePtr = flat_shared_ptr<PositionToPositionEdge> ; 

class RoutingNode : public Bump {
public:
    int k_id ; // ID for flat_map, and non-seen in final design
    RoutingNode(DieType die, BumpType type, int id, int k_id = -1, const point_xy& point = {0,0}) : Bump(die, type, id, point), k_id(k_id) {}
    bool operator<(const RoutingNode& node) const { return k_id < node.k_id; }
    bool operator>(const RoutingNode& node) const { return k_id > node.k_id; }
    bool operator>=(const RoutingNode& node) const { return k_id >= node.k_id; }
    bool operator<=(const RoutingNode& node) const { return k_id <= node.k_id; }
    bool operator==(const RoutingNode& node) const { return k_id == node.k_id; }
    bool operator!=(const RoutingNode& node) const { return k_id != node.k_id; }
} ; 

class ViaNode : public RoutingNode {
public:
    ViaNode(DieType die, BumpType type, int id, int k_id = -1, const point_xy& point = {0,0}) : RoutingNode(die, type, id, k_id, point) {}
};

class TileNode : public RoutingNode {
public:
    TileNode(DieType die, BumpType type, int id, int k_id = -1, const point_xy& point = {0,0}) : RoutingNode(die, type, id, k_id, point) {}
};

class EdgeNode : public RoutingNode {
public:
    EdgeType edgeType ; 
    EdgeNode(DieType die, BumpType type, int id, int k_id = -1, const point_xy& point = {0,0}, EdgeType edgetype = BaseEdge) 
        : RoutingNode(die, type, id, k_id, point), edgeType(edgeType) {}
};

class PositionNode : public RoutingNode { // 跟隨EdgeNode的資訊, 每個EdgeNode可以生成與其 容量 相同的PositionNodes, 每個PositionNodes只能被一種線路佔據
public:
    BumpType occupyType = DUMMY ; // 佔據的線路
    int occupyCount = 0 ; // 佔據的線段數 
    int coverCount = 0 ; // Buffer後包圍的數量


    bool isViaNode ;
    PositionNode(DieType die, BumpType type, int id, int k_id = -1, const point_xy& point = {0,0}, BumpType occupyType = DUMMY, bool isViaNode = false) 
        : RoutingNode(die, type, id, k_id, point), isViaNode(isViaNode) {
            if(occupyType!=DUMMY){
                add_occupy(occupyType) ;
            }
        }
    
    void clear() ; 
    bool is_compitable(BumpType& netType) ;

    void add_occupy(BumpType newOccupyType) ;
    void reduce_occupy() ;

    void add_cover() ;
    void reduce_cover() ;

};

template <typename T> // 無向關係, node1ㄧ定<=node2
class RoutingEdge : public pair<T, T> {
public:
    int k_id ; 
    T node1, node2 ; 

    RoutingEdge(const T& n1, const T& n2, int k_id) : k_id(k_id) {
        if(n1 < n2){
            node1 = n1 ; 
            node2 = n2 ; 
        }else{
            node1 = n2 ; 
            node2 = n1 ; 
        }
    }

    bool contain(const T& node) const {
        return node1==node || node2==node ; 
    }

    T& connected_node(const T& node) {
        if(node==node1) return node2 ;
        else if(node==node2) return node1 ;
        throw runtime_error("Node doesn't on the edge.") ; 
    }

    bool operator<(const RoutingEdge& routingEdge) const {
        return k_id < routingEdge.k_id ; 
    }
} ;

class ViaToViaEdge : public RoutingEdge<ViaNodePtr> { 
public: 
    // EdgeNodePtr intermediateEdgeNode ; 
    ViaToViaEdge(ViaNodePtr node1, ViaNodePtr node2, int k_id) : RoutingEdge(node1, node2, k_id) {}
} ; 

class TileToTileEdge : public RoutingEdge<TileNodePtr> { 
public:
    // EdgeNodePtr intermediateEdgeNode ; 
    int capacity ;  
    int currentCapacity ;  
    TileToTileEdge(TileNodePtr node1, TileNodePtr node2, int k_id, int capacity) 
            : RoutingEdge(node1, node2, k_id), capacity(capacity), currentCapacity(0) {}
} ;

class EdgeToEdgeEdge : public RoutingEdge<EdgeNodePtr> {
public:
    // flat_shared_ptr<ViaToViaEdge> crossingViaEdge ; 
    // flat_shared_ptr<TileToTileEdge> crossingTileEdge ; 
    // vector<PositionNodePtr> includedPositionNodes ; 
    EdgeToEdgeEdge(EdgeNodePtr node1, EdgeNodePtr node2, int k_id) : RoutingEdge(node1, node2, k_id) {}
} ;

class PositionToPositionEdge : public RoutingEdge<PositionNodePtr> {
public:
    // EdgeNodePtr intermediateEdgeNode ; 
    PositionToPositionEdge(PositionNodePtr node1, PositionNodePtr node2, int k_id) : RoutingEdge(node1, node2, k_id) {}
} ;

class ChannelGlobalNet : public vector<EdgeNodePtr> { // 描述2-pin net的 由左至右的連接關係
public:
    ViaNodePtr startVia  ; 
    ViaNodePtr targetVia ;

    ChannelGlobalNet(ViaNodePtr startVia = nullptr, ViaNodePtr targetVia = nullptr, const vector<EdgeNodePtr>& seq = {}) 
        : startVia(startVia), targetVia(targetVia), vector(seq) {}; 
} ; 

struct DetailedNet : public BI_Key, public map<PositionNodePtr, set<PositionNodePtr>>{
    DetailedNet(BumpType type=DUMMY, int id=-1) : BI_Key(type, id) {} ;

    Net to_design_net() ; 
    void to_segements(vector<pair<PositionNodePtr, PositionNodePtr>>& segements) ; 
    void get_end_points(vector<PositionNodePtr>& endPoints) ;
    void combine(DetailedNet& detailedNet) ; 
    void add_connection(PositionNodePtr& p1, PositionNodePtr& p2) ; 
} ;

struct ChannelDetailedNet : public vector<PositionNodePtr> {
    ViaNodePtr startVia  ; 
    ViaNodePtr targetVia ;

    ChannelDetailedNet(ViaNodePtr startVia = nullptr, ViaNodePtr targetVia = nullptr, const vector<PositionNodePtr>& seq = {}) 
        : startVia(startVia), targetVia(targetVia), vector(seq) {}; 

    DetailedNet to_detailed_net(RoutingGraph& graph) ;
} ;

class RoutingGraph {
public:
    flat_set<ViaNodePtr> vias ;
    flat_set<TileNodePtr> tiles ;
    flat_set<EdgeNodePtr> edges ;
    flat_set<PositionNodePtr> positions ;

    flat_map<ViaNodePtr, flat_map<ViaNodePtr, ViaToViaEdgePtr>> connectedVias ;
    flat_map<TileNodePtr, flat_map<TileNodePtr, TileToTileEdgePtr>> connectedTiles;
    flat_map<EdgeNodePtr, flat_map<EdgeNodePtr, EdgeToEdgeEdgePtr>> connectedEdges ;
    flat_map<PositionNodePtr, flat_map<PositionNodePtr, PositionToPositionEdgePtr>> connectedPositions ;

    flat_set<ViaToViaEdgePtr> v2vEdges ;
    flat_set<TileToTileEdgePtr> t2tEdges ;
    flat_set<EdgeToEdgeEdgePtr> e2eEdges ;
    flat_set<PositionToPositionEdgePtr> p2pEdges ;

    double_linked_flat_relation<ViaToViaEdgePtr, EdgeNodePtr> viaCrossingEdges ; // v2vEdge間的edge node
    double_linked_flat_relation<TileToTileEdgePtr, EdgeNodePtr> tileCrossingEdges ; // t2tEdge間的edge node

    double_linked_flat_relation<EdgeNodePtr, PositionNodePtr> edgeExtendedPositions ; // 在edge上的position
    double_linked_flat_relation<ViaNodePtr, PositionNodePtr> viaExtendedPositions ; // 在edge上的position

    double_linked_flat_relation<TileNodePtr, ViaNodePtr> tileSurroundVias ; 
    double_linked_flat_relation<TileNodePtr, EdgeNodePtr> tileSurroundEdges ; 
    double_linked_flat_relation<TileNodePtr, PositionNodePtr> tileSurroundPositions ; 
    
    vector<vector<ViaNodePtr>> viaMatrix ; 
    vector<vector<ViaNodePtr>> die1ViaMatrix ; 
    vector<vector<ViaNodePtr>> die2ViaMatrix ; 
};

struct RoutingInfo {
    double bumpDistance ; 
    box_xy chipBoundary ; 
    box_xy die1Boundary ; 
    box_xy die2Boundary ; 
    
    map<string, vector<Label>> debugLabelMapping ; 
    map<string, vector<Point>> debugPointMapping ; 
    map<string, vector<Edge>> debugEdgeMapping ; 
    map<string, vector<Bump>> debugBumpMapping ; 
     
    vector<Edge> gridEdges ; 

    vector<Bump> routingBumps ; 
    vector<OffsetVia> offsetVias ; 

    vector<vector<Bump>> bumpMatrix ; 

    map<Bump, ViaNodePtr> bump2ViaNodeMapping ; 
    RoutingGraph graph ; 

    vector<ChannelGlobalNet> globalNets ; 
    vector<DetailedNet> detailedNets ; 
    vector<Net> designNets ; 
    vector<Teardrop> teardrops ;
    
    vector<Net> violatedNets ; 
    map<TileNodePtr, vector<pair<PositionNodePtr, PositionNodePtr>>> netSequence ; 

    // vector<DetailedNet> detailedNet ; 
} ;



struct ChipInfo {
    double bumpDistance ; 
    box_xy chipBoundary ; 
    box_xy die1Boundary ; 
    box_xy die2Boundary ; 
    
    vector<Bump> routingBumps ; 
    vector<OffsetVia> offsetVias ; 

    vector<vector<Bump*>> die1BumpMatrix ;
    vector<vector<Bump*>> die2BumpMatrix ;
    vector<vector<Bump*>> combinedBumpMatrix ;
} ;

struct GraphInfo {
    flat_set<ViaNodePtr> vias ;
    flat_set<TileNodePtr> tiles ;
    flat_set<EdgeNodePtr> edges ;
    flat_set<PositionNodePtr> positions ;

    flat_map<ViaNodePtr, flat_map<ViaNodePtr, ViaToViaEdgePtr>> connectedVias ;
    flat_map<TileNodePtr, flat_map<TileNodePtr, TileToTileEdgePtr>> connectedTiles;
    flat_map<EdgeNodePtr, flat_map<EdgeNodePtr, EdgeToEdgeEdgePtr>> connectedEdges ;
    flat_map<PositionNodePtr, flat_map<PositionNodePtr, PositionToPositionEdgePtr>> connectedPositions ;

    flat_set<ViaToViaEdgePtr> v2vEdges ;
    flat_set<TileToTileEdgePtr> t2tEdges ;
    flat_set<EdgeToEdgeEdgePtr> e2eEdges ;
    flat_set<PositionToPositionEdgePtr> p2pEdges ;

    double_linked_flat_relation<ViaToViaEdgePtr, EdgeNodePtr> viaCrossingEdges ; // v2vEdge間的edge node
    double_linked_flat_relation<TileToTileEdgePtr, EdgeNodePtr> tileCrossingEdges ; // t2tEdge間的edge node

    
    double_linked_flat_relation<EdgeNodePtr, PositionNodePtr> edgeExtendedPositions ; // 在edge上的position
    double_linked_flat_relation<ViaNodePtr, PositionNodePtr> viaExtendedPositions ; // 在edge上的position

    double_linked_flat_relation<TileNodePtr, ViaNodePtr> tileSurroundVias ; 
    double_linked_flat_relation<TileNodePtr, EdgeNodePtr> tileSurroundEdges ; 
    double_linked_flat_relation<TileNodePtr, PositionNodePtr> tileSurroundPositions ; 
    
    // flat_map<TileNode, vector<pair<PositionNode, PositionNode>>> 

    vector<vector<ViaNodePtr>> viaMatrix ; 
    vector<vector<ViaNodePtr>> die1ViaMatrix ; 
    vector<vector<ViaNodePtr>> die2ViaMatrix ; 
} ;



template<typename T>
vector<Bump> routing_node_ptrs_to_bumps(T& nodePtrs){
    vector<Bump> bumps ;
    for(auto& node : nodePtrs){
        bumps.push_back( static_cast<Bump>(*node) ) ;
    }
    return bumps ; 
}


template<typename T> 
vector<Edge> routing_edge_ptrs_to_edge(T& edgePtrs, const string& colorCode = "#FFFFFF"){
    vector<Edge> edges ;
     for(auto& edge : edgePtrs){
        auto node1 = edge->node1 ; 
        auto node2 = edge->node2 ; 
        Edge newEdge(segment_xy({node1->x(),node1->y()}, {node2->x(),node2->y()}), 0.5, colorCode) ; 
        edges.push_back(newEdge) ; 
    }
    return edges ; 
}
