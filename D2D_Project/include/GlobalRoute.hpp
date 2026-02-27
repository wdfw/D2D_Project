#pragma once
#include <array>
#include "Router.hpp"

using namespace std ; 

struct Channel ;
struct ChannelPath ;
struct ChannelCode ; 
struct ChannelRepresentation ;
struct Chromosome ;

ostream& operator<<(ostream& os, const Channel& channel) ; 
ostream& operator<<(ostream& os, const ChannelPath& path) ; 
ostream& operator<<(ostream& os, const ChannelCode& code) ; 
ostream& operator<<(ostream& os, const ChannelRepresentation& representation) ; 
ostream& operator<<(ostream& os, const Chromosome& chromosome) ; 

enum ChannelDirection {
    UP = 0,
    DOWN = 1,
    LEFT = 2, 
    RIGHT = 3,
} ;

struct DirectionalT2TEdge {
    int ny ;
    int nx ; 
    TileToTileEdgePtr t2tEdge ; 
} ;

struct Channel { // 以二維表取代鏈節表
    vector<vector<TileNodePtr>> channelTileMatrix ; // 當前點的TileNode
    vector<vector< array<DirectionalT2TEdge, 4> >> channelT2TEdgeMaze ; // 該點往4種方向走所經過的TileToTileEdgePtr以及到達的新座標
};

struct ChannelPath : public vector<TileToTileEdgePtr> {
    int channelRow ; 
    pair<int, int> topChannelIndexes ; // [from, to)
    pair<int, int> bottomChannelIndexes ; // [from, to)
    ChannelPath(const pair<int, int>& topChannelIndexes={-1, -1}, const pair<int, int>& bottomChannelIndexes={-1, -1}, int channelRow=-1) : 
                topChannelIndexes(topChannelIndexes), bottomChannelIndexes(bottomChannelIndexes), channelRow(channelRow) {}
};

struct ChannelCode : public vector<int> {
    
    int startIndex ; // [from, start from startIndex-th crossable tile
    int endIndex ; // to), end before endIndex-th crossable tile
    
    ChannelCode(int startIndex=-1, int endIndex=-1) : startIndex(startIndex), endIndex(endIndex) {resize(endIndex-startIndex);}
    bool operator<(const ChannelCode& genotype) const ;
    bool operator>(const ChannelCode& genotype) const ;
};

struct ChannelRepresentation { // 表示一條線路的連接
    void synchronize_code(const Channel& channel) ; // code -> path
    void synchronize_path(const Channel& channel) ; // path -> code
    void randomize_code(const Channel& channel) ;
    void randomize_code() ;
    void legalize_code() ; 

    ViaNodePtr startVia ;
    ViaNodePtr targetVia ; 
    ChannelPath path ;
    ChannelCode code ;
};


struct Chromosome : public vector<ChannelRepresentation> { // 表示多組的線路連接
    using vector<ChannelRepresentation>::vector ;
    // void to_global_net(vector<GlobalNet>& nets) ;
    // bool valid = false ;
    double fitness = -1.0 ; // -1.0 代表尚未被計算過
};


class GlobalRoute : public Router {
protected:

    using ChromosomePtr = shared_ptr<Chromosome> ;
    double global_route(RoutingInfo& routingInfo, CostStruct& costResult) override ; 
        void construct_routing_channel(RoutingInfo& routingInfo, Channel& channel) ;
        void connect_routing_channel(RoutingInfo& routingInfo, Channel& channel) ;
        void generate_refenced_chromosome(RoutingInfo& routingInfo, Channel& channel, Chromosome& chromosome) ;

        double caclute_cost(const Channel& channel, Chromosome& chromosome, CostStruct& costResult) ; 
            double caclute_wire_length(const Chromosome& chromosome) ;
            int caclute_conflict_count(const Chromosome& chromosome) ; 
            int caclute_excessive_capacity(const Chromosome& chromosome) ; 

        void genetic_algorithm_process(const Channel& channel, const Chromosome& referencedChromosome,  Chromosome& bestChromosome) ;
            void initialize_population(const Channel& channel, const Chromosome& referencedChromosome, vector<ChromosomePtr>& population) ;
            void select_parents(const vector<ChromosomePtr>& population, vector<ChromosomePtr>& parents) ;
            void crossover(const vector<ChromosomePtr>& parents, vector<ChromosomePtr>& offsprings) ;
            void mutation(vector<ChromosomePtr>& offsprings) ; 
            void select_surviors(const vector<ChromosomePtr>& population, const vector<ChromosomePtr>& mutatedOffsprings, vector<ChromosomePtr>& survivors) ;
            void evaluate(const Channel& channel, vector<ChromosomePtr>& population) ; 
            pair<double, double> statistically_evaluate(const Channel& channel, vector<ChromosomePtr>& population, CostStruct& avgCostResult, CostStruct& stdCostResult) ;

            void update_and_legalize(const Channel& channel, vector<ChromosomePtr>& population) ; 

        void add_shielding(Chromosome& chromosome) ;
        void sort_routing_order(Chromosome& chromosome) ; // 將Chromosome內的ChannelRepresentation由上到下排序
        void generate_global_nets(const RoutingInfo& routingInfo, const Chromosome& chromosome, vector<ChannelGlobalNet>& globalNets) ; // 根據ChannelRepresentation的上下順序產生global nets 

public:
    using Router::Router ; 
} ;