#include "GlobalRoute.hpp"
bool DebugFlag = false ; 
ostream& operator<<(ostream& os, const ChannelPath& path) {
    for(auto& t2tEdge : path){
        os << "(" << *t2tEdge->node1 << " | " << *t2tEdge->node2 << ") " ; 
    }
    return os ; 
}

ostream& operator<<(ostream& os, const ChannelCode& code) {
    for(auto& c : code){
        os << c << " " ;
    }
    return os ; 
}

ostream& operator<<(ostream& os, const ChannelRepresentation& representation) {
    os << *representation.startVia << " " << *representation.targetVia << ":\n" ;
    os << representation.path << "\n" ;
    os << representation.code << "\n" ;
    return os ;
}

ostream& operator<<(ostream& os, const Chromosome& chromosome) {
    for(auto& representation : chromosome){
        os << representation << "\n" ;
    }
    return os ;
}

bool ChannelCode::operator<(const ChannelCode& code2) const {
    return code2 > *this ;
}

bool ChannelCode::operator>(const ChannelCode& code2) const {
    const ChannelCode& code1 = *this ;
    if(code1.endIndex<code2.startIndex || code2.endIndex<code1.startIndex) return false ; 

    int conflictCount = 0 ;
    int s = min(code1.startIndex, code2.startIndex) ;
    int t = max(code1.endIndex, code2.endIndex) ;
    
    int polarity = 0 ; // 0無向性, 1表示code1比code2上方, -1表示code1比code2下方
    int code1Bit = 0, code2Bit = 0; 
    for(int k=s; k<t; ++k){
        if(k>=code1.startIndex && k>=code2.startIndex && k<code1.endIndex && k<code2.endIndex){
            code1Bit = code1[k-code1.startIndex] ; code2Bit = code2[k-code2.startIndex] ; 
        }else{
            if(k==code1.startIndex-1 && k>=code2.startIndex){
                code1Bit = code1[0]/2 ; code2Bit = code2[k-code2.startIndex] ; 
            }else if(k>=code1.startIndex && k==code2.startIndex-1){
                code1Bit = code1[k-code1.startIndex] ; code2Bit = code2[0]/2 ;
            }

            if(k==code1.endIndex && k<code2.endIndex){
                code1Bit = code1.back()/2 ; code2Bit = code2[k-code2.startIndex] ; 
            }else if(k<code1.endIndex && k==code2.endIndex){
                code1Bit = code1[k-code1.startIndex] ; code2Bit = code2.back()/2 ;
            }
        }

        if(code1Bit>code2Bit){
            if(polarity==-1) ++ conflictCount ;
            polarity = 1 ;
        }else if(code1Bit<code2Bit){
            if(polarity==1) ++ conflictCount ;
            polarity = -1 ;
        }else if(code1Bit==code2Bit){
            polarity = polarity ;
        }
    }
    
    return conflictCount==0 && polarity==1 ; 
}

// void ChannelRepresentation::synchronize_code(const Channel& channel) ; // code -> path
void ChannelRepresentation::synchronize_path(const Channel& channel) {

    const int channelRow = path.channelRow ; // the path will go on row [channelRow, channelRow+1] 
    const pair<int, int>& topChannelIndexes = path.topChannelIndexes ; 
    const pair<int, int>& bottomChannelIndexes = path.bottomChannelIndexes ; 

    int currentCode = code[0] ; 
    int cx = (currentCode==2) ? topChannelIndexes.first : bottomChannelIndexes.first, cy = (currentCode==2) ? channelRow : channelRow+1 ;
    int nx, ny ; 

    path.clear() ;

    for(int i=0; i<code.size()-1; ++i){
        // cout <<"\t" << cx << " " << cy << " " << channel.channelT2TEdgeMaze.size() << " " << channel.channelT2TEdgeMaze[0].size() << "\n" ; 
        
        do {
            path.push_back(channel.channelT2TEdgeMaze[cy][cx][RIGHT].t2tEdge) ; 
            nx = channel.channelT2TEdgeMaze[cy][cx][RIGHT].nx ; ny = channel.channelT2TEdgeMaze[cy][cx][RIGHT].ny ;
            cx = nx ; cy = ny ; 
            // cout <<"\t" << cx << " " << cy << " " << channel.channelT2TEdgeMaze.size() << " " << channel.channelT2TEdgeMaze[0].size() << "\n" ; 

        }while(
            (currentCode==-2 && !channel.channelT2TEdgeMaze[cy][cx][UP].t2tEdge) || 
            (currentCode==2 && !channel.channelT2TEdgeMaze[cy][cx][DOWN].t2tEdge) ) ;     
        
        if(currentCode==-2 && code[i+1]==2){
            path.push_back(channel.channelT2TEdgeMaze[cy][cx][UP].t2tEdge) ; 
            nx = channel.channelT2TEdgeMaze[cy][cx][UP].nx ; ny = channel.channelT2TEdgeMaze[cy][cx][UP].ny ; 
        }else if(currentCode==2 && code[i+1]==-2){
            path.push_back(channel.channelT2TEdgeMaze[cy][cx][DOWN].t2tEdge) ; 
            nx = channel.channelT2TEdgeMaze[cy][cx][DOWN].nx ; ny = channel.channelT2TEdgeMaze[cy][cx][DOWN].ny ; 
        }else{
            nx = cx ; ny = cy ; 
        }

        cx = nx ; cy = ny ; 
        // cout <<"\t" << cx << " " << cy << " " << channel.channelT2TEdgeMaze.size() << " " << channel.channelT2TEdgeMaze[0].size() << "\n" ; 

        currentCode = code[i+1] ; 
    }

}

void ChannelRepresentation::randomize_code(const Channel& channel) {
    randomize_code() ;
    legalize_code() ; 
    synchronize_path(channel) ;
}

void ChannelRepresentation::randomize_code() {
    int p = (GlobalRandomTool.rand(0,1)) ? 2 : -2 ;
    for(auto& v : code) v = p ;
}

void ChannelRepresentation::legalize_code() {
    code.back() = code[code.size()-2] ; 
}

void GlobalRoute::construct_routing_channel(RoutingInfo& routingInfo, Channel& channel) {
    RoutingGraph& graph = routingInfo.graph ;
  
    vector<tuple<double, double>> die1Segements, die2Segements ; 
    vector<TileNodePtr> die1Tiles, die2Tiles, die12Tiles ;
    vector<vector<TileNodePtr>> die1TileMatrix, die2TileMatrix, die12TileMatrix ;
    vector<vector<TileNodePtr>> combinedTileMatix ;
    
    for(auto& tile : graph.tiles){
        if(tile->die==DIE1) die1Tiles.push_back(tile) ;
        else if(tile->die==DIE2) die2Tiles.push_back(tile) ;
        else if(tile->die==DIE12) die12Tiles.push_back(tile) ;
    }

    for(int i=0; i<graph.die1ViaMatrix.size()-1; ++i){
        die1Segements.push_back({graph.die1ViaMatrix[i][0]->y(), graph.die1ViaMatrix[i+1][0]->y()}) ; 
        die2Segements.push_back({graph.die2ViaMatrix[i][0]->y(), graph.die2ViaMatrix[i+1][0]->y()}) ; 
    }

    matrixlize(die1Tiles, die1TileMatrix, die1Segements) ; 
    matrixlize(die2Tiles, die2TileMatrix, die2Segements) ;
    matrixlize(die12Tiles, die12TileMatrix, GlobalEpsilonY) ;

    combinedTileMatix = die1TileMatrix ; 
    for(int i=0; i<die2TileMatrix.size(); ++i){
        combinedTileMatix[i].insert(combinedTileMatix[i].end(), die12TileMatrix[i].begin(), die12TileMatrix[i].end()) ;
        combinedTileMatix[i].insert(combinedTileMatix[i].end(), die2TileMatrix[i].begin(), die2TileMatrix[i].end()) ;
    }

    channel.channelTileMatrix = combinedTileMatix ;
}

void GlobalRoute::connect_routing_channel(RoutingInfo& routingInfo, Channel& channel) {
    RoutingGraph& graph = routingInfo.graph ;

    vector<vector<TileNodePtr>>& tileMatrix = channel.channelTileMatrix ; 

    channel.channelT2TEdgeMaze.clear() ; channel.channelT2TEdgeMaze.resize(tileMatrix.size()) ;

    for(int i=0; i<tileMatrix.size(); ++i){
        channel.channelT2TEdgeMaze[i].resize(tileMatrix[i].size()) ;
    }
    
    // 水平向連結
    for(int i=0; i<tileMatrix.size(); ++i){
        for(int j=0; j<tileMatrix[i].size()-1; ++j){
            TileNodePtr& leftTile = tileMatrix[i][j] ; 
            TileNodePtr& rightTile = tileMatrix[i][j+1] ; 
            TileToTileEdgePtr& t2tEdge = graph.connectedTiles[leftTile][rightTile] ; 
            channel.channelT2TEdgeMaze[i][j][RIGHT] = {i, j+1, t2tEdge} ; 
            channel.channelT2TEdgeMaze[i][j+1][LEFT] = {i, j, t2tEdge} ; 
        }
    }

    // 垂直向連結
    for(int i=0; i<tileMatrix.size()-1; ++i){
        for(int j=0; j<tileMatrix[i].size(); ++j){
            for(int k=0; k<tileMatrix[i+1].size(); ++k){
                TileNodePtr& upperTile = tileMatrix[i][j] ; 
                TileNodePtr& lowerTile = tileMatrix[i+1][k] ; 
                if(graph.connectedTiles[upperTile].contains(lowerTile)){
                    TileToTileEdgePtr& t2tEdge = graph.connectedTiles[upperTile][lowerTile] ; 
                    channel.channelT2TEdgeMaze[i][j][DOWN] = {i+1, k, t2tEdge} ;
                    channel.channelT2TEdgeMaze[i+1][k][UP] = {i, j, t2tEdge} ;
                }
            }
        }
    }
}

void GlobalRoute::generate_refenced_chromosome(RoutingInfo& routingInfo, Channel& channel, Chromosome& chromosome) {
    RoutingGraph& graph = routingInfo.graph ;
    map<ViaNodePtr, ViaNodePtr> routingViaNodePairs ; 

    for(auto& via1 : graph.vias){
        for(auto& via2 : graph.vias){
            if( via1->type==SIGNAL && via1->die==DIE1 && via2->die==DIE2 && via1->type==via2->type && 
                via1->id==via2->id && via1->isRouting && via2->isRouting){
                routingViaNodePairs[via1] = via2 ;
            }
        }   
    }

    vector<vector<TileNodePtr>>& channelTileMatrix = channel.channelTileMatrix ; 
    vector<vector< array<DirectionalT2TEdge, 4> >>& channelT2TEdgeMaze = channel.channelT2TEdgeMaze ; 
    chromosome.clear() ; 

    for(auto& via : graph.vias){
        if(!routingViaNodePairs.contains(via)) continue ; 
        
        auto& via1 = via ; 
        auto& via2 = routingViaNodePairs[via] ; 

        ChannelRepresentation newChannelRepresentation ; 
        newChannelRepresentation.startVia = via1 ; 
        newChannelRepresentation.targetVia = via2 ; 
        int isAtUpperChannel = -1 ; 
        for(int i=0; i<channelTileMatrix.size(); ++i){
            for(int j=0, codeIndex=0; j<channelTileMatrix[i].size() && isAtUpperChannel!=2; ++j){
                auto& tile = channelTileMatrix[i][j] ; 
                if(isAtUpperChannel==-1 || isAtUpperChannel==1){
                    if(via1->id==54){
                        cout << *via2 << " | " << *tile << " | " << codeIndex << "\n" ;
                    }
                    if(graph.tileSurroundVias.at(tile).contains(via1)){
                        if(isAtUpperChannel==-1){
                            newChannelRepresentation.path.channelRow = i ;
                            isAtUpperChannel = 1 ;
                        }

                        newChannelRepresentation.code.startIndex = codeIndex ; 
                        newChannelRepresentation.path.topChannelIndexes.first = j ; 
                    }

                    if(graph.tileSurroundVias.at(tile).contains(via2)){
                        newChannelRepresentation.code.endIndex = codeIndex+1 ; 
                        newChannelRepresentation.path.topChannelIndexes.second = j+1 ; 
                        isAtUpperChannel = 0 ;

                       

                        break ;
                    }

                    if(channelT2TEdgeMaze[i][j][DOWN].t2tEdge) ++codeIndex ;
                }else if(isAtUpperChannel==0){
                    if(graph.tileSurroundVias.at(tile).contains(via1)){
                        newChannelRepresentation.path.bottomChannelIndexes.first = j ; 
                    }

                    if(graph.tileSurroundVias.at(tile).contains(via2)){
                        newChannelRepresentation.path.bottomChannelIndexes.second = j+1 ; 
                        isAtUpperChannel = 2 ;
                        break ;
                    }
                }
            }
        }

        newChannelRepresentation.code.resize(newChannelRepresentation.code.endIndex-newChannelRepresentation.code.startIndex) ; 
        chromosome.push_back(newChannelRepresentation) ; 

        // cout << *via1 << " " << *via2 << " | " << 
        //         newChannelRepresentation.code.startIndex << " " <<
        //         newChannelRepresentation.code.endIndex << " | " <<
        //         newChannelRepresentation.path.topChannelIndexes.first << " " <<
        //         newChannelRepresentation.path.topChannelIndexes.second << " " <<
        //         newChannelRepresentation.path.bottomChannelIndexes.first << " " <<
        //         newChannelRepresentation.path.bottomChannelIndexes.second << " | " <<
        //         newChannelRepresentation.path.channelRow << "\n" ;
    }
    
}

void GlobalRoute::update_and_legalize(const Channel& channel, vector<ChromosomePtr>& population) {
    for(auto& chromosome : population){
        for(auto& representation : *chromosome){
            representation.legalize_code() ;
            representation.synchronize_path(channel) ;
        }
    }
}

void GlobalRoute::initialize_population(const Channel& channel, const Chromosome& referencedChromosome, vector<ChromosomePtr>& population) {
    population.clear() ; 
    for(int i=0; i<GAConfig.populationSize; ++i){
        population.push_back(make_shared<Chromosome>(referencedChromosome)) ;
    }

    for(auto& chromosome : population){
        for(auto& representation : *chromosome){
            representation.randomize_code(channel) ;
        }
    }
}

double GlobalRoute::caclute_wire_length(const Chromosome& chromosome){
    double wirtLength = 0.0 ; 
    for(auto& representation : chromosome){
        for(auto& t2tEdge : representation.path){
            wirtLength += distance_sc(*t2tEdge->node1, *t2tEdge->node2) ;
        }
    }
    return wirtLength ; 
}


int GlobalRoute::caclute_excessive_capacity(const Chromosome& chromosome){
    int excessiveCapacity = 0 ; 

     for(auto& representation : chromosome){
        for(auto& t2tEdge : representation.path){
           ++ t2tEdge->currentCapacity ; 
        //    cout << *t2tEdge->node1 << " " << *t2tEdge->node2 << " | " << t2tEdge->capacity << " ?\n" ;
        }
    }
    // if(DebugFlag){
    //     for(auto& representation : chromosome){
    //         for(auto& t2tEdge : representation.path){
    //             auto noed1 = t2tEdge->node1 ;
    //             auto noed2 = t2tEdge->node2 ;
    //             cout << *noed1 << " " << *noed2 << " | " << t2tEdge->currentCapacity << " " << t2tEdge->capacity << "\n" ;
    //         }
    //     }
    // }

    for(auto& representation : chromosome){
        for(auto& t2tEdge : representation.path){
           if(t2tEdge->currentCapacity > t2tEdge->capacity){
                excessiveCapacity += (t2tEdge->currentCapacity - t2tEdge->capacity) ;
           }
           t2tEdge->currentCapacity = 0 ;
        }
    }

    return excessiveCapacity ; 
}

int GlobalRoute::caclute_conflict_count(const Chromosome& chromosome){
    int conflictCount = 0 ;
    for(int i=0; i<chromosome.size(); ++i){
        for(int j=i+1; j<chromosome.size(); ++j){
            if(chromosome[i].path.channelRow!=chromosome[j].path.channelRow) continue ; 
            const ChannelCode& code1 = chromosome[i].code ;
            const ChannelCode& code2 = chromosome[j].code ;
            if(code1.endIndex<code2.startIndex || code2.endIndex<code1.startIndex) continue ;

            int s = min(code1.startIndex, code2.startIndex) ;
            int t = max(code1.endIndex, code2.endIndex) ;
            
            int polarity = 0 ; // 0無向性, 1表示code1比code2上方, -1表示code1比code2下方
            int code1Bit, code2Bit ; 
            bool cmpFlag ; 
            for(int k=s; k<t; ++k){
                cmpFlag = false ; 
                if(k>=code1.startIndex && k>=code2.startIndex && k<code1.endIndex && k<code2.endIndex){
                    code1Bit = code1[k-code1.startIndex] ; code2Bit = code2[k-code2.startIndex] ; cmpFlag = true ;
                }else{
                    if(k==code1.startIndex-1 && k>=code2.startIndex){
                        code1Bit = code1[0]/2 ; code2Bit = code2[k-code2.startIndex] ; cmpFlag = true ;
                    }else if(k>=code1.startIndex && k==code2.startIndex-1){
                        code1Bit = code1[k-code1.startIndex] ; code2Bit = code2[0]/2 ; cmpFlag = true ;
                    }

                    if(k==code1.endIndex && k<code2.endIndex){
                        code1Bit = code1.back()/2 ; code2Bit = code2[k-code2.startIndex] ; cmpFlag = true ;
                    }else if(k<code1.endIndex && k==code2.endIndex){
                        code1Bit = code1[k-code1.startIndex] ; code2Bit = code2.back()/2 ; cmpFlag = true ;
                    }
                }

                if(code1Bit>code2Bit){
                    if(polarity==-1) ++ conflictCount ;
                     polarity = 1 ;
                }else if(code1Bit<code2Bit){
                    if(polarity==1) ++ conflictCount ;
                    polarity = -1 ;
                }else if(code1Bit==code2Bit){
                    polarity = polarity ;
                }
                // cout << k << " " << code1Bit << " " << code2Bit << " " << polarity << "|" << conflictCount << " | " << cmpFlag << " " << cond  << "\n" ;
            }

            // cout << code1 << "\n" << code2 << "\n" << *chromosome[i].startVia << " " << *chromosome[j].startVia << "\n" ;
            // cout << code1.startIndex << " " << code1.endIndex << " " <<  code2.startIndex << " " << code2.endIndex  << "\n" ;
            // cout << conflictCount << "\n" ;
            // throw logic_error("\n\n") ;
        }
    }
    return conflictCount ; 
}

double GlobalRoute::caclute_cost(const Channel& channel, Chromosome& chromosome, CostStruct& costResult) {
    costResult.wireLength = caclute_wire_length(chromosome) ;
    costResult.excessiveCapacity = caclute_excessive_capacity(chromosome) ;
    costResult.conflictCount = caclute_conflict_count(chromosome) ; 
    costResult.differentialLength = 0 ;

    chromosome.fitness = GAConfig.alpha*costResult.wireLength + GAConfig.beta*costResult.excessiveCapacity + GAConfig.gamma*costResult.conflictCount + GAConfig.delta*costResult.differentialLength ;
    return chromosome.fitness ;
}

void GlobalRoute::evaluate(const Channel& channel, vector<ChromosomePtr>& population) {
    CostStruct foo ;
    for(auto& chromosomePtr : population){
        caclute_cost(channel, *chromosomePtr, foo) ;
    }
}

void GlobalRoute::select_parents(const vector<ChromosomePtr>& population, vector<ChromosomePtr>& parents) {
    vector<ChromosomePtr> candidates = population ;
    parents.clear() ; 

    sort(candidates.begin(), candidates.end(), 
        [](const ChromosomePtr& c1, const ChromosomePtr& c2){return c1->fitness<c2->fitness;}
    ) ; 
    
    vector<double> roll(candidates.size(), 0) ; for(int i=0; i<candidates.size(); ++i) roll[i] = 0.01 * (1.1-(i*(0.2049)/(101))) ; 
    double rr = 0.0 ; for(auto& r : roll) rr += r ; 

    for(int i=0; i<GAConfig.populationSize*2; ++i){
        bool f = false ; 
        double p = rand()%10000000/double(10000000)* rr ; 
        for(int j=0; j<roll.size(); ++j){
            p -= roll[j] ;
            if(p<=0){
                parents.push_back(candidates[j]) ; 
                f = 1 ; 
                break;
            }
        }
        if(!f) parents.push_back(candidates.back()) ; 
    }

    // for(int i=0; parents.size()<GAConfig.populationSize*2; ++i){
    //     candidates.clear() ; 
    //     for(int j=0; j<GAConfig.tournamentSize; j++){
    //         candidates.push_back(population[ GlobalRandomTool.rand(0, population.size()-1) ]) ; 
    //     }

    //     ChromosomePtr& winner = *min_element(candidates.begin(), candidates.end(), [](ChromosomePtr& p1, ChromosomePtr& p2){return p1->fitness < p2->fitness;}) ; 
    //     parents.push_back(winner) ; 
    // }
}

void GlobalRoute::crossover(const vector<ChromosomePtr>& parents, vector<ChromosomePtr>& offsprings) {
    offsprings.clear() ;
    double succ = 0.0, fail = 0.0 ; 
    for(int i=0, p1, p2; offsprings.size()<GAConfig.populationSize; ++i){
        const ChromosomePtr& parent1 = parents[2*i] ;
        const ChromosomePtr& parent2 = parents[2*i+1] ;
        ChromosomePtr offspring1, offspring2 ;

        offspring1 = make_shared<Chromosome>(*parent1) ; offspring2 = make_shared<Chromosome>(*parent2) ; 
 
        for(int j=0; j<offspring1->size(); ++j){
            if( GlobalRandomTool.Bernoulli_trial(GAConfig.crossoverRate) ){
                // int p1 = GlobalRandomTool.rand(0, offspring1->at(j).code.size()) ; p2 = GlobalRandomTool.rand(0, offspring1->at(j).code.size()) ;
                // if(p1 > p2) swap(p1, p2) ;
                // for(int k=p1; k<p2; k++) swap(offspring1->at(j).code[k], offspring2->at(j).code[k]) ;
                swap(offspring1->at(j), offspring2->at(j)) ;
            }
        }

        offsprings.push_back(offspring1) ; offsprings.push_back(offspring2) ;
    }

}

void GlobalRoute::mutation(vector<ChromosomePtr>& offsprings){
    for(auto& offspring : offsprings){
        for(int j=0; j<offspring->size(); ++j){
            if( GlobalRandomTool.Bernoulli_trial(GAConfig.mutationRate) ){
                int p = (GlobalRandomTool.rand(0,1) ? 2 : -2) ;
                int s = GlobalRandomTool.rand(0, offspring->at(j).code.size()-1) ;
                int t = GlobalRandomTool.rand(0, offspring->at(j).code.size()-1) ;
                if(s>t) swap(t,s) ; 
                for(int k=s; k<=t; ++k){
                    offspring->at(j).code[k] = p ; 
                }
            }
        }
    }
}

void GlobalRoute::select_surviors(const vector<ChromosomePtr>& population, const vector<ChromosomePtr>& mutatedOffsprings, vector<ChromosomePtr>& survivors) {
    vector<ChromosomePtr> selectionPool = population ; for(auto& offspring : mutatedOffsprings) selectionPool.push_back(offspring) ;
    survivors.clear() ; 

    sort(selectionPool.begin(), selectionPool.end(), [](ChromosomePtr& c1, ChromosomePtr& c2){return c1->fitness > c2->fitness;}) ; 

    for(int i=0; i<GAConfig.populationSize/2; ++i){
        survivors.push_back(selectionPool.back()) ;
        selectionPool.pop_back() ;
    }

    for(int i=0; survivors.size()<GAConfig.populationSize; ++i){
        int s = GlobalRandomTool.rand(0, selectionPool.size()-1) ;
        survivors.push_back(selectionPool[s]) ;
        swap(selectionPool.back(), selectionPool[s]) ; selectionPool.pop_back() ; 
    }
}

pair<double, double> GlobalRoute::statistically_evaluate(const Channel& channel, vector<ChromosomePtr>& population, CostStruct& avgCostResult, CostStruct& stdCostResult){
    vector<CostStruct> costResults(population.size()) ; 
    vector<double> costs ; 
    double totalCost = 0.0, avgCost = 0.0, stdCost = 0.0;

    for(int i=0; i<population.size(); ++i){
        ChromosomePtr& chromosomePtr = population[i] ;
        costs.push_back(caclute_cost(channel, *chromosomePtr, costResults[i])) ;
    }

    avgCostResult = CostStruct{0, 0, 0, 0} ;
    for(int i=0; i<population.size(); ++i){
        avgCostResult.wireLength += costResults[i].wireLength ; 
        avgCostResult.excessiveCapacity += costResults[i].excessiveCapacity ; 
        avgCostResult.conflictCount += costResults[i].conflictCount ; 
        avgCostResult.differentialLength += costResults[i].differentialLength ; 
        avgCost += costs[i] ;
    }
    
    avgCostResult.wireLength = double(avgCostResult.wireLength)/population.size() ;
    avgCostResult.excessiveCapacity = double(avgCostResult.excessiveCapacity)/population.size() ;
    avgCostResult.conflictCount = double(avgCostResult.conflictCount)/population.size() ;
    avgCostResult.differentialLength = double(avgCostResult.differentialLength)/population.size() ;
    avgCost = avgCost/population.size() ;

    stdCostResult = CostStruct{0, 0, 0, 0} ;

    for(int i=0; i<population.size(); ++i){
        stdCostResult.wireLength += pow(costResults[i].wireLength - avgCostResult.wireLength, 2) ; 
        stdCostResult.excessiveCapacity += pow(costResults[i].excessiveCapacity - avgCostResult.excessiveCapacity, 2) ; 
        stdCostResult.conflictCount += pow(costResults[i].conflictCount - avgCostResult.conflictCount, 2) ; 
        stdCostResult.differentialLength += pow(costResults[i].differentialLength - avgCostResult.differentialLength, 2) ; 
        stdCost += pow(costs[i]-avgCost, 2) ; 

    }

    stdCostResult.wireLength = sqrt(double(stdCostResult.wireLength)/population.size()) ;
    stdCostResult.excessiveCapacity = sqrt(double(stdCostResult.excessiveCapacity)/population.size()) ;
    stdCostResult.conflictCount = sqrt(double(stdCostResult.conflictCount)/population.size()) ;
    stdCostResult.differentialLength = sqrt(double(stdCostResult.differentialLength)/population.size()) ;
    stdCost = sqrt(stdCost/population.size()) ;
    return {avgCost, stdCost} ; 
}

void GlobalRoute::genetic_algorithm_process(const Channel& channel, const Chromosome& referencedChromosome, Chromosome& bestChromosome) {

    vector<ChromosomePtr> population, parents, offsprings, survivors ;
    CostStruct costResult, bestCostResult ;
    double bestCost ; 
    cout << "----------Genetic Algorithm Start----------\n" ;
    initialize_population(channel, referencedChromosome, population) ; 

    cout << "GA: Step A\n" ;

    update_and_legalize(channel, population) ;
    cout << "GA: Step B\n" ;
    evaluate(channel, population) ;

    cout << "GA: Step C\n" ;
    for(int i=1; i<=GAConfig.generations; ++i){
        select_parents(population, parents) ; 
        crossover(parents, offsprings) ; 
        mutation(offsprings) ; 
        update_and_legalize(channel, offsprings) ;
        evaluate(channel, offsprings) ;
        select_surviors(population, offsprings, survivors) ; 
        population = survivors ;

        if(i%50==0){
            cout << "----------Iteration " << i << "----------\n" ;
            CostStruct avgCostResult, stdCostResult ;
            pair<double, double> costSTAs = statistically_evaluate(channel, population, avgCostResult, stdCostResult) ; 
            
            cout << "AVE: " << avgCostResult << "| AVE Cost: " << costSTAs.first << "\n" ;
            cout << "STD: " << stdCostResult << "| STD Cost: " << costSTAs.second << "\n" ;
            cout << "------------------------------------------\n" ;
        }
    }

    bestChromosome = **min_element(population.begin(), population.end(), [](ChromosomePtr& p1, ChromosomePtr& p2){return p1->fitness < p2->fitness;}) ; 

    bestCost = caclute_cost(channel, bestChromosome, bestCostResult) ; 

    cout << "Best Result:\n" ;
    // cout << "Chromosome: " << bestChromosome << "\n" ;
    // cout << "Cost Structure: " << bestCostResult << "\n" ;
    cout << "Cost: " << bestCost << "\n" ;
    cout << "Wire Length: " << bestCostResult.wireLength << "\n" ; 
    cout << "Excessive Capacity: " << bestCostResult.excessiveCapacity << "\n" ;
    cout << "Conflict Count: " << bestCostResult.conflictCount << "\n" ;

}

void GlobalRoute::add_shielding(Chromosome& chromosome) {
    Chromosome newChromosome(chromosome.size()*3) ; 

    for(int i=0; i<chromosome.size(); ++i){
        newChromosome[3*i] = chromosome[i] ; newChromosome[3*i].startVia = newChromosome[3*i].targetVia = nullptr ; // Upper ground net
        newChromosome[3*i+1] = chromosome[i] ; // Signal net
        newChromosome[3*i+2] = chromosome[i] ; newChromosome[3*i+2].startVia = newChromosome[3*i+2].targetVia = nullptr ; // Lower ground net
    }

    chromosome = newChromosome ;
}

void GlobalRoute::sort_routing_order(Chromosome& chromosome) {
    sort(chromosome.begin(), chromosome.end(), 
        [](const ChannelRepresentation& c1, const ChannelRepresentation& c2 ){
            return  (c1.path.channelRow < c2.path.channelRow) ||
                    (c1.path.channelRow == c2.path.channelRow) && (c1.code > c2.code) ;
        }
    );

}

void GlobalRoute::generate_global_nets(const RoutingInfo& routingInfo, const Chromosome& chromosome, vector<ChannelGlobalNet>& globalNets) {
    globalNets.clear() ;
    
    for(auto& representation : chromosome){
        ChannelGlobalNet newGlobalNet ; 
        newGlobalNet.startVia = representation.startVia ; 
        newGlobalNet.targetVia = representation.targetVia ; 

        for(auto& t2tEdge : representation.path){

            
// if(newGlobalNet.startVia && (newGlobalNet.startVia->id==40 || newGlobalNet.startVia->id==36)){
    // cout << *t2tEdge->node1 << " " << *t2tEdge->node2 << " " << **routingInfo.graph.tileCrossingEdges.at(t2tEdge).begin() << "\n" ;
// }
            newGlobalNet.push_back( *routingInfo.graph.tileCrossingEdges.at(t2tEdge).begin() ) ; 
        }
        globalNets.push_back(newGlobalNet) ;
        // cout << "\n" ;
    }
}
double GlobalRoute::global_route(RoutingInfo& routingInfo, CostStruct& costResult) {
    Timer timer; timer.set_clock() ;

    Channel channel ; 
    Chromosome referencedChromosome, bestChromosome ;
    vector<ChannelGlobalNet> globalNets ; 
    construct_routing_channel(routingInfo, channel) ;
    connect_routing_channel(routingInfo, channel) ;
    generate_refenced_chromosome(routingInfo, channel, referencedChromosome) ; 
    genetic_algorithm_process(channel, referencedChromosome, bestChromosome) ; 
    sort_routing_order(bestChromosome) ; 
    add_shielding(bestChromosome) ;
    generate_global_nets(routingInfo, bestChromosome, routingInfo.globalNets) ;

    cout << "Global Route Elapsed Time: " << timer.get_duration_milliseconds() << " ms\n"  ;
    // for(auto& globalNet : routingInfo.globalNets){
    //     vector<Bump> bumps = routing_node_ptrs_to_bumps(globalNet) ;
    //     for(auto& bump : bumps) routingInfo.debugBumpMapping["GR_NETs"].push_back(bump) ;
    // }

    vector<vector< array<DirectionalT2TEdge, 4> >>& channelT2TEdgeMaze = channel.channelT2TEdgeMaze ; 
    vector<TileToTileEdgePtr> newEdges ; 
    for(int i=0; i<channelT2TEdgeMaze.size(); ++i){
        for(int j=0; j<channelT2TEdgeMaze[i].size(); ++j){
            for(auto e : {UP, DOWN, LEFT, RIGHT}){
                if(channelT2TEdgeMaze[i][j][e].t2tEdge){
                    newEdges.push_back(channelT2TEdgeMaze[i][j][e].t2tEdge) ;
                }
            }
        }
    }
    routingInfo.debugEdgeMapping["CHANNEL_T2T_Edges"] = routing_edge_ptrs_to_edge(newEdges, "#A5405A") ;


    // costResult result ;
    return 0.0 ;
}
