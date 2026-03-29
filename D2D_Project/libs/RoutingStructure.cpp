#include "RoutingStructure.hpp"


bool PositionNode::is_compitable(BumpType& netType){

    if(coverCount){ //occupyCount && coverCount
        if(occupyCount && occupyType==netType) return true ; 
    }else{ 
        if(occupyType==DUMMY) return true ;
        else throw logic_error("Unoccupied position with non DUMMY type.") ;
    }
    
    return false ; 
}

void PositionNode::clear() {
    occupyType = DUMMY ; 
    occupyCount = 0 ; 
    coverCount = 0 ;
}

void PositionNode::add_occupy(BumpType newOccupyType) {
    if(newOccupyType==DUMMY){
        throw logic_error("Add DUMMY occuption.") ; 
    }

    if(occupyType != DUMMY && occupyType != newOccupyType){
        cerr << *this << " | " << occupyType << " " << occupyCount << "|" << newOccupyType << "\n";
         throw logic_error("PositionNode has 2 different occupyType.") ; 
    }

    occupyType = newOccupyType ; 
    ++occupyCount ; 
    add_cover() ; 
}


void PositionNode::add_cover() {
    ++coverCount ; 
}

void PositionNode::reduce_cover() {
    if(coverCount==0){
        throw logic_error("PositionNode has not been covered.") ; 
    }
    --coverCount ; 
}

void PositionNode::reduce_occupy() {
    if(occupyType == DUMMY && occupyCount == 0){
         throw logic_error("PositionNode has not been occupied.") ; 
    }
    if(!--occupyCount) occupyType = DUMMY ; 
    reduce_cover() ; 
}