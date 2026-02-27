#pragma once
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>

using boost::container::flat_map ; 
using boost::container::flat_set ; 

template<typename T1, typename T2>
class double_linked_flat_relation {
protected:
    flat_set<T1> type1_empty ;
    flat_set<T2> type2_empty ;
    flat_map<T1, flat_set<T2>> type1_to_type2_relation ;
    flat_map<T2, flat_set<T1>> type2_to_type1_relation ;  
public:
    void clear(){
        type1_to_type2_relation.clear() ; 
        type2_to_type1_relation.clear() ;
    } ; 

    bool contains(const T1& key) const {
        return type1_to_type2_relation.contains(key) ; 
    }

    bool contains(const T2& key) const {
        return type2_to_type1_relation.contains(key) ; 
    }

    bool contains(const T1& key1, const T2& key2) const {
        return type1_to_type2_relation.contains(key1) && type1_to_type2_relation[key1].contains(key2) ; 
    }

    // T2& operator[](const T1& key) {
    //     return type1_to_type2_relation[key] ;
    // }
    // T1& operator[](const T2& key) {
    //     return type2_to_type1_relation[key] ;
    // }
    void insert(const std::pair<T1, T2>& key1Key2){
        type1_to_type2_relation[key1Key2.first].insert(key1Key2.second) ; 
        type2_to_type1_relation[key1Key2.second].insert(key1Key2.first) ; 
    }

    const flat_set<T2>& at(const T1& key) const {
        if(!contains(key)) return type2_empty ;
        return type1_to_type2_relation.at(key) ; 
    }
    // T2& at(const T1& key) {
    //     return type1_to_type2_relation.at(key) ; 
    // }
    const flat_set<T1>& at(const T2& key) const {
        if(!contains(key)) return type1_empty ;
        return type2_to_type1_relation.at(key) ; 
    }
    // T1& at(const T2& key) {
    //     return type2_to_type1_relation.at(key) ; 
    // }
    size_t size() const {
        return type1_to_type2_relation.size() ; 
    }

    auto begin(const T1& key) const { 
        return at(key).begin() ; 
    }
    auto end(const T1& key) const { 
        return at(key).end() ; 
    }

    auto begin(const T2& key) const { 
        return at(key).begin() ; 
    }
    auto end(const T2& key) const { 
        return at(key).end() ; 
    }
} ;