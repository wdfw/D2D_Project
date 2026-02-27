#pragma once 
#include <boost/geometry.hpp>
#include <cmath>
#include <memory>
#include <limits>

namespace bg = boost::geometry;
namespace bgsb = bg::strategy::buffer ;
namespace bgi = boost::geometry::index;

// using bg::distance ; 
// using bg::crosses ; 
// using bg::read_wkt ;

using point_xy = bg::model::d2::point_xy<double> ; 
using box_xy = bg::model::box<point_xy> ;
using segment_xy = bg::model::segment<point_xy> ;
using polygon_xy = bg::model::polygon<point_xy> ;
using multi_point_xy = bg::model::multi_point<point_xy> ;
using multi_polygon_xy = boost::geometry::model::multi_polygon<polygon_xy>;
using linestring_xy = bg::model::linestring<point_xy> ;
using multi_linestring_xy = bg::model::multi_linestring<linestring_xy> ;

template <typename T> class routing_shared_ptr ;
// point_xy& operator+(const point_xy& p1, const& point_xy p2) ;
// point_xy& operator-(const point_xy& p1, const& point_xy p2) ;
// point_xy& operator*(const point_xy& p1, double m) ;
// point_xy& operator/(const point_xy& p1, double d) ;

std::ostream& operator<<(std::ostream& os, const point_xy& point) ;
std::ostream& operator<<(std::ostream& os, const box_xy& box) ;
std::ostream& operator<<(std::ostream& os, const segment_xy& segment) ;
std::ostream& operator<<(std::ostream& os, const polygon_xy& polygon) ;
std::ostream& operator<<(std::ostream& os, const multi_point_xy& multi_point) ;
std::ostream& operator<<(std::ostream& os, const linestring_xy& linestring) ;

template <typename T1, typename T2>
double distance_sc(const T1& p1, const T2& p2) { return bg::distance(static_cast<const point_xy&>(p1), static_cast<const point_xy&>(p2));}

template <typename T1, typename T2, typename T3, typename T4>
double is_crossed_sc(const T1& p11, const T2& p12, const T3& p21, const T4& p22) { 
    segment_xy seg1(static_cast<const point_xy&>(p11), static_cast<const point_xy&>(p12)) ;
    segment_xy seg2(static_cast<const point_xy&>(p21), static_cast<const point_xy&>(p22)) ;
    return bg::intersects(seg1, seg2);
}


template <typename T>
const T& unwrap(const T& v) { return v; }

template <typename T>
const T& unwrap(const std::shared_ptr<T>& v) { return *v; }

template <typename T>
const T& unwrap(const routing_shared_ptr<T>& v) { return *v; }

template<typename T> // 把元素根據y軸進行分列
void find_elements_in_each_row(const std::vector<T>& elements, std::vector<std::vector<T>>& rowElements, double epsilonY = 1e-6) ;

template<typename T> // 把元素根據segements的區段進行分列，若有元素在區段外則報錯
void find_elements_in_each_row(const std::vector<T>& elements, std::vector<std::vector<T>>& rowElements, const std::vector<std::tuple<double, double>>& segements) ; 

template <typename T> // 把元素根據y軸分列並再以x軸排序
void matrixlize(const std::vector<T>& elements, std::vector<std::vector<T>>& matrix, double epsilonY = 1e-6) ;

template <typename T> // 把元素根據segements的區段分列並再以x軸排序
void matrixlize(const std::vector<T>& elements, std::vector<std::vector<T>>& matrix, const std::vector<std::tuple<double, double>>& segements) ;

template <typename T> // 把元素根據segements的區段分列並再以x軸排序
double minimum_distance_between_elements(const std::vector<T>& elements) ;

template<typename T> 
void find_elements_in_each_row(const std::vector<T>& elements, std::vector<std::vector<T>>& rowElements, double epsilonY){ 
    std::vector<T> sortedElements = elements ; sort(sortedElements.begin(), sortedElements.end(), 
        [](const T& a, const T& b) {return unwrap(a).y() < unwrap(b).y();});
    
    rowElements.clear() ; 

    for (const auto& element : sortedElements) {
        if(!rowElements.size()) rowElements.push_back({element}) ;
        else if(fabs(unwrap(rowElements.back().back()).y() - unwrap(element).y()) >= epsilonY) rowElements.push_back({element}) ;
        else rowElements.back().push_back(element) ;
    }
}

template<typename T> // 把元素根據y軸進行分類
void find_elements_in_each_row(const std::vector<T>& elements, std::vector<std::vector<T>>& rowElements, const std::vector<std::tuple<double, double>>& segements){ 
    std::vector<std::vector<T>> temp ; 
    std::vector<std::vector<int>> groups(segements.size()) ;  

    find_elements_in_each_row(elements, temp) ; 

    for(int i=0; i<temp.size(); ++i){
        double y = unwrap(temp[i][0]).y() ;
        int groupIndex = -1 ; 
        for(int j=0; j<segements.size(); ++j){
            if(std::get<0>(segements[j])<=y && y<std::get<1>(segements[j])){
                groupIndex = j ; break ;
            }
        }

        if(groupIndex==-1){
            std::cout << unwrap(temp[i][0]).x() << " " << unwrap(temp[i][0]).y() << "\n"; 
            for(auto& p : segements) std::cout << std::get<0>(p) << "," << std::get<1>(p) << "\n" ;
            throw std::runtime_error("Can't find a valid segement in Matrixlize.") ;
        } 
        groups[groupIndex].push_back(i) ; 
    }

    rowElements.clear() ; 
    for(int i=0; i<groups.size(); ++i){
        if(!groups[i].size()) continue ;
        rowElements.push_back(std::vector<T>()) ; 
        for(int j=0; j<groups[i].size(); ++j){
            int k = groups[i][j] ; 
            rowElements.back().insert(rowElements.back().end(), temp[k].begin(), temp[k].end()) ; 
        }
    }
}

template <typename T>
void matrixlize(const std::vector<T>& elements, std::vector<std::vector<T>>& matrix, double epsilonY) {
    find_elements_in_each_row(elements, matrix, epsilonY) ; 
    for(auto& row : matrix) sort(row.begin(), row.end(), [](const T& a, const T& b) {return unwrap(a).x() < unwrap(b).x();}) ;
}

template <typename T>
void matrixlize(const std::vector<T>& elements, std::vector<std::vector<T>>& matrix, const std::vector<std::tuple<double, double>>& segements) {
    find_elements_in_each_row(elements, matrix, segements) ; 
    for(auto& row : matrix) sort(row.begin(), row.end(), [](const T& a, const T& b) {return unwrap(a).x() < unwrap(b).x();}) ;
}



template <typename T> // 把元素根據segements的區段分列並再以x軸排序
double minimum_distance_between_elements(const std::vector<T>& elements) {
    double minimumDistance = std::numeric_limits<double>::max() ; 
    for(int i=0; i<elements.size(); ++i){
        for(int j=i+1; j<elements.size(); ++j){
            const T& e1 = elements[i] ;
            const T& e2 = elements[j] ;
            double distance = std::sqrt(std::pow(unwrap(e1).x()-unwrap(e2).x(), 2) + std::pow(unwrap(e1).y()-unwrap(e2).y(), 2)) ;
            minimumDistance = std::min(minimumDistance, distance) ;
        }
    }
    return minimumDistance ;
}

template <typename T> 
class routing_shared_ptr : public std::shared_ptr<T> {
public:
    using std::shared_ptr<T>::shared_ptr;
    routing_shared_ptr(std::shared_ptr<T>&& other) : std::shared_ptr<T>(std::move(other)) {}
    routing_shared_ptr(const std::shared_ptr<T>& other) : std::shared_ptr<T>(other) {}

    bool operator<(const routing_shared_ptr& ptr) const { return **this < *ptr ; }
    bool operator>(const routing_shared_ptr& ptr) const { return **this > *ptr ; }
    bool operator>=(const routing_shared_ptr& ptr) const { return **this >= *ptr ; }
    bool operator<=(const routing_shared_ptr& ptr) const { return **this <= *ptr ; }
    bool operator==(const routing_shared_ptr& ptr) const { return **this == *ptr ; }
    bool operator!=(const routing_shared_ptr& ptr) const { return **this != *ptr ; }
};
