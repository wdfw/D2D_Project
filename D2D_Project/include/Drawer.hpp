#pragma once

#include <QMainWindow>
#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QScrollBar>
#include <QMouseEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>
#include <QWidget>
#include <QGraphicsProxyWidget>

#include <map>

#include "Utils.hpp"
#include "Parser.hpp"
#include "DesignRule.hpp"
#include "Geometry.hpp"
#include "DesignStructure.hpp"
#include "RoutingStructure.hpp"

using namespace std ;

#define QT_DRAWER_OFFSET -0.5 // 修正顯示問題的參數

class Drawer {
private:
    bgsb::distance_symmetric<double> viaStrategy = bgsb::distance_symmetric<double>(0.0) ; // for UI
    bgsb::distance_symmetric<double> viaPadStrategy = bgsb::distance_symmetric<double>(0.0) ; // for UI
    bgsb::distance_symmetric<double> lineWidthStrategy = bgsb::distance_symmetric<double>(0.0) ; // for UI
    bgsb::join_round joinStrategy = bgsb::join_round(0) ; // for UI
    bgsb::end_round endStrategy = bgsb::end_round(0) ; // for UI
    bgsb::point_circle circleStrategy = bgsb::point_circle(0) ; // for UI
    bgsb::side_straight sideStrategy = bgsb::side_straight() ; // for UI
    double teadropRadius = 0.0 ; // for UI

    bgsb::distance_symmetric<double> DRC_LineSapcingStrategy = bgsb::distance_symmetric<double>(0.0) ; // for DRC
    bgsb::distance_symmetric<double> DRC_LineWidthStrategy = bgsb::distance_symmetric<double>(0.0) ; // for DRC
    bgsb::distance_symmetric<double> DRC_ViaPadSapcingStrategy = bgsb::distance_symmetric<double>(0.0) ; // for DRC

    QGraphicsScene *scene = nullptr ;
    DesignRule designRule ;
public:
    Drawer() = default ; 
    Drawer(const DesignRule& designRule, QGraphicsScene *scene=nullptr) ;
    void set_design_rule(const DesignRule& designRule) ; // 設定Design rule, 並根據規則生成緩衝區策略
    void set_design_scence(QGraphicsScene *scene) ; // 設定要畫的QT場景
    
    void draw_label(const Label& label, vector<QGraphicsItem*>& itemBuffer) ;
    void draw_point(const Point& point, vector<QGraphicsItem*>& itemBuffer) ;
    void draw_edge(const Edge& edge, vector<QGraphicsItem*>& itemBuffer) ; 
    void draw_bump(const Bump& bump, vector<QGraphicsItem*>& itemBuffer) ;
    void draw_offset_via(const OffsetVia& offsetVia, vector<QGraphicsItem*>& itemBuffer) ; 
    void draw_net(const Net& net, vector<QGraphicsItem*>& itemBuffer) ; 
    
    void draw_violations(const vector<Bump>& bumps, const vector<Net>& nets, vector<QGraphicsItem*>& itemBuffer) ;

    // void DrawOffsetVia(const OffestVia& offsetVia) ; 
    // void DrawNet(const Net& net, Qt::GlobalColor netColor=Qt::yellow) ; // 畫單條Net, netColor可以指定顏色, 當netColor為黃色時會根據Net的種類畫預設的顏色(e.g. 不能畫黃色的Net)
    // void DrawTeardrop(const tuple<Bump, double, double, double, double>& teardrop) ; // 畫一顆Teardrop
    // void DrawDieBoundary(const vector<double>& coordinate) ; // 畫Die的邊界, 目前邊界僅為參考用
    // void DrawTriangulation(const tuple<double, double, double, double>& edge) ; // 畫三角化的邊
    // void DrawDebugLabel(const tuple<string, double, double>& label) ;

    // void DrawOneRDL(vector<Bump> &bumps, vector<double> &coordinate) ; 
    // void DrawAllRDL(vector<Bump> &bumps, vector<double> &coordinate) ; 
} ;
