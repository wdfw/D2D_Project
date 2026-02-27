#include "Drawer.hpp"

map<BumpType, Qt::GlobalColor> BumpColor = {{DUMMY, Qt::gray}, {SIGNAL, Qt::red}, {VDD, Qt::blue}, {VSS, Qt::green}} ;
map<BumpType, Qt::GlobalColor> OffsetBumpColor = {{DUMMY, Qt::darkGray}, {SIGNAL, Qt::darkRed}, {VDD, Qt::darkBlue}, {VSS, Qt::darkGreen}} ;
map<BumpType, Qt::GlobalColor> NetColor = {{DUMMY, Qt::gray}, {SIGNAL, Qt::red}, {VDD, Qt::blue}, {VSS, Qt::green}} ;
map<BumpType, Qt::GlobalColor> SpacingColor = {{DUMMY, Qt::darkGray}, {SIGNAL, Qt::darkRed}, {VDD, Qt::darkBlue}, {VSS, Qt::darkGreen}} ;

//-------------------------------------------------------------------------------------

Drawer::Drawer(const DesignRule& designRule, QGraphicsScene *scene){
    set_design_rule(designRule) ;
    set_design_scence(scene) ;
}

void Drawer::set_design_rule(const DesignRule& designRule){
    this->designRule = designRule ;

    // 設定緩衝區策略
    const int points_per_circle = 36;

    viaStrategy = bgsb::distance_symmetric<double>(designRule.viaRadius + QT_DRAWER_OFFSET);
    viaPadStrategy = bgsb::distance_symmetric<double>(designRule.viaPadRadius - designRule.viaRadius) ;
    lineWidthStrategy = bgsb::distance_symmetric<double>(designRule.minimumLineWidth/2 + QT_DRAWER_OFFSET);

    DRC_LineWidthStrategy = bgsb::distance_symmetric<double>(designRule.minimumLineWidth/2 );
    DRC_LineSapcingStrategy = bgsb::distance_symmetric<double>(designRule.minimumLineWidth/2 + designRule.minimumLineSpacing) ; 
    DRC_ViaPadSapcingStrategy = bgsb::distance_symmetric<double>(designRule.viaPadRadius + designRule.minimumLineViaPadSpacing) ; 

    joinStrategy = bgsb::join_round(points_per_circle);
    endStrategy = bgsb::end_round(points_per_circle);
    circleStrategy = bgsb::point_circle(points_per_circle);
    sideStrategy = bgsb::side_straight();

    teadropRadius = designRule.viaPadRadius ;
}

void Drawer::set_design_scence(QGraphicsScene *scene){
    this->scene = scene ; 
}

void Drawer::draw_label(const Label& label, vector<QGraphicsItem*>& itemBuffer){
    QString QLabelStr = QString::fromStdString(label.labelStr) ; 
    QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(QLabelStr);

    textItem->setBrush(QColor(label.color.c_str()));
    textItem->setFont(QFont("Arial", label.fontSize));  
    textItem->setPos(label.x(), label.y()) ;  
    scene->addItem(textItem);  

    itemBuffer.push_back(textItem) ;
}


void Drawer::draw_point(const Point& point, vector<QGraphicsItem*>& itemBuffer){
    multi_polygon_xy buffer;
    point_xy cpt = point ; 

    QBrush innerdBrush(QColor(point.color.c_str())); 
    QPen innerPen(QColor(point.color.c_str())); innerPen.setWidth(point.radius);
    QPolygonF qred ;
    qred << QPointF(cpt.x(), cpt.y());
    itemBuffer.push_back(scene->addPolygon(qred, innerPen, innerdBrush)) ;
    // boost::geometry::buffer(cpt, buffer, viaStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);

    // for (const auto &poly : buffer) { 
    //     QPolygonF qred;
    //     for (const auto &pt : poly.outer()) qred << QPointF(cpt.x(), cpt.y());
    //     itemBuffer.push_back(scene->addPolygon(qred, innerPen, innerdBrush)) ;
    // }

}

void Drawer::draw_edge(const Edge& edge, vector<QGraphicsItem*>& itemBuffer){
    QPen pen(QColor(edge.color.c_str())); pen.setWidth(edge.width);
    itemBuffer.push_back(scene->addLine(get<0>(edge).x(), get<0>(edge).y(), get<1>(edge).x(), get<1>(edge).y(), pen)) ;
}


void Drawer::draw_bump(const Bump &bump, vector<QGraphicsItem*>& itemBuffer){
    multi_polygon_xy buffer, final_outer_buffer;
    point_xy pt(bump) ; 
    
    QBrush outerBrush(Qt::yellow); 
    QPen outerPen(Qt::yellow); outerPen.setWidth(1);
    Qt::GlobalColor bumpColor = BumpColor[bump.type] ;
    QBrush innerdBrush(bumpColor);
    QPen innerPen(bumpColor); innerPen.setWidth(1);

    boost::geometry::buffer(pt, buffer, viaStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
    boost::geometry::buffer(buffer, final_outer_buffer, viaPadStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);

    for (const auto &poly : final_outer_buffer) {
        QPolygonF qpoly;
        for (const auto &pt : poly.outer()){
            qpoly << QPointF(pt.x(), pt.y());
        }
        itemBuffer.push_back(scene->addPolygon(qpoly, outerPen, outerBrush)) ;
    }

    for (const auto &poly : buffer) { 
        QPolygonF qred;
        for (const auto &pt : poly.outer()){
            qred << QPointF(pt.x(), pt.y());
        }
        itemBuffer.push_back(scene->addPolygon(qred, innerPen, innerdBrush)) ;
    }
    
    // 繪製 bump id 文字
    if (1){
        QString label = QString::number(bump.id);
        QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(label);
        textItem->setBrush(Qt::white);
        textItem->setFont(QFont("Arial", 6));  // 字小一點
        textItem->setPos(bump.x() - 5, bump.y() - 5);  // bump 位置
        scene->addItem(textItem);  // 直接加進 scene，不加進 group
        itemBuffer.push_back(textItem) ;
    }
}
void Drawer::draw_offset_via(const OffsetVia &offsetVia, vector<QGraphicsItem*>& itemBuffer){
    
    const Bump& bump = get<0>(offsetVia) ; 
    const Bump& matchedBump = get<1>(offsetVia) ; 

    point_xy center1(bump.x(), bump.y());
    point_xy center2(matchedBump.x(), matchedBump.y());  // 兩個圓心相距 50，彼此不相交
    polygon_xy convex_hull;
    multi_polygon_xy buffer1, buffer2, merged_area, final_outer_buffer;

    QBrush outerYellowBrush(Qt::yellow);
    QPen outerYellowPen(Qt::yellow);  outerYellowPen.setWidth(1);

    boost::geometry::buffer(center1, buffer1, viaStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
    boost::geometry::buffer(center2, buffer2, viaStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
    boost::geometry::union_(buffer1, buffer2, merged_area);
    boost::geometry::convex_hull(merged_area, convex_hull);
    boost::geometry::buffer(convex_hull, final_outer_buffer, viaPadStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);

    for (const auto &poly : final_outer_buffer) {
        QPolygonF qpoly;
        for (const auto &pt : poly.outer()) {
            qpoly << QPointF(pt.x(), pt.y());
        }
        itemBuffer.push_back(scene->addPolygon(qpoly, outerYellowPen, outerYellowBrush)) ;
    }

    draw_bump(bump, itemBuffer) ; 
    draw_bump(matchedBump, itemBuffer) ; 
} 

void Drawer::draw_net(const Net& net, vector<QGraphicsItem*>& itemBuffer){
    Qt::GlobalColor netColor = NetColor[net.type] ; 
    QBrush brush(netColor);
    QPen pen(netColor); pen.setWidth(1);

    for(auto& segment : net){
        linestring_xy line ;
        multi_polygon_xy buffer ;
        bg::append(line, point_xy(segment.first.x() , segment.first.y())) ;
        bg::append(line, point_xy(segment.second.x() , segment.second.y())) ;
        boost::geometry::buffer(line, buffer, lineWidthStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
        for (auto &poly : buffer) {
            QPolygonF qpoly;
            for (auto &pt : poly.outer()) {
                qpoly << QPointF(pt.x(), pt.y());
            }
            itemBuffer.push_back(scene->addPolygon(qpoly, pen, brush)) ;
        }
    }

}

void Drawer::draw_violations( const vector<Bump>& bumps, const vector<Net>& nets, vector<QGraphicsItem*>& itemBuffer) {
    // Net : vector<segment_xy>
    
    vector<vector<multi_polygon_xy>> lineDetectionRegionBuffers(nets.size()) ;
    vector<vector<multi_polygon_xy>> lineWidthBuffers(nets.size()) ;
    vector<multi_polygon_xy> viaPadDetectionRegionBuffers(bumps.size()) ;

    for(int i=0; i<bumps.size(); ++i){
        boost::geometry::buffer(point_xy(bumps[i]), viaPadDetectionRegionBuffers[i], DRC_ViaPadSapcingStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);
    }


    for(int i=0; i<nets.size(); ++i){
        const Net& net = nets[i] ; 
        for(auto& segment : net){
            linestring_xy line ;
            multi_polygon_xy buffer1, buffer2 ;
            bg::append(line, segment.first) ;
            bg::append(line, segment.second) ;
            boost::geometry::buffer(line, buffer1, DRC_LineSapcingStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
            boost::geometry::buffer(line, buffer2, DRC_LineWidthStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy) ;
            lineDetectionRegionBuffers[i].push_back(buffer1) ; 
            lineWidthBuffers[i].push_back(buffer2) ; 
        }
    }

    for(int i=0; i<bumps.size(); ++i){
        if(bumps[i].type==DUMMY) continue;

        for(int j=0; j<nets.size(); ++j){
             if(bumps[i].type==nets[j].type){
                if(bumps[i].type==SIGNAL){
                    if(bumps[i].id==nets[j].id) continue ;
                }else{
                    continue;
                }
            }

            for(auto& netBuffer : lineWidthBuffers[j]){
                multi_polygon_xy& vaiPadBuffer = viaPadDetectionRegionBuffers[i] ; 
                bool isContacted = bg::intersects(vaiPadBuffer, netBuffer); 

                if(isContacted){
                    Qt::GlobalColor netColor = SpacingColor[bumps[i].type] ; 
                    QBrush brush(netColor) ;
                    QPen pen(netColor); pen.setWidth(1) ;
                    for (auto &poly : vaiPadBuffer) {
                        QPolygonF qpoly;
                        for (auto &pt : poly.outer()) qpoly << QPointF(pt.x(), pt.y());
                        itemBuffer.push_back(scene->addPolygon(qpoly, pen, brush)) ;
                    }
                }
            }
        }
    }
    
    for(int i=0; i<nets.size(); ++i){
        auto& net = nets[i] ; 
        for(int j=0; j<nets.size(); ++j){
            if(i==j) continue;
            for(auto& buffer1 : lineDetectionRegionBuffers[i]){
                for(auto& buffer2 : lineWidthBuffers[j]){
                    bool isContacted = bg::intersects(buffer1, buffer2); 
                    if(isContacted){
                        Qt::GlobalColor netColor = SpacingColor[net.type] ; 
                        QBrush brush(netColor) ;
                        QPen pen(netColor); pen.setWidth(1) ;
                        for (auto &poly : buffer1) {
                            QPolygonF qpoly;
                            for (auto &pt : poly.outer()) qpoly << QPointF(pt.x(), pt.y());
                            itemBuffer.push_back(scene->addPolygon(qpoly, pen, brush)) ;
                        }
                    }
                }
            }
        }
    }

}
