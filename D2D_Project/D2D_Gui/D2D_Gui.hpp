#ifndef D2D_GUI_HPP
#define D2D_GUI_HPP

#include <QElapsedTimer>
#include <QMainWindow>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <QStandardItemModel>
#include <QStandardItem>

#include "Drawer.hpp"

#include <unistd.h>
#include <sys/wait.h>
#include <iostream>
#include <filesystem>
#include <vector>
#include <map>
#include <tuple>
#include <algorithm>
#include <cmath>

class D2DGui ;

// 視窗相關的設定
class QtView : public QGraphicsView {
    Q_OBJECT
public:
    friend class D2DGui ;
    QtView(QWidget *parent = nullptr) : QGraphicsView(parent) {
        setRenderHint(QPainter::Antialiasing);
        setDragMode(QGraphicsView::ScrollHandDrag);
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

        setStyleSheet("background-color: black;");
        // setBackgroundBrush(Qt::NoBrush);
        // setStyleSheet("background: transparent; border: none;");
        // setAttribute(Qt::WA_TranslucentBackground);
    }

    QtView(QGraphicsScene *scene, QWidget *parent = nullptr) : QGraphicsView(scene, parent) {
        setRenderHint(QPainter::Antialiasing);
        setDragMode(QGraphicsView::ScrollHandDrag);
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

        setStyleSheet("background-color: black;");

        
        // setBackgroundBrush(Qt::NoBrush);
        // setStyleSheet("background: transparent; border: none;");
        // setAttribute(Qt::WA_TranslucentBackground);
    }

signals:
    void mouseMoved(const QPointF &pos); // 定義信號    
    void mouseQuicrPressAndRelease(const vector<QPointF> &poss) ; 
protected:
    bool rightMousePressed = false;
    bool rightMouseMoved = false;
    QPoint lastMousePos;

    QElapsedTimer prevPressedTimer ;
    vector<QPointF> prevPressedPoss ;

    int prevPressedState = 0 ; 

    // 滑鼠按下事件
    void mousePressEvent(QMouseEvent *event) override {
        prevPressedTimer.start() ;
       
        if (event->button() == Qt::RightButton) {
            rightMousePressed = true;
        } else {
            QGraphicsView::mousePressEvent(event);
        }
        
    }


    // 滑鼠移動事件
    void mouseMoveEvent(QMouseEvent *event) override {

        if (rightMousePressed) {
            rightMouseMoved = true ;
        } else {
            QPointF scenePos = mapToScene(event->pos()); // 轉為場景座標
            emit mouseMoved(scenePos); // 發射信號
            QGraphicsView::mouseMoveEvent(event); // 呼叫父類處理
        }
    }

    // 滑鼠釋放事件
    void mouseReleaseEvent(QMouseEvent *event) override {
        qint64 interval = prevPressedTimer.elapsed() ; // ms
        
        if (event->button() == Qt::RightButton) {
            if( interval < 200.0 && !rightMouseMoved){ // 
                prevPressedPoss.push_back(mapToScene(event->pos())); // 轉為場景座標
                emit mouseQuicrPressAndRelease( prevPressedPoss ); // 發射信號
                if(prevPressedPoss.size()>=2) prevPressedPoss.clear() ;
            }

            rightMousePressed = false;
            rightMouseMoved = false ;
        } else {
            QGraphicsView::mouseReleaseEvent(event);
        }
    }


    void wheelEvent(QWheelEvent *event) override {
        if (event->modifiers() & Qt::ControlModifier) {
            double factor = (event->angleDelta().y() > 0) ? 1.1 : 0.9;
            scale(factor, factor);
        } else {
            QGraphicsView::wheelEvent(event);
        }
    }

    void keyPressEvent(QKeyEvent *event) override {
        if (event->key() == Qt::Key_Z) {
            resetTransform();  // 重置縮放和平移
            // centerOn(0, 0);     // 回到初始座標位置 (0, 0)
        } else {
            QGraphicsView::keyPressEvent(event);
        }
    }

};



class D2DMainWindow : public QMainWindow {
    Q_OBJECT
public:
    D2DMainWindow(int argc, char *argv[], QWidget *parent = nullptr);
    ~D2DMainWindow();
private:
    D2DGui *ui ; 
};


using namespace std ; 

class D2DGui {
 protected:
    QGraphicsScene* scenece ;
    QAction *actionOpen_result_folder;
    QAction *actionOpen_execution_file;
    QWidget *centralwidget;
    QPushButton *rerunButton ;
    QPushButton *reloadButton ;
    QListView *listView ;
    QtView *graphicsView;
    QSpinBox *spinBox;
    QStatusBar *statusbar;
    QMenuBar *menubar ;
    QMenu *menuFile ;
    QLabel *posLabel ;
    QLabel *distLabel ;

    QStandardItemModel *layerModel ;
    QMap<QString, QList<QGraphicsItem*>> layerGroups ; 
    
    QHBoxLayout *mainLayout ;
    QVBoxLayout *rightLayout ;
    vector<QGraphicsScene*> sceneces ;

    string executionFile ; //"./bin/D2D" ;
    string designBumpPath ; //"testcase/standard_44/bumps.loc" ;
    string designRulePath ; //"./testcase/standard_44/rule.txt" ;
    string resultDirectory ;//  "./result/" ;


    // QGraphicsScene* scene ; //scene.setSceneRect(0, 0, 5000, 2000) ;

    using GroupType = tuple<int, string> ;
    map<GroupType, vector<QGraphicsItem*>> graphicsItemControlTable ; 
    // map<string, map<string, QStandardItem*>> layerItemGroups ; 
    map<QStandardItem*, GroupType> layerItemMapping ; 
    map<GroupType, Qt::CheckState> backupStates ;

    void setLayerVisible(const GroupType& group, Qt::CheckState visible) ;
    void changeLayer(int layer) ;
    void setWidgetText(QMainWindow *D2D_Gui) ; 
    void retunExecutable(const string& executionFile, const string& designBumpPath, const string& designRulePath, const string& resultDirectory) ; 
    void reloadLayoutResult(const string& resultDirectory, const string& designRulePath) ; 
public:    
    D2DGui(     const string executionFile="", const string designBumpPath="",
                const string designRulePath="", const string resultDirectory="") 
                : executionFile(executionFile), designBumpPath(designBumpPath), designRulePath(designRulePath), resultDirectory(resultDirectory) {}
            
    void setGUI(QMainWindow *D2D_Gui) ; 
} ; 
#endif // D2D_GUI_HPP

