#include "D2D_Gui.hpp"
#include "./ui_D2D_Gui.h"

#include <QApplication>

D2DMainWindow::D2DMainWindow(int argc, char *argv[], QWidget *parent){
    if (argc<4) {
        cerr << "Usage: " << argv[0] << " <exectuable> <design_bump_file> <design_rule_file> <output_dir>" << "\n";
        throw runtime_error("Invalid parameters.") ; 
    }
    string executionFile = argv[1] ; 
    string designBumpPath = argv[2] ; 
    string designRulePath = argv[3] ;
    string resultDirectory = argv[4] ;

    ui = new D2DGui(executionFile, designBumpPath, designRulePath, resultDirectory) ;
    ui->setGUI(this) ; 
}

D2DMainWindow::~D2DMainWindow(){
    delete ui;
}

void D2DGui::setGUI(QMainWindow *D2D_Gui){
    actionOpen_result_folder = new QAction(D2D_Gui) ;
    actionOpen_execution_file = new QAction(D2D_Gui) ;

    menubar = new QMenuBar(D2D_Gui) ;
    menuFile = new QMenu(menubar) ;
    
    menubar->setGeometry(QRect(0, 0, 1222, 19)) ;
    menubar->addAction(menuFile->menuAction()) ; 
    menuFile->addAction(actionOpen_result_folder) ; 
    menuFile->addAction(actionOpen_execution_file) ;

    centralwidget = new QWidget(D2D_Gui) ;  
    mainLayout = new QHBoxLayout(centralwidget) ;
    rightLayout = new QVBoxLayout() ;

    rerunButton = new QPushButton(centralwidget) ; 
    reloadButton = new QPushButton(centralwidget) ; 
    spinBox = new QSpinBox(centralwidget) ; 
    layerModel = new QStandardItemModel();
    listView = new QListView(centralwidget) ; 

    graphicsView = new QtView(centralwidget); graphicsView->setMouseTracking(true) ;
    
    posLabel = new QLabel("", graphicsView);
    distLabel = new QLabel("", graphicsView);

    statusbar = new QStatusBar(D2D_Gui) ; statusbar->addPermanentWidget(posLabel); statusbar->addPermanentWidget(distLabel);
    
    D2D_Gui->setCentralWidget(centralwidget) ; 
    D2D_Gui->setStatusBar(statusbar) ; 
    D2D_Gui->setMenuBar(menubar); 
    D2D_Gui->resize(1222, 601);
        
    listView->setFixedWidth(151); listView->setModel(layerModel);
    spinBox->setFixedWidth(151); spinBox->setFixedHeight(51) ;
    rerunButton->setFixedWidth(151); rerunButton->setFixedHeight(51) ;
    reloadButton->setFixedWidth(151); reloadButton->setFixedHeight(51) ;

    

    rightLayout->addWidget(listView) ;
    rightLayout->addWidget(spinBox) ;
    rightLayout->addSpacing(10) ; // 間距
    rightLayout->addWidget(rerunButton) ;
    rightLayout->addWidget(reloadButton) ;

    mainLayout->addWidget(graphicsView);
    mainLayout->addLayout(rightLayout);
    mainLayout->setStretch(0, 1); // graphicsView 會自動拉伸
    mainLayout->setStretch(1, 0); // 右側 Layout 保持元件原始寬度

    setWidgetText(D2D_Gui) ;
    reloadLayoutResult(resultDirectory, designRulePath) ; 


    QObject::connect(reloadButton, &QPushButton::clicked, 
        [this](){ 
            reloadLayoutResult(resultDirectory, designRulePath) ; 
        }
    );

    QObject::connect(rerunButton, &QPushButton::clicked, 
        [this](){ 
            retunExecutable(executionFile, designBumpPath, designRulePath, resultDirectory) ;
            reloadLayoutResult(resultDirectory, designRulePath) ; 
        }
    );

    QObject::connect(graphicsView, &QtView::mouseMoved, 
        [this](const QPointF &pos) {
            QString text = QString("X: %1, Y: %2 |").arg(pos.x(), 0, 'f', 1).arg(pos.y(), 0, 'f', 1);
            posLabel->setText(text);
        }
    );

    QObject::connect(graphicsView, &QtView::mouseQuicrPressAndRelease, 
        [this](const vector<QPointF> &poss) {
            QString text ; 
            if(poss.size()==1){
                text = QString("Selected Position: (%1, %2)").arg(poss[0].x(), 0, 'f', 1).arg(poss[0].y(), 0, 'f', 1);
            }else if(poss.size()==2){
                double distance = sqrt(pow(poss[1].x()-poss[0].x(), 2)+pow(poss[1].y()-poss[0].y(), 2)) ; 
                text = QString("Selected Position: (%1, %2) (%3, %4) | Distance : %5").arg(poss[0].x(), 0, 'f', 1).arg(poss[0].y(), 0, 'f', 1).arg(poss[1].x(), 0, 'f', 1).arg(poss[1].y(), 0, 'f', 1).arg(distance, 0, 'f', 1) ;
            }
            distLabel->setText(text);
        }
    );

    QObject::connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
        [this](int layer) {
            changeLayer(layer) ;
        }
    );

    QObject::connect(layerModel, &QStandardItemModel::itemChanged, 
        [this](QStandardItem *item){
            if(get<1>(layerItemMapping[item]) == "ALL"){
                int layer = get<0>(layerItemMapping[item]) ; 
                for(auto& [subItem, groupType] : layerItemMapping){
                    if(get<0>(groupType) == layer && get<1>(groupType) !="ALL"){
                        subItem->setCheckState(item->checkState()) ;
                    }
                }
                backupStates[layerItemMapping[item]] = item->checkState() ;
            }else{
                backupStates[layerItemMapping[item]] = item->checkState() ;
                setLayerVisible(layerItemMapping[item], item->checkState()) ;
            }
        }
    )  ;
}

void D2DGui::setWidgetText(QMainWindow *D2D_Gui){
    D2D_Gui->setWindowTitle(QCoreApplication::translate("D2D_Gui", "D2D_Gui", nullptr));
    actionOpen_result_folder->setText(QCoreApplication::translate("D2D_Gui", "Open result folder", nullptr));
    actionOpen_execution_file->setText(QCoreApplication::translate("D2D_Gui", "Open execution file", nullptr));
    reloadButton->setText(QCoreApplication::translate("D2D_Gui", "Reload", nullptr));
    rerunButton->setText(QCoreApplication::translate("D2D_Gui", "Rerun", nullptr));
} 

void D2DGui::changeLayer(int layer){
    graphicsView->setScene(sceneces[layer]) ;
    layerModel->clear() ; 
    layerItemMapping.clear() ;
    
    graphicsItemControlTable[{layer, "ALL"}] = {} ; 
    for(auto& [group, graphicsItem] : graphicsItemControlTable){
        if(layer == get<0>(group)){
            QStandardItem *newItem = new QStandardItem() ;
            Qt::CheckState visible ; 

            layerItemMapping[newItem] = group ; 
            newItem->setText(QString::fromStdString(get<1>(group))) ; newItem->setCheckable(true) ;  

            if(backupStates.contains(group)) visible = backupStates[group] ;
            else visible = backupStates[group] = Qt::CheckState::Checked ;

            newItem->setCheckState(visible) ;
            layerModel->appendRow(newItem) ;
        }
    }

}

void D2DGui::setLayerVisible(const GroupType& group, Qt::CheckState visible) {
    for(auto& graphicsItem : graphicsItemControlTable[group]){
        if(visible==Qt::CheckState::Checked) graphicsItem->show() ; 
        else graphicsItem->hide() ;
    }
}
void D2DGui::retunExecutable(const string& executionFile, const string& designBumpPath, const string& designRulePath, const string& resultDirectory){
    pid_t pid = fork();
    if(pid==0){
        execlp( executionFile.c_str(), executionFile.c_str(), designBumpPath.c_str(), designRulePath.c_str(), resultDirectory.c_str(),
                "-p", "300", "-g", "200", "-m", "1", "-c", "1", "-t", "4", "-r", 10, NULL) ;
        // Example: ./bin/D2D testcase/standard_44/bumps.loc testcase/standard_44/rule.txt ./result -p 300 -g 200 -m 1 -c 1
    }else{
        cout << "Wating result...\n" ; 
        wait(NULL) ;
        cout << "Parent: Sub-process finished.\n" ;
    }
}

void D2DGui::reloadLayoutResult(const string& resultDirectory, const string& designRulePath) {
    vector<string> RDLDirectories ; 
    for(auto& dir : filesystem::directory_iterator(resultDirectory)){
        if(string(dir.path()).find("RDL")==string::npos) throw runtime_error("Invalid folder " + string(dir.path())) ;
        RDLDirectories.push_back(dir.path()) ; 
    }

    if(!RDLDirectories.size()){
        cerr << "Empty folder " << resultDirectory ; return ;
    }else{
        sort(RDLDirectories.begin(), RDLDirectories.end()) ;
    }

    for(int i=0; i<sceneces.size(); ++i) delete sceneces[i] ; sceneces.clear() ; 
    sceneces.resize(RDLDirectories.size()) ;
    
    for(int i=0; i<sceneces.size(); i++){
        sceneces[i] = new QGraphicsScene() ; sceneces[i]->setSceneRect(-100, -100, 5000, 2000) ;
    }

    // for(auto& [_, ptrs] : graphicsItemControlTable){
    //     for(auto& ptr : ptrs){
    //         delete ptr ; 
    //     }
    // }

    graphicsItemControlTable.clear() ; 
    layerItemMapping.clear() ; 
    backupStates.clear() ; 

    string filePath, bumpFilePath, netFilePath ; 
    DesignRule designRule ; parse_design_rule(designRulePath, designRule) ;
    Drawer drawer ; drawer.set_design_rule(designRule) ; 


    box_xy boundary ; 
    vector<Label> labels ; 
    vector<Point> points ; 
    vector<Edge> edges ; 
    vector<Bump> bumps ; 
    vector<OffsetVia> offsetVias ; 
    vector<Net> nets ; 
    
    for(int i=0; i<RDLDirectories.size(); i++){
        
        string directory = RDLDirectories[i] ;
        int layerNumber = i ; // stoi(get_trailing_number(directory)) ; 
        set<string> mainFileNames ; 

        for(const auto& entry : std::filesystem::directory_iterator(directory)) {
            mainFileNames.insert(get_string_before(entry.path().filename(), '.')) ; // get the main filenames 
        } 
        
        drawer.set_design_scence(sceneces[i]) ;

        for(auto& mainFileName : mainFileNames){
            bumpFilePath = directory + "/" + mainFileName + ".bump" ; 
            netFilePath = directory + "/" + mainFileName + ".net" ;

            if(filesystem::exists(bumpFilePath) && filesystem::exists(netFilePath)){
                GroupType group = {layerNumber, mainFileName} ;
                parse_design_bump(bumpFilePath, bumps, boundary) ;
                parse_net(netFilePath, nets) ;
                drawer.draw_violations(bumps, nets, graphicsItemControlTable[group]) ; 
            }
        }

        for(auto& extension : {".label", ".point", ".edge", ".net", ".bump", ".offset_via", ".teardrop"}){
            for(auto& mainFileName : mainFileNames){
                filePath = directory + "/" + mainFileName + extension ; 
                GroupType group = {layerNumber, mainFileName} ;
                if(filesystem::exists(filePath)){
                    if(extension==".label"){
                        parse_label(filePath, labels) ;
                        for(auto& label : labels) drawer.draw_label(label, graphicsItemControlTable[group] ) ; 
                    }else if(extension==".point"){
                        parse_point(filePath, points) ;
                        for(auto& point : points) drawer.draw_point(point, graphicsItemControlTable[group] ) ; 
                    }else if(extension==".edge"){
                        parse_edge(filePath, edges) ;
                        for(auto& edge : edges) drawer.draw_edge(edge, graphicsItemControlTable[group] ) ; 
                    }else if(extension==".bump"){
                        parse_design_bump(filePath, bumps, boundary) ;
                        for(auto& bump : bumps) drawer.draw_bump(bump, graphicsItemControlTable[group] ) ; 
                    }else if(extension==".offset_via"){
                        parse_offset_via(filePath, offsetVias) ;
                        for(auto& offsetVia : offsetVias) drawer.draw_offset_via(offsetVia, graphicsItemControlTable[group] ) ; 
                    }else if(extension==".net"){
                        parse_net(filePath, nets) ;
                        
                        for(auto& net : nets) drawer.draw_net(net, graphicsItemControlTable[group]) ; 

                    }else if(extension==".teardrop"){
                    }
            
                }
            }
        }
    }

    graphicsView->setScene(sceneces[0]) ; graphicsView->centerOn(0, 0) ;
    spinBox->setRange(0, sceneces.size()-1) ; spinBox->setValue(0) ; 
    changeLayer(0) ;
}