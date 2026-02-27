/********************************************************************************
** Form generated from reading UI file 'D2D_Gui.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_D2D_GUI_H
#define UI_D2D_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_D2D_Gui
{
public:
    QAction *actionOpen_result_folder;
    QAction *actionOpen_execution_file;
    QWidget *centralwidget;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QListView *listView_2;
    QGraphicsView *graphicsView;
    QSpinBox *spinBox;
    QStatusBar *statusbar;
    QMenuBar *menubar;
    QMenu *menuFile;

    void setupUi(QMainWindow *D2D_Gui)
    {
        if (D2D_Gui->objectName().isEmpty())
            D2D_Gui->setObjectName(QString::fromUtf8("D2D_Gui"));
        D2D_Gui->resize(1222, 601);
        D2D_Gui->setMouseTracking(true);
        actionOpen_result_folder = new QAction(D2D_Gui);
        actionOpen_result_folder->setObjectName(QString::fromUtf8("actionOpen_result_folder"));
        actionOpen_execution_file = new QAction(D2D_Gui);
        actionOpen_execution_file->setObjectName(QString::fromUtf8("actionOpen_execution_file"));
        centralwidget = new QWidget(D2D_Gui);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(1060, 440, 151, 51));
        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(1060, 500, 151, 51));
        listView_2 = new QListView(centralwidget);
        listView_2->setObjectName(QString::fromUtf8("listView_2"));
        listView_2->setGeometry(QRect(1060, 10, 151, 371));
        graphicsView = new QGraphicsView(centralwidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(15, 11, 1031, 541));
        spinBox = new QSpinBox(centralwidget);
        spinBox->setObjectName(QString::fromUtf8("spinBox"));
        spinBox->setGeometry(QRect(1060, 392, 151, 41));
        D2D_Gui->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(D2D_Gui);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        D2D_Gui->setStatusBar(statusbar);
        menubar = new QMenuBar(D2D_Gui);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1222, 19));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        D2D_Gui->setMenuBar(menubar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionOpen_result_folder);
        menuFile->addAction(actionOpen_execution_file);

        retranslateUi(D2D_Gui);

        QMetaObject::connectSlotsByName(D2D_Gui);
    } // setupUi

    void retranslateUi(QMainWindow *D2D_Gui)
    {
        D2D_Gui->setWindowTitle(QCoreApplication::translate("D2D_Gui", "D2D_Gui", nullptr));
        actionOpen_result_folder->setText(QCoreApplication::translate("D2D_Gui", "Open result folder", nullptr));
        actionOpen_execution_file->setText(QCoreApplication::translate("D2D_Gui", "Open execution file", nullptr));
        pushButton->setText(QCoreApplication::translate("D2D_Gui", "Rerun", nullptr));
        pushButton_2->setText(QCoreApplication::translate("D2D_Gui", "Reload", nullptr));
#if QT_CONFIG(tooltip)
        listView_2->setToolTip(QCoreApplication::translate("D2D_Gui", "<html><head/><body><p>123</p><p>2121</p><p>123</p></body></html>", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        listView_2->setWhatsThis(QCoreApplication::translate("D2D_Gui", "123", nullptr));
#endif // QT_CONFIG(whatsthis)
        menuFile->setTitle(QCoreApplication::translate("D2D_Gui", "File", nullptr));
    } // retranslateUi

};

namespace Ui {
    class D2D_Gui: public Ui_D2D_Gui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_D2D_GUI_H
