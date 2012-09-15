/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Sep 14 15:42:01 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTableWidget>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionSave_Current;
    QWidget *centralWidget;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label_8;
    QLineEdit *edit_text_roll;
    QLabel *label_9;
    QLineEdit *edit_text_pitch;
    QLabel *label_10;
    QLineEdit *edit_text_yaw;
    QSpacerItem *horizontalSpacer;
    QPushButton *add_button;
    QLabel *label_2;
    QLabel *label_7;
    QLabel *label;
    QTableWidget *table_widget;
    QPushButton *compute_button;
    QLabel *label_15;
    QLabel *label_4;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_6;
    QLineEdit *edit_text_resolution;
    QPushButton *visualise_button;
    QWidget *layoutWidget_2;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_offset_roll;
    QLineEdit *edit_text_offset_roll;
    QLabel *label_offset_pitch;
    QLineEdit *edit_text_offset_pitch;
    QLabel *label_offset_yaw;
    QLineEdit *edit_text_offset_yaw;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_11;
    QCheckBox *tool_offset_enabled;
    QWidget *widget2;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_offset_x;
    QLineEdit *edit_text_offset_x;
    QLabel *label_offset_y;
    QLineEdit *edit_text_offset_y;
    QLabel *label_offset_z;
    QLineEdit *edit_text_offset_z;
    QWidget *widget3;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;
    QWidget *widget4;
    QHBoxLayout *horizontalLayout_11;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_5;
    QLabel *label_19;
    QHBoxLayout *horizontalLayout_10;
    QVBoxLayout *verticalLayout_3;
    QLineEdit *edit_text_origin_x;
    QLineEdit *edit_text_size_x;
    QSpacerItem *horizontalSpacer_2;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *edit_text_origin_y;
    QLineEdit *edit_text_size_y;
    QSpacerItem *horizontalSpacer_3;
    QVBoxLayout *verticalLayout;
    QLineEdit *edit_text_origin_z;
    QLineEdit *edit_text_size_z;
    QLabel *frame_id_label;
    QLabel *name_label;
    QMenuBar *menuBar;
    QMenu *menuMoveIt_Kinematics_Tool;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(442, 828);
        actionSave_Current = new QAction(MainWindow);
        actionSave_Current->setObjectName(QString::fromUtf8("actionSave_Current"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(60, 190, 321, 29));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout->addWidget(label_8);

        edit_text_roll = new QLineEdit(layoutWidget);
        edit_text_roll->setObjectName(QString::fromUtf8("edit_text_roll"));

        horizontalLayout->addWidget(edit_text_roll);

        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout->addWidget(label_9);

        edit_text_pitch = new QLineEdit(layoutWidget);
        edit_text_pitch->setObjectName(QString::fromUtf8("edit_text_pitch"));

        horizontalLayout->addWidget(edit_text_pitch);

        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout->addWidget(label_10);

        edit_text_yaw = new QLineEdit(layoutWidget);
        edit_text_yaw->setObjectName(QString::fromUtf8("edit_text_yaw"));

        horizontalLayout->addWidget(edit_text_yaw);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        add_button = new QPushButton(layoutWidget);
        add_button->setObjectName(QString::fromUtf8("add_button"));

        horizontalLayout->addWidget(add_button);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(41, 51, 64, 17));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(60, 160, 91, 17));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(41, 21, 43, 17));
        table_widget = new QTableWidget(centralWidget);
        table_widget->setObjectName(QString::fromUtf8("table_widget"));
        table_widget->setGeometry(QRect(60, 220, 321, 141));
        compute_button = new QPushButton(centralWidget);
        compute_button->setObjectName(QString::fromUtf8("compute_button"));
        compute_button->setGeometry(QRect(40, 690, 341, 27));
        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(40, 130, 66, 17));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_15->setFont(font);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(40, 480, 81, 17));
        label_4->setFont(font);
        widget = new QWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(60, 620, 171, 29));
        horizontalLayout_2 = new QHBoxLayout(widget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_6 = new QLabel(widget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_2->addWidget(label_6);

        edit_text_resolution = new QLineEdit(widget);
        edit_text_resolution->setObjectName(QString::fromUtf8("edit_text_resolution"));

        horizontalLayout_2->addWidget(edit_text_resolution);

        visualise_button = new QPushButton(centralWidget);
        visualise_button->setObjectName(QString::fromUtf8("visualise_button"));
        visualise_button->setGeometry(QRect(260, 620, 121, 27));
        layoutWidget_2 = new QWidget(centralWidget);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(60, 440, 261, 29));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget_2);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        label_offset_roll = new QLabel(layoutWidget_2);
        label_offset_roll->setObjectName(QString::fromUtf8("label_offset_roll"));

        horizontalLayout_8->addWidget(label_offset_roll);

        edit_text_offset_roll = new QLineEdit(layoutWidget_2);
        edit_text_offset_roll->setObjectName(QString::fromUtf8("edit_text_offset_roll"));

        horizontalLayout_8->addWidget(edit_text_offset_roll);

        label_offset_pitch = new QLabel(layoutWidget_2);
        label_offset_pitch->setObjectName(QString::fromUtf8("label_offset_pitch"));

        horizontalLayout_8->addWidget(label_offset_pitch);

        edit_text_offset_pitch = new QLineEdit(layoutWidget_2);
        edit_text_offset_pitch->setObjectName(QString::fromUtf8("edit_text_offset_pitch"));

        horizontalLayout_8->addWidget(edit_text_offset_pitch);

        label_offset_yaw = new QLabel(layoutWidget_2);
        label_offset_yaw->setObjectName(QString::fromUtf8("label_offset_yaw"));

        horizontalLayout_8->addWidget(label_offset_yaw);

        edit_text_offset_yaw = new QLineEdit(layoutWidget_2);
        edit_text_offset_yaw->setObjectName(QString::fromUtf8("edit_text_offset_yaw"));

        horizontalLayout_8->addWidget(edit_text_offset_yaw);

        widget1 = new QWidget(centralWidget);
        widget1->setObjectName(QString::fromUtf8("widget1"));
        widget1->setGeometry(QRect(60, 380, 176, 23));
        horizontalLayout_3 = new QHBoxLayout(widget1);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_11 = new QLabel(widget1);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        horizontalLayout_3->addWidget(label_11);

        tool_offset_enabled = new QCheckBox(widget1);
        tool_offset_enabled->setObjectName(QString::fromUtf8("tool_offset_enabled"));

        horizontalLayout_3->addWidget(tool_offset_enabled);

        widget2 = new QWidget(centralWidget);
        widget2->setObjectName(QString::fromUtf8("widget2"));
        widget2->setGeometry(QRect(60, 410, 261, 29));
        horizontalLayout_7 = new QHBoxLayout(widget2);
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        label_offset_x = new QLabel(widget2);
        label_offset_x->setObjectName(QString::fromUtf8("label_offset_x"));

        horizontalLayout_7->addWidget(label_offset_x);

        edit_text_offset_x = new QLineEdit(widget2);
        edit_text_offset_x->setObjectName(QString::fromUtf8("edit_text_offset_x"));

        horizontalLayout_7->addWidget(edit_text_offset_x);

        label_offset_y = new QLabel(widget2);
        label_offset_y->setObjectName(QString::fromUtf8("label_offset_y"));

        horizontalLayout_7->addWidget(label_offset_y);

        edit_text_offset_y = new QLineEdit(widget2);
        edit_text_offset_y->setObjectName(QString::fromUtf8("edit_text_offset_y"));

        horizontalLayout_7->addWidget(edit_text_offset_y);

        label_offset_z = new QLabel(widget2);
        label_offset_z->setObjectName(QString::fromUtf8("label_offset_z"));

        horizontalLayout_7->addWidget(label_offset_z);

        edit_text_offset_z = new QLineEdit(widget2);
        edit_text_offset_z->setObjectName(QString::fromUtf8("edit_text_offset_z"));

        horizontalLayout_7->addWidget(edit_text_offset_z);

        widget3 = new QWidget(centralWidget);
        widget3->setObjectName(QString::fromUtf8("widget3"));
        widget3->setGeometry(QRect(110, 510, 271, 20));
        horizontalLayout_9 = new QHBoxLayout(widget3);
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        horizontalLayout_9->setContentsMargins(0, 0, 0, 0);
        label_20 = new QLabel(widget3);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(label_20);

        label_21 = new QLabel(widget3);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(label_21);

        label_22 = new QLabel(widget3);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(label_22);

        widget4 = new QWidget(centralWidget);
        widget4->setObjectName(QString::fromUtf8("widget4"));
        widget4->setGeometry(QRect(60, 530, 321, 66));
        horizontalLayout_11 = new QHBoxLayout(widget4);
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        horizontalLayout_11->setContentsMargins(0, 0, 0, 0);
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_5 = new QLabel(widget4);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_4->addWidget(label_5);

        label_19 = new QLabel(widget4);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        verticalLayout_4->addWidget(label_19);


        horizontalLayout_11->addLayout(verticalLayout_4);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        edit_text_origin_x = new QLineEdit(widget4);
        edit_text_origin_x->setObjectName(QString::fromUtf8("edit_text_origin_x"));

        verticalLayout_3->addWidget(edit_text_origin_x);

        edit_text_size_x = new QLineEdit(widget4);
        edit_text_size_x->setObjectName(QString::fromUtf8("edit_text_size_x"));

        verticalLayout_3->addWidget(edit_text_size_x);


        horizontalLayout_10->addLayout(verticalLayout_3);

        horizontalSpacer_2 = new QSpacerItem(50, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_2);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        edit_text_origin_y = new QLineEdit(widget4);
        edit_text_origin_y->setObjectName(QString::fromUtf8("edit_text_origin_y"));

        verticalLayout_2->addWidget(edit_text_origin_y);

        edit_text_size_y = new QLineEdit(widget4);
        edit_text_size_y->setObjectName(QString::fromUtf8("edit_text_size_y"));

        verticalLayout_2->addWidget(edit_text_size_y);


        horizontalLayout_10->addLayout(verticalLayout_2);

        horizontalSpacer_3 = new QSpacerItem(50, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_3);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        edit_text_origin_z = new QLineEdit(widget4);
        edit_text_origin_z->setObjectName(QString::fromUtf8("edit_text_origin_z"));

        verticalLayout->addWidget(edit_text_origin_z);

        edit_text_size_z = new QLineEdit(widget4);
        edit_text_size_z->setObjectName(QString::fromUtf8("edit_text_size_z"));

        verticalLayout->addWidget(edit_text_size_z);


        horizontalLayout_10->addLayout(verticalLayout);


        horizontalLayout_11->addLayout(horizontalLayout_10);

        frame_id_label = new QLabel(centralWidget);
        frame_id_label->setObjectName(QString::fromUtf8("frame_id_label"));
        frame_id_label->setGeometry(QRect(129, 51, 161, 27));
        name_label = new QLabel(centralWidget);
        name_label->setObjectName(QString::fromUtf8("name_label"));
        name_label->setGeometry(QRect(129, 21, 161, 27));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 442, 25));
        menuMoveIt_Kinematics_Tool = new QMenu(menuBar);
        menuMoveIt_Kinematics_Tool->setObjectName(QString::fromUtf8("menuMoveIt_Kinematics_Tool"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuMoveIt_Kinematics_Tool->menuAction());
        menuMoveIt_Kinematics_Tool->addAction(actionSave_Current);

        retranslateUi(MainWindow);
        QObject::connect(add_button, SIGNAL(clicked()), MainWindow, SLOT(addRow()));
        QObject::connect(compute_button, SIGNAL(clicked()), MainWindow, SLOT(compute()));
        QObject::connect(tool_offset_enabled, SIGNAL(clicked(bool)), MainWindow, SLOT(showOffset(bool)));
        QObject::connect(visualise_button, SIGNAL(clicked()), MainWindow, SLOT(visualiseWorkspace()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionSave_Current->setText(QApplication::translate("MainWindow", "Save Current", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "R:", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "P:", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "Y:", 0, QApplication::UnicodeUTF8));
        add_button->setText(QApplication::translate("MainWindow", "+", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Frame ID:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "Orientation:", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Name:", 0, QApplication::UnicodeUTF8));
        compute_button->setText(QApplication::translate("MainWindow", "Compute", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "Arm:", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Workspace", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Resolution:", 0, QApplication::UnicodeUTF8));
        visualise_button->setText(QApplication::translate("MainWindow", "Visualise Points", 0, QApplication::UnicodeUTF8));
        label_offset_roll->setText(QApplication::translate("MainWindow", "R:", 0, QApplication::UnicodeUTF8));
        label_offset_pitch->setText(QApplication::translate("MainWindow", "P:", 0, QApplication::UnicodeUTF8));
        label_offset_yaw->setText(QApplication::translate("MainWindow", "Y:", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "Tool Offset (optional):", 0, QApplication::UnicodeUTF8));
        tool_offset_enabled->setText(QString());
        label_offset_x->setText(QApplication::translate("MainWindow", "X:", 0, QApplication::UnicodeUTF8));
        label_offset_y->setText(QApplication::translate("MainWindow", "Y:", 0, QApplication::UnicodeUTF8));
        label_offset_z->setText(QApplication::translate("MainWindow", "Z:", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("MainWindow", "Z", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Origin:", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("MainWindow", "Size:", 0, QApplication::UnicodeUTF8));
        frame_id_label->setText(QString());
        name_label->setText(QString());
        menuMoveIt_Kinematics_Tool->setTitle(QApplication::translate("MainWindow", "MoveIt! Kinematics Tool", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
