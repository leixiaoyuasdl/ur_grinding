#include <QVBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>
#include <QWidget>
#include <QTabWidget>

#include "ur_grinding_rviz_plugin.h"

namespace fanuc_grinding_rviz_plugin
{
    FanucGrindingRvizPlugin::FanucGrindingRvizPlugin(QWidget* parent) :
            rviz::Panel(parent)
    {
        // Create Tabs
        tab_widget_ = new QTabWidget();
        scanning_widget_ = new ScanningWidget();
        path_planning_widget_ = new PathPlanningWidget();
        tab_widget_->addTab(scanning_widget_, "Scanning");
        tab_widget_->addTab(path_planning_widget_, "Path planning");
        tab_widget_->setTabEnabled(0, true);
        tab_widget_->setTabEnabled(1, false);

        // Bottom status layout
        QVBoxLayout* status_layout = new QVBoxLayout;
        status_layout->addWidget(new QLabel("Status:"));

        // Global Layout
        QVBoxLayout* global_layout = new QVBoxLayout;
        global_layout->addWidget(tab_widget_);
        global_layout->addLayout(status_layout);
        status_label_ = new QLabel;
        global_layout->addWidget(status_label_);
        setLayout(global_layout);

        // Connect handlers
        // SCANNING
        // Will display a status in general status label ( from scanning widget )
        connect(scanning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
        connect(scanning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
                SLOT(displayMsgBoxHandler(QString, QString, QString)));

        // Call configChanged at each time that scanning_widget_ is modified
        connect(scanning_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
        // Enable general panel when scanning_widget_ send the SIGNAL
        connect(scanning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));

        // Will send information about cad and scan in the other widgets
        connect(scanning_widget_, SIGNAL(sendCADDatas(QString)), this, SLOT(setCADDatas(QString)));
        connect(scanning_widget_, SIGNAL(sendScanDatas(QString)), this, SLOT(setScanDatas(QString)));
        // For the demonstrator, we will skip alignment and comparison parts for the moment
        connect(scanning_widget_, SIGNAL(enablePanelPathPlanning()), this, SLOT(enablePanelPathPlanningHandler()));

        

        //PATH PLANNING
        // Will display a status in general status label ( from path_planning widget )
        connect(path_planning_widget_, SIGNAL(sendStatus(QString)), this, SLOT(displayStatusHandler(QString)));
        connect(path_planning_widget_, SIGNAL(sendMsgBox(QString, QString , QString)), this,
                SLOT(displayMsgBoxHandler(QString, QString, QString)));
        // Call configChanged at each time that path_planning_widget is modified
        connect(path_planning_widget_, SIGNAL(guiChanged()), this, SLOT(triggerSave()));
        // Enable general panel when path_planning send the SIGNAL
        connect(path_planning_widget_, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
        // Received a signal from comparison widget in order to get CAD and scan params
        connect(path_planning_widget_, SIGNAL(getCADAndScanParams()), this, SLOT(sendCADAndScanDatasSlot()));
        // Send a signal to comparison widget in order to give CAD and scan params
        connect(this, SIGNAL(sendCADAndScanDatas(const QString, const QString)),
                path_planning_widget_, SLOT(setCADAndScanParams(const QString, const QString)));

        connect(this, SIGNAL(displayStatus(const QString)), this, SLOT(displayStatusHandler(const QString)));
    }

    FanucGrindingRvizPlugin::~FanucGrindingRvizPlugin()
    {}

    void FanucGrindingRvizPlugin::enablePanelPathPlanningHandler()
    {
        tab_widget_->setTabEnabled(1, true);
    }

    void FanucGrindingRvizPlugin::enablePanelHandler(const bool status)
    {
        setEnabled(status);
    }

    void FanucGrindingRvizPlugin::displayStatusHandler(const QString message)
    {
        status_label_->setText(message);
    }

    void FanucGrindingRvizPlugin::displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg)
    {
        enablePanelHandler(false);
        QMessageBox msg_box;
        msg_box.setWindowTitle(title);
        msg_box.setText(msg);
        msg_box.setInformativeText(info_msg);
        msg_box.setIcon(QMessageBox::Critical);
        msg_box.setStandardButtons(QMessageBox::Ok);
        msg_box.exec();
        enablePanelHandler(true);
    }

    void FanucGrindingRvizPlugin::triggerSave()
    {
        Q_EMIT configChanged();
    }

    void FanucGrindingRvizPlugin::setCADDatas(const QString cad_filename)
    {
        cad_filename_ = cad_filename;
    }

    void FanucGrindingRvizPlugin::setScanDatas(const QString scan_filename)
    {
        scan_filename_ = scan_filename;
    }


    void FanucGrindingRvizPlugin::sendCADAndScanDatasSlot()
    {
        Q_EMIT sendCADAndScanDatas(cad_filename_, scan_filename_);
    }

// Save all configuration data from this panel to the given Config object
    void FanucGrindingRvizPlugin::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
        scanning_widget_->save(config);
        path_planning_widget_->save(config);
    }

// Load all configuration data for this panel from the given Config object.
    void FanucGrindingRvizPlugin::load(const rviz::Config& config)
    {
        rviz::Panel::load(config);
        scanning_widget_->load(config);
        path_planning_widget_->load(config);
    }

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fanuc_grinding_rviz_plugin::FanucGrindingRvizPlugin, rviz::Panel)
