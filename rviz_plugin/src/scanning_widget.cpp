#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTabWidget>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QFileDialog>
#include <QFileInfo>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>

#include "scanning_widget.h"
using namespace std;
fanuc_grinding_rviz_plugin::ScanningWidget::ScanningWidget(QWidget* parent) :
    QWidget(parent),
    package_name_("fanuc_grinding_scanning")
{
  setObjectName("ScanningWidget_");
  QLabel* cad_meshname_label = new QLabel("CAD mesh file:");
  cad_meshname_ = new QLineEdit;
  cad_meshname_->setReadOnly(true);
  cad_meshname_browse_button_ = new QPushButton("...");
  cad_meshname_browse_button_->setMaximumSize(QSize(30,30));
  QHBoxLayout* cad_meshname_layout = new QHBoxLayout;
  cad_meshname_layout->addWidget(cad_meshname_);
  cad_meshname_layout->addWidget(cad_meshname_browse_button_);

  QLabel* cad_marker_name_label = new QLabel("CAD marker name:");
  cad_marker_name_line_ = new QLineEdit;
  QHBoxLayout* cad_marker_name_layout = new QHBoxLayout;
  cad_marker_name_layout->addWidget(cad_marker_name_label);
  cad_marker_name_layout->addWidget(cad_marker_name_line_);

  import_cad_button_ = new QPushButton("Import CAD file");

  scan_choice_container_ = new QWidget;
  scan_choice_container_->setEnabled(true);
  QVBoxLayout* scan_choice_layout = new QVBoxLayout(scan_choice_container_);

  scan_choice_widget_ = new QTabWidget;
  scan_choice_layout->addWidget(scan_choice_widget_);
 // start_scan_tab_ = new QWidget();
  import_scan_tab_ = new QWidget();
  scan_choice_widget_->addTab(import_scan_tab_, "Import point cloud");
 // scan_choice_widget_->addTab(start_scan_tab_, "Scan");

//  // Import a yaml file to load robot's joint states and start scan
//  QLabel* trajectory_yaml_label = new QLabel("Scan trajectory YAML file:");
//  traj_yaml_file_ = new QLineEdit;
//  traj_yaml_file_->setReadOnly(true);
//  traj_yaml_browse_button_ = new QPushButton("...");
//  traj_yaml_browse_button_->setMaximumSize(QSize(30,30));
//  QHBoxLayout* traj_yaml_layout = new QHBoxLayout;
//  traj_yaml_layout->addWidget(traj_yaml_file_);
//  traj_yaml_layout->addWidget(traj_yaml_browse_button_);
//  // Parameters of the SLS-2
//  QLabel* sls_2_server_name_label = new QLabel("SLS-2 server name:");
//  sls_2_server_name_ = new QLineEdit;
//  QHBoxLayout* sls_2_server_name_layout = new QHBoxLayout;
//  sls_2_server_name_layout->addWidget(sls_2_server_name_label);
//  sls_2_server_name_layout->addWidget(sls_2_server_name_);
//  QLabel* sls_2_ip_address_label = new QLabel("SLS-2 IP address:");
//  sls_2_ip_address_ = new QLineEdit;
//  sls_2_ip_address_->setInputMask("000.000.000.000;_");
//  QHBoxLayout* sls_2_ip_address_layout = new QHBoxLayout;
//  sls_2_ip_address_layout->addWidget(sls_2_ip_address_label);
//  sls_2_ip_address_layout->addWidget(sls_2_ip_address_);
//  // Import SLS-2 calibration matrix
//  QLabel* calibration_yaml_label = new QLabel("SLS-2 calibration YAML file:");
//  calibration_yaml_file_ = new QLineEdit;
//  calibration_yaml_file_->setReadOnly(true);
//  calibration_yaml_browse_button_ = new QPushButton("...");
//  calibration_yaml_browse_button_->setMaximumSize(QSize(30,30));
//  QHBoxLayout* calibration_yaml_layout = new QHBoxLayout;
//  calibration_yaml_layout->addWidget(calibration_yaml_file_);
//  calibration_yaml_layout->addWidget(calibration_yaml_browse_button_);

//  QVBoxLayout* scan_yaml_widget_layout = new QVBoxLayout(start_scan_tab_);
//  scan_yaml_widget_layout->addWidget(trajectory_yaml_label);
//  scan_yaml_widget_layout->addLayout(traj_yaml_layout);
//  scan_yaml_widget_layout->addStretch(1);
//  scan_yaml_widget_layout->addWidget(calibration_yaml_label);
//  scan_yaml_widget_layout->addLayout(calibration_yaml_layout);
//  scan_yaml_widget_layout->addStretch(1);
//  scan_yaml_widget_layout->addLayout(sls_2_server_name_layout);
//  scan_yaml_widget_layout->addLayout(sls_2_ip_address_layout);

//  QLabel* down_sampling_label = new QLabel("Point cloud down-sampling leaf size:");
//  down_sampling_leaf_size_ = new QDoubleSpinBox;
//  down_sampling_leaf_size_->setRange(0.0001, 0.5);
//  down_sampling_leaf_size_->setDecimals(4);
//  down_sampling_leaf_size_->setSingleStep(0.001);
//  down_sampling_leaf_size_->setSuffix(" meters");
//  scan_yaml_widget_layout->addWidget(down_sampling_label);
//  scan_yaml_widget_layout->addWidget(down_sampling_leaf_size_);
//
//  start_scan_ = new QPushButton("Start scanning");
//  scan_yaml_widget_layout->addWidget(start_scan_);

  //Import point cloud from an older scan
  QLabel* scan_mesh_label = new QLabel("Import a point cloud file:");
  scan_file_ = new QLineEdit;
  scan_file_->setReadOnly(true);
  scan_file_browse_button_ = new QPushButton("...");
  scan_file_browse_button_->setMaximumSize(QSize(30,30));
  QHBoxLayout* scan_file_label_layout = new QHBoxLayout;
  scan_file_label_layout->addWidget(scan_file_);
  scan_file_label_layout->addWidget(scan_file_browse_button_);
  import_scan_ = new QPushButton("Import point cloud");
  QVBoxLayout* import_scan_widget_layout = new QVBoxLayout (import_scan_tab_);
  import_scan_widget_layout->addWidget(scan_mesh_label);
  import_scan_widget_layout->addLayout(scan_file_label_layout);
  import_scan_widget_layout->addStretch(1);
  import_scan_widget_layout->addWidget(import_scan_);

  QLabel* scan_marker_name_label = new QLabel("Point_cloud marker name:");
  scan_marker_name_line_ = new QLineEdit;
  QVBoxLayout* scan_marker_name_layout = new QVBoxLayout;
  scan_choice_layout->addLayout(scan_marker_name_layout);
  scan_marker_name_layout->addWidget(scan_marker_name_label);
  scan_marker_name_layout->addWidget(scan_marker_name_line_);

  QVBoxLayout* scanning_layout = new QVBoxLayout(this);
  scanning_layout->addWidget(cad_meshname_label);
  scanning_layout->addLayout(cad_meshname_layout);
  scanning_layout->addLayout(cad_marker_name_layout);
  scanning_layout->addWidget(import_cad_button_);
  scanning_layout->addStretch(2);
  scanning_layout->addWidget(scan_choice_container_);
  scanning_layout->addStretch(1);

  connect(cad_meshname_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
//  connect(traj_yaml_file_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
//  connect(sls_2_server_name_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
//  connect(sls_2_ip_address_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
//  connect(down_sampling_leaf_size_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
//  connect(calibration_yaml_file_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(cad_marker_name_line_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(scan_file_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
  connect(scan_marker_name_line_, SIGNAL(textChanged(QString)), this, SLOT(triggerSave()));
//  connect(start_scan_, SIGNAL(released()), this, SLOT(scanningButtonHandler()));
  connect(cad_meshname_browse_button_, SIGNAL(released()), this, SLOT(browseCADFiles()));
//  connect(traj_yaml_browse_button_, SIGNAL(released()), this, SLOT(browseTrajectoryFiles()));
//  connect(calibration_yaml_browse_button_, SIGNAL(released()), this, SLOT(browseCalibrationFiles()));
  connect(scan_file_browse_button_, SIGNAL(released()), this, SLOT(browseScannedFiles()));
  connect(import_scan_, SIGNAL(released()), this, SLOT(importScanFileButtonHandler()));
  connect(import_cad_button_, SIGNAL(released()), this, SLOT(importCADFileButtonHandler()));

  connect(this, SIGNAL(enableScanWidget()), this, SLOT(enableScanWidgetHandler()));

  // Subscriber to receive messages from the exterior
  status_sub_ = nh_.subscribe("scanning_status", 1, &fanuc_grinding_rviz_plugin::ScanningWidget::newStatusMessage, this);

  // Setup client
  scanning_service_ = nh_.serviceClient<fanuc_grinding_scanning::ScanningService>("scanning_service");
  ros::service::waitForService("publish_meshfile_service");
  publish_meshfile_service_ = nh_.serviceClient<fanuc_grinding_publish_meshfile::PublishMeshfileService>("publish_meshfile_service");

  QFuture<void> future = QtConcurrent::run(this, &fanuc_grinding_rviz_plugin::ScanningWidget::connectToServices);
}

void fanuc_grinding_rviz_plugin::ScanningWidget::newStatusMessage(const std_msgs::String::ConstPtr &msg)
{
  Q_EMIT sendStatus(QString::fromStdString(msg->data));
}

void fanuc_grinding_rviz_plugin::ScanningWidget::setScanningParams(const fanuc_grinding_scanning::ScanningService::Request &params)
{
//  srv_scanning_.request.YamlFileName = params.YamlFileName;
//  srv_scanning_.request.SLS2ServerName = params.SLS2ServerName;
//  srv_scanning_.request.SLS2IpAddress = params.SLS2IpAddress;
//  srv_scanning_.request.YamlCalibrationFileName = params.YamlCalibrationFileName;
  srv_scanning_.request.CADName = params.CADName;
  srv_scanning_.request.MarkerName = params.MarkerName;
  updateGUI();
}

void fanuc_grinding_rviz_plugin::ScanningWidget::setPublishParams(const fanuc_grinding_publish_meshfile::PublishMeshfileService::Request &params)
{
  srv_publish_meshfile_.request.MeshName = params.MeshName;
  srv_publish_meshfile_.request.MarkerName = params.MarkerName;
  srv_publish_meshfile_.request.PosX = params.PosX;
  srv_publish_meshfile_.request.PosY = params.PosY;
  srv_publish_meshfile_.request.PosZ = params.PosZ;
  srv_publish_meshfile_.request.RotX = params.RotX;
  srv_publish_meshfile_.request.RotY = params.RotY;
  srv_publish_meshfile_.request.RotZ = params.RotZ;
  srv_publish_meshfile_.request.RotW = params.RotW;
  srv_publish_meshfile_.request.ColorR = params.ColorR;
  srv_publish_meshfile_.request.ColorG = params.ColorG;
  srv_publish_meshfile_.request.ColorB = params.ColorB;
  srv_publish_meshfile_.request.ColorA = params.ColorA;
  srv_publish_meshfile_.request.WaitForSubscriber = params.WaitForSubscriber;
  updateGUI();
}

void fanuc_grinding_rviz_plugin::ScanningWidget::updateGUI()
{
//  traj_yaml_file_->setText(QString::fromStdString(srv_scanning_.request.YamlFileName));
//  sls_2_server_name_->setText(QString::fromStdString(srv_scanning_.request.SLS2ServerName));
//  sls_2_ip_address_->setText(QString::fromStdString(srv_scanning_.request.SLS2IpAddress));
//  calibration_yaml_file_->setText(QString::fromStdString(srv_scanning_.request.YamlCalibrationFileName));
  cad_meshname_->setText(QString::fromStdString(srv_publish_meshfile_.request.MeshName));
  cad_marker_name_line_->setText(QString::fromStdString(srv_publish_meshfile_.request.MarkerName));
  scan_marker_name_line_->setText(QString::fromStdString(srv_scanning_.request.MarkerName));
 // down_sampling_leaf_size_->setValue(srv_scanning_.request.VoxelGridLeafSize);
}

void fanuc_grinding_rviz_plugin::ScanningWidget::updateInternalValues()
{
//  srv_scanning_.request.YamlFileName = traj_yaml_file_->text().toStdString();
//  srv_scanning_.request.SLS2ServerName = sls_2_server_name_->text().toStdString();
//  srv_scanning_.request.SLS2IpAddress = sls_2_ip_address_->text().toStdString();
//  srv_scanning_.request.YamlCalibrationFileName = calibration_yaml_file_->text().toStdString();
  srv_scanning_.request.MarkerName = scan_marker_name_line_->text().toStdString();
  srv_scanning_.request.CADName = cad_meshname_->text().toStdString();
//  srv_scanning_.request.VoxelGridLeafSize = down_sampling_leaf_size_->value();
  srv_publish_meshfile_.request.MeshName = cad_meshname_->text().toStdString();
  srv_publish_meshfile_.request.MarkerName = cad_marker_name_line_->text().toStdString();
}

void fanuc_grinding_rviz_plugin::ScanningWidget::browseCADFiles()
{
  QFileDialog cad_mesh_browser;
  std::string meshes_path = ros::package::getPath(package_name_) + "/meshes/";
  QString file_path = cad_mesh_browser.getOpenFileName(0, tr("Import File Dialog"), QString::fromStdString(meshes_path), tr("Meshfiles (*.ply *.stl *.obj)"));
  if(file_path != "")
  {
    cad_meshname_->setText(file_path);
  }
}

void fanuc_grinding_rviz_plugin::ScanningWidget::browseTrajectoryFiles()
{
  QFileDialog yaml_browser;
  std::string yaml_path = ros::package::getPath(package_name_) + "/yaml/";
  QString file_path = yaml_browser.getOpenFileName(0, tr("Import File Dialog"), QString::fromStdString(yaml_path), tr("Yaml files (*.yaml)"));
  if(file_path != "")
  {
    traj_yaml_file_->setText(file_path);
  }
}

void fanuc_grinding_rviz_plugin::ScanningWidget::browseCalibrationFiles()
{
  QFileDialog yaml_browser;
  std::string yaml_path = ros::package::getPath(package_name_) + "/yaml/";
  QString file_path = yaml_browser.getOpenFileName(0, tr("Import File Dialog"), QString::fromStdString(yaml_path), tr("Yaml files (*.yaml)"));
  if(file_path != "")
  {
    calibration_yaml_file_->setText(file_path);
  }
}

void fanuc_grinding_rviz_plugin::ScanningWidget::browseScannedFiles()
{
  QFileDialog scan_file_browser;
  QString file_path = scan_file_browser.getOpenFileName(0, tr("Import File Dialog"), "/home/dell/catkin_ws/src/fanuc_grinding/meshes");
  if(file_path != "")
  {
    scan_file_->setText(file_path);
  }
}

void fanuc_grinding_rviz_plugin::ScanningWidget::enableScanWidgetHandler()
{
  scan_choice_container_->setEnabled(true);
}

void fanuc_grinding_rviz_plugin::ScanningWidget::importCADFileButtonHandler()
{
  QFileInfo Fout(cad_meshname_->text());
  if (!Fout.exists())
  {
    Q_EMIT sendMsgBox("Error loading CAD file",
                      "The specified file does not exist",
                      cad_meshname_->text());
    Q_EMIT sendStatus("CAD file does not exist");
    return;
  }

  if (cad_marker_name_line_->text().isEmpty())
  {
    Q_EMIT sendMsgBox("Error publishing CAD marker",
                      "The marker name cannot be empty", "");
    Q_EMIT sendStatus("Marker name is empty");
    return;
  }

  // Fill in the request
  srv_publish_meshfile_.request.MeshName = cad_meshname_->text().toStdString();
  srv_publish_meshfile_.request.MarkerName = cad_marker_name_line_->text().toStdString();
  srv_publish_meshfile_.request.PosX = 1.4;
  srv_publish_meshfile_.request.PosY = 0.0;
  srv_publish_meshfile_.request.PosZ = -0.065;
  srv_publish_meshfile_.request.RotX = 0.0;
  srv_publish_meshfile_.request.RotY = 0.0;
  srv_publish_meshfile_.request.RotZ = 0.0;
  srv_publish_meshfile_.request.RotW = 1.0;
  srv_publish_meshfile_.request.ColorR = 130 / 255.0;
  srv_publish_meshfile_.request.ColorG = 75 / 255.0;
  srv_publish_meshfile_.request.ColorB = 75 / 255.0;
  srv_publish_meshfile_.request.ColorA = 1.0;
  srv_publish_meshfile_.request.WaitForSubscriber = false;

  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &ScanningWidget::publishCADMeshOrCloudFile);
}

void fanuc_grinding_rviz_plugin::ScanningWidget::importScanFileButtonHandler()
{

  if(scan_marker_name_line_->text().isEmpty())
  {
    Q_EMIT sendMsgBox("Error publishing scan marker",
                      "The marker name cannot be empty","");
    Q_EMIT sendStatus("Marker name is empty");
    return;
  }
  else
  {
    // Fill in the request
    srv_publish_meshfile_.request.MeshName = scan_file_->text().toStdString();
    srv_publish_meshfile_.request.MarkerName = scan_marker_name_line_->text().toStdString();
    srv_publish_meshfile_.request.PosX = 0.0;
    srv_publish_meshfile_.request.PosY = 0.0;
    srv_publish_meshfile_.request.PosZ = 0.0;
    srv_publish_meshfile_.request.RotX = 0.0;
    srv_publish_meshfile_.request.RotY = 0.0;
    srv_publish_meshfile_.request.RotZ = 0.0;
    srv_publish_meshfile_.request.RotW = 1.0;
    srv_publish_meshfile_.request.ColorR = 75 / 255.0;
    srv_publish_meshfile_.request.ColorG = 75 / 255.0;
    srv_publish_meshfile_.request.ColorB = 130 / 255.0;
    srv_publish_meshfile_.request.ColorA = 1.0;
    srv_publish_meshfile_.request.WaitForSubscriber = true;

    // Start client service call in an other thread
    QFuture<void> future = QtConcurrent::run(this, &ScanningWidget::publishScanMeshOrCloudFile);
  }
}

void fanuc_grinding_rviz_plugin::ScanningWidget::publishCADMeshOrCloudFile()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service

  publish_meshfile_service_.call(srv_publish_meshfile_);

  Q_EMIT sendStatus(QString::fromStdString(srv_publish_meshfile_.response.ReturnMessage));

  if(srv_publish_meshfile_.response.ReturnStatus == true)
  {
    Q_EMIT enableScanWidget();
    Q_EMIT sendCADDatas(QString::fromStdString(srv_publish_meshfile_.request.MeshName));
  }
  else
  {
    Q_EMIT sendMsgBox("Error importing mesh/point cloud file",
                      QString::fromStdString(srv_publish_meshfile_.response.ReturnMessage),
                      QString::fromStdString(srv_publish_meshfile_.request.MeshName));
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void fanuc_grinding_rviz_plugin::ScanningWidget::publishScanMeshOrCloudFile()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service
  publish_meshfile_service_.call(srv_publish_meshfile_);
  Q_EMIT sendStatus(QString::fromStdString(srv_publish_meshfile_.response.ReturnMessage));

  if(srv_publish_meshfile_.response.ReturnStatus == true)
  {
    //Q_EMIT enablePanelAlignment();
    Q_EMIT enablePanelPathPlanning();
    Q_EMIT sendScanDatas(QString::fromStdString(srv_publish_meshfile_.request.MeshName));
  }
  else
  {
    Q_EMIT sendMsgBox("Error importing mesh/point cloud file",
                      QString::fromStdString(srv_publish_meshfile_.response.ReturnMessage),
                      QString::fromStdString(srv_publish_meshfile_.request.MeshName));
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void fanuc_grinding_rviz_plugin::ScanningWidget::scanningButtonHandler()
{
//  QFileInfo traj_yaml_file(traj_yaml_file_->text());
//  if (!traj_yaml_file.exists())
//  {
//    Q_EMIT sendMsgBox("Error starting scanning",
//                      "The specified trajectory YAML file does not exist", "");
//    Q_EMIT sendStatus("Trajectory YAML file does not exist");
//    return;
//  }

//  QFileInfo calibration_yaml_file(calibration_yaml_file_->text());
//  if (!calibration_yaml_file.exists())
//  {
//    Q_EMIT sendMsgBox("Error starting scanning",
//                      "The specified calibration YAML file does not exist", "");
//    return;
//  }

  if (scan_marker_name_line_->text().isEmpty())
  {
    Q_EMIT sendMsgBox("Error starting scanning", "Please specify the scan marker name", "");
    Q_EMIT sendStatus("Scan marker name is empty");
    return;
  }

//  if(sls_2_server_name_->text().isEmpty())
//  {
//    Q_EMIT sendMsgBox("Error starting scanning",
//                      "Please specify the SLS-2 server name", "");
//    Q_EMIT sendStatus("SLS-2 server name is empty");
//    return;
//  }

  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &ScanningWidget::scanning);
}

void fanuc_grinding_rviz_plugin::ScanningWidget::scanning()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call client service
  scanning_service_.call(srv_scanning_);
  Q_EMIT sendStatus(QString::fromStdString(srv_scanning_.response.ReturnMessage));

  if(srv_scanning_.response.ReturnStatus == true)
  {
    //Q_EMIT enablePanelAlignment();
    Q_EMIT enablePanelPathPlanning();
    Q_EMIT sendScanDatas(QString::fromStdString(srv_scanning_.response.ScanMeshName));
  }
  else
  {
    Q_EMIT sendMsgBox("Error scanning",
                      QString::fromStdString(srv_scanning_.response.ReturnMessage),
                      "");
  }

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void fanuc_grinding_rviz_plugin::ScanningWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT sendStatus("Connecting to service");
  while (ros::ok())
  {
    // We don't check status_sub_ number of publishers, it is not mandatory to have a status publisher.

    if (scanning_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(objectName().toStdString() + " RViz panel connected to the service " << scanning_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + scanning_service_.getService()));
      break;
    }
    else
    {
      ROS_WARN_STREAM(objectName().toStdString() + " RViz panel could not connect to ROS service:\n\t" << scanning_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + scanning_service_.getService()));
      sleep(1);
    }
  }
  while (ros::ok())
  {
    if (publish_meshfile_service_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(objectName().toStdString() +  " RViz panel connected to the service " << publish_meshfile_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + publish_meshfile_service_.getService()));
      break;
    }
    else
    {
      ROS_WARN_STREAM(objectName().toStdString() +  " RViz panel could not connect to ROS service:\n\t" << publish_meshfile_service_.getService());
      Q_EMIT sendStatus(QString::fromStdString("RViz panel connected to the service: " + publish_meshfile_service_.getService()));
      sleep(1);
    }
  }

  ROS_INFO_STREAM(objectName().toStdString() + " service connections have been made");
  Q_EMIT sendStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

void fanuc_grinding_rviz_plugin::ScanningWidget::triggerSave()
{
  Q_EMIT guiChanged();
  updateInternalValues();
  updateGUI();
}

// Save all configuration data from this panel to the given Config object
void fanuc_grinding_rviz_plugin::ScanningWidget::save(rviz::Config config)
{
  // Save offset value into the config file
//  config.mapSetValue(objectName() + "traj_yaml_file", traj_yaml_file_->text());
//  config.mapSetValue(objectName() + "sls_2_server_name", sls_2_server_name_->text());
//  config.mapSetValue(objectName() + "sls_2_ip_address", sls_2_ip_address_->text());
  config.mapSetValue(objectName() + "cad_file", cad_meshname_->text());
  config.mapSetValue(objectName() + "cad_marker_name", cad_marker_name_line_->text());
  config.mapSetValue(objectName() + "scan_marker_name", scan_marker_name_line_->text());
 // config.mapSetValue(objectName() + "calibration_yaml_file", calibration_yaml_file_->text());
  config.mapSetValue(objectName() + "scan_filename", scan_file_->text());
 // config.mapSetValue(objectName() + "down_sampling_leaf_size", down_sampling_leaf_size_->value());
}

// Load all configuration data for this panel from the given Config object.
void fanuc_grinding_rviz_plugin::ScanningWidget::load(const rviz::Config& config)
{
  QString tmp;
  // Load offset value from config file (if it exists)

  if (config.mapGetString(objectName() + "cad_file", &tmp))
    cad_meshname_->setText(tmp);
  else
  {
    std::string meshes_path = ros::package::getPath(package_name_) + "/meshes/";
    cad_meshname_->setText(QString::fromStdString(meshes_path+"DAC_580813.stl"));
  }

  if (config.mapGetString(objectName() + "cad_marker_name", &tmp))
    cad_marker_name_line_->setText(tmp);
  else
    cad_marker_name_line_->setText("cad");

//  if (config.mapGetString(objectName() + "traj_yaml_file", &tmp))
//    traj_yaml_file_->setText(tmp);
//  else
//  {
//    std::string meshes_path = ros::package::getPath(package_name_) + "/yaml/";
//    traj_yaml_file_->setText(QString::fromStdString(meshes_path+"trajectory_DaC.yaml"));
//  }
//
//  if (config.mapGetString(objectName() + "sls_2_server_name", &tmp))
//    sls_2_server_name_->setText(tmp);
//  else
//    sls_2_server_name_->setText("m4500");
//
//  if (config.mapGetString(objectName() + "sls_2_ip_address", &tmp))
//    sls_2_ip_address_->setText(tmp);
//  else
//    sls_2_ip_address_->setText("192.168.100.50");
//
//  if (config.mapGetString(objectName() + "calibration_yaml_file", &tmp))
//    calibration_yaml_file_->setText(tmp);
//  else
//  {
//    std::string yaml_path = ros::package::getPath(package_name_) + "/yaml/";
//    calibration_yaml_file_->setText(QString::fromStdString(yaml_path+"camera_calibration.yaml"));
//  }

  if (config.mapGetString(objectName() + "scan_file", &tmp))
    scan_file_->setText(tmp);
  else
  {
    std::string yaml_path = ros::package::getPath(package_name_) + "/meshes/";
    scan_file_->setText(QString::fromStdString(yaml_path+"DAC_580813_scan.ply"));
  }

  if (config.mapGetString(objectName() + "scan_marker_name", &tmp))
    scan_marker_name_line_->setText(tmp);
  else
    scan_marker_name_line_->setText("point_cloud");

//  float tmp_float;
//  if (config.mapGetFloat(objectName() + "down_sampling_leaf_size", &tmp_float))
//      down_sampling_leaf_size_->setValue(tmp_float);
//    else
//      down_sampling_leaf_size_->setValue(0.03);
}
