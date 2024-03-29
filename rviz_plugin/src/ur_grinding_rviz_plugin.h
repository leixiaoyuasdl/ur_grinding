#ifndef FANUC_GRINDING_RVIZ_PLUGIN_H
#define FANUC_GRINDING_RVIZ_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include "scanning_widget.h"
#include "path_planning_widget.h"

class QTabWidget;
class QLabel;
class QVBoxLayout;

namespace fanuc_grinding_rviz_plugin
{
class FanucGrindingRvizPlugin : public rviz::Panel
{
  Q_OBJECT
public:
  FanucGrindingRvizPlugin(QWidget* parent = 0);
  virtual ~FanucGrindingRvizPlugin();

  Q_SIGNALS:
  void enableWidget(const bool);
  void displayStatus(const QString);
  void sendCADAndScanDatas(const QString, const QString);

protected Q_SLOTS:
  virtual void triggerSave();
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  void displayStatusHandler(const QString message);
  void displayMsgBoxHandler(const QString title, const QString msg, const QString info_msg);

  void enablePanelHandler(const bool);
  void enablePanelPathPlanningHandler();
  void enablePanelPostProcessorHandler();
  void setCADDatas(const QString cad_path);
  void setScanDatas(const QString scan_path);
  void sendCADAndScanDatasSlot();

protected:
  QString cad_filename_;
  QString scan_filename_;

  QTabWidget* tab_widget_;
  ScanningWidget* scanning_widget_;
  PathPlanningWidget* path_planning_widget_;

  QLabel* status_label_;
};

}  // end namespace

#endif // FANUC_GRINDING_RVIZ_PLUGIN_H
