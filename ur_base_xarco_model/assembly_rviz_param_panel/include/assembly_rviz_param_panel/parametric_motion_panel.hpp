#ifndef ASSEMBLY_RVIZ_PARAM_PANEL__PARAMETRIC_MOTION_PANEL_HPP_
#define ASSEMBLY_RVIZ_PARAM_PANEL__PARAMETRIC_MOTION_PANEL_HPP_

#include <memory>
#include <string>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "assembly_parametric_motion/srv/execute_cached_plan.hpp"
#include "assembly_parametric_motion/srv/manage_experiment_config.hpp"
#include "assembly_parametric_motion/srv/preview_plan.hpp"
#include "assembly_parametric_motion/srv/validate_plan.hpp"

namespace assembly_rviz_param_panel
{
class ParametricMotionPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ParametricMotionPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void on_load_clicked();
  void on_save_clicked();
  void on_validate_clicked();
  void on_preview_clicked();
  void on_execute_clicked();
  void on_experiment_changed(int index);

private:
  template<typename ServiceT, typename RequestT>
  typename ServiceT::Response::SharedPtr call_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const std::shared_ptr<RequestT> & request,
    double timeout_sec,
    std::string & error);

  void set_status(const std::string & text, bool is_error);
  void update_gates();
  void block_inputs(bool blocked);

  std::string config_path() const;
  void apply_active_values(
    const assembly_parametric_motion::srv::ManageExperimentConfig::Response::SharedPtr & response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<assembly_parametric_motion::srv::ManageExperimentConfig>::SharedPtr manage_client_;
  rclcpp::Client<assembly_parametric_motion::srv::ValidatePlan>::SharedPtr validate_client_;
  rclcpp::Client<assembly_parametric_motion::srv::PreviewPlan>::SharedPtr preview_client_;
  rclcpp::Client<assembly_parametric_motion::srv::ExecuteCachedPlan>::SharedPtr execute_client_;

  QLineEdit * yaml_path_edit_{nullptr};
  QComboBox * experiment_id_box_{nullptr};
  QDoubleSpinBox * depth_spin_{nullptr};
  QDoubleSpinBox * speed_spin_{nullptr};
  QDoubleSpinBox * angle_spin_{nullptr};
  QPushButton * load_btn_{nullptr};
  QPushButton * save_btn_{nullptr};
  QPushButton * validate_btn_{nullptr};
  QPushButton * preview_btn_{nullptr};
  QPushButton * execute_btn_{nullptr};
  QLabel * status_label_{nullptr};

  bool validated_ok_{false};
  bool preview_ok_{false};
  bool suppress_combo_event_{false};
};
}  // namespace assembly_rviz_param_panel

#endif  // ASSEMBLY_RVIZ_PARAM_PANEL__PARAMETRIC_MOTION_PANEL_HPP_
