#include "assembly_rviz_param_panel/parametric_motion_panel.hpp"

#include <chrono>
#include <sstream>

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace assembly_rviz_param_panel
{
using namespace std::chrono_literals;

ParametricMotionPanel::ParametricMotionPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout(this);
  auto * form = new QFormLayout();

  yaml_path_edit_ = new QLineEdit(this);
  yaml_path_edit_->setText(
    "/root/ur10_ws/src/ur_base_xarco_model/assembly_description/config/parametric_experiment.yaml");
  form->addRow("YAML", yaml_path_edit_);

  experiment_id_box_ = new QComboBox(this);
  form->addRow("Experiment", experiment_id_box_);

  depth_spin_ = new QDoubleSpinBox(this);
  depth_spin_->setRange(0.0, 200.0);
  depth_spin_->setSuffix(" mm");
  depth_spin_->setDecimals(3);
  form->addRow("Depth", depth_spin_);

  speed_spin_ = new QDoubleSpinBox(this);
  speed_spin_->setRange(0.0, 5.0);
  speed_spin_->setDecimals(3);
  form->addRow("Speed", speed_spin_);

  angle_spin_ = new QDoubleSpinBox(this);
  angle_spin_->setRange(-180.0, 180.0);
  angle_spin_->setSuffix(" deg");
  angle_spin_->setDecimals(3);
  form->addRow("Angle", angle_spin_);
  root_layout->addLayout(form);

  auto * row1 = new QHBoxLayout();
  load_btn_ = new QPushButton("Load", this);
  save_btn_ = new QPushButton("Save", this);
  validate_btn_ = new QPushButton("Validate", this);
  row1->addWidget(load_btn_);
  row1->addWidget(save_btn_);
  row1->addWidget(validate_btn_);
  root_layout->addLayout(row1);

  auto * row2 = new QHBoxLayout();
  preview_btn_ = new QPushButton("Plan Preview", this);
  execute_btn_ = new QPushButton("Execute", this);
  row2->addWidget(preview_btn_);
  row2->addWidget(execute_btn_);
  root_layout->addLayout(row2);

  status_label_ = new QLabel(this);
  status_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);
  root_layout->addStretch();

  connect(load_btn_, &QPushButton::clicked, this, &ParametricMotionPanel::on_load_clicked);
  connect(save_btn_, &QPushButton::clicked, this, &ParametricMotionPanel::on_save_clicked);
  connect(validate_btn_, &QPushButton::clicked, this, &ParametricMotionPanel::on_validate_clicked);
  connect(preview_btn_, &QPushButton::clicked, this, &ParametricMotionPanel::on_preview_clicked);
  connect(execute_btn_, &QPushButton::clicked, this, &ParametricMotionPanel::on_execute_clicked);
  connect(
    experiment_id_box_,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    &ParametricMotionPanel::on_experiment_changed);

  set_status("Ready. Run Load first.", false);
  update_gates();
}

void ParametricMotionPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    set_status("RViz ROS node unavailable.", true);
    block_inputs(true);
    return;
  }
  node_ = node_abstraction->get_raw_node();
  manage_client_ = node_->create_client<assembly_parametric_motion::srv::ManageExperimentConfig>(
    "/assembly/config/load_save");
  validate_client_ = node_->create_client<assembly_parametric_motion::srv::ValidatePlan>(
    "/assembly/plan/validate");
  preview_client_ = node_->create_client<assembly_parametric_motion::srv::PreviewPlan>(
    "/assembly/plan/preview");
  execute_client_ = node_->create_client<assembly_parametric_motion::srv::ExecuteCachedPlan>(
    "/assembly/plan/execute");
}

template<typename ServiceT, typename RequestT>
typename ServiceT::Response::SharedPtr ParametricMotionPanel::call_service(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const std::shared_ptr<RequestT> & request,
  double timeout_sec,
  std::string & error)
{
  if (!client) {
    error = "client not initialized";
    return nullptr;
  }
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    error = "service unavailable";
    return nullptr;
  }
  auto future = client->async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(50ms) == std::future_status::ready) {
      return future.get();
    }
  }
  error = "service timeout";
  return nullptr;
}

void ParametricMotionPanel::set_status(const std::string & text, bool is_error)
{
  if (is_error) {
    status_label_->setStyleSheet("QLabel { color: rgb(200, 40, 40); }");
  } else {
    status_label_->setStyleSheet("QLabel { color: rgb(40, 180, 80); }");
  }
  status_label_->setText(QString::fromStdString(text));
}

void ParametricMotionPanel::update_gates()
{
  preview_btn_->setEnabled(validated_ok_);
  execute_btn_->setEnabled(preview_ok_);
}

void ParametricMotionPanel::block_inputs(bool blocked)
{
  yaml_path_edit_->setEnabled(!blocked);
  experiment_id_box_->setEnabled(!blocked);
  depth_spin_->setEnabled(!blocked);
  speed_spin_->setEnabled(!blocked);
  angle_spin_->setEnabled(!blocked);
  load_btn_->setEnabled(!blocked);
  save_btn_->setEnabled(!blocked);
  validate_btn_->setEnabled(!blocked);
  preview_btn_->setEnabled(!blocked && validated_ok_);
  execute_btn_->setEnabled(!blocked && preview_ok_);
}

std::string ParametricMotionPanel::config_path() const
{
  return yaml_path_edit_->text().toStdString();
}

void ParametricMotionPanel::apply_active_values(
  const assembly_parametric_motion::srv::ManageExperimentConfig::Response::SharedPtr & response)
{
  depth_spin_->setValue(response->penetration_depth_mm);
  speed_spin_->setValue(response->speed_setting);
  angle_spin_->setValue(response->entry_angle_deg);

  depth_spin_->setRange(response->min_depth_mm, response->max_depth_mm);
  speed_spin_->setRange(response->min_speed, response->max_speed);
  angle_spin_->setRange(response->min_angle_deg, response->max_angle_deg);

  suppress_combo_event_ = true;
  experiment_id_box_->clear();
  for (const auto & id : response->experiment_ids) {
    experiment_id_box_->addItem(QString::fromStdString(id));
  }
  const int idx = experiment_id_box_->findText(QString::fromStdString(response->active_experiment_id));
  if (idx >= 0) {
    experiment_id_box_->setCurrentIndex(idx);
  }
  suppress_combo_event_ = false;
}

void ParametricMotionPanel::on_load_clicked()
{
  auto req = std::make_shared<assembly_parametric_motion::srv::ManageExperimentConfig::Request>();
  req->command = "load";
  req->yaml_path = config_path();

  std::string error;
  auto resp = call_service<assembly_parametric_motion::srv::ManageExperimentConfig>(
    manage_client_, req, 3.0, error);
  if (!resp) {
    set_status("Load failed: " + error, true);
    return;
  }
  if (!resp->success) {
    set_status("Load failed: " + resp->message, true);
    return;
  }

  apply_active_values(resp);
  validated_ok_ = false;
  preview_ok_ = false;
  update_gates();
  set_status("Load ok.", false);
}

void ParametricMotionPanel::on_save_clicked()
{
  const QString current_id = experiment_id_box_->currentText();
  if (current_id.isEmpty()) {
    set_status("Save failed: empty experiment id.", true);
    return;
  }

  auto req = std::make_shared<assembly_parametric_motion::srv::ManageExperimentConfig::Request>();
  req->command = "save";
  req->yaml_path = config_path();
  req->active_experiment_id = current_id.toStdString();
  req->penetration_depth_mm = depth_spin_->value();
  req->speed_setting = speed_spin_->value();
  req->entry_angle_deg = angle_spin_->value();

  std::string error;
  auto resp = call_service<assembly_parametric_motion::srv::ManageExperimentConfig>(
    manage_client_, req, 3.0, error);
  if (!resp) {
    set_status("Save failed: " + error, true);
    return;
  }
  if (!resp->success) {
    set_status("Save failed: " + resp->message, true);
    return;
  }

  apply_active_values(resp);
  validated_ok_ = false;
  preview_ok_ = false;
  update_gates();
  set_status("Save ok.", false);
}

void ParametricMotionPanel::on_validate_clicked()
{
  auto req = std::make_shared<assembly_parametric_motion::srv::ValidatePlan::Request>();
  req->penetration_depth_mm = depth_spin_->value();
  req->speed_setting = speed_spin_->value();
  req->entry_angle_deg = angle_spin_->value();

  std::string error;
  auto resp = call_service<assembly_parametric_motion::srv::ValidatePlan>(
    validate_client_, req, 15.0, error);
  if (!resp) {
    validated_ok_ = false;
    preview_ok_ = false;
    update_gates();
    set_status("Validate failed: " + error, true);
    return;
  }
  if (!resp->success) {
    validated_ok_ = false;
    preview_ok_ = false;
    update_gates();
    set_status("Validate failed: " + resp->message, true);
    return;
  }

  validated_ok_ = true;
  preview_ok_ = false;
  update_gates();
  std::ostringstream oss;
  oss << "Validate ok. fraction=" << resp->cartesian_fraction
      << ", vel_scale=" << resp->velocity_scaling
      << ", acc_scale=" << resp->acceleration_scaling
      << ", est_mass_g=" << resp->estimated_mass_g;
  set_status(oss.str(), false);
}

void ParametricMotionPanel::on_preview_clicked()
{
  if (!validated_ok_) {
    set_status("Preview blocked: run Validate first.", true);
    return;
  }

  auto req = std::make_shared<assembly_parametric_motion::srv::PreviewPlan::Request>();
  req->penetration_depth_mm = depth_spin_->value();
  req->speed_setting = speed_spin_->value();
  req->entry_angle_deg = angle_spin_->value();

  std::string error;
  auto resp = call_service<assembly_parametric_motion::srv::PreviewPlan>(
    preview_client_, req, 15.0, error);
  if (!resp) {
    preview_ok_ = false;
    update_gates();
    set_status("Preview failed: " + error, true);
    return;
  }
  if (!resp->success) {
    preview_ok_ = false;
    update_gates();
    set_status("Preview failed: " + resp->message, true);
    return;
  }

  preview_ok_ = true;
  update_gates();
  std::ostringstream oss;
  oss << "Preview ok. fraction=" << resp->cartesian_fraction
      << ", path_m=" << resp->path_length_m
      << ", planning_s=" << resp->planning_time_s;
  set_status(oss.str(), false);
}

void ParametricMotionPanel::on_execute_clicked()
{
  if (!preview_ok_) {
    set_status("Execute blocked: successful Preview required.", true);
    return;
  }

  auto req = std::make_shared<assembly_parametric_motion::srv::ExecuteCachedPlan::Request>();
  req->execute = true;

  std::string error;
  auto resp = call_service<assembly_parametric_motion::srv::ExecuteCachedPlan>(
    execute_client_, req, 120.0, error);
  if (!resp) {
    set_status("Execute failed: " + error, true);
    return;
  }
  if (!resp->success) {
    set_status("Execute failed: " + resp->message, true);
    preview_ok_ = false;
    update_gates();
    return;
  }
  preview_ok_ = false;
  update_gates();
  set_status("Execute success.", false);
}

void ParametricMotionPanel::on_experiment_changed(int index)
{
  if (suppress_combo_event_ || index < 0) {
    return;
  }

  auto req = std::make_shared<assembly_parametric_motion::srv::ManageExperimentConfig::Request>();
  req->command = "set_active";
  req->yaml_path = config_path();
  req->active_experiment_id = experiment_id_box_->currentText().toStdString();

  std::string error;
  auto resp = call_service<assembly_parametric_motion::srv::ManageExperimentConfig>(
    manage_client_, req, 3.0, error);
  if (!resp) {
    set_status("Set active failed: " + error, true);
    return;
  }
  if (!resp->success) {
    set_status("Set active failed: " + resp->message, true);
    return;
  }

  apply_active_values(resp);
  validated_ok_ = false;
  preview_ok_ = false;
  update_gates();
  set_status("Active experiment switched.", false);
}
}  // namespace assembly_rviz_param_panel

PLUGINLIB_EXPORT_CLASS(assembly_rviz_param_panel::ParametricMotionPanel, rviz_common::Panel)
