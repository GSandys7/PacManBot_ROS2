#ifndef PACMANBOT_RVIZ_PLUGINS__GAME_CONTROL_PANEL_HPP_
#define PACMANBOT_RVIZ_PLUGINS__GAME_CONTROL_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QStringList>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class QDoubleSpinBox;
class QProcess;
class QVBoxLayout;

namespace pacmanbot_rviz_plugins
{

class GameControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GameControlPanel(QWidget * parent = nullptr);
  ~GameControlPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onStartLocalizationPressed();
  void onStartNav2Pressed();
  void onStartGameNodesPressed();
  void onStartPressed();
  void onResetPressed();
  void onClydeSpeedChanged(double value);
  void setState(const QString & state);
  void setStateText(const QString & text);
  void setResultText(const QString & text);
  void setPelletsSpawned(int value);
  void setPelletsCollected(int value);
  void setScore(int value);
  void setRound(int value);
  void setServiceFeedback(const QString & text);

private:
  using Trigger = std_srvs::srv::Trigger;

  void sendTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const QString & waiting_text);
  void startLaunchProcess(
    QProcess *& process,
    QPushButton * button,
    const QStringList & arguments,
    const QString & label);
  void shutdownLaunchProcesses();
  void refreshPelletText();

  QLabel * state_label_;
  QLabel * round_label_;
  QLabel * pellets_label_;
  QLabel * score_label_;
  QLabel * result_label_;
  QLabel * service_feedback_label_;
  QPushButton * localization_button_;
  QPushButton * nav2_button_;
  QPushButton * game_nodes_button_;
  QPushButton * start_button_;
  QPushButton * reset_button_;
  QDoubleSpinBox * clyde_speed_spin_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr clyde_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr base_clyde_speed_pub_;
  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spawned_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr collected_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr score_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr round_sub_;

  QString current_state_;
  int pellets_spawned_;
  int pellets_collected_;

  QProcess * localization_process_;
  QProcess * nav2_process_;
  QProcess * game_nodes_process_;
};

}  // namespace pacmanbot_rviz_plugins

#endif  // PACMANBOT_RVIZ_PLUGINS__GAME_CONTROL_PANEL_HPP_
