#include "pacmanbot_rviz_plugins/game_control_panel.hpp"

#include <exception>
#include <functional>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QProcess>
#include <QStringList>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace pacmanbot_rviz_plugins
{

GameControlPanel::GameControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  state_label_(new QLabel("State: unknown")),
  round_label_(new QLabel("Round: 1")),
  pellets_label_(new QLabel("Pellets: 0 / 0")),
  score_label_(new QLabel("Score: 0")),
  result_label_(new QLabel("Result:")),
  service_feedback_label_(new QLabel("Waiting for game controller")),
  localization_button_(new QPushButton("Start Localization")),
  nav2_button_(new QPushButton("Start Nav2")),
  game_nodes_button_(new QPushButton("Start Game Nodes")),
  start_button_(new QPushButton("Start Game")),
  reset_button_(new QPushButton("Reset Round")),
  clyde_speed_spin_(new QDoubleSpinBox),
  current_state_("unknown"),
  pellets_spawned_(0),
  pellets_collected_(0),
  localization_process_(nullptr),
  nav2_process_(nullptr),
  game_nodes_process_(nullptr)
{
  auto * layout = new QVBoxLayout;

  auto * title = new QLabel("<b>PacMan Game</b>");
  layout->addWidget(title);
  layout->addWidget(state_label_);
  layout->addWidget(round_label_);
  layout->addWidget(pellets_label_);
  layout->addWidget(score_label_);
  layout->addWidget(result_label_);

  auto * bringup_layout = new QHBoxLayout;
  bringup_layout->addWidget(localization_button_);
  bringup_layout->addWidget(nav2_button_);
  bringup_layout->addWidget(game_nodes_button_);
  layout->addLayout(bringup_layout);

  auto * button_layout = new QHBoxLayout;
  button_layout->addWidget(start_button_);
  button_layout->addWidget(reset_button_);
  layout->addLayout(button_layout);

  clyde_speed_spin_->setRange(0.05, 2.0);
  clyde_speed_spin_->setSingleStep(0.05);
  clyde_speed_spin_->setDecimals(2);
  clyde_speed_spin_->setSuffix(" m/s");
  clyde_speed_spin_->setValue(0.25);

  auto * clyde_speed_layout = new QHBoxLayout;
  clyde_speed_layout->addWidget(new QLabel("Clyde speed"));
  clyde_speed_layout->addWidget(clyde_speed_spin_);
  layout->addLayout(clyde_speed_layout);

  layout->addWidget(service_feedback_label_);
  setLayout(layout);

  connect(
    localization_button_, &QPushButton::clicked,
    this, &GameControlPanel::onStartLocalizationPressed);
  connect(nav2_button_, &QPushButton::clicked, this, &GameControlPanel::onStartNav2Pressed);
  connect(
    game_nodes_button_, &QPushButton::clicked,
    this, &GameControlPanel::onStartGameNodesPressed);
  connect(start_button_, &QPushButton::clicked, this, &GameControlPanel::onStartPressed);
  connect(reset_button_, &QPushButton::clicked, this, &GameControlPanel::onResetPressed);
  connect(
    clyde_speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &GameControlPanel::onClydeSpeedChanged);
}

GameControlPanel::~GameControlPanel()
{
  shutdownLaunchProcesses();
}

void GameControlPanel::onInitialize()
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    setServiceFeedback("RViz ROS node unavailable");
    return;
  }

  node_ = ros_node_abstraction->get_raw_node();
  const auto qos = rclcpp::QoS(1).reliable().transient_local();

  start_client_ = node_->create_client<Trigger>("/robot_15/game/start");
  reset_client_ = node_->create_client<Trigger>("/robot_15/game/reset");
  clyde_speed_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/robot_15/clyde/speed",
    qos
  );
  base_clyde_speed_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/robot_15/game/base_clyde_speed",
    qos
  );

  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_15/game_state",
    qos,
    [this](std_msgs::msg::String::SharedPtr msg) {
      const auto state = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(
        this, "setState", Qt::QueuedConnection, Q_ARG(QString, state));
    });

  result_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_15/game_result",
    qos,
    [this](std_msgs::msg::String::SharedPtr msg) {
      QString text = "Result:";
      if (!msg->data.empty()) {
        text = QString::fromStdString("Result: " + msg->data);
      }
      QMetaObject::invokeMethod(
        this, "setResultText", Qt::QueuedConnection, Q_ARG(QString, text));
    });

  spawned_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/robot_15/game/pellets_spawned",
    qos,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setPelletsSpawned", Qt::QueuedConnection, Q_ARG(int, msg->data));
    });

  collected_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/robot_15/game/pellets_collected",
    qos,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setPelletsCollected", Qt::QueuedConnection, Q_ARG(int, msg->data));
    });

  score_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/robot_15/game/score",
    qos,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setScore", Qt::QueuedConnection, Q_ARG(int, msg->data));
    });

  round_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/robot_15/game/round",
    qos,
    [this](std_msgs::msg::Int32::SharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setRound", Qt::QueuedConnection, Q_ARG(int, msg->data));
    });

  setServiceFeedback("Ready");
  onClydeSpeedChanged(clyde_speed_spin_->value());
}

void GameControlPanel::onStartLocalizationPressed()
{
  startLaunchProcess(
    localization_process_,
    localization_button_,
    QStringList({"launch", "pacmanbot_package", "pacman_start_localization.launch.py"}),
    "localization");
}

void GameControlPanel::onStartNav2Pressed()
{
  startLaunchProcess(
    nav2_process_,
    nav2_button_,
    QStringList({
      "launch",
      "pacmanbot_package",
      "pacman_nav2.launch.py",
      "namespace:=/robot_15",
      "tf_topic:=/robot_15/tf",
      "tf_static_topic:=/robot_15/tf_static",
    }),
    "Nav2");
}

void GameControlPanel::onStartGameNodesPressed()
{
  startLaunchProcess(
    game_nodes_process_,
    game_nodes_button_,
    QStringList({"launch", "pacmanbot_package", "pacman_game_nodes.launch.py"}),
    "game nodes");
}

void GameControlPanel::onStartPressed()
{
  sendTrigger(start_client_, "Starting game...");
}

void GameControlPanel::onResetPressed()
{
  sendTrigger(reset_client_, "Resetting round...");
}

void GameControlPanel::onClydeSpeedChanged(double value)
{
  if (!clyde_speed_pub_) {
    return;
  }

  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(value);
  clyde_speed_pub_->publish(msg);
  if (base_clyde_speed_pub_ && (current_state_ == "menu" || current_state_ == "unknown")) {
    base_clyde_speed_pub_->publish(msg);
  }
  setServiceFeedback(QString("Clyde speed set to %1 m/s").arg(value, 0, 'f', 2));
}

void GameControlPanel::setState(const QString & state)
{
  current_state_ = state;
  state_label_->setText("State: " + state);
}

void GameControlPanel::setStateText(const QString & text)
{
  state_label_->setText(text);
}

void GameControlPanel::setResultText(const QString & text)
{
  result_label_->setText(text);
}

void GameControlPanel::setPelletsSpawned(int value)
{
  pellets_spawned_ = value;
  refreshPelletText();
}

void GameControlPanel::setPelletsCollected(int value)
{
  pellets_collected_ = value;
  refreshPelletText();
}

void GameControlPanel::setScore(int value)
{
  score_label_->setText(QString("Score: %1").arg(value));
}

void GameControlPanel::setRound(int value)
{
  round_label_->setText(QString("Round: %1").arg(value));
}

void GameControlPanel::setServiceFeedback(const QString & text)
{
  service_feedback_label_->setText(text);
}

void GameControlPanel::sendTrigger(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const QString & waiting_text)
{
  if (!node_ || !client) {
    setServiceFeedback("Game controller unavailable");
    return;
  }

  if (!client->service_is_ready()) {
    setServiceFeedback("Game controller service not ready");
    return;
  }

  setServiceFeedback(waiting_text);
  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(
    request,
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      QString feedback;
      try {
        const auto response = future.get();
        feedback = QString::fromStdString(response->message);
        if (!response->success) {
          feedback = "Rejected: " + feedback;
        }
      } catch (const std::exception & ex) {
        feedback = QString("Service call failed: %1").arg(ex.what());
      }

      QMetaObject::invokeMethod(
        this, "setServiceFeedback", Qt::QueuedConnection, Q_ARG(QString, feedback));
    });
}

void GameControlPanel::startLaunchProcess(
  QProcess *& process,
  QPushButton * button,
  const QStringList & arguments,
  const QString & label)
{
  if (process && process->state() != QProcess::NotRunning) {
    setServiceFeedback(QString("%1 launch is already running").arg(label));
    return;
  }

  if (process) {
    process->deleteLater();
  }

  process = new QProcess(this);
  process->setProgram("ros2");
  process->setArguments(arguments);
  process->setProcessChannelMode(QProcess::ForwardedChannels);

  connect(
    process,
    QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this,
    [this, button, label](int exit_code, QProcess::ExitStatus exit_status) {
      if (button) {
        button->setEnabled(true);
      }

      const QString status =
        exit_status == QProcess::NormalExit ? QString("exit %1").arg(exit_code) : "crashed";
      setServiceFeedback(QString("%1 launch stopped (%2)").arg(label, status));
    });

  process->start();
  if (!process->waitForStarted(3000)) {
    process->deleteLater();
    process = nullptr;
    setServiceFeedback(QString("Failed to start %1 launch").arg(label));
    return;
  }

  if (button) {
    button->setEnabled(false);
  }
  setServiceFeedback(QString("Started %1 launch").arg(label));
}

void GameControlPanel::shutdownLaunchProcesses()
{
  QProcess * processes[] = {
    game_nodes_process_,
    nav2_process_,
    localization_process_,
  };

  for (auto * process : processes) {
    if (!process || process->state() == QProcess::NotRunning) {
      continue;
    }

    process->terminate();
  }

  for (auto * process : processes) {
    if (!process || process->state() == QProcess::NotRunning) {
      continue;
    }

    if (!process->waitForFinished(3000)) {
      process->kill();
      process->waitForFinished(1000);
    }
  }
}

void GameControlPanel::refreshPelletText()
{
  pellets_label_->setText(
    QString("Pellets: %1 / %2").arg(pellets_collected_).arg(pellets_spawned_));
}

}  // namespace pacmanbot_rviz_plugins

PLUGINLIB_EXPORT_CLASS(pacmanbot_rviz_plugins::GameControlPanel, rviz_common::Panel)
