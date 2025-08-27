#ifndef ROS_PHOENIX_PHOENIX_NODES
#define ROS_PHOENIX_PHOENIX_NODES

#include "ros_phoenix/phoenix_node.hpp"

namespace ros_phoenix {

class TalonFXNode
    : public PhoenixNode<ctre::phoenix::motorcontrol::can::TalonFX,
                         TalonFXConfiguration, TalonFXFeedbackDevice,
                         TalonFXControlMode> {
 public:
  TalonFXNode(const std::string &name,
              const NodeOptions &options = NodeOptions());

  virtual ~TalonFXNode() = default;

  virtual void configure_current_limit(TalonFXConfiguration &config);
  virtual void configure_sensor();
  virtual double get_output_current();
};

class TalonSRXNode
    : public PhoenixNode<ctre::phoenix::motorcontrol::can::TalonSRX,
                         TalonSRXConfiguration, TalonSRXFeedbackDevice,
                         TalonSRXControlMode> {
 public:
  TalonSRXNode(const std::string &name,
               const NodeOptions &options = NodeOptions());

  virtual ~TalonSRXNode() = default;

  virtual void configure_current_limit(TalonSRXConfiguration &config);
  virtual void configure_sensor();
  virtual double get_output_current();
};

class VictorSPXNode
    : public PhoenixNode<ctre::phoenix::motorcontrol::can::VictorSPX,
                         VictorSPXConfiguration, FeedbackDevice,
                         VictorSPXControlMode> {
 public:
  VictorSPXNode(const std::string &name,
                const NodeOptions &options = NodeOptions());

  virtual ~VictorSPXNode() = default;

  virtual void configure_current_limit(VictorSPXConfiguration &config);
  virtual void configure_sensor();
  virtual double get_output_current();
};

}  // namespace ros_phoenix

#endif  // ROS_PHOENIX_PHOENIX_NODES