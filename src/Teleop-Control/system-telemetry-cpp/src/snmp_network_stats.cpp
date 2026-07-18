#include "snmp_network_stats.hpp"

#include <cstring>
#include <limits>
#include <vector>

extern "C" {
#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>
}

SnmpNetworkStatsNode::SnmpNetworkStatsNode()
    : Node("snmp_network_stats"),
      ip_address_(
          this->declare_parameter<std::string>("ip_address", "192.168.0.3")),
      frequency_(this->declare_parameter<double>("frequency", 1.0)),
      community_("public"), snmp_version_(SNMP_VERSION_1),
      timeout_us_(1000000L), retries_(0) {
  pub_bandwidth_tx_ =
      this->create_publisher<std_msgs::msg::Float32>("~/bandwidth_tx", 10);
  pub_bandwidth_rx_ =
      this->create_publisher<std_msgs::msg::Float32>("~/bandwidth_rx", 10);
  pub_throughput_tx_ =
      this->create_publisher<std_msgs::msg::Float32>("~/throughput_tx", 10);
  pub_throughput_rx_ =
      this->create_publisher<std_msgs::msg::Float32>("~/throughput_rx", 10);
  pub_signal_strength_ =
      this->create_publisher<std_msgs::msg::Float32>("~/signal_strength", 10);
  pub_noise_floor_ =
      this->create_publisher<std_msgs::msg::Float32>("~/noise_floor", 10);
  pub_ccq_tx_ = this->create_publisher<std_msgs::msg::Float32>("~/ccq_tx", 10);

  if (frequency_ <= 0.0) {
    frequency_ = 1.0;
  }

  init_snmp("snmp_network_stats");

  const auto period = std::chrono::duration<double>(1.0 / frequency_);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SnmpNetworkStatsNode::poll_and_publish, this));
  RCLCPP_INFO(this->get_logger(), "Initialized with IP: %s, Frequency: %.2f Hz",
              ip_address_.c_str(), frequency_);
}

std::optional<int>
SnmpNetworkStatsNode::snmp_get_int(const std::string &oid_str) {
  netsnmp_session session{};
  snmp_sess_init(&session);

  session.peername = strdup(ip_address_.c_str());
  session.version = snmp_version_;
  session.community =
      reinterpret_cast<u_char *>(const_cast<char *>(community_.c_str()));
  session.community_len = community_.size();
  session.timeout = timeout_us_;
  session.retries = retries_;

  netsnmp_session *ss = snmp_open(&session);
  if (!ss) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to open SNMP session to %s",
                         ip_address_.c_str());
    free(session.peername);
    return std::nullopt;
  }

  netsnmp_pdu *pdu = snmp_pdu_create(SNMP_MSG_GET);

  oid an_oid[MAX_OID_LEN];
  size_t an_oid_len = MAX_OID_LEN;
  if (!snmp_parse_oid(oid_str.c_str(), an_oid, &an_oid_len)) {
    snmp_free_pdu(pdu);
    snmp_close(ss);
    free(session.peername);
    return std::nullopt;
  }

  snmp_add_null_var(pdu, an_oid, an_oid_len);

  netsnmp_pdu *response = nullptr;
  const int status = snmp_synch_response(ss, pdu, &response);

  std::optional<int> result = std::nullopt;

  if (status == STAT_SUCCESS && response &&
      response->errstat == SNMP_ERR_NOERROR) {
    for (netsnmp_variable_list *vars = response->variables; vars;
         vars = vars->next_variable) {
      switch (vars->type) {
      case ASN_INTEGER:
      case ASN_COUNTER:
      case ASN_GAUGE:
      case ASN_TIMETICKS:
      case ASN_UINTEGER:
        result = static_cast<int>(*vars->val.integer);
        break;
      case ASN_COUNTER64:
        result = static_cast<int>(vars->val.counter64->low);
        break;
      default:
        break;
      }
      if (result.has_value()) {
        break;
      }
    }
  }

  if (response) {
    snmp_free_pdu(response);
  }
  snmp_close(ss);
  free(session.peername);

  return result;
}

std::optional<int>
SnmpNetworkStatsNode::snmp_walk_first_int(const std::string &base_oid_str) {
  netsnmp_session session{};
  snmp_sess_init(&session);

  session.peername = strdup(ip_address_.c_str());
  session.version = snmp_version_;
  session.community =
      reinterpret_cast<u_char *>(const_cast<char *>(community_.c_str()));
  session.community_len = community_.size();
  session.timeout = timeout_us_;
  session.retries = retries_;

  netsnmp_session *ss = snmp_open(&session);
  if (!ss) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to open SNMP session to %s",
                         ip_address_.c_str());
    free(session.peername);
    return std::nullopt;
  }

  oid root_oid[MAX_OID_LEN];
  size_t root_oid_len = MAX_OID_LEN;
  if (!snmp_parse_oid(base_oid_str.c_str(), root_oid, &root_oid_len)) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse OID: %s",
                base_oid_str.c_str());
    snmp_close(ss);
    free(session.peername);
    return std::nullopt;
  }

  oid current_oid[MAX_OID_LEN];
  size_t current_oid_len = root_oid_len;
  std::memcpy(current_oid, root_oid, root_oid_len * sizeof(oid));

  std::optional<int> result = std::nullopt;

  while (true) {
    netsnmp_pdu *pdu = snmp_pdu_create(SNMP_MSG_GETNEXT);
    snmp_add_null_var(pdu, current_oid, current_oid_len);

    netsnmp_pdu *response = nullptr;
    const int status = snmp_synch_response(ss, pdu, &response);

    if (status != STAT_SUCCESS || !response ||
        response->errstat != SNMP_ERR_NOERROR) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "SNMP walk failed for OID: %s",
                           base_oid_str.c_str());
      if (response) {
        snmp_free_pdu(response);
      }
      break;
    }

    netsnmp_variable_list *vars = response->variables;
    if (!vars) {
      RCLCPP_WARN(this->get_logger(), "No vars: %s", base_oid_str.c_str());
      snmp_free_pdu(response);
      break;
    }

    if (snmp_oid_compare(root_oid, root_oid_len, vars->name,
                         vars->name_length) != 0 &&
        (vars->name_length < root_oid_len ||
         std::memcmp(root_oid, vars->name, root_oid_len * sizeof(oid)) != 0)) {
      RCLCPP_WARN(this->get_logger(), "Bad compare: %s", base_oid_str.c_str());
      snmp_free_pdu(response);
      break;
    }

    switch (vars->type) {
    case ASN_INTEGER:
    case ASN_COUNTER:
    case ASN_GAUGE:
    case ASN_TIMETICKS:
    case ASN_UINTEGER:
      result = static_cast<int>(*vars->val.integer);
      break;
    case ASN_COUNTER64:
      result = static_cast<int>(vars->val.counter64->low);
      break;
    default:
      break;
    }

    snmp_free_pdu(response);
    break;
  }

  snmp_close(ss);
  free(session.peername);

  return result;
}

void SnmpNetworkStatsNode::publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &publisher,
    float value) {
  std_msgs::msg::Float32 msg;
  msg.data = value;
  publisher->publish(msg);
}

std::optional<float> SnmpNetworkStatsNode::compute_throughput_bps(
    uint32_t current_counter, uint32_t previous_counter, double dt_sec) const {
  if (dt_sec <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid time delta for throughput calculation: %.6f seconds",
                dt_sec);
    return std::nullopt;
  }

  uint64_t delta_bytes = 0;
  if (current_counter >= previous_counter) {
    delta_bytes = static_cast<uint64_t>(current_counter) -
                  static_cast<uint64_t>(previous_counter);
  } else {
    delta_bytes = (static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) -
                   previous_counter + 1ULL) +
                  current_counter;
  }

  return static_cast<float>((static_cast<double>(delta_bytes) * 8.0) / dt_sec);
}

void SnmpNetworkStatsNode::poll_and_publish() {
  const auto now = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "Polling SNMP data at time: %.2f seconds",
               std::chrono::duration<double>(now.time_since_epoch()).count());

  const auto signal_strength = snmp_get_int(oid_signal_strength_base_);
  const auto noise_floor = snmp_get_int(oid_noise_floor_base_);
  const auto ccq_tx = snmp_get_int(oid_ccq_tx_base_);
  const auto bandwidth_tx = snmp_get_int(oid_bandwidth_tx_base_);
  const auto bandwidth_rx = snmp_get_int(oid_bandwidth_rx_base_);

  const auto rx_counter_raw = snmp_get_int(oid_rx_counter_);
  const auto tx_counter_raw = snmp_get_int(oid_tx_counter_);

  if (signal_strength.has_value()) {
    publish_float(pub_signal_strength_,
                  static_cast<float>(signal_strength.value()));
  }
  if (noise_floor.has_value()) {
    publish_float(pub_noise_floor_, static_cast<float>(noise_floor.value()));
  }
  if (ccq_tx.has_value()) {
    publish_float(pub_ccq_tx_, static_cast<float>(ccq_tx.value()));
  }
  if (bandwidth_tx.has_value()) {
    publish_float(pub_bandwidth_tx_, static_cast<float>(bandwidth_tx.value()));
  }
  if (bandwidth_rx.has_value()) {
    publish_float(pub_bandwidth_rx_, static_cast<float>(bandwidth_rx.value()));
  }

  if (rx_counter_raw.has_value() && tx_counter_raw.has_value()) {
    const uint32_t rx_counter = static_cast<uint32_t>(rx_counter_raw.value());
    const uint32_t tx_counter = static_cast<uint32_t>(tx_counter_raw.value());

    if (prev_time_.has_value() && prev_rx_counter_.has_value() &&
        prev_tx_counter_.has_value()) {
      const double dt =
          std::chrono::duration<double>(now - prev_time_.value()).count();

      const auto throughput_rx =
          compute_throughput_bps(rx_counter, prev_rx_counter_.value(), dt);
      const auto throughput_tx =
          compute_throughput_bps(tx_counter, prev_tx_counter_.value(), dt);

      if (throughput_rx.has_value()) {
        publish_float(pub_throughput_rx_, throughput_rx.value());
      }
      if (throughput_tx.has_value()) {
        publish_float(pub_throughput_tx_, throughput_tx.value());
      }
    }

    prev_rx_counter_ = rx_counter;
    prev_tx_counter_ = tx_counter;
    prev_time_ = now;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SnmpNetworkStatsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}