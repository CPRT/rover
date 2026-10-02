#include "interface_manager.h"

#include <asm/types.h>
#include <fcntl.h>
#include <libudev.h>
#include <libusb-1.0/libusb.h>
#include <linux/can/netlink.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include "interfaces/msg/usb_device.hpp"

/* Based on copy_rtnl_link_stats() from kernel at net/core/rtnetlink.c */
static void copy_rtnl_link_stats64(struct rtnl_link_stats64 *stats64,
                                   const struct rtnl_link_stats *stats) {
  __u64 *a = (__u64 *)stats64;
  const __u32 *b = (const __u32 *)stats;
  const __u32 *e = b + sizeof(*stats) / sizeof(*b);

  while (b < e)
    *a++ = *b++;
}

int get_rtnl_link_stats_rta(struct rtnl_link_stats64 *stats64,
                            struct rtattr *tb[]) {
  struct rtnl_link_stats stats;
  void *s;
  struct rtattr *rta;
  int size, len;

  if (tb[IFLA_STATS64]) {
    rta = tb[IFLA_STATS64];
    size = sizeof(struct rtnl_link_stats64);
    s = stats64;
  } else if (tb[IFLA_STATS]) {
    rta = tb[IFLA_STATS];
    size = sizeof(struct rtnl_link_stats);
    s = &stats;
  } else {
    return -1;
  }

  len = RTA_PAYLOAD(rta);
  if (len < size)
    memset(s + len, 0, size - len);
  else
    len = size;

  memcpy(s, RTA_DATA(rta), len);

  if (s != stats64)
    copy_rtnl_link_stats64(stats64, (const rtnl_link_stats *)s);
  return size;
}

#define USB_MAX_DEPTH 7

#define SYSFS_DEV_ATTR_PATH "/sys/bus/usb/devices/%s/%s"
struct udev *udev = NULL;
struct udev_hwdb *hwdb = NULL;
int get_sysfs_name(char *buf, size_t size, libusb_device *dev) {
  int len = 0;
  uint8_t bnum = libusb_get_bus_number(dev);
  uint8_t pnums[USB_MAX_DEPTH];
  int num_pnums;

  buf[0] = '\0';

  num_pnums = libusb_get_port_numbers(dev, pnums, sizeof(pnums));
  if (num_pnums == LIBUSB_ERROR_OVERFLOW) {
    return -1;
  } else if (num_pnums == 0) {
    /* Special-case root devices */
    return snprintf(buf, size, "usb%d", bnum);
  }

  len = snprintf(buf, size, "%d-", bnum);
  if (len < 0 || (size_t)len >= size)
    return -1;
  for (int i = 0; i < num_pnums; i++) {
    int n = snprintf(buf + len, size - len, i ? ".%d" : "%d", pnums[i]);
    if ((n < 0) || (n >= (int)(size - len)))
      break;
    len += n;
  }

  return len;
}

int read_sysfs_prop(char *buf, size_t size, const char *sysfs_name,
                    const char *propname) {
  int n, fd;
  char path[PATH_MAX];

  buf[0] = '\0';
  snprintf(path, sizeof(path), SYSFS_DEV_ATTR_PATH, sysfs_name, propname);
  fd = open(path, O_RDONLY);

  if (fd == -1)
    return 0;

  n = read(fd, buf, size - 1);

  if (n > 0) {
    buf[n] = '\0';
    /* Strip trailing newline if present */
    if (n > 0 && buf[n - 1] == '\n')
      buf[n - 1] = '\0';
    for (int i = 0; buf[i]; i++)
      if ((unsigned char)buf[i] < 0x20 || buf[i] == 0x7f)
        buf[i] = '?';
  }

  close(fd);
  return n;
}

static const char *hwdb_get(const char *modalias, const char *key) {
  struct udev_list_entry *entry;

  udev_list_entry_foreach(
      entry, udev_hwdb_get_properties_list_entry(
                 hwdb, modalias,
                 0)) if (strcmp(udev_list_entry_get_name(entry), key) ==
                         0) return udev_list_entry_get_value(entry);

  return NULL;
}

const char *names_vendor(uint16_t vendorid) {
  char modalias[64];

  snprintf(modalias, sizeof(modalias), "usb:v%04X*", vendorid);
  return hwdb_get(modalias, "ID_VENDOR_FROM_DATABASE");
}

const char *names_product(uint16_t vendorid, uint16_t productid) {
  char modalias[64];

  snprintf(modalias, sizeof(modalias), "usb:v%04Xp%04X*", vendorid, productid);
  return hwdb_get(modalias, "ID_MODEL_FROM_DATABASE");
}

int get_vendor_string(char *buf, size_t size, uint16_t vid) {
  const char *cp;

  if (size < 1)
    return 0;
  *buf = 0;
  if (!(cp = names_vendor(vid)))
    return 0;
  return snprintf(buf, size, "%s", cp);
}

int get_product_string(char *buf, size_t size, uint16_t vid, uint16_t pid) {
  const char *cp;

  if (size < 1)
    return 0;
  *buf = 0;
  if (!(cp = names_product(vid, pid)))
    return 0;
  return snprintf(buf, size, "%s", cp);
}

InterfaceManagerNode::InterfaceManagerNode(const std::string name,
                                           const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {

  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&InterfaceManagerNode::timer_callback, this));

  this->can0_status_pub_ =
      this->create_publisher<interfaces::msg::CANStatus>("/can0_status", 10);

  this->can1_status_pub_ =
      this->create_publisher<interfaces::msg::CANStatus>("/can1_status", 10);

  this->usb_list_pub_ =
      this->create_publisher<interfaces::msg::USBList>("/usb_list", 10);

  if (rtnl_open(&rth, 0) < 0)
    exit(1);

  udev = udev_new();
  // if (!udev)
  // return -1;

  hwdb = udev_hwdb_new(udev);

  int err = libusb_init(&ctx);
  if (err) {
    fprintf(stderr, "unable to initialize libusb: %i\n", err);
    // return EXIT_FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Interface Manager Node started");
}

/*

hwdb = udev_hwdb_unref(hwdb);
        udev = udev_unref(udev);
  libusb_exit(ctx);
*/

void InterfaceManagerNode::timer_callback() {
  this->send_can("vcan0", this->can0_status_pub_);
  this->send_can("can1", this->can1_status_pub_);
  this->send_usb();
}

void InterfaceManagerNode::send_can(
    char *name, rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr pub) {
  interfaces::msg::CANStatus msg;

  struct iplink_req req;
  req.n.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
  req.n.nlmsg_flags = NLM_F_REQUEST;
  req.n.nlmsg_type = RTM_GETLINK;
  req.i.ifi_family = AF_PACKET;
  req.i.ifi_change = 0;
  req.i.ifi_flags = 0;
  req.i.ifi_index = 0;
  req.i.ifi_type = 0;

  struct nlmsghdr *n;

  addattr_l(&req.n, sizeof(req), IFLA_IFNAME, name, strlen(name) + 1);

  if (rtnl_talk(&rth, &req.n, &n) < 0) {
    perror("Cannot send link request");
    return;
  }

  struct ifinfomsg *ifi = (ifinfomsg *)NLMSG_DATA(n);

  struct rtattr *tb[IFLA_MAX + 1];

  int len = n->nlmsg_len;

  len -= NLMSG_LENGTH(sizeof(*ifi));
  if (len < 0)
    return;

  parse_rtattr_flags(tb, IFLA_MAX, IFLA_RTA(ifi), len, NLA_F_NESTED);

  msg.up = ifi->ifi_flags & IFF_UP;

  struct rtattr *linkinfo[IFLA_INFO_MAX + 1];
  parse_rtattr_nested(linkinfo, IFLA_INFO_MAX, tb[IFLA_LINKINFO]);
  if (linkinfo[IFLA_INFO_DATA]) {
    struct rtattr *attr[IFLA_CAN_MAX + 1];
    parse_rtattr_nested(attr, IFLA_CAN_MAX, linkinfo[IFLA_INFO_DATA]);

    if (attr[IFLA_CAN_STATE]) {
      msg.state = *(__u32 *)RTA_DATA(attr[IFLA_CAN_STATE]);
    }

    if (attr[IFLA_CAN_BERR_COUNTER]) {
      struct can_berr_counter *bc =
          (can_berr_counter *)RTA_DATA(attr[IFLA_CAN_BERR_COUNTER]);
      msg.txerr = bc->txerr;
      msg.rxerr = bc->rxerr;
    }
  }

  parse_rtattr(tb, IFLA_MAX, IFLA_RTA(ifi),
               n->nlmsg_len - NLMSG_LENGTH(sizeof(*ifi)));

  struct rtnl_link_stats64 _s, *s = &_s;
  int ret;

  ret = get_rtnl_link_stats_rta(s, tb);
  if (ret < 0)
    return;

  msg.tx_packets = s->tx_packets;
  msg.tx_packet_errs = s->tx_errors;
  msg.tx_packet_drops = s->tx_dropped;
  msg.rx_packets = s->rx_packets;
  msg.rx_packet_errs = s->rx_errors;
  msg.rx_packet_drops = s->rx_dropped;

  struct rtattr *xstats = linkinfo[IFLA_INFO_XSTATS];

  struct can_device_stats *stats;

  if (xstats && RTA_PAYLOAD(xstats) == sizeof(*stats)) {
    stats = (can_device_stats *)RTA_DATA(xstats);

    msg.restarts = stats->restarts;
    msg.bus_error = stats->bus_error;
    msg.arbitration_lost = stats->arbitration_lost;
    msg.error_warning = stats->error_warning;
    msg.error_passive = stats->error_passive;
    msg.bus_off = stats->bus_off;
  }

  free(n);

  if (pub) {
    pub->publish(msg);
  }
}

void InterfaceManagerNode::send_usb() {
  interfaces::msg::USBList msg;
  libusb_device **list;
  struct libusb_device_descriptor desc;
  char vendor[128], product[128];
  ssize_t num_devs, i;

  num_devs = libusb_get_device_list(ctx, &list);

  struct libusb_device *dev, *dev_next;
  int bnum, bnum_next, dnum, dnum_next;
  int sorted;
  sorted = 0;
  do {
    sorted = 1;
    for (i = 0; i < num_devs - 1; ++i) {
      dev = list[i];
      dev_next = list[i + 1];
      bnum = libusb_get_bus_number(dev);
      dnum = libusb_get_device_address(dev);
      bnum_next = libusb_get_bus_number(dev_next);
      dnum_next = libusb_get_device_address(dev_next);
      if ((bnum == bnum_next && dnum > dnum_next) || bnum > bnum_next) {
        list[i] = dev_next;
        list[i + 1] = dev;
        sorted = 0;
      }
    }
  } while (!sorted);

  for (i = 0; i < num_devs; ++i) {
    interfaces::msg::USBDevice dev_msg;
    dev = list[i];
    uint8_t bnum = libusb_get_bus_number(dev);
    uint8_t dnum = libusb_get_device_address(dev);

    libusb_get_device_descriptor(dev, &desc);

    char sysfs_name[PATH_MAX];
    bool have_vendor, have_product;

    /* set to "[unknown]" by default unless something below finds a string */
    snprintf(vendor, sizeof(vendor), "[unknown]");
    snprintf(product, sizeof(product), "[unknown]");

    have_vendor = !!get_vendor_string(vendor, sizeof(vendor), desc.idVendor);
    have_product = !!get_product_string(product, sizeof(product), desc.idVendor,
                                        desc.idProduct);

    if (get_sysfs_name(sysfs_name, sizeof(sysfs_name), dev) >= 0) {
      if (!have_vendor)
        read_sysfs_prop(vendor, sizeof(vendor), sysfs_name, "manufacturer");
      if (!have_product)
        read_sysfs_prop(product, sizeof(product), sysfs_name, "product");
    }

    dev_msg.bus = bnum;
    dev_msg.dev = dnum;
    dev_msg.vendor = desc.idVendor;
    dev_msg.product = desc.idProduct;

    std::string name;
    name.append(vendor);
    name.append(" ");
    name.append(product);
    dev_msg.name = name;

    // dumpdev(dev);
    msg.devices.push_back(dev_msg);
  }

  libusb_free_device_list(list, 1);

  if (this->usb_list_pub_) {
    this->usb_list_pub_->publish(msg);
  }
}