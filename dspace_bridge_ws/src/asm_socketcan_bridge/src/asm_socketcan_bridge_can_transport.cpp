#include "asm_socketcan_bridge.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace asm_socketcan_bridge {

  int AsmSocketCanBridgeNode::open_socket(const std::string &iface)
  {
    RCLCPP_INFO(get_logger(), "Socket open...");
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
      RCLCPP_ERROR(get_logger(),
                  "socket() failed for interface %s: %s",
                  iface.c_str(), strerror(errno));
      return -1;
    }

    struct ifreq ifr {};
    strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(),
                  "ioctl(SIOCGIFINDEX) failed for %s: %s",
                  iface.c_str(), strerror(errno));
      close(s);
      return -1;
    }

    RCLCPP_INFO(get_logger(), "Socket bind...");
    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(),
                  "bind() failed for %s: %s",
                  iface.c_str(), strerror(errno));
      close(s);
      return -1;
    }

    return s;
  }

  void AsmSocketCanBridgeNode::can_write(int sock, const struct can_frame &frame)
  {
    if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(frame)) {
      perror("Write");
      return;
    }
  }

  void AsmSocketCanBridgeNode::finalizeCanMessage(const PreparedCanMessage &message)
  {
    if (sentMessagePrinting && message.metadata) {
      RCLCPP_INFO(get_logger(), "can_out::%s", message.metadata->name);
      RCLCPP_INFO(get_logger(),
                  "send: 0x%03X [%d] ",
                  message.metadata->id,
                  static_cast<int>(message.frame.can_dlc));
      for (int i = 0; i < message.frame.can_dlc; i++) {
        RCLCPP_INFO(get_logger(), "send: %02X ", message.frame.data[i]);
      }
    }
    const std::lock_guard<std::mutex> socket_lock(can_socket_mutex_);
    can_write(can_socket, message.frame);
  }

} // namespace asm_socketcan_bridge
