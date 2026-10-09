#include "npc_controller.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace controller
{

    int ControllerNode::open_socket(const std::string &iface)
    {
        RCLCPP_INFO(get_logger(), "Socket open...");
        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        RCLCPP_INFO(get_logger(), "CAN socket opened fd=%d", sock);
        if (sock < 0) {
            RCLCPP_ERROR(get_logger(),
                        "socket() failed for interface %s: %s",
                        iface.c_str(), strerror(errno));
            return -1;
        }

        struct ifreq ifr {};
        strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(get_logger(),
                        "ioctl(SIOCGIFINDEX) failed for %s: %s",
                        iface.c_str(), strerror(errno));
            close(sock);
            return -1;
        }

        RCLCPP_INFO(get_logger(), "Socket bind...");
        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(),
                        "bind() failed for %s: %s",
                        iface.c_str(), strerror(errno));
            close(sock);
            return -1;
        }

        RCLCPP_INFO(get_logger(), "CAN socket bound fd=%d", sock);
        return sock;
    }

    void ControllerNode::can_write(int sock, const struct can_frame &frame)
    {
        if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(frame)) {
        perror("Write");
        return;
        }
    }

    void ControllerNode::finalizeCanMessage(const PreparedCanMessage &message)
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

    std::optional<ControllerNode::PreparedCanMessage>
    ControllerNode::prepareCanMessage(std::string_view message_name)
    {
        const auto *message = findMessageByName(message_name);
        if (!message) {
        RCLCPP_ERROR(get_logger(),
                    "CAN metadata missing for message %.*s",
                    static_cast<int>(message_name.size()),
                    message_name.data());
        return std::nullopt;
        }
        PreparedCanMessage prepared{};
        prepared.metadata = message;
        prepared.frame.can_id = message->id;
        prepared.frame.can_dlc = message->dlc;
        std::memset(prepared.frame.data, 0, sizeof(prepared.frame.data));
        return prepared;
    }

} // namespace controller