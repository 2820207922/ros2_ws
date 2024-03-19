#include "rclcpp/rclcpp.hpp"
#include "common/protocol_rs.h"

#define DEVICE "/dev/ttyUSB0"

class Communication : public ProtocolRS, public rclcpp::Node
{
    public:
        Communication(const char* port_name) : ProtocolRS(port_name), rclcpp::Node("communication_node")
        {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Communication::timer_callback, this));
        }

        void timer_callback()
        {
            // printf("yaw: %f, pitch: %f, roll: %f\n", msg_imu_info.yaw, msg_imu_info.pitch, msg_imu_info.roll);
            msg_move_cmd_t move_cmd;
            move_cmd.vx = 1.1;
            move_cmd.vy = 1.2;
            move_cmd.vw = 1.3;
            move_cmd.top_flag = 0;
            rs_send(MSG_MOVE_CMD_ID, (uint8_t*)&move_cmd, sizeof(msg_move_cmd_t));
        }

    private:

        // rclcpp::Publisher<communication::Message>::SharedPtr publisher_;
        // rclcpp::Subscription<communication::Message>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};

void serial_init()
{
    const char *password = "123"; // 设置密码
  // 要执行的命令
    const char *cmd = "sudo -S chmod 777 /dev/ttyUSB0"; 
    // 打开管道以与sudo交互
    FILE *pipe = popen(cmd, "w");
    if (!pipe) {
        perror("Unable to open pipeline!\n");
        return;
    }
    // 将密码写入sudo
    fprintf(pipe, "%s\n", password);
    fflush(pipe);
    // 关闭管道
    pclose(pipe);
}

int main(int argc, char** argv)
{
    serial_init();
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Communication>(DEVICE);

    // Run the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
