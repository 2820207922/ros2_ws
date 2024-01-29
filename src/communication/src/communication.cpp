#include "rclcpp/rclcpp.hpp"
#include "common/protocol_rs.h"

#define DEVICE "/dev/ttyCH341USB0"

class Communication : public ProtocolRS, public rclcpp::Node
{
    public:
        Communication(const char* port_name) : ProtocolRS(port_name), rclcpp::Node("communication_node")
        {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Communication::timer_callback, this));
        }

        void timer_callback()
        {
            game_result_t test;
            test.winner = 2;
            rs_send(GAME_RESULT_ID, (uint8_t *)&test, sizeof(game_result_t));

            test = *(game_result_t *)get_rs_data(GAME_RESULT_ID);
            printf("winner: %d\n", test.winner);
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
    const char *cmd = "sudo -S chmod 777 /dev/ttyCH341USB0"; 
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
