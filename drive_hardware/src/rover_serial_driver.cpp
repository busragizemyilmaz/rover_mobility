#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp> // For encoder data
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <vector>
#include <cstring>
#include <chrono> 

using namespace std;
using std::placeholders::_1;

/*
 * RoverSerialDriver
 *
 * Responsibilities:
 * - Subscribe to /wheel_speeds topic (Float32MultiArray)
 * - Convert normalized wheel speeds [-1, 1] to byte values [0, 255]
 * - Transmit packet to STM32 via UART
 * - Enforce communication timeout safety
 *
* Packet Format (6 bytes):
 * [0] 0xFF        — Header
 * [1] mode        — Control mode (0=PWM, 1=PID)
 * [2] Front Left  — sol_on
 * [3] Rear Left   — sol_ark
 * [4] Front Right — sag_on
 * [5] Rear Right  — sag_ark
 */

class RoverSerialDriver : public rclcpp::Node
{
public:
    RoverSerialDriver()
        : Node("rover_serial_driver")
    {
        // ---------------- PARAMETERS ----------------
        this->declare_parameter<string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("timeout_sec", 0.5);

        port_name_   = this->get_parameter("port_name").as_string();
        baud_rate_   = this->get_parameter("baud_rate").as_int();
        timeout_sec_ = this->get_parameter("timeout_sec").as_double();

        // ---------------- SERIAL INITIALIZATION ----------------
        if (openSerialPort())
        {
            RCLCPP_INFO(this->get_logger(),
                        "Serial connected: %s @ %d baud",
                        port_name_.c_str(), baud_rate_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to open serial port: %s",
                         port_name_.c_str());
        }

        // ---------------- ROS INTERFACES ----------------
        // 5-Element array from teleop [ControlMode, FL, RL, FR, RR]
        speed_subscriber_ =
            this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/wheel_speeds",
                10,
                bind(&RoverSerialDriver::speedCallback, this, _1));
                
        // Encoder raw data publisher  
        encoder_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/encoder_raw", 10);

        // Timer to continuously read the serial port (50 Hz / 20ms)
        read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), 
            bind(&RoverSerialDriver::read_serial_data, this));

        // Control loop runs at fixed 20 Hz (50 ms)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            bind(&RoverSerialDriver::controlLoop, this));

        last_msg_time_ = this->now();
        wheel_speeds_  = {0.0f, 0.0f, 0.0f, 0.0f};

        RCLCPP_INFO(this->get_logger(), "Rover Serial Driver Started. Full-Duplex (Read/Write) Active.");
    }

    ~RoverSerialDriver()
    {
        // Send neutral packet on shutdown to stop the rover
        wheel_speeds_ = {0.0f, 0.0f, 0.0f, 0.0f};
        sendPacket();

        if (serial_fd_ != -1)
            close(serial_fd_);
    }

private:

    // ------------------------------------------------
    // Callback: receives new wheel speeds
    // ------------------------------------------------
    void speedCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        last_msg_time_ = this->now();

        // Validate packet structure
        if (msg->data.size() >= 5)
        {
            // Extract Mode (Index 0)
            uint8_t new_mode = static_cast<uint8_t>(msg->data[0]);
            if (new_mode != control_mode_)
            {
                control_mode_ = new_mode;
                RCLCPP_INFO(this->get_logger(),
                            "Control mode updated: %s (%d)",
                            control_mode_ == 1 ? "PID" : "PWM",
                            control_mode_);
            }

            // Extract Speeds (Indices 1 to 4)
            wheel_speeds_ = {msg->data[1], msg->data[2], msg->data[3], msg->data[4]};
        }
    }
    
    // ------------------------------------------------
    // Callback: reads encoder data from STM32
    // ------------------------------------------------
    void read_serial_data()
    {
        if (serial_fd_ < 0) return;

        uint8_t temp_buf[256];
        int bytes_read = read(serial_fd_, temp_buf, sizeof(temp_buf));

        if (bytes_read > 0) 
        {
            // Append new incoming data to the buffer
            rx_buffer_.insert(rx_buffer_.end(), temp_buf, temp_buf + bytes_read);
        }

        // Each encoder packet is 20 bytes
        while (rx_buffer_.size() >= 20) 
        {
            // Check headers: 0xBA, 0xAB and 0xCD, 0xDC
            if (rx_buffer_[0] == 0xBA && rx_buffer_[1] == 0xAB &&
                rx_buffer_[10] == 0xCD && rx_buffer_[11] == 0xDC) 
            {
                // Bit Shifting to convert 4 bytes into a 32-bit Integer (Equivalent to Python struct.unpack)
                uint32_t v1 = (rx_buffer_[2] << 24) | (rx_buffer_[3] << 16) | (rx_buffer_[4] << 8) | rx_buffer_[5];
                uint32_t v2 = (rx_buffer_[6] << 24) | (rx_buffer_[7] << 16) | (rx_buffer_[8] << 8) | rx_buffer_[9];
                uint32_t v3 = (rx_buffer_[12] << 24) | (rx_buffer_[13] << 16) | (rx_buffer_[14] << 8) | rx_buffer_[15];
                uint32_t v4 = (rx_buffer_[16] << 24) | (rx_buffer_[17] << 16) | (rx_buffer_[18] << 8) | rx_buffer_[19];

                // Publish the raw data to the ROS network
                auto enc_msg      = std_msgs::msg::Int64MultiArray();
                // Explicitly cast to int64_t to match the ROS message type
                enc_msg.data      = {
                    static_cast<int64_t>(v1),
                    static_cast<int64_t>(v2),
                    static_cast<int64_t>(v3),
                    static_cast<int64_t>(v4)
                };
                encoder_pub_->publish(enc_msg);

                // Erase the processed 20 bytes from the buffer
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 20);
            } 
            else 
            {
                // If headers don't match, we might be out of sync. Drop 1 byte and search again.
                rx_buffer_.erase(rx_buffer_.begin());
            }
        }
    }

    // ------------------------------------------------
    // Main control loop (20 Hz)
    // Handles:
    //  - Timeout detection
    //  - Packet transmission
    // ------------------------------------------------
    void controlLoop()
    {
        // Attempt reconnection if port was not opened
       if (serial_fd_ == -1)
        {
            if (openSerialPort()) {
                RCLCPP_INFO(this->get_logger(), "✅ Serial reconnected successfully!");
            }
            return;
        }

        double elapsed = (this->now() - last_msg_time_).seconds();

        // Safety: stop rover if command stream lost
        if (elapsed > timeout_sec_)
        {
            if (!emergency_active_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Command timeout. Sending neutral values.");
                emergency_active_ = true;
            }

            wheel_speeds_ = {0.0f, 0.0f, 0.0f, 0.0f};
        }
        else if (emergency_active_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Command stream restored.");
            emergency_active_ = false;
        }

        sendPacket();
    }

    // ------------------------------------------------
    // Maps normalized speed [-1, 1] to byte [0, 255]
    //  -1 → 0
    //   0 → 127
    //  +1 → 255
    // ------------------------------------------------
    uint8_t mapSpeedToByte(float speed)
    {
        float clamped = max(-1.0f, min(speed, 1.0f));
        return static_cast<uint8_t>((clamped + 1.0f) * 127.5f);
    }

    // ------------------------------------------------
    // Builds and sends packet to STM32
    // ------------------------------------------------
    void sendPacket()
    {
        if (serial_fd_ == -1)
            return;

        uint8_t packet[6];

        packet[0] = 0xFF;
        packet[1] = control_mode_;
        packet[2] = mapSpeedToByte(wheel_speeds_[0]);  // Front Left  (sol_on)
        packet[3] = mapSpeedToByte(wheel_speeds_[1]);  // Rear Left   (sol_ark)
        packet[4] = mapSpeedToByte(wheel_speeds_[2]);  // Front Right (sag_on)
        packet[5] = mapSpeedToByte(wheel_speeds_[3]);  // Rear Right  (sag_ark)

        ssize_t bytes_written = write(serial_fd_, packet, sizeof(packet));

        if (bytes_written != sizeof(packet))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "❌ Serial write error. Cable disconnected!");
            
            // Auto-Reconnect mekanizmasını tetiklemek için portu kapat
            close(serial_fd_);
            serial_fd_ = -1; 
        }
    }

    // ------------------------------------------------
    // Configures and opens UART using POSIX termios
    // ------------------------------------------------
    bool openSerialPort()
    {
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

        if (serial_fd_ < 0)
            return false;

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        speed_t baud_constant;

        switch (baud_rate_)
        {
            case 115200: baud_constant = B115200; break;
            case 57600:  baud_constant = B57600;  break;
            case 38400:  baud_constant = B38400;  break;
            default:
                RCLCPP_WARN(this->get_logger(),
                            "Unsupported baud rate. Defaulting to 115200.");
                baud_constant = B115200;
        }

        cfsetospeed(&tty, baud_constant);
        cfsetispeed(&tty, baud_constant);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        return true;
    }

private:
    // Subscribers & Publishers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr speed_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    rclcpp::Time last_msg_time_;
    string port_name_;
    int baud_rate_;
    double timeout_sec_;
    int serial_fd_       = -1;
    uint8_t control_mode_    = 0;      // 0=PWM, 1=PID
    bool emergency_active_ = false;

    vector<float> wheel_speeds_;
    vector<uint8_t> rx_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RoverSerialDriver>());
    rclcpp::shutdown();
    return 0;
}
