#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <fstream>
#include <atomic>
#include <cstdlib>
#include <pthread.h>    // ← per pthread_cancel

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("ros2_talker"), running_(true), last_value_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/micro_ros_sub_topic", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
    reader_thread_ = std::thread(&Talker::read_loop, this);

    // Avvia nuova bash per scrivere nella named pipe
    std::system(
      "gnome-terminal -- bash -c '"
      "echo Inserisci un numero e premi invio; "
      "while true; do read line; echo $line > /tmp/input_pipe; done'"
    );
  }

  ~Talker()
  {
    // 1) segnalo al thread di fermarsi (opzionale)
    running_ = false;

    // 2) termino brutalmente il thread
    pthread_cancel(reader_thread_.native_handle());
    reader_thread_.detach();

    // 3) rimuovo la pipe
    std::system("rm -f /tmp/input_pipe");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = last_value_.load();
    RCLCPP_INFO(this->get_logger(), "Sto pubblicando: %d", message.data);
    publisher_->publish(message);
  }

  void read_loop()
  {
    // Creazione della named pipe (se non esiste)
    std::system("mkfifo -m 0666 /tmp/input_pipe");

    while (running_) {
      std::ifstream pipe("/tmp/input_pipe");
      if (!pipe.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Impossibile aprire la pipe.");
        std::this_thread::sleep_for(1s);
        continue;
      }

      std::string line;
      while (running_ && std::getline(pipe, line)) {
        try {
          int value = std::stoi(line);
          last_value_.store(value);
          RCLCPP_INFO(this->get_logger(), "Ricevuto da pipe: %d", value);
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Input non valido: %s", line.c_str());
        }
      }

      RCLCPP_WARN(this->get_logger(), "EOF sulla pipe, riapro...");
      pipe.close();
      std::this_thread::sleep_for(100ms);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread reader_thread_;
  std::atomic<bool> running_;
  std::atomic<int> last_value_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}



