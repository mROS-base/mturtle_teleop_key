#include <functional>
#include <stdexcept>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <cstdio>
#include <cstring>
#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_Q = 0x71;

bool running = true;

class KeyboardReader final {
public:
  KeyboardReader() {
#ifdef _WIN32
    hstdin_ = GetStdHandle(STD_INPUT_HANDLE);
    if (hstdin_ == INVALID_HANDLE_VALUE) {
      throw std::runtime_error("Failed to get stdin handle");
    }
    if (!GetConsoleMode(hstdin_, &old_mode_)) {
      throw std::runtime_error("Failed to get old console mode");
    }
    DWORD new_mode = ENABLE_PROCESSED_INPUT; // for Ctrl-C processing
    if (!SetConsoleMode(hstdin_, new_mode)) {
      throw std::runtime_error("Failed to set new console mode");
    }
#else
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0) {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    std::memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0) {
      throw std::runtime_error("Failed to set new console mode");
    }
#endif
  }

  char readOne() {
    char c = 0;

#ifdef _WIN32
    INPUT_RECORD record;
    DWORD num_read;
    switch (WaitForSingleObject(hstdin_, 100)) {
    case WAIT_OBJECT_0:
      if (!ReadConsoleInput(hstdin_, &record, 1, &num_read)) {
        throw std::runtime_error("Read failed");
      }

      if (record.EventType != KEY_EVENT || !record.Event.KeyEvent.bKeyDown) {
        break;
      }

      if (record.Event.KeyEvent.wVirtualKeyCode == VK_LEFT) {
        c = KEYCODE_LEFT;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_UP) {
        c = KEYCODE_UP;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT) {
        c = KEYCODE_RIGHT;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_DOWN) {
        c = KEYCODE_DOWN;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x51) {
        c = KEYCODE_Q;
      }
      break;

    case WAIT_TIMEOUT:
      break;
    }

#else
    int rc = read(0, &c, 1);
    if (rc < 0) {
      throw std::runtime_error("read failed");
    }
#endif

    return c;
  }

  ~KeyboardReader() {
#ifdef _WIN32
    SetConsoleMode(hstdin_, old_mode_);
#else
    tcsetattr(0, TCSANOW, &cooked_);
#endif
  }

private:
#ifdef _WIN32
  HANDLE hstdin_;
  DWORD old_mode_;
#else
  struct termios cooked_;
#endif
};

class TeleopTurtle final {
public:
  TeleopTurtle() {
    nh_ = rclcpp::Node::make_shared("teleop_turtle");
    nh_->declare_parameter("scale_angular", 2.0);
    nh_->declare_parameter("scale_linear", 2.0);

    twist_pub_ =
        nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
  }

  int keyLoop() {
    char c;

    std::thread{std::bind(&TeleopTurtle::spin, this)}.detach();

    std::puts("Reading from keyboard");
    std::puts("---------------------------");
    std::puts("Use arrow keys to move the turtle.");
    std::puts("'Q' to quit.");

    while (running) {
      // get the next event from the keyboard
      try {
        c = input_.readOne();
      } catch (const std::runtime_error &) {
        std::perror("read():");
        return -1;
      }

      double linear = 0.0;
      double angular = 0.0;

      RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

      switch (c) {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        angular = 1.0;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        angular = -1.0;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        linear = 1.0;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        linear = -1.0;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        running = false;
        break;
      default:
        // This can happen if the read returned when there was no data, or
        // another key was pressed.  In these cases, just silently ignore the
        // press.
        break;
      }

      if (running && (linear != 0.0 || angular != 0.0)) {
        geometry_msgs::msg::Twist twist;
        twist.angular.z =
            nh_->get_parameter("scale_angular").as_double() * angular;
        twist.linear.x =
            nh_->get_parameter("scale_linear").as_double() * linear;
        twist_pub_->publish(twist);
      }
    }

    return 0;
  }

private:
  void spin() { rclcpp::spin(nh_); }

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  KeyboardReader input_;
};

#ifdef _WIN32
BOOL WINAPI quit(DWORD ctrl_type) {
  (void)ctrl_type;
  running = false;
  return true;
}
#else
void quit(int sig) {
  (void)sig;
  running = false;
}
#endif

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#ifdef _WIN32
  SetConsoleCtrlHandler(quit, TRUE);
#else
  std::signal(SIGINT, quit);
#endif

  TeleopTurtle teleop_turtle;

  int rc = teleop_turtle.keyLoop();

  rclcpp::shutdown();

  return rc;
}
