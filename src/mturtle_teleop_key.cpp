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

namespace {
constexpr char KEYCODE_U = 'u';
constexpr char KEYCODE_I = 'i';
constexpr char KEYCODE_O = 'o';
constexpr char KEYCODE_L = 'l';
constexpr char KEYCODE_PERIOD = '.';
constexpr char KEYCODE_COMMA = ',';
constexpr char KEYCODE_M = 'm';
constexpr char KEYCODE_J = 'j';
constexpr char KEYCODE_K = 'k';
constexpr char KEYCODE_Q = 'q';
constexpr char KEYCODE_W = 'w';
constexpr char KEYCODE_X = 'x';
constexpr char KEYCODE_E = 'e';
constexpr char KEYCODE_C = 'c';
constexpr char KEYCODE_R = 'r';
constexpr char KEYCODE_V = 'v';
} // namespace

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

      if (record.Event.KeyEvent.wVirtualKeyCode == 0x55) {
        c = KEYCODE_U;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x49) {
        c = KEYCODE_I;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x4F) {
        c = KEYCODE_O;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x4C) {
        c = KEYCODE_L;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_OEM_PERIOD) {
        c = KEYCODE_PERIOD;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == VK_OEM_COMMA) {
        c = KEYCODE_COMMA;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x4D) {
        c = KEYCODE_M;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x4A) {
        c = KEYCODE_J;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x4B) {
        c = KEYCODE_K;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x51) {
        c = KEYCODE_Q;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x57) {
        c = KEYCODE_W;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x58) {
        c = KEYCODE_X;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x45) {
        c = KEYCODE_E;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x43) {
        c = KEYCODE_C;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x52) {
        c = KEYCODE_R;
      } else if (record.Event.KeyEvent.wVirtualKeyCode == 0x56) {
        c = KEYCODE_V;
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
    nh_ = rclcpp::Node::make_shared("mteleop_turtle");
    nh_->declare_parameter("scale_angular", 2.0);
    nh_->declare_parameter("scale_linear", 2.0);

    twist_pub_ =
        nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }

  int keyLoop() {
    char c;

    std::thread{std::bind(&TeleopTurtle::spin, this)}.detach();

    std::puts(R"(Reading from keyboard
---------------------------
O|L|.|,|M|J|U|I keys to move around.
'K' to stop the turtle.
W|X to increase/decrease maximum speeds by 10%.
E|C to increase/decrease linear speed by 10%.
R|V to increase/decrease angular speed by 10%.
'Q' to quit.
)");

    double linear = 0.5;
    double angular = 1.0;

    while (running) {
      // get the next event from the keyboard
      try {
        c = input_.readOne();
      } catch (const std::runtime_error &) {
        std::perror("read():");
        return -1;
      }

      RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

      bool publish = false;
      double lin = 0.0;
      double ang = 0.0;

      switch (c) {
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        linear *= 1.1;
        angular *= 1.1;
        break;
      case KEYCODE_X:
        RCLCPP_DEBUG(nh_->get_logger(), "X");
        linear *= 0.9;
        angular *= 0.9;
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        linear *= 1.1;
        break;
      case KEYCODE_C:
        RCLCPP_DEBUG(nh_->get_logger(), "C");
        linear *= 0.9;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "R");
        angular *= 1.1;
        break;
      case KEYCODE_V:
        RCLCPP_DEBUG(nh_->get_logger(), "V");
        angular *= 0.9;
        break;
      case KEYCODE_U:
        RCLCPP_DEBUG(nh_->get_logger(), "U");
        lin = linear;
        ang = angular;
        publish = true;
        break;
      case KEYCODE_I:
        RCLCPP_DEBUG(nh_->get_logger(), "I");
        lin = linear;
        ang = 0.0;
        publish = true;
        break;
      case KEYCODE_O:
        RCLCPP_DEBUG(nh_->get_logger(), "O");
        lin = linear;
        ang = -angular;
        publish = true;
        break;
      case KEYCODE_L:
        RCLCPP_DEBUG(nh_->get_logger(), "L");
        lin = 0.0;
        ang = -angular;
        publish = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), ".");
        lin = -linear;
        ang = angular;
        publish = true;
        break;
      case KEYCODE_COMMA:
        RCLCPP_DEBUG(nh_->get_logger(), ",");
        lin = -linear;
        ang = 0.0;
        publish = true;
        break;
      case KEYCODE_M:
        RCLCPP_DEBUG(nh_->get_logger(), "M");
        lin = -linear;
        ang = -angular;
        publish = true;
        break;
      case KEYCODE_J:
        RCLCPP_DEBUG(nh_->get_logger(), "J");
        lin = 0.0;
        ang = angular;
        publish = true;
        break;
      case KEYCODE_K:
        RCLCPP_DEBUG(nh_->get_logger(), "J");
        lin = 0.0;
        ang = 0.0;
        publish = true;
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

      if (running && publish) {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = nh_->get_parameter("scale_angular").as_double() * ang;
        twist.linear.x = nh_->get_parameter("scale_linear").as_double() * lin;
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
