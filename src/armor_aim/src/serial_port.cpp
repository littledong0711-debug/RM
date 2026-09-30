#include "armor_aim/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>

namespace armor_aim
{
namespace
{

speed_t ToTermiosSpeed(const int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}

}  // namespace

SerialPort::~SerialPort() {Close();}

bool SerialPort::Open(const std::string & device, const int baud_rate)
{
  Close();
  file_descriptor_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (file_descriptor_ < 0) {
    return false;
  }

  termios settings{};
  if (tcgetattr(file_descriptor_, &settings) != 0) {
    Close();
    return false;
  }
  cfmakeraw(&settings);
  const speed_t speed = ToTermiosSpeed(baud_rate);
  cfsetispeed(&settings, speed);
  cfsetospeed(&settings, speed);
  settings.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  settings.c_cflag &= static_cast<tcflag_t>(~PARENB);
  settings.c_cflag = (settings.c_cflag & static_cast<tcflag_t>(~CSIZE)) | CS8;
  if (tcsetattr(file_descriptor_, TCSANOW, &settings) != 0) {
    Close();
    return false;
  }
  tcflush(file_descriptor_, TCIOFLUSH);
  return true;
}

void SerialPort::Close()
{
  if (file_descriptor_ >= 0) {
    close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

bool SerialPort::IsOpen() const {return file_descriptor_ >= 0;}

bool SerialPort::WriteAll(const std::uint8_t * data, const std::size_t size) const
{
  // write可能只发送部分字节，循环补齐；错误由节点关闭串口后重连。
  if (!IsOpen()) {
    return false;
  }
  std::size_t written = 0;
  while (written < size) {
    const ssize_t result = write(file_descriptor_, data + written, size - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

bool SerialPort::SendAim(const float angle_degrees) const
{
  // 小端x86 WSL：0x01后跟4字节角度，无换行；跨端序移植须显式编码。
  static_assert(sizeof(float) == 4U, "Protocol requires IEEE-754 float32");
  std::array<std::uint8_t, 5> packet{0x01U, 0U, 0U, 0U, 0U};
  std::memcpy(packet.data() + 1, &angle_degrees, sizeof(angle_degrees));
  return WriteAll(packet.data(), packet.size());
}

bool SerialPort::SendFire() const
{
  constexpr std::uint8_t packet = 0x02U;
  return WriteAll(&packet, 1U);
}

}  // namespace armor_aim
