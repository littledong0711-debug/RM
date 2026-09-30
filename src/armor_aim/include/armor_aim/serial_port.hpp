#ifndef ARMOR_AIM__SERIAL_PORT_HPP_
#define ARMOR_AIM__SERIAL_PORT_HPP_

#include <cstdint>
#include <string>

namespace armor_aim
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool Open(const std::string & device, int baud_rate);
  void Close();
  bool IsOpen() const;
  bool SendAim(float angle_degrees) const;
  bool SendFire() const;

private:
  bool WriteAll(const std::uint8_t * data, std::size_t size) const;

  int file_descriptor_{-1};
};

}  // namespace armor_aim

#endif  // ARMOR_AIM__SERIAL_PORT_HPP_
