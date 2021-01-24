#pragma once

#include <string>

class SerialPort {
  private:
	int fd;

  public:
	explicit SerialPort(const std::string &tty_path);
	~SerialPort();
	bool send(const void *data, size_t len);
};
