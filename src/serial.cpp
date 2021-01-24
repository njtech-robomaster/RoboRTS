#include "serial.h"
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

#define BAUD_RATE B1152000

SerialPort::SerialPort(const std::string &tty_path) {
	fd = open(tty_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd == -1) {
		throw std::runtime_error("Couldn't open serial port " + tty_path);
	}

	termios tOption;
	tcgetattr(fd, &tOption);
	cfmakeraw(&tOption);
	cfsetispeed(&tOption, BAUD_RATE);
	cfsetospeed(&tOption, BAUD_RATE);
	tcsetattr(fd, TCSANOW, &tOption);
	tOption.c_cflag &= ~PARENB;
	tOption.c_cflag &= ~CSTOPB;
	tOption.c_cflag &= ~CSIZE;
	tOption.c_cflag |= CS8;
	tOption.c_cflag &= ~INPCK;
	tOption.c_cflag |= (BAUD_RATE | CLOCAL | CREAD);
	tOption.c_cflag &= ~(INLCR | ICRNL);
	tOption.c_cflag &= ~(IXON);
	tOption.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tOption.c_oflag &= ~OPOST;
	tOption.c_oflag &= ~(ONLCR | OCRNL);
	tOption.c_iflag &= ~(ICRNL | INLCR);
	tOption.c_iflag &= ~(IXON | IXOFF | IXANY);
	tOption.c_cc[VTIME] = 1;
	tOption.c_cc[VMIN] = 1;
	tcflush(fd, TCIOFLUSH);
}

SerialPort::~SerialPort() {
	tcflush(fd, TCIOFLUSH);
	if (close(fd) == -1) {
		std::cerr << "Couldn't close serial port fd " << fd << std::endl;
	}
}

bool SerialPort::send(const void *data, size_t len) {
	tcflush(fd, TCOFLUSH);

	int bytes_sent;
	try {
		bytes_sent = write(fd, data, len);
	} catch (std::exception &e) {
		std::cerr << "Error writing to serial port: " << e.what() << std::endl;
		return false;
	}

	if (bytes_sent == -1) {
		std::cerr << "Error writing to serial port, no bytes sent" << std::endl;
		return false;
	} else if (bytes_sent < static_cast<int>(len)) {
		std::cerr << "Error writing to serial port, " << bytes_sent << "/"
		          << len << " bytes sent" << std::endl;
		return false;
	} else {
		return true;
	}
}
