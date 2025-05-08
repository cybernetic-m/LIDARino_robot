#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cstring>

int main() {
    int serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }

    struct termios tty;
    tcgetattr(serial_port, &tty);

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // No flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcsetattr(serial_port, TCSANOW, &tty);

    char read_buf[256];
    while (true) {
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        if (num_bytes < 0) {
            std::cerr << "Error reading from serial port" << std::endl;
            break;
        }

        std::cout << "Received: " << read_buf;
    }

    close(serial_port);
    return 0;
}
