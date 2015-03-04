#include "serialStuff.h"

int openPort(char* device)
{
  int flags=0;
  //read write
  flags=O_RDWR;
  int m_fd=open(device, flags | O_NDELAY);
  // flushing is to be done after opening. This prevents first read and write to be spam'ish.
  tcflush(m_fd, TCIOFLUSH);
  int n = fcntl(m_fd, F_GETFL, 0);
  fcntl(m_fd, F_SETFL, n & ~O_NDELAY);
  struct termios newtio;
  tcgetattr(m_fd, &newtio);
  cfsetospeed(&newtio, B115200);
  cfsetispeed(&newtio, B115200);
  newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~(PARENB | PARODD);
  newtio.c_cflag &= ~CRTSCTS;
  newtio.c_cflag &= ~CSTOPB;
  newtio.c_iflag=IGNBRK;
  newtio.c_iflag &= ~(IXON|IXOFF|IXANY);
  newtio.c_lflag=0;
  newtio.c_oflag=0;
  newtio.c_cc[VTIME]=1;
  newtio.c_cc[VMIN]=1;
  tcsetattr(m_fd, TCSANOW, &newtio);
  int mcs=0;
  ioctl(m_fd, TIOCMGET, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(m_fd, TIOCMSET, &mcs);
  tcgetattr(m_fd, &newtio);
  newtio.c_cflag &= ~CRTSCTS;
  tcsetattr(m_fd, TCSANOW, &newtio);
  return m_fd;
}
