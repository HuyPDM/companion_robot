#include "camera_tracking.h"

const char *address_tty ="/dev/ttyUSB0";

struct termios tty;

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(uint8_t id, int16_t angle, uint16_t time, int serial_p)
{
  int position = int(angle*25/6);
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  write(serial_p, buf, 10);
}

// Open
void Open(int &serial_p){
  serial_p = open(address_tty, O_RDWR);
  // Check for errors
  if (serial_p < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
  }
}

void Config_SerialPort(int serial_p){
  // Read in existing settings, and handle any error
  if(tcgetattr(serial_p, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      // return 1;
      exit(1);
  }
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, Baud_rate);
  cfsetospeed(&tty, Baud_rate);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_p, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      exit(1);
  }
}

void CameraTracking(int frameWidth,int recX, int recWidth, int &pan, int serial_p, int time)
{
  int objX = recX + recWidth/2; 
  int errorPan=objX-frameWidth/2;
  if(abs(errorPan)>15)
    pan = pan - errorPan/75;
  
  if(pan>180)
  {
    pan = 180;
    // std:: cout<<"pan out of range "<<std::endl;
  }
  if(pan<0)
  {
    pan = 0;
    // std:: cout<<"pan out of range "<<std::endl;
  }
  //std:: cout<<"pan"<<pan<<std::endl;
  LobotSerialServoMove(1, pan, time, serial_p);
}
