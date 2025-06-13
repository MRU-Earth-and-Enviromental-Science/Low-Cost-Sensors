#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/poll.h>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

constexpr char IN_PORT[] = "/dev/ttyUSB0";
constexpr char OUT_PORT[] = "/dev/ttyAMA0";
constexpr speed_t BAUD = B115200;
constexpr int TIMEOUT_MS = 1000;

class SerialPort
{
public:
    SerialPort() : fd_(-1) {}
    ~SerialPort()
    {
        if (fd_ != -1)
            close(fd_);
    }

    bool openPort(const char *path, speed_t baud)
    {
        fd_ = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ == -1)
        {
            perror(path);
            return false;
        }
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
        {
            perror("tcgetattr");
            return false;
        }
        cfmakeraw(&tty);
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS; // no HW flow-control
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            perror("tcsetattr");
            return false;
        }
        return true;
    }

    ssize_t readByte(char &c) { return ::read(fd_, &c, 1); }
    ssize_t writeData(const std::string &s)
    {
        return ::write(fd_, s.data(), s.size());
    }

private:
    int fd_;
};

uint8_t hex2byte(const std::string &hx)
{
    return static_cast<uint8_t>(std::strtoul(hx.c_str(), nullptr, 16));
}

bool validChecksum(const std::string &nmea)
{
    auto star = nmea.find('*');
    if (star == std::string::npos || star + 3 > nmea.size())
        return false;
    uint8_t given = hex2byte(nmea.substr(star + 1, 2));

    uint8_t calc = 0;
    for (size_t i = 1; i < star; ++i) // XOR from char after $ until '*'
        calc ^= static_cast<uint8_t>(nmea[i]);

    return calc == given;
}

double dm2deg(const std::string &dm, const std::string &hemisphere)
{
    if (dm.empty())
        return 0.0;
    double val = std::stod(dm);
    int degs = static_cast<int>(val / 100);
    double mins = val - (degs * 100);
    double dec = degs + mins / 60.0;
    if (hemisphere == "S" || hemisphere == "W")
        dec = -dec;
    return dec;
}

struct Fix
{
    std::string time_hms;
    double latitude = 0.0;
    double longitude = 0.0;
};

bool parseGPRMC(const std::string &line, Fix &fix)
{
    if (line.rfind("$GPRMC,", 0) != 0)
        return false;
    if (!validChecksum(line))
        return false;

    std::vector<std::string> f;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ','))
        f.push_back(token);
    if (f.size() < 7)
        return false;

    const std::string &status = f[2];
    if (status != "A")
        return false;

    const std::string &t = f[1];
    if (t.size() < 6)
        return false;
    fix.time_hms = t.substr(0, 2) + ":" + t.substr(2, 2) + ":" + t.substr(4, 2);

    fix.latitude = dm2deg(f[3], f[4]);
    fix.longitude = dm2deg(f[5], f[6]);

    return true;
}

int main()
{
    SerialPort in, out;
    if (!in.openPort(IN_PORT, BAUD) ||
        !out.openPort(OUT_PORT, BAUD))
    {
        std::cerr << "Failed to open serial ports.\n";
        return 1;
    }
    std::cout << "Listening on " << IN_PORT
              << " and forwarding clean GPRMC sentences to "
              << OUT_PORT << std::endl;

    std::string currentLine;
    pollfd pfd{.fd = 0, .events = 0};

    while (true)
    {
        char c;
        if (in.readByte(c) == 1)
        {
            if (c == '\r')
                continue; // ignore CR
            if (c == '\n')
            {
                if (!currentLine.empty())
                {
                    Fix fix;
                    if (parseGPRMC(currentLine, fix))
                    {
                        // Forward full line with CRLF
                        out.writeData(currentLine + "\r\n");

                        // Show parsed data
                        std::cout << std::fixed << std::setprecision(6);
                        std::cout << "[" << fix.time_hms << "]  "
                                  << "Lat: " << fix.latitude
                                  << "  Lon: " << fix.longitude
                                  << std::endl;
                    }
                    currentLine.clear();
                }
            }
            else
            {
                currentLine += c;
                if (currentLine.size() > 120)
                    currentLine.clear();
            }
        }
        else
        {
            usleep(1000);
        }
    }
    return 0;
}