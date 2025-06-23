#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <cstdint>
#include <chrono>
#include <thread>


const char *IN_PORT = "/dev/ttyUSB0";
const char *OUT_PORT = "/dev/ttyAMA0";
constexpr speed_t BAUD = B115200;

double parseCoord(const std::string &dm, const std::string &hemi)
{
    if (dm.empty())
        return 0.0;
    double val = std::stod(dm);
    int deg = int(val / 100);
    double min = val - deg * 100;
    double dec = deg + min / 60.0;
    return (hemi == "S" || hemi == "W") ? -dec : dec;
}

bool hasValidChecksum(const std::string &sentence)
{
    auto star = sentence.find('*');
    if (star == std::string::npos || star + 2 >= sentence.size())
        return false;
    uint8_t expected = std::strtoul(sentence.substr(star + 1, 2).c_str(), nullptr, 16);
    uint8_t actual = 0;
    for (size_t i = 1; i < star; ++i)
        actual ^= sentence[i];
    return expected == actual;
}

int main()
{
    int in_fd = -1;
    while (in_fd < 0) {
	in_fd = open(IN_PORT, O_RDONLY | O_NOCTTY);
	if (in_fd < 0) {
	    std::cerr << "Waiting for" << IN_PORT << "...\n";
	    std::this_thread::sleep_for(std::chrono::seconds(2));
	}
    }

    if (in_fd < 0)
    {
        perror("open input");
        return 1;
    }

    termios tty{};
    tcgetattr(in_fd, &tty);
    cfmakeraw(&tty);
    cfsetspeed(&tty, BAUD);
    tcsetattr(in_fd, TCSANOW, &tty);

    int out_fd = open(OUT_PORT, O_WRONLY | O_NOCTTY);
    if (out_fd < 0)
    {
        std::cerr << "⚠️  Warning: can't open " << OUT_PORT << " (continuing without forwarding)\n";
    }
    else
    {
        termios tty_out{};
        tcgetattr(out_fd, &tty_out);
        cfmakeraw(&tty_out);
        cfsetspeed(&tty_out, BAUD);
        tcsetattr(out_fd, TCSANOW, &tty_out);
    }

    std::string buffer;
    char c;

    std::regex gprmc_pattern(R"(\$GPRMC,[^*]*\*[0-9A-Fa-f]{2})");

    while (read(in_fd, &c, 1) == 1)
    {
        if (isprint(c))
        {
            buffer += c;
        }
        else if (c == '\n' || c == '\r')
        {
            std::smatch match;
            if (std::regex_search(buffer, match, gprmc_pattern))
            {
                std::string gprmc = match.str();

                if (out_fd >= 0)
                {
                    std::string toSend = gprmc + "\r\n";
                    write(out_fd, toSend.c_str(), toSend.size());
                }

                if (hasValidChecksum(gprmc))
                {
                    std::stringstream ss(gprmc);
                    std::string field;
                    std::vector<std::string> f;
                    while (std::getline(ss, field, ','))
                        f.push_back(field);

                    if (f.size() > 6 && f[2] == "A")
                    {
                        std::string t = f[1];
                        std::string lat = f[3], ns = f[4];
                        std::string lon = f[5], ew = f[6];
                        if (t.size() >= 6)
                        {
                            std::string time = t.substr(0, 2) + ":" + t.substr(2, 2) + ":" + t.substr(4, 2);
                            double latitude = parseCoord(lat, ns);
                            double longitude = parseCoord(lon, ew);
                            std::cout << "✅ GPRMC " << time << " | Lat: " << latitude << " | Lon: " << longitude << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "⚠️  GPRMC (no fix): " << gprmc << std::endl;
                    }
                }
                else
                {
                    std::cout << "❌ Checksum invalid: " << gprmc << std::endl;
                }
            }

            buffer.clear();
        }

        if (buffer.length() > 300)
            buffer.clear();
    }

    if (in_fd >= 0)
        close(in_fd);
    if (out_fd >= 0)
        close(out_fd);
    return 0;
}
