#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "ouster/os1.h"

namespace ouster {
namespace OS1 {

using ns = std::chrono::nanoseconds;

/**
 * @note Added to support advanced mode parameters configuration for Autoware
 */
static OperationMode _operation_mode = ouster::OS1::MODE_1024x10;
static PulseMode _pulse_mode = ouster::OS1::PULSE_STANDARD;
static std::string _operation_mode_str = "";
static std::string _pulse_mode_str = "";
//----------------

struct client {
    int lidar_fd;
    int imu_fd;
    ~client() {
        close(lidar_fd);
        close(imu_fd);
    }
};

static int udp_data_socket(int port) {
    int sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        std::cerr << "socket: " << std::strerror(errno) << std::endl;
        return -1;
    }

    struct sockaddr_in my_addr;
    memset((char*)&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_fd, (struct sockaddr*)&my_addr, sizeof(my_addr)) < 0) {
        std::cerr << "bind: " << std::strerror(errno) << std::endl;
        return -1;
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                   sizeof(timeout)) < 0) {
        std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
        return -1;
    }

    return sock_fd;
}

static int cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
        std::cerr << "getaddrinfo: " << gai_strerror(ret) << std::endl;
        return -1;
    }
    if (info_start == NULL) {
        std::cerr << "getaddrinfo: empty result" << std::endl;
        return -1;
    }

    int sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock_fd < 0) {
            std::cerr << "socket: " << std::strerror(errno) << std::endl;
            continue;
        }

        if (connect(sock_fd, ai->ai_addr, ai->ai_addrlen) == -1) {
            close(sock_fd);
            continue;
        }

        break;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        return -1;
    }

    return sock_fd;
}

std::shared_ptr<client> init_client(const std::string& hostname,
                                    const std::string& udp_dest_host,
                                    int lidar_port, int imu_port) {
    int sock_fd = cfg_socket(hostname.c_str());

    if (sock_fd < 0) return std::shared_ptr<client>();

    std::string cmd;

    auto do_cmd = [&](const std::string& op, const std::string& val) {
        const size_t max_res_len = 4095;
        char read_buf[max_res_len + 1];

        ssize_t len;
        std::string cmd = op + " " + val + "\n";

        len = write(sock_fd, cmd.c_str(), cmd.length());
        if (len != (ssize_t)cmd.length()) {
            //std::cerr << "init_client: failed to send command" << std::endl;
            return std::string("[error] init_client: failed to send command");
        }

        len = read(sock_fd, read_buf, max_res_len);
        if (len < 0) {
            //std::cerr << "read: " << std::strerror(errno) << std::endl;
            return std::string("[error] read: ") + std::strerror(errno);
        }
        read_buf[len] = '\0';

        auto res = std::string(read_buf);
        res.erase(res.find_last_not_of(" \r\n\t") + 1);
        
        return res;
    };
    
    auto do_cmd_chk = [&](const std::string& op, const std::string& val) {
    	auto res = do_cmd(op, val);
    	if (res.find("[error]") != std::string::npos) {
    		std::cerr << res << std::endl;
    		return false;
    	} else if (res != op) {
            std::cerr << "init_client: command \"" << op << " " << val << "\" failed with \""
                      << res << "\"" << std::endl;
            return false;
        }
        return true;
    };
    
    auto get_cmd = [&](const std::string& op, const std::string& val) {
    	auto res = do_cmd(op, val);
    	if (res.find("error") != std::string::npos) {
    		std::cerr << res << std::endl;
    		return std::string("");
    	} 
        return res;
    };
    
    auto str_tok = [&](const std::string &str, const std::string delim) {
    	auto start = str.find_first_not_of(delim);
    	auto end = start;
		std::vector<std::string> tokens;

		while (start != std::string::npos){
		    end = str.find_first_of(delim, start);
		    tokens.push_back(str.substr(start, end-start));
		    start = str.find_first_not_of(delim, end);
		}
		
		return tokens;
    };

    bool success = true;
    success &= do_cmd_chk("set_udp_port_lidar", std::to_string(lidar_port));
    success &= do_cmd_chk("set_udp_port_imu", std::to_string(imu_port));
    success &= do_cmd_chk("set_udp_ip", udp_dest_host);

    if (!success) return std::shared_ptr<client>();
    
    /**
     * @note Added to support advanced mode parameters configuration
     */
    //read the sensor information
    std::string version_str = std::string("");
    std::string product_str = std::string("");
    bool has_pulsemode = true;
    auto sensor_info_str = get_cmd("get_sensor_info", "");
    auto tokens = str_tok(sensor_info_str, std::string(",: \""));
    auto pos = std::find(tokens.begin(), tokens.end(), "build_rev");
    if (pos != tokens.end()) {
    	version_str = *(pos+1);
    }
    pos = std::find(tokens.begin(), tokens.end(), "prod_line");
    if (pos != tokens.end()) {
    	product_str = *(pos+1);
    }
    if (product_str == std::string("") || version_str == std::string("")) {
		std::cout << "Error: Failed to read product name and firmware version." << std::endl;
    	return std::shared_ptr<client>();    	
    }
    std::cout << "Ouster model \"" << product_str << "\", firmware version \"" << version_str << "\"" << std::endl;
    //TODO: support OS-1-16 and OS-1-128
    if (product_str != std::string("OS-1-64")) {
    	std::cout << "Error: this driver currently only supports Ouster model \"OS-1-64\"." << std::endl;
    	return std::shared_ptr<client>();
    }
    
    auto ver_numbers = str_tok(version_str, std::string("v."));
    //check the minor version
    auto ver_major = std::stoi(ver_numbers[0], nullptr);
    auto ver_minor = std::stoi(ver_numbers[1], nullptr);
    if (ver_major < 1 || ver_minor < 7) {
    	std::cout << "Error: Firmware version \"" << version_str << "\" is not supported, please upgrade" << std::endl;
    	return std::shared_ptr<client>();
    }
    if (std::stoi(ver_numbers[1], nullptr) >= 10) {
    	std::cout << "On firmware version \"" << version_str << "\" the \"pulse_mode\" parameter is no longer available, will ignore it." << std::endl;
    	has_pulsemode = false;
    }
    
    //read the current settings
    auto curr_operation_mode_str = get_cmd("get_config_param", "active lidar_mode");
    auto curr_pulse_mode_str = std::string("");
    if (has_pulsemode) {
    	curr_pulse_mode_str = get_cmd("get_config_param", "active pulse_mode");
   	}
    auto curr_window_rejection_str = get_cmd("get_config_param", "active window_rejection_enable");
    bool do_configure = false;
    success = true;
    
    //setting advanced mode parameters (if necessary)
    if (curr_operation_mode_str != _operation_mode_str) {
    	success &= do_cmd_chk("set_config_param", "lidar_mode " + _operation_mode_str);
    	do_configure = true;
   	}
   	if (has_pulsemode && (curr_pulse_mode_str != _pulse_mode_str)) {
	    success &= do_cmd_chk("set_config_param", "pulse_mode " + _pulse_mode_str);
	    do_configure = true;
    }
    
    if (!success) return std::shared_ptr<client>();
    
    //reinitialize to take effect (if necessary)
	if (do_configure) {
		success &= do_cmd_chk("reinitialize", "");
		if (!success) return std::shared_ptr<client>();
		std::cout << "Parameters configured, reinitializing sensor" << std::endl;
   	} else {
   		std::cout << "Parameters already configured, no need to reinitialize" << std::endl;
   	}
    //----------------
    
    close(sock_fd);

    int lidar_fd = udp_data_socket(lidar_port);
    int imu_fd = udp_data_socket(imu_port);
    auto cli = std::make_shared<client>();
    cli->lidar_fd = lidar_fd;
    cli->imu_fd = imu_fd;
    return cli;
}

client_state poll_client(const client& c) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(c.lidar_fd, &rfds);
    FD_SET(c.imu_fd, &rfds);

    int max_fd = std::max(c.lidar_fd, c.imu_fd);

    int retval = select(max_fd + 1, &rfds, NULL, NULL, NULL);

    client_state res = client_state(0);
    if (retval == -1) {
        std::cerr << "select: " << std::strerror(errno) << std::endl;
        res = client_state(res | ERROR);
    } else if (retval) {
        if (FD_ISSET(c.lidar_fd, &rfds)) res = client_state(res | LIDAR_DATA);
        if (FD_ISSET(c.imu_fd, &rfds)) res = client_state(res | IMU_DATA);
    }
    return res;
}

static bool recv_fixed(int fd, void* buf, size_t len) {
    ssize_t n = recvfrom(fd, buf, len + 1, 0, NULL, NULL);
    if (n == (ssize_t)len)
        return true;
    else if (n == -1)
        std::cerr << "recvfrom: " << std::strerror(errno) << std::endl;
    else
        std::cerr << "Unexpected udp packet length: " << n << std::endl;
    return false;
}

bool read_lidar_packet(const client& cli, uint8_t* buf) {
    return recv_fixed(cli.lidar_fd, buf, lidar_packet_bytes);
}

bool read_imu_packet(const client& cli, uint8_t* buf) {
    return recv_fixed(cli.imu_fd, buf, imu_packet_bytes);
}

/**
 * @note Added to support advanced mode parameters configuration for Autoware
 */
//----------------
void set_advanced_params(std::string operation_mode_str, std::string pulse_mode_str)
{
   _operation_mode_str = operation_mode_str;
   _pulse_mode_str = pulse_mode_str;
   
   _operation_mode = ouster::OS1::MODE_1024x10;
   if (_operation_mode_str == std::string("512x10")) {
       _operation_mode = ouster::OS1::MODE_512x10;
   } else if (_operation_mode_str == std::string("1024x10")) {
       _operation_mode = ouster::OS1::MODE_1024x10;
   } else if (_operation_mode_str == std::string("2048x10")) {
       _operation_mode = ouster::OS1::MODE_2048x10;
   } else if (_operation_mode_str == std::string("512x20")) {
       _operation_mode = ouster::OS1::MODE_512x20;
   } else if (_operation_mode_str == std::string("1024x20")) {
       _operation_mode = ouster::OS1::MODE_1024x20;
   } else {
   	   std::cout << "Selected operation mode " << _operation_mode_str << " is invalid, using default mode \"1024x10\"" << std::endl;
   }

   _pulse_mode = ouster::OS1::PULSE_STANDARD;
   if (_pulse_mode_str == std::string("STANDARD")) {
   	   _pulse_mode = ouster::OS1::PULSE_STANDARD;
   } else if (_pulse_mode_str == std::string("NARROW")) {
   	   _pulse_mode = ouster::OS1::PULSE_NARROW;
   } else {
   	   std::cout << "Selected pulse mode " << _pulse_mode_str << " is invalid, using default mode \"STANDARD\"" << std::endl;
   }
}
//----------------

}
}
