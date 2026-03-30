#pragma once

#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <iostream> // For printing error messages

class SocketBroker
{
private:
    int sockfd;
    sockaddr_in broadcastAddr;
    std::string m_ip;
    int m_port;

public:
    SocketBroker(const std::string &ip, int port)
        : sockfd(-1), m_ip(ip), m_port(port)
    {
        memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    }

    ~SocketBroker()
    {
        if (sockfd != -1)
        {
            close(sockfd);
        }
    }

    // Method to open the socket and set broadcast options
    std::string open()
    {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1)
        {
            return "Error creating socket: " + std::string(strerror(errno));
        }
        
        int broadcastEnable = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) == -1)
        {
            close(sockfd);
            sockfd = -1; // Mark socket as invalid
            return "Error setting broadcast option: " + std::string(strerror(errno));
        }

        broadcastAddr.sin_family = AF_INET;
        broadcastAddr.sin_port = htons(m_port);
        broadcastAddr.sin_addr.s_addr = inet_addr(m_ip.c_str());

        return ""; // Success
    }

    // Method to send the message
    std::string send(const std::string &msg)
    {
        if (sockfd == -1)
        {
            return "Socket is not open.";
        }

        int result = sendto(sockfd, msg.c_str(), msg.size(), 0, (sockaddr *)&broadcastAddr, sizeof(broadcastAddr));
        if (result == -1)
        {
            return "Error sending data: " + std::string(strerror(errno));
        }

        return ""; // Success
    }
};
