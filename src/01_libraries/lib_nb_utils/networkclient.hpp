#pragma once

#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <iostream>  // For printing error messages
#include <sys/time.h>  // For timeout support
#include <fcntl.h>     // For fcntl() to set non-blocking mode

class SocketClient {
private:
    int sockfd;
    sockaddr_in recvAddr;
    std::string buffer;
    std::string m_ip;
    int m_port;

    // Private method to clear the internal software buffer
    void clearInternalBuffer() {
        buffer.clear();
    }

    // Private method to clear the socket's kernel buffer
    void clearSocketBuffer() {
        if (sockfd == -1) {
            return;
        }

        char tempBuffer[8192];
        // Drain the socket buffer using MSG_DONTWAIT (no fcntl overhead)
        while (recvfrom(sockfd, tempBuffer, sizeof(tempBuffer), MSG_DONTWAIT, nullptr, nullptr) > 0) {
            // Keep reading until no more data
        }
    }

    // Method to read from the socket, filling the contents string and returning the number of bytes read
    // Optional timeout_ms parameter (default 0 means no timeout)
    int readSocket(std::string &contents, int timeout_ms = 0) {
        if (sockfd == -1) {
            std::cerr << "Socket is not open." << std::endl;
            return -1;
        }

        // Set timeout if requested
        if (timeout_ms > 0) {
            struct timeval tv;
            tv.tv_sec = timeout_ms / 1000;
            tv.tv_usec = (timeout_ms % 1000) * 1000;
            setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }

        char tempBuffer[8192];
        memset(tempBuffer, 0, sizeof(tempBuffer));

        int n = recvfrom(sockfd, tempBuffer, sizeof(tempBuffer) - 1, 0, nullptr, nullptr);
        
        // Reset timeout to blocking if it was set
        if (timeout_ms > 0) {
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 0;
            setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }
        
        if (n > 0) {
            buffer.append(tempBuffer, n);  // Append received data to internal buffer
        } 
        contents = buffer;  // Return the current buffer content
        return n;
    }

public:
    SocketClient(const std::string &ip, int port) 
        : sockfd(-1), m_ip(ip), m_port(port), buffer("") {
        memset(&recvAddr, 0, sizeof(recvAddr));
    }

    ~SocketClient() {
        close();
    }

    // Method to open the socket and bind it, returning a string indicating success (empty) or failure (error message)
    std::string open() {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) {
            return "Error creating socket: " + std::string(strerror(errno));
        }

        // Set a large kernel receive buffer (1MB) to prevent packet drops
        int rcvbuf_size = 1024 * 1024;  // 1 MB
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) == -1) {
            // Non-fatal: log warning but continue
            std::cerr << "Warning: Failed to set SO_RCVBUF: " << strerror(errno) << std::endl;
        }

        recvAddr.sin_family = AF_INET;
        recvAddr.sin_port = htons(m_port);
        recvAddr.sin_addr.s_addr = inet_addr(m_ip.c_str());

        if (bind(sockfd, (sockaddr *) &recvAddr, sizeof(recvAddr)) == -1) {
            ::close(sockfd);
            sockfd = -1;  // Mark socket as invalid
            return "Error binding socket: " + std::string(strerror(errno));
        }

        return "";  // Success
    }

    // Helper to extract one sentence from the buffer if available
    bool extractSentence(std::string &sentence, const std::string &start, const std::string &end) {
        size_t start_pos = buffer.find(start);
        if (start_pos != std::string::npos) {
            size_t end_pos = buffer.find(end, start_pos);
            if (end_pos != std::string::npos) {
                sentence = buffer.substr(start_pos, end_pos - start_pos + 1);
                buffer.erase(0, end_pos + 1);
                return true;
            }
        }
        return false;
    }

    // Drain all available datagrams using MSG_DONTWAIT (no fcntl overhead)
    // MSG_DONTWAIT returns EAGAIN when kernel buffer is empty, so no infinite loop risk
    void drainSocketToBuffer() {
        if (sockfd == -1) return;

        char tempBuffer[8192];
        int n;

        while ((n = recvfrom(sockfd, tempBuffer, sizeof(tempBuffer) - 1, MSG_DONTWAIT, nullptr, nullptr)) > 0) {
            tempBuffer[n] = '\0';
            buffer.append(tempBuffer, n);
        }
    }

    // Method to read a sentence from the buffer using specified start and end characters
    int readSentence(std::string &sentence, const std::string &start, const std::string &end) {
        // FIRST: Check if we already have a complete sentence in the buffer
        if (extractSentence(sentence, start, end)) {
            return sentence.size();
        }

        // No complete sentence - drain any queued datagrams (non-blocking)
        drainSocketToBuffer();

        // Check again after draining
        if (extractSentence(sentence, start, end)) {
            return sentence.size();
        }

        // Still no sentence - do ONE blocking read to wait for data
        if (sockfd == -1) return -1;

        char tempBuffer[8192];
        int n = recvfrom(sockfd, tempBuffer, sizeof(tempBuffer) - 1, 0, nullptr, nullptr);
        if (n > 0) {
            tempBuffer[n] = '\0';
            buffer.append(tempBuffer, n);

            // After blocking read, drain any additional queued data
            drainSocketToBuffer();
        }

        // Final attempt to extract a sentence
        if (extractSentence(sentence, start, end)) {
            return sentence.size();
        }

        return -1;  // No complete sentence found
    }

    // Method to read a sentence and return an error message (empty if successful, or the error message)
    std::string readSentenceAndReturnError(std::string &sentence, std::string start, std::string end, bool print = false) {
        int n = readSentence(sentence, start, end);
        if (n < 0) {
            if (print) {
                std::cerr << "Error reading sentence from socket." << std::endl;
            }
            return std::string(strerror(errno));
        }
        return ""; 
    }

    std::string getRawBuffer() const {
        return buffer;
    }

    // Clear the internal buffer
    void clearBuffer() {
        clearInternalBuffer();
    }

    // Get the size of the internal buffer
    size_t getBufferSize() const {
        return buffer.size();
    }

    // Clear both internal and socket buffers
    void clear() {
        clearInternalBuffer();
        clearSocketBuffer();
    }

    // Check if socket is open
    bool isOpen() const {
        return sockfd != -1;
    }

    // Close the socket
    void close() {
        if (sockfd != -1) {
            ::close(sockfd);
            sockfd = -1;
        }
    }

    // Reopen the socket
    std::string reopen() {
        close();
        return open();
    }
};
