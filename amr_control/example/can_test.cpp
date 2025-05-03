/*
    can통신
    일단 무조건 값을 읽고 출력한다.
*/
#include <iostream>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <array>

struct PacketPair {
    can_frame msg1;
    can_frame msg2;
    bool msg1_received = false;
    bool msg2_received = false;
};

float bytesToFloat(const uint8_t* bytes) {
    float f;
    std::memcpy(&f, bytes, sizeof(float));
    return f;
}

int main() {
    const char* ifname = "can0";
    // can 소켓 생성
    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        std::cout<<"Error creating socket: " << std::endl;
        return 1;
    }

    sockaddr_can addr {};
    //ifreq << linux/can.h>에서 정의된 구조체, socket의 인터페이스 정보를 담고 있다.
    // ioctl호출에 사용되는 구조체
    ifreq ifr {};
    //ifr_name에 can0을 넣어준다.
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    // can0라는 이름이 실제로 몇번 인터페이스에 존재하는지 불러온다.
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        std::cout<<"Error getting interface index: " << std::endl;
        return 1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }

    std::cout << "Listening on " << ifname << "...\n";

    std::array<PacketPair, 256> packetBuffer;

    while (true) {
        can_frame frame;
        int nbytes = read(sockfd, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("Read");
            break;
        }

        if (frame.can_id == 0x100 || frame.can_id == 0x101) {
            uint8_t packet_id = frame.data[0];
            auto& pkt = packetBuffer[packet_id];

            if (frame.can_id == 0x100) {
                pkt.msg1 = frame;
                pkt.msg1_received = true;
            } else if (frame.can_id == 0x101) {
                pkt.msg2 = frame;
                pkt.msg2_received = true;
            }

            if (pkt.msg1_received && pkt.msg2_received) {
                float pose_x = bytesToFloat(&pkt.msg1.data[2]);

                uint8_t pose_y_bytes[4] = {
                    pkt.msg1.data[6],
                    pkt.msg1.data[7],
                    pkt.msg2.data[2],
                    pkt.msg2.data[3]
                };
                float pose_y = bytesToFloat(pose_y_bytes);

                float orientation_theta = bytesToFloat(&pkt.msg2.data[4]);

                std::cout << "[packet_id " << static_cast<int>(packet_id) << "] "
                          << "pose_x: " << pose_x << ", "
                          << "pose_y: " << pose_y << ", "
                          << "theta: " << orientation_theta << '\n';
c
                pkt.msg1_received = false;
                pkt.msg2_received = false;
            }
        }
    }

    close(sockfd);
    return 0;
}
