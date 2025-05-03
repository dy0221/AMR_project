/*
    https://www.kernel.org/doc/html/latest/networking/can.html
*/
#include "amr_control/amr_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace


float bytesToFloat(const uint8_t* bytes) {
    float f;
    std::memcpy(&f, bytes, sizeof(float));
    return f;
}

void resetPacketBuffer(PacketPair &buf) {
    buf.msg1_received = false;
    buf.msg2_received = false;
    buf.msg1 = {};
    buf.msg2 = {};
}

amr_control::AmrControllorNode::AmrControllorNode()
    : Node("twai2control")
{
    create_topic();
}
amr_control::AmrControllorNode::~AmrControllorNode()
{
    close(sockfd_);
}

bool amr_control::AmrControllorNode::twai_init()
{
    const char* ifname = "can0";
    // can 소켓 생성
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
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
    if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("getting interface index");
        return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    // 소켓 바인딩
    if (bind(sockfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("binding socket");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "CAN socket initialized on %s", ifname);
    return true;
}

void amr_control::AmrControllorNode::create_topic(){
    // initialize command subscriber
    //  cmd_vel을 sub 하는 코드
    velocity_command_subscriber_ = this->create_subscription<TwistStamped>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<TwistStamped> msg) -> void // 람다함수 (callback)
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(this->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }
          //time stamp가 0 일경우 << 의미없는 값 << 현재 시간으로 바꿈
          if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
          {
            RCLCPP_WARN_ONCE(
              this->get_logger(),
              "Received TwistStamped with zero timestamp, setting it to current "
              "time, this message will only be shown once");
            msg->header.stamp = this->now();
          }
   
         // 너무 오래된 메세지는 무시 (0.5)
          const auto current_time_diff = this->now() - msg->header.stamp;
    
          if (
            cmd_vel_timeout_ == rclcpp::Duration::from_seconds(0.0) ||
            current_time_diff < cmd_vel_timeout_)
          {
           // RT 버퍼에 메세지 저장
            received_velocity_msg_ptr_.writeFromNonRT(msg);
          }
          else
          {
            //오래된 메세지 무시
            RCLCPP_WARN(
              this->get_logger(),
              "Ignoring the received message (timestamp %.10f) because it is older than "
              "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
              rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
              cmd_vel_timeout_.seconds());
          }
        });

    // odom publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());

    //  접두어 붙이기 (name space와 비슷한 기능으로 tf frame이 충돌이 되지 않도록함)
    std::string tf_prefix = "";
    tf_prefix = std::string(this->get_namespace());
    if (!tf_prefix.empty() && tf_prefix != "/")  tf_prefix += "/";

    const auto odom_frame_id = tf_prefix + "odom";
    const auto base_frame_id = tf_prefix + "base_link";

    odometry_message_.header.frame_id = odom_frame_id;
    odometry_message_.child_frame_id = base_frame_id;

    // limit the publication on the topics /odom and /tf
    //double publish_rate_ = 50.0;
    //20ms
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

    // initialize odom values zeros 
    odometry_message_.twist =
        geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    // 분산(신뢰도)를 테스트이므로 그냥 0으로 설정
    //constexpr << 컴파일과정중에 계산해서 상수처럼 사용
    constexpr std::array<double, 6> pose_cov_diag = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    constexpr std::array<double, 6> twist_cov_diag = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;
        odometry_message_.pose.covariance[diagonal_index] =  pose_cov_diag[index];
        odometry_message_.twist.covariance[diagonal_index] = twist_cov_diag[index];
    }

    // odom -> base link 변환 publisher
    odometry_transform_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
        DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

    // keeping track of odom and base_link transforms only
    odometry_transform_message_.transforms.resize(1);
    odometry_transform_message_.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message_.transforms.front().child_frame_id = base_frame_id;
    
    previous_update_timestamp_ = this->get_clock()->now();

    subscriber_is_active_ = true;
}

void amr_control::AmrControllorNode::read_and_publish(){
    PacketPair packetBuffer;
    // 패킷 ID는 0-99 사이의 값
    uint8_t packet_id = 0;
    uint8_t packet_task = 0;
    while (rclcpp::ok()) {
        can_frame frame;
        int nbytes = read(sockfd_, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("can raw socket read");
            break;
        }

        switch (packet_task)
        {
        case 0:
            if (frame.can_id == 0x100){
                packet_id = frame.data[0];
                packetBuffer.msg1 = frame;
                packetBuffer.msg1_received = true;
                packet_task = 1;
            }
            break;
        case 1:
            if (frame.can_id == 0x101){
                // 패킷 ID가 일치하는지 확인
                if (frame.data[0] != packet_id){
                    std::cout << "Packet ID mismatch: expected " << static_cast<int>(packet_id) << ", got " << static_cast<int>(frame.data[0]) << std::endl;
                    packet_task = 0;
                    resetPacketBuffer(packetBuffer);
                    break;
                }
                // 패킷 ID가 일치하면 msg2에 저장
                packetBuffer.msg2 = frame;
                packetBuffer.msg2_received = true;
                packet_task = 0;
            }
            break;
        default:
            std::cout << "Invalid packet task state: " << packet_task << std::endl;
            packet_task = 0;
            resetPacketBuffer(packetBuffer);
            break;
        }

        if (packetBuffer.msg1_received && packetBuffer.msg2_received) {
            float pose_x = bytesToFloat(&packetBuffer.msg1.data[2]);

            uint8_t pose_y_bytes[4] = {
                packetBuffer.msg1.data[6],
                packetBuffer.msg1.data[7],
                packetBuffer.msg2.data[2],
                packetBuffer.msg2.data[3]
            };
            float pose_y = bytesToFloat(pose_y_bytes);

            float orientation_theta = bytesToFloat(&packetBuffer.msg2.data[4]);

            // std::cout << "[packet_id " << static_cast<int>(packet_id) << "] "
            //           << "pose_x: " << pose_x << ", "
            //           << "pose_y: " << pose_y << ", "
            //           << "theta: " << orientation_theta << '\n';

            resetPacketBuffer(packetBuffer);
            // 메세지 publish
            rclcpp::Time current_time = this->get_clock()->now();
            double dt = (current_time - previous_time_).seconds();
            if (dt < 0.0001) { // 너무 작으면 division error 방지
                previous_pose_x_ = pose_x;
                previous_pose_y_ = pose_y;
                previous_theta_ = orientation_theta;
                previous_time_ = current_time;
                RCLCPP_WARN(this->get_logger(), "dt is very small");
                continue;
            }

            double dx = pose_x - previous_pose_x_;
            double dy = pose_y - previous_pose_y_;
            double dtheta = std::atan2(std::sin(orientation_theta - previous_theta_), 
                                    std::cos(orientation_theta - previous_theta_));

            double linear_velocity = std::sqrt(dx * dx + dy * dy) / dt;
            double angular_velocity = dtheta / dt;

            //  쿼터니언으로 바꿈
            tf2::Quaternion orientation;
            //  2D 로봇의 heading(yaw) 값을 3D 쿼터니언으로 변환
            orientation.setRPY(0.0, 0.0, orientation_theta);

            bool should_publish = false;
            try{     
              //            이전에 pub한 시간    + pub주기          < time보다 작으면 << pub해야할 시간
              if (previous_publish_timestamp_ + publish_period_ < current_time){
                previous_publish_timestamp_ += publish_period_;
                should_publish = true;
              }
            } 
            catch (const std::runtime_error &){
                // Handle exceptions when the time source changes and initialize publish timestamp
                previous_publish_timestamp_ = current_time;
                should_publish = true;
            }

            if (should_publish){
                odometry_message_.header.stamp = current_time;
                odometry_message_.pose.pose.position.x = pose_x / 100; //자동으로 float에서 double로 형변환
                odometry_message_.pose.pose.position.y = pose_y / 100; //cm 에서 m로 바꿔야함
                odometry_message_.pose.pose.orientation.x = orientation.x();
                odometry_message_.pose.pose.orientation.y = orientation.y();
                odometry_message_.pose.pose.orientation.z = orientation.z();
                odometry_message_.pose.pose.orientation.w = orientation.w();
                odometry_message_.twist.twist.linear.x = linear_velocity;
                odometry_message_.twist.twist.angular.z = angular_velocity;

                odometry_publisher_->publish(odometry_message_);

                odometry_transform_message_.transforms.front().header.stamp = current_time;
                odometry_transform_message_.transforms.front().transform.translation.x = pose_x / 100;
                odometry_transform_message_.transforms.front().transform.translation.y = pose_y / 100;
                odometry_transform_message_.transforms.front().transform.rotation.x = orientation.x();
                odometry_transform_message_.transforms.front().transform.rotation.y = orientation.y();
                odometry_transform_message_.transforms.front().transform.rotation.z = orientation.z();
                odometry_transform_message_.transforms.front().transform.rotation.w = orientation.w();

                odometry_transform_publisher_->publish(odometry_transform_message_);

            }

            previous_pose_x_ = pose_x;
            previous_pose_y_ = pose_y;
            previous_theta_ = orientation_theta;
            previous_time_ = current_time;
        }


    }


}

void amr_control::AmrControllorNode::clean_msg(){
    odometry_message_.header.stamp =  rclcpp::Time(0, 0, get_clock()->get_clock_type());
    odometry_message_.pose.pose.position.x = 0.0;
    odometry_message_.pose.pose.position.y = 0.0;
    odometry_message_.pose.pose.orientation.x = 0.0;
    odometry_message_.pose.pose.orientation.y = 0.0;
    odometry_message_.pose.pose.orientation.z = 0.0;
    odometry_message_.pose.pose.orientation.w = 1.0; // 회전없음
    odometry_message_.twist.twist.linear.x = 0.0;
    odometry_message_.twist.twist.angular.z = 0.0;
    
    odometry_transform_message_.transforms.front().header.stamp =  rclcpp::Time(0, 0, get_clock()->get_clock_type());
    odometry_transform_message_.transforms.front().transform.translation.x = 0.0;
    odometry_transform_message_.transforms.front().transform.translation.y = 0.0;
    odometry_transform_message_.transforms.front().transform.rotation.x = 0.0;
    odometry_transform_message_.transforms.front().transform.rotation.y = 0.0;
    odometry_transform_message_.transforms.front().transform.rotation.z = 0.0;
    odometry_transform_message_.transforms.front().transform.rotation.w = 1.0; // 회전없음
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // ROS 2 초기화

    auto node = std::make_shared<amr_control::AmrControllorNode>();

    if (!node->twai_init()) {
        rclcpp::shutdown();  // 실패 시 종료
        return 1;
    }

    // 별도 쓰레드에서 read_and_publish()를 실행
    std::thread can_read_thread([&node]() {
        node->read_and_publish();
    });

    rclcpp::spin(node);  // ROS 콜백 spin

    can_read_thread.join();  // 종료 시 CAN 스레드 대기
    rclcpp::shutdown();      // ROS 종료
    return 0;
}
