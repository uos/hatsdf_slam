/**
 * @author Marcel Flottmann
 */

#include <util/time.h>
#include <system_error>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/file.h>
#include <poll.h>

#include <driver/lidar/velodyne.h>
#include <util/logging/logger.h>

using namespace fastsense::driver;
using namespace fastsense::msg;
using namespace fastsense::util;
using namespace fastsense::util::logging;

// Magic constants of the sensor
constexpr uint8_t PROD_ID_VLP16 = 0x22;
constexpr uint8_t MODE_STRONGEST = 0x37;
constexpr uint8_t MODE_LAST = 0x38;
constexpr uint8_t MODE_DUAL = 0x39;
constexpr uint16_t BLOCK_FLAG = 0xEEFF;

/**
 * @brief Calculate degrees to radians.
 *
 * @param deg Angle in degrees.
 * @return float Calcualted angle in radians.
 */
constexpr float deg_to_rad(float deg)
{
    return deg * M_PIf32 / 180.f;
}

/// Lookup table to convert the laser id to vertical angles in radians.
constexpr float LASER_ID_TO_VERT_ANGLE[16] =
{
    deg_to_rad(-15),
    deg_to_rad(1),
    deg_to_rad(-13),
    deg_to_rad(3),
    deg_to_rad(-11),
    deg_to_rad(5),
    deg_to_rad(-9),
    deg_to_rad(7),
    deg_to_rad(-7),
    deg_to_rad(9),
    deg_to_rad(-5),
    deg_to_rad(11),
    deg_to_rad(-3),
    deg_to_rad(13),
    deg_to_rad(-1),
    deg_to_rad(15)
};

/// Lookup table to get the offset of a laser.
constexpr float LASER_ID_TO_OFFSET[16] =
{
    11.2f,
    -0.7f,
    9.7f,
    -2.2f,
    8.1f,
    -3.7f,
    6.6f,
    -5.1f,
    5.1f,
    -6.6f,
    3.7f,
    2.2f,
    -9.7f,
    0.7f,
    -11.2f
};

/// Lookup table to get ring of laser.
constexpr uint8_t LASER_ID_TO_RING[16] =
{
    15,
    13,
    11,
    9,
    7,
    5,
    3,
    1,
    14,
    12,
    10,
    8,
    6,
    4,
    2,
    0
};

VelodyneDriver::VelodyneDriver(uint16_t port, const PointCloudPtrStampedBuffer::Ptr& buffer) :
    port_{port},
    sockfd_{},
    packet_{},
    az_last_{0.f},
    scan_buffer_{buffer},
    current_scan_{std::make_shared<PointCloud>()}
{
    // open socket
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        throw std::system_error(errno, std::generic_category(),  "failed to open socket");
    }

    // set socket port
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    // bind to port
    if (bind(sockfd_, (sockaddr*)&addr, sizeof(addr)) == -1)
    {
        close(sockfd_);
        throw std::system_error(errno, std::generic_category(),  "failed to bind");
    }

    // set non blocking mode
    if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        close(sockfd_);
        throw std::system_error(errno, std::generic_category(),  "failed to set non-blocking mode");
    }
}

VelodyneDriver::~VelodyneDriver()
{
    ProcessThread::stop();
    close(sockfd_);
}

void VelodyneDriver::start()
{
    if (running == false)
    {
        running = true;
        az_last_ = 0.f;
        current_scan_ = std::make_shared<PointCloud>();
        scan_buffer_->clear();
        worker = std::thread(&VelodyneDriver::thread_run, this);
    }
}

fastsense::msg::PointCloudPtrStamped VelodyneDriver::getScan()
{
    PointCloudPtrStamped pcs;
    scan_buffer_->pop(&pcs);
    return pcs;
}

void VelodyneDriver::thread_run()
{
    constexpr int POLL_TIMEOUT = 1000;
    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;

    sockaddr_in sender;
    socklen_t senderLength = sizeof(sender);

    while (running)
    {
        // wait for packet
        do
        {
            int ret = poll(fds, 1, POLL_TIMEOUT);
            if (ret < 0)
            {
                if (errno != EINTR)
                {
                    throw std::system_error(errno, std::generic_category(),  "poll failed");
                }
            }
            else if (ret == 0)
            {
                Logger::error("Timeout waiting for LIDAR data!");
            }
            else
            {
                if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
                {
                    throw std::runtime_error("poll error");
                }
            }
        }
        while (!(fds[0].revents & POLLIN));

        // receive packet
        ssize_t size = recvfrom(sockfd_, &packet_, sizeof(VelodynePacket), 0, (sockaddr*)&sender, &senderLength);

        if (size < 0)
        {
            throw std::system_error(errno, std::generic_category(),  "recvfrom failed");
        }

        // process packet
        if (size == sizeof(VelodynePacket))
        {
            decode_packet();
        }
    }
}

void VelodyneDriver::decode_packet()
{
    if (packet_.produkt_id != PROD_ID_VLP16)
    {
        throw std::runtime_error("wrong sensor");
    }

    if (packet_.mode != MODE_STRONGEST && packet_.mode != MODE_LAST)
    {
        throw std::runtime_error("wrong mode");
    }

    for (int b = 0; b < BLOCKS_IN_PACKET; b++)
    {
        if (packet_.blocks[b].flag == BLOCK_FLAG)
        {
            float az_block = packet_.blocks[b].azimuth * 0.01f; // 0.01 degree

            // add new scan to queue when azimuth overflows
            if (az_block < az_last_)
            {
                current_scan_->rings_ = 16;
                current_scan_->scaling_ = 1.0f;
                // TODO std::move()
                scan_buffer_->push_nb(Stamped<PointCloud::Ptr>{current_scan_, util::HighResTime::now()}, true);
                current_scan_ = std::make_shared<PointCloud>();
            }
            az_last_ = az_block;

            // allocate points for current block
            size_t startIdx = current_scan_->points_.size();
            current_scan_->points_.resize(startIdx + POINTS_IN_BLOCK);

            // calculate azimuth difference between current and next block for interpolation
            // last block uses previous difference for simplification
            float az_diff;
            if (b + 1 < BLOCKS_IN_PACKET)
            {
                az_diff = (packet_.blocks[b + 1].azimuth * 0.01f) - az_block;
                if (packet_.blocks[b + 1].azimuth < packet_.blocks[b].azimuth)
                {
                    az_diff += 360.f;
                }
            }

            for (int p = 0; p < POINTS_IN_BLOCK; p++)
            {
                // select point
                auto& new_point = current_scan_->points_[startIdx + LASER_ID_TO_RING[p % 16] + (p < 16 ? 0 : 16)];

                // interpolate azimuth (VLP-16 User Manual, Ch. 9.5)
                float az;
                if (p < 16)
                {
                    az = az_block + (az_diff * 2.304f * p) / 55.296f;
                }
                else
                {
                    az = az_block + (az_diff * 2.304f * (p - 16)) / (2 * 55.296f);
                }

                // check angle overflow
                if (az > 360.f)
                {
                    az -= 360.f;
                }

                az = deg_to_rad(az);

                // calculate XYZ and fill new point
                if (packet_.blocks[b].points[p].distance > 0 && packet_.blocks[b].points[p].distance <= std::numeric_limits<ScanPointType>::max())
                {
                    float r = packet_.blocks[b].points[p].distance * 2 - LASER_ID_TO_OFFSET[p % 16]; // 2 mm
                    float cos_vertical = cos(LASER_ID_TO_VERT_ANGLE[p % 16]);
                    new_point.x() = r * cos_vertical * sin(az);
                    new_point.y() = r * cos_vertical * cos(az);
                    new_point.z() = r * sin(LASER_ID_TO_VERT_ANGLE[p % 16]);
                }
                else
                {
                    // out of range: set to zero
                    new_point.x() = 0;
                    new_point.y() = 0;
                    new_point.z() = 0;
                }
            }
        }
    }
}
