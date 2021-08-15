#pragma once

/**
 * @file velodyne.h
 * @author Marcel Flottmann
 */

#include <util/time.h>
#include <msg/point_cloud.h>
#include <util/process_thread.h>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::driver
{

constexpr uint8_t POINTS_IN_BLOCK = 32;
constexpr uint8_t BLOCKS_IN_PACKET = 12;

// Do not pad the following structs as they represent encoded data
#pragma pack(push, 1)

/**
 * @brief Represents a single data point in a Velodyne packet.
 *
 */
struct VelodyneDataPoint
{
    uint16_t distance;
    uint8_t intensity;
};

/**
 * @brief Represents a block in a Velodyne packet.
 *
 */
struct VelodyneBlock
{
    uint16_t flag;
    uint16_t azimuth;
    VelodyneDataPoint points[POINTS_IN_BLOCK];
};

/**
 * @brief Represents a complete Velodyne packet.
 *
 */
struct VelodynePacket
{
    VelodyneBlock blocks[BLOCKS_IN_PACKET];
    uint32_t timestamp;
    uint8_t mode;
    uint8_t produkt_id;
};
#pragma pack(pop)

// A packet must be exactly 1206 bytes long. There might be a padding error or wrong struct definition if not.
static_assert(sizeof(VelodynePacket) == 1206);

/**
 * @brief Driver for the Velodyne LIDAR.
 *
 * A new Thread is started that receives, decodes and bundles the data as point clouds.
 *
 */
class VelodyneDriver : public fastsense::util::ProcessThread
{
public:
    using UPtr = std::unique_ptr<VelodyneDriver>;

    /**
     * @brief Construct a new Velodyne Driver object.
     *
     * @param port Port for receiving the sensor data.
     * @param buffer Ring buffer for storing the sensor data and transfer to the next step.
     */
    VelodyneDriver(uint16_t port, const fastsense::msg::PointCloudPtrStampedBuffer::Ptr& buffer);

    /**
     * @brief Destroy the Velodyne Driver object.
     *
     */
    ~VelodyneDriver() override;

    /// delete copy assignment operator
    VelodyneDriver& operator=(const VelodyneDriver& other) = delete;

    /// delete move assignment operator
    VelodyneDriver& operator=(VelodyneDriver&&) noexcept = delete;

    /// delete copy constructor
    VelodyneDriver(const VelodyneDriver&) = delete;

    /// delete move constructor
    VelodyneDriver(VelodyneDriver&&) = delete;

    /**
     * @brief Start receiver thread. The buffer will be cleared.
     *
     */
    void start() override;

    /**
     * @brief Get the next scan.
     *
     * @return PointCloudStamped The next scan with timestamp
     */
    fastsense::msg::PointCloudPtrStamped getScan();

protected:
    /**
     * @brief Receives a packet. This is the main receiver thread function.
     *
     */
    void thread_run() override;

    /**
     * @brief Decode the packet and make point clouds from the data.
     *
     */
    void decode_packet();

    /// Port for receiving data
    uint16_t port_;

    /// Socket file descriptor
    int sockfd_;

    /// Current packet
    VelodynePacket packet_;

    /// Last azimuth
    float az_last_;

    /// Buffer to write scans to
    fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>::Ptr scan_buffer_;

    /// Current scan
    fastsense::msg::PointCloud::Ptr current_scan_;
};

} // namespace fastsense::driver
