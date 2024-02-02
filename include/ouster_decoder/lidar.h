/*! Kumar Robotics
 *  January 2024
 *  Structs for data organization around lidar scans, lidar metadata and range/signal images
 *  Authors: Chao Qu and Jason Hughes
 */

#pragma once

#include <boost/make_shared.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "ouster_ros/os_ros.h"

inline constexpr double Deg2Rad(double deg) { return deg * M_PI / 180.0; }

/*!
 * @brief struct for organize image data in a scan
 */ 
struct ImageData 
{
    // @brief: x,y,z values
    float x{};
    float y{};
    float z{};
    // @brief: range
    uint16_t r16u{};
    // @brief:signal
    uint16_t s16u{};

    /*!
    * @breif: set the raw range value in image struct
    * @param: range in meters
    * @param: scale thet range is in
    */
    void set_range(float range, double scale) noexcept 
    {
        r16u = static_cast<uint16_t>(
            std::min(range * scale,
                     static_cast<double>(std::numeric_limits<uint16_t>::max())));
    }

    /*!
    * @breif: set range and signal to zero approaching inf.
    */
    void set_bad() noexcept {
        x = y = z = std::numeric_limits<float>::quiet_NaN();
        r16u = s16u = 0;
    }
};

static_assert(sizeof(ImageData) == sizeof(float) * 4,
              "Size of ImageData must be 4 floats (16 bytes)");

/*!
* @brief: Stores SensorInfo from ouster with some other useful data
*/
struct LidarModel 
{
    LidarModel() = default;
    /*!
    * @breif: parse metadata and save important info from it.
    * @param: string in json format containing ouster metadata.
    */
    explicit LidarModel(const std::string& metadata);

    /*!
    * @brief: whether this model is ready
    */
    bool Initialized() const { return !altitudes.empty(); }

    // @breif: number of beams
    int rows{};               
    // @breif: number of columns in a full scan
    int cols{};                     
    // @breif: frequency
    int freq{};
    // @breif: delta time between two columns in secs
    double dt_col{};
    // @breif: delta time between two packets in secs 
    double dt_packet{};             
    // @breif: delta angle between two columns in radians
    double d_azimuth{};            
    // @brief: distance between beam and origin
    double beam_offset{};
    // @brief: alitude angles, high to low in radians
    std::vector<double> altitudes;
    // @breif: azimuth affset angles in radians
    std::vector<double> azimuths;
    // @breif: metadata in sensor info formet
    ouster_ros::sensor::sensor_info info;
    // @breif: packet format from ouster 
    ouster_ros::sensor::packet_format const* pf{nullptr};  

    const auto& pixel_shifts() const noexcept 
    {
        return info.format.pixel_shift_by_row;
    }

    /*!
    * @brief Convert lidar range data to xyz
    * @details see software manual 3.1.2 Lidar Range to XYZ
    *
    * y    r
    * ^   / -> rotate clockwise
    * |  /
    * | /
    * |/  theta
    * o ---------> x  (connector)
    * @param: range of point
    * @param: calcualted angle of column
    * @param: id of row
    */
    Eigen::Vector3f ToPoint(float range, float theta_enc, int row) const;

    /*!
    * @brief: calculate unique id for a measurement
    * @return: calculated uid
    */
    int Uid(int fid, int mid) const noexcept { return fid * cols + mid; }

    /*!
    * @brief: update camera info with current info
    * @param: recently recieved camera info ros msg
    */
    void UpdateCameraInfo(sensor_msgs::CameraInfo& cinfo) const;
};

/*!
* @brief: Stores data for a (sub)scan
*/
struct LidarScan 
{
    // @breif: column index
    int icol{0};           
    // @breif: subscan index
    int iscan{0};
    // @brief: previous uid
    int prev_uid{-1};
    // @breif: minimum range
    double min_range{0.25};
    // @breif: maximum range
    double max_range{256.0};
    // @breif: raw range is uint32_t, divide by scale to meter
    double range_scale{};
    // @breif: whether to destagger scan
    bool destagger{false}; 

    // @breif: each pixel is ImageData
    sensor_msgs::ImagePtr image_ptr;  
    // @breif: each point is x,y,z,itensity
    sensor_msgs::PointCloud2 cloud;
    // @breif: all time stamps in nanoseconds
    std::vector<uint64_t> times;      

    /*!
    * @brief: update point cloud ptr
    * @param: r: pixel index
    * @param: c: column of cloud
    * @return: updated point
    */
    float* CloudPtr(int r, int c)
    {
        const auto i = r * cols() + c;
        return reinterpret_cast<float*>(cloud.data.data() + i * cloud.point_step);
    }

    /*!
    * @breif: update image prt data
    * @param: r: pixel index
    * @param: c: column of image
    * @return: updated image data struct
    */
    ImageData* ImagePtr(int r, int c) 
    {
        const auto i = r * cols() + c;
        return reinterpret_cast<ImageData*>(image_ptr->data.data() +
                                        i * sizeof(ImageData));
    }

    /*!
    * @breif: getter for number of rows in cloud
    * @return: int height of cloud
    */
    int rows() const noexcept { return static_cast<int>(cloud.height); }
    /*!
    * @brief: getter for number of columns in cloud
    * @return: width of cloud
    */
    int cols() const noexcept { return static_cast<int>(cloud.width); }

    /*!
    * @brief: whether this scan is full
    * @return: true if scan is full
    */
    bool IsFull() const noexcept { return icol >= cols(); }

    /*!
    * @brief: Starting column of this scan
    * @return: index of starting column of scan
    */
    int StartingCol() const noexcept { return iscan * cols(); }

    /*!
    * @brief: Detect if there is a jump in the lidar data
    * @return: 0 - no jump, >0 - jump forward in time, <0 - jump backward in time
    */
    int DetectJump(int uid) noexcept;

    /*!
    * @brief: Allocate storage for the scan
    * @param: rows: number of rows in scan
    * @param: cols: number of columns in scan
    */
    void Allocate(int rows, int cols);

    /*!
    * @brief: Hard reset internal counters and prev_uid
    */
    void HardReset() noexcept;

    /*!
    * @brief: Try to reset the internal counters if it is full
    * @params: integer indicating column is full
    */
    void SoftReset(int full_col) noexcept;

    /*!
    * @brief: Invalidate an entire column
    * @param: delta time between two columns
    */
    void InvalidateColumn(double dt_col) noexcept;

    /*!
    * @brief: Decode column
    * @param: column buffer from raw lidar packet
    * @param: lidar model containing lidar info
    */
    void DecodeColumn(const uint8_t* const col_buf, const LidarModel& model);

    /*!
    * @brief: Update camera info roi data with this scan
    * @param: msg contianing camera info
    */
    void UpdateCinfo(sensor_msgs::CameraInfo& cinfo) const noexcept;

    /*!
    * @breif: prepares the PointField msg
    * @returns: vector of PointField msg.
    */
    std::vector<sensor_msgs::PointField> MakePointFieldsXYZI() noexcept;
};
