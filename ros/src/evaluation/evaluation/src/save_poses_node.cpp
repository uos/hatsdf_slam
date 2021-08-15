#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <evaluation/SavePoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>
#include <chrono>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * This Class saves incoming geometry_msgs/SavePoseStamped (id + PoseStamped) in a file called
 * poses_<ID>_<TIMESTAMP>.txt
 */
struct PoseSaver
{
    /// Default destructor
    PoseSaver() = default;

    /// Destructor: save files
    ~PoseSaver()
    {
        for (auto& [id, stream]: pose_streams)
        {
            ROS_INFO_STREAM("Closing filestream for: " << id);
            stream.close();
        }
    }

    /**
     * Creates filestream poses_<id>_<timestamp>
     * @param id part of file name
     * @return true if file could be opened, false if not
     */
    bool createFStream(const std::string& id)
    {
        std::ostringstream filename;
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        filename << "poses_" << id << "_" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S") << ".txt";

        std::ofstream pose_stream;
        pose_stream.open(filename.str());

        if (!pose_stream)
        {
            ROS_ERROR_STREAM("Cannot save pose path!");
            return false;
        }

        ROS_INFO_STREAM("Saving poses for id '" << id << "' in ~/ros/" << filename.str());

        pose_streams[id] = std::move(pose_stream);
        return true;
    }

    /**
     * Save Pose to file, in one line: x y z xx yy zz ww
     * @param id id of filestream
     * @param pose_stamped stamped Pose to save
     */
    void savePose(const std::string& id, const geometry_msgs::PoseStamped& pose_stamped)
    {
        auto& pose_stream = pose_streams.at(id);
        // pose_stream << pose_stamped.pose.position.x << " " << pose_stamped.pose.position.y << " " << pose_stamped.pose.position.z << " "
        //              << pose_stamped.pose.orientation.x << " " << pose_stamped.pose.orientation.y << " " << pose_stamped.pose.orientation.z << " "
        //              << pose_stamped.pose.orientation.w << std::endl;
    
        tf2::Quaternion q(-pose_stamped.pose.orientation.y, 
                          -pose_stamped.pose.orientation.z, 
                          -pose_stamped.pose.orientation.x, 
                           pose_stamped.pose.orientation.w);

        tf2::Matrix3x3 rot_matrix(q);

        pose_stream << rot_matrix[0][0] << " " << rot_matrix[0][1] << " " << rot_matrix[0][2] << " " <<  -pose_stamped.pose.position.y << " "
                    << rot_matrix[1][0] << " " << rot_matrix[1][1] << " " << rot_matrix[1][2] << " " <<   pose_stamped.pose.position.z << " "
                    << rot_matrix[2][0] << " " << rot_matrix[2][1] << " " << rot_matrix[2][2] << " " <<   pose_stamped.pose.position.x << std::endl;
    }

    /**
     * Callback that saves pose in a file, line by line.
     * Creates a new file for each id that is received in the SavePoseStamped message
     * @param savepose Pose to save in filename with specific id
     */
    void saveCallback(const evaluation::SavePoseStamped::ConstPtr& savepose)
    {
        const auto& [id, pose] = *savepose;
        if (pose_streams.count(id) == 0)
        {
            if (createFStream(id))
            {
                savePose(id, pose);
            }
        }
        else
        {
            savePose(id, pose);
        }
    }

    std::unordered_map<std::string, std::ofstream> pose_streams;
};

/**
 * Node that saves pose in a file, line by line.
 * Creates a new file for each id that is received in the SavePoseStamped message
 * @param argc n arguments
 * @param argv arguments
 * @return 0 if successful
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_saver_subscriber");
    ros::NodeHandle n("~");

    ROS_WARN_STREAM("This node must be run from a launch file for the shutdown hook to trigger the file to save all its data!");

    PoseSaver saver;
    ros::Subscriber sub = n.subscribe("/evaluation/save_pose", 10000, &PoseSaver::saveCallback, &saver);
    ros::spin();

    return 0;
}
