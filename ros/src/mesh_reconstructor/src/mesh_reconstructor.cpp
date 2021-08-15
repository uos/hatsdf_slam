/**
 * @file mesh_reconstructor.cpp
 * @author Steffen Hinderink
 * 
 * This is a ROS node that reconstructs a mesh from the tsdf values.
 * The tsdf values are given as markers.
 * That way the node can work with the data that is already being published for visualization.
 * The reconstructed mesh is then published.
 * The visualization of the mesh in rviz can give clearer information than the raw tsdf values.
 */

#include <util/constants.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <cmath>
#include <string>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>

/// Marker topic that the node subscribes to
constexpr char MARKER_TOPIC[] = "from_trenz_bridge/tsdf";
/// Mesh topic that the node publishes to
constexpr char MESH_TOPIC[] = "reconstructed_mesh";
/**
 * Truncation or maximum distance of tsdf values.
 * This value is used to rescale the tsdf values that are stored as color values.
 */ 
constexpr int TAU = 600;

/// ROS publisher that publishes the meshes
ros::Publisher pub;

/**
 * Callback function of the ROS subscriber that subscribes to the markers.
 * The marker message is interpreted as a tsdf map.
 * The points of the message are the cell positions and
 * the colors encode the distance.
 * Red color stands for positive distance and green for negative.
 * The brighter the color the greater is the distance.
 * A mesh is reconstructed from the tsdf map and published.
 * @param marker Marker message
 */
void callback(const visualization_msgs::Marker& marker)
{
    std::cout << "Receive marker" << std::endl;

    std::vector<geometry_msgs::Point> points = marker.points;
    std::vector<std_msgs::ColorRGBA> colors = marker.colors;

    // Bounding box
    lvr2::BaseVector<int> min(0, 0, 0); 
    lvr2::BaseVector<int> max(0, 0, 0);
    for (int i = 0; i < points.size(); i++)
    {
        if (points[i].x < min[0]) min[0] = points[i].x;
        if (points[i].x > max[0]) max[0] = points[i].x;
        if (points[i].y < min[1]) min[1] = points[i].y;
        if (points[i].y > max[1]) max[1] = points[i].y;
        if (points[i].z < min[2]) min[2] = points[i].z;
        if (points[i].z > max[2]) max[2] = points[i].z;
    }
    for (int i = 0; i < 3; i++)
    {
        min[i] = std::floor(min[i] * 1000 / MAP_RESOLUTION);
        max[i] = std::ceil(max[i] * 1000 / MAP_RESOLUTION);
    }
    lvr2::BoundingBox<lvr2::BaseVector<int>> bb(min, max);

    // Grid
    auto grid = std::make_shared<lvr2::HashGrid<lvr2::BaseVector<int>, lvr2::FastBox<lvr2::BaseVector<int>>>>(1, bb);
    for (int i = 0; i < points.size(); i++)
    {
        int x = std::round(points[i].x * 1000 / MAP_RESOLUTION);
        int y = std::round(points[i].y * 1000 / MAP_RESOLUTION);
        int z = std::round(points[i].z * 1000 / MAP_RESOLUTION);
        int tsdf_value = 0;
        if (colors[i].r > 0)
        {
            tsdf_value = colors[i].r * TAU;
        }
        else
        {
            tsdf_value = -colors[i].g * TAU;
        }
        grid->addLatticePoint(x, y, z, tsdf_value);
    }

    // Reconstruction
    lvr2::FastReconstruction<lvr2::BaseVector<int>, lvr2::FastBox<lvr2::BaseVector<int>>> reconstruction(grid);
    lvr2::HalfEdgeMesh<lvr2::BaseVector<int>> mesh;
    reconstruction.getMesh(mesh);

    // Publish
    std::vector<geometry_msgs::Point> vertices;
    for (auto vi = mesh.verticesBegin(); vi != mesh.verticesEnd(); ++vi)
    {
        lvr2::BaseVector<int> v = mesh.getVertexPosition(*vi);
        geometry_msgs::Point vertex;
        // - min because the mesh is shifted by min, this fixed the weird translations
        vertex.x = ((float) v[0] - min[0]) * MAP_RESOLUTION / 1000;
        vertex.y = ((float) v[1] - min[1]) * MAP_RESOLUTION / 1000;
        vertex.z = ((float) v[2] - min[2]) * MAP_RESOLUTION / 1000;
        vertices.push_back(vertex);
    }
    std::vector<mesh_msgs::TriangleIndices> faces;
    for (auto fi = mesh.facesBegin(); fi != mesh.facesEnd(); ++fi)
    {
        std::array<lvr2::VertexHandle, 3> f = mesh.getVerticesOfFace(*fi); 
        boost::array<int, 3> a;
        for (int i = 0; i < 3; i++)
        {
            a[i] = f[i].idx();
        }
        mesh_msgs::TriangleIndices face;
        face.vertex_indices = a;
        faces.push_back(face);
    }
    mesh_msgs::MeshGeometry mesh_geometry;
    mesh_geometry.vertices = vertices;
    mesh_geometry.faces = faces;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    mesh_msgs::MeshGeometryStamped mesh_geometry_stamped;
    mesh_geometry_stamped.header = header;
    mesh_geometry_stamped.uuid = "";
    mesh_geometry_stamped.mesh_geometry = mesh_geometry;
    pub.publish(mesh_geometry_stamped);

    std::cout << "Publish reconstructed mesh" << std::endl;
}

/**
 * Main function of the ROS node.
 * The subscriber and publisher are initialized and the node spins.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_reconstructor");
    ros::NodeHandle n;
    pub = n.advertise<mesh_msgs::MeshGeometryStamped>(MESH_TOPIC, 0);
    ros::Subscriber sub = n.subscribe(MARKER_TOPIC, 0, callback);
    ros::spin();
    return 0;
}
