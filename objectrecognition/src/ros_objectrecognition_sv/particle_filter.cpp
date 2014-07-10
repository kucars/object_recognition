#include <ros_objectrecognition/particle_filter.h>
#include <pcl/io/vtk_lib_io.h>

//typedef  pcl::PointNormal PointType;
typedef pcl::PointXYZRGBA PointType;

void usage (char** argv)
{
    std::cout << "usage: " << argv[0] << " <device_id> [-C] [-g]\n\n";
    std::cout << " -C: initialize the pointcloud to track without plane segmentation"
              << std::endl;
    std::cout << " -D: visualizing with non-downsampled pointclouds."
              << std::endl;
    std::cout << " -P: not visualizing particle cloud."
              << std::endl;
    std::cout << " -fixed: use the fixed number of the particles."
              << std::endl;
    std::cout << " -d <value>: specify the grid size of downsampling (defaults to 0.01)."
              << std::endl;
}

int main (int argc, char** argv)
{
    bool use_convex_hull = true;
    bool visualize_non_downsample = false;
    bool visualize_particles = true;
    bool use_fixed = false;

    double downsampling_grid_size = 0.01;

    std::string mesh_file_vtk_="/home/kuri/catkin_ws/devel/lib/objectrecognition/coke_can.ply";
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY(mesh_file_vtk_, mesh);
    pcl::PointCloud<PointType>::Ptr model_cloud(new pcl::PointCloud<PointType>);
    pcl::fromPCLPointCloud2(mesh.cloud, *model_cloud);


    if (pcl::console::find_argument (argc, argv, "-C") > 0)
        use_convex_hull = false;
    if (pcl::console::find_argument (argc, argv, "-D") > 0)
        visualize_non_downsample = true;
    if (pcl::console::find_argument (argc, argv, "-P") > 0)
        visualize_particles = false;
    if (pcl::console::find_argument (argc, argv, "-fixed") > 0)
        use_fixed = true;
    pcl::console::parse_argument (argc, argv, "-d", downsampling_grid_size);
    if (argc < 2)
    {
        usage (argv);
        exit (1);
    }

    std::string device_id = std::string (argv[1]);

    if (device_id == "--help" || device_id == "-h")
    {
        usage (argv);
        exit (1);
    }

    // open kinect
    //pcl::PointXYZRGBA
    OpenNISegmentTracking<PointType> v(device_id, 8, downsampling_grid_size,
                                               use_convex_hull,
                                               visualize_non_downsample, visualize_particles,
                                               use_fixed,
                                               model_cloud);
    v.run ();
}
