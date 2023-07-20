//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include <boost/format.hpp>
#include <cstdlib>
#include <erasor/OfflineMapUpdater.h>

string DATA_DIR;
int INTERVAL, INIT_IDX;
float VOXEL_SIZE;
bool STOP_FOR_EACH_FRAME;
std::string filename = "/staticmap_via_erasor.pcd";

using PointType = pcl::PointXYZI;


vector<float> split_line(string input, char delimiter) {
    vector<float> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));
    }
    return answer;
}

void load_all_poses(string txt, vector<Eigen::Matrix4f >& poses){
    // These poses are already w.r.t body frame!
    // Thus, tf4x4 by pose * corresponding cloud -> map cloud
    cout<<"Target path: "<< txt <<endl;
    poses.clear();
    poses.reserve(3000);
    std::ifstream in(txt);
    std::string line;
    while (getline(in, line)) {
        float _data[12];
        stringstream strstream(line);
        string _out;
        int i = 0;
        while(strstream >> _out) {
            float a = stof(_out);
            _data[i] = a;
            i++;
        }
        Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
        t(0, 0) = _data[0], t(0, 1) = _data[1], t(0, 2) = _data[2], t(0, 3) = _data[3];
        t(1, 0) = _data[4], t(1, 1) = _data[5], t(1, 2) = _data[6], t(1, 3) = _data[7];
        t(2, 0) = _data[8], t(2, 1) = _data[9], t(2, 2) = _data[10], t(2, 3) = _data[11];
        poses.push_back(t.matrix());
    }
    in.close();
    std::cout<<"Total "<< poses.size() <<" poses are loaded"<<std::endl;
}

void readBinFile(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInptr, string& filePath){
        cloudInptr->clear();
        int32_t num = 10000000;
        float *data = (float*)malloc(num * sizeof(float));
        float *px = data + 0;
        float *py = data + 1;
        float *pz = data + 2;
        float *pr = data + 3;//反射强度
        fstream input(filePath.c_str(), ios::in | ios::binary);
        if(!input.good()){
        cerr << "Couldn't read in_file: " << filePath << endl;
        }
        for (int i = 0; input.good() && !input.eof(); i++) {
            pcl::PointXYZI point;
            input.read((char*) &point.x, 3*sizeof(float));
            input.read((char*) &point.intensity, sizeof(float));
            cloudInptr->push_back(point);
        }
        input.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erasor_in_your_env");
    ros::NodeHandle nh;
    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();

    nh.param<string>("/data_dir", DATA_DIR, "/");
    nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
    nh.param<int>("/init_idx", INIT_IDX, 0);
    nh.param<int>("/interval", INTERVAL, 2);
    nh.param<bool>("/stop_for_each_frame", STOP_FOR_EACH_FRAME, false);

    // Set ROS visualization publishers
    ros::Publisher NodePublisher = nh.advertise<erasor::node>("/node/combined/optimized", 100);
    ros::Rate loop_rate(10);

    /***
     * Set target data
     * Note that pcd files are in `pcd_dir`
     * And check that each i-th pose corresponds to i-th pcd file
     *
     * In our example, transformation result (pose * point cloud) denotes
     * partial part of map cloud directly
     * (i.e. there are no additional transformation matrix, e.g. lidar2body)
     *
     * In your own case, be careful to set max_h and min_h correctly!
     */
    // Set target data
    cout << "\033[1;32mTarget directory:" << DATA_DIR << "\033[0m" << endl;
    string pose_path = DATA_DIR + "/pose.txt";
    string pcd_dir = DATA_DIR + "/velodyne"; //
    // Load raw pointcloud

    vector<Eigen::Matrix4f> poses;
    load_all_poses(pose_path, poses);

    int N = poses.size();

    for (int i = INIT_IDX; i < N; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);

        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
        string pcd_name = (boost::format("%s/%06d.bin") % pcd_dir % i).str();
        readBinFile(srcCloud, pcd_name);
        erasor::node node;
        node.header.seq = i;
        node.odom = erasor_utils::eigen2geoPose(poses[i]);
        node.lidar = erasor_utils::cloud2msg(*srcCloud);
        NodePublisher.publish(node);
        ros::spinOnce();
        loop_rate.sleep();

        if (STOP_FOR_EACH_FRAME) {
            cout<< "[Debug]: STOP! Press any button to continue" <<endl;
            cin.ignore();
        }
    }

    updater.save_static_map(0.2);

    cout<< "Static map building complete!" << endl;

    return 0;
}