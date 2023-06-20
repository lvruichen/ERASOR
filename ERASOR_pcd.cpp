/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 21:03
 * Description: No ROS version, speed up the process
 *
 * Input: PCD files + Prior raw global map , check our benchmark in dufomap
 * Output: Cleaned global map
 */

#include <glog/logging.h>

// log
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include "MapUpdater.h"

namespace fs = std::filesystem;

// global variable
std::shared_ptr<spdlog::logger> logger;
pcl::PointCloud<PointT>::Ptr map_global_orig;
pcl::PointCloud<PointT>::Ptr map_global_curr;
pcl::VoxelGrid<PointT> downsize_filter;

// total
vector<string> sequence_scan_names;
vector<string> sequence_scan_paths;
vector<Eigen::Matrix4d> sequence_scan_poses;
vector<Eigen::Matrix4d> sequence_scan_inverse_poses;
// submap
vector<string> sequence_valid_scan_paths;
vector<string> sequence_valid_scan_names;
vector<Eigen::Matrix4d> scan_poses;
vector<Eigen::Matrix4d> scan_inverse_poses;
vector<pcl::PointCloud<PointT>::Ptr> scans;
vector<pcl::PointCloud<PointT>::Ptr> scans_origin;
// param
bool isScanFileKITTIFormat;
int start_idx;
int end_idx;
double kDownsampleVoxelSize;

void allocateMemory() {
    downsize_filter.setLeafSize(kDownsampleVoxelSize, kDownsampleVoxelSize, kDownsampleVoxelSize);
    map_global_orig.reset(new pcl::PointCloud<PointT>());
    map_global_curr.reset(new pcl::PointCloud<PointT>());
}

void readBin(std::string _bin_path, pcl::PointCloud<PointT>::Ptr _pcd_ptr)
{
 	std::fstream input(_bin_path.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << _bin_path << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
  
	for (int ii=0; input.good() && !input.eof(); ii++) {
		PointT point;

		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));

		_pcd_ptr->push_back(point);
	}
	input.close();
}

std::vector<double> splitPoseLine(std::string _str_line, char _delimiter) {
    std::vector<double> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter)) {
        parsed.push_back(std::stod(temp)); // convert string to "double"
    }
    return parsed;
}

void loadPoseFile(const string& _sequence_pose_dir) {
    std::ifstream pose_file_handle (_sequence_pose_dir);
    int num_poses {0};
    std::string strOneLine;
    if (isScanFileKITTIFormat) {
        while (getline(pose_file_handle, strOneLine)) 
        {
        // str to vec
        std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' ');
        if(ith_pose_vec.size() == 12) {
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(1.0));
        }
        // vec to eig
        Eigen::Matrix4d ith_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ith_pose_vec.data(), 4, 4);
        Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();
        // save (move)
        // cout << "Pose of scan: " << sequence_scan_names_.at(num_poses) << endl;
        // cout << ith_pose << endl;
        sequence_scan_poses.emplace_back(ith_pose);
        sequence_scan_inverse_poses.emplace_back(ith_pose_inverse);
        num_poses++;
        }
    }
    else {
        while (getline(pose_file_handle, strOneLine)) 
            {
                // str to vec
                std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' ');
                if(ith_pose_vec.size() == 12) {
                    ith_pose_vec.emplace_back(double(0.0)); 
                    ith_pose_vec.emplace_back(double(0.0)); 
                    ith_pose_vec.emplace_back(double(0.0)); 
                    ith_pose_vec.emplace_back(double(1.0));
                }
                std::stringstream ss(strOneLine);
                vector<string> _res;
                string item;
                while(std::getline(ss, item, ' '))  {
                    _res.push_back(item);
                }
                vector<double> _pose = {stod(_res[2]), stod(_res[3]), stod(_res[4]), stod(_res[5]), stod(_res[6]), stod(_res[7]), stod(_res[8])};
                Eigen::Quaterniond _q(_pose[6], _pose[3], _pose[4], _pose[5]);
                Eigen::Vector3d _trans(_pose[0], _pose[1], _pose[2]);
                Eigen::Isometry3d _isom = Eigen::Isometry3d::Identity();
                _isom.rotate(_q);
                _isom.pretranslate(_trans);       
                // vec to eig
                Eigen::Matrix4d ith_pose = _isom.matrix();
                Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();
                // save (move)
                // cout << "Pose of scan: " << sequence_scan_names_.at(num_poses) << endl;
                // cout << ith_pose << endl;
                sequence_scan_poses.emplace_back(ith_pose);
                sequence_scan_inverse_poses.emplace_back(ith_pose_inverse);
                num_poses++;
            }
    }

}

void loadScanInfo(const string& _sequence_scan_dir) {
    for(auto& _entry : fs::directory_iterator(_sequence_scan_dir)) {
        string file_name = _entry.path().filename();
        if(file_name.substr(file_name.size() - 3, 3) == "txt") {
            continue;
        }
        sequence_scan_names.emplace_back(_entry.path().filename());
        sequence_scan_paths.emplace_back(_entry.path());
    }
    if (isScanFileKITTIFormat) {
        std::sort(sequence_scan_names.begin(), sequence_scan_names.end());
         std::sort(sequence_scan_paths.begin(), sequence_scan_paths.end());
    }
    else {
        std::sort(sequence_scan_names.begin(), sequence_scan_names.end(), [](string& s1, string& s2) {
        string _s1 = s1.substr(0, s1.size() - 4);
        string _s2 = s2.substr(0, s2.size() - 4);
        return std::stoi(_s1) < std::stoi(_s2);
         });

        std::sort(sequence_scan_paths.begin(), sequence_scan_paths.end(), [](string& s1, string& s2) {
        string _s1;
        for (int i = s1.size()-1; i >=0; --i) {
            if(s1[i] == '/')
                break;
            _s1 = s1[i] + _s1;
        }
        _s1 = _s1.substr(0, _s1.size() - 4);
        string _s2;
        for (int i = s2.size()-1; i >=0; --i) {
            if(s2[i] == '/')
                break;
            _s2 = s2[i] + _s2;
        }
        _s2 = _s2.substr(0, _s2.size() - 4);
        return std::stoi(_s1) < std::stoi(_s2);
        });
    }
}

void parseValidScanInfo() {
    for (int curr_idx = start_idx; curr_idx < int(sequence_scan_paths.size()) && curr_idx < end_idx; curr_idx++) {
        sequence_valid_scan_paths.emplace_back(sequence_scan_paths.at(curr_idx));
        sequence_valid_scan_names.emplace_back(sequence_scan_names.at(curr_idx));
        scan_poses.emplace_back(sequence_scan_poses.at(curr_idx)); // used for local2global
        scan_inverse_poses.emplace_back(sequence_scan_inverse_poses.at(curr_idx)); // used for global2local
    }
}

void readValidScans() {
    for(auto& _scan_path : sequence_valid_scan_paths) 
    {
        // read bin files and save  
        pcl::PointCloud<PointT>::Ptr points (new pcl::PointCloud<PointT>); // pcl::PointCloud Ptr is a shared ptr so this points will be automatically destroyed after this function block (because no others ref it).
        if( isScanFileKITTIFormat ) {
            readBin(_scan_path, points); // For KITTI (.bin)
        } else {
            pcl::io::loadPCDFile<PointT> (_scan_path, *points); // saved from SC-LIO-SAM's pcd binary (.pcd)
        }
        scans.emplace_back(points);
        scans_origin.emplace_back(points);
    }
}

void mergeScansWithinGlobalCoord( 
        const std::vector<pcl::PointCloud<PointT>::Ptr>& _scans, 
        const std::vector<Eigen::Matrix4d>& _scans_poses,
        pcl::PointCloud<PointT>::Ptr& _ptcloud_to_save ) {
    for(std::size_t scan_idx = 0 ; scan_idx < _scans.size(); scan_idx++) {
        auto ii_scan = _scans.at(scan_idx); // pcl::PointCloud<PointType>::Ptr
        auto ii_pose = _scans_poses.at(scan_idx); // Eigen::Matrix4d 

        // local to global (local2global)
        pcl::PointCloud<PointT>::Ptr scan_global_coord(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*ii_scan, *scan_global_coord, ii_pose);
        // merge the scan into the global map
        *_ptcloud_to_save += *scan_global_coord;
    }
}

void makeGlobalMap() {
    map_global_orig->clear();
    map_global_curr->clear();
    mergeScansWithinGlobalCoord(scans, scan_poses, map_global_orig);
    downsize_filter.setInputCloud(map_global_orig);
    downsize_filter.filter(*map_global_curr);
}

void resetParameter() {

}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);
    logger = spdlog::stdout_color_mt("main");

    if (argc < 2) {
        logger->error("Usage: {}[config_file]", argv[0]);
        return 0;
    }
    std::string config_file = argv[1]; // we assume that rawmap is in pcd_parent;
    // check if the config_file exists
    if (!std::filesystem::exists(config_file)) {
        logger->error("Config file does not exist: {}", config_file);
        return 0;
    }

    erasor::MapUpdater map_updater(config_file);

    // load config file
    YAML::Node yaml_node;
    yaml_node = YAML::LoadFile(config_file);
    isScanFileKITTIFormat            = yaml_node["isScanFileKITTIFormat"].as<bool>();
    kDownsampleVoxelSize         = yaml_node["leafSize"].as<double>(); 
    start_idx                                        = yaml_node["start_idx"].as<int>();
    end_idx                                         = yaml_node["end_idx"].as<int>();
    string sequence_scan_dir     = yaml_node["sequence_scan_dir"].as<string>();
    string sequence_pose_path = yaml_node["sequence_pose_path"].as<string>();
    string save_pcd_folder    = yaml_node["save_pcd_folder"].as<string>();

    // load pose and scan info
    allocateMemory();
    loadPoseFile(sequence_pose_path);
    loadScanInfo(sequence_scan_dir);
    assert(sequence_scan_paths.size() == sequence_scan_poses.size());
    int num_total_scans_of_sequence = sequence_scan_paths.size();
    logger->info("Total load {} scan", num_total_scans_of_sequence);

    // load valid scan for each submap
    logger->info("start_idx: {}, end_idx: {}", start_idx, end_idx);
    parseValidScanInfo();
    readValidScans();
    makeGlobalMap();
    logger->info("origin submap contains : {} points", map_global_orig->size());
    logger->info("downsize submap contains : {} points", map_global_curr->size());

    map_updater.setRawMap(map_global_orig);
    for (int i = 0; i < scans.size(); ++i) {
        map_updater.timing.start(" One Scan Cost  ");
        map_updater.run(scans[i], scan_poses[i]);
        map_updater.timing.stop(" One Scan Cost  ");
    }
    map_updater.saveMap(save_pcd_folder);
    pcl::io::savePCDFileBinary(save_pcd_folder + "erasor_output_ori.pcd", *map_global_curr);
    return 0;
}
