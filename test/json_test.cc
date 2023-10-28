#include <fstream>
#include <iostream>
#include <string>

#include <json/json.h>
#include <opencv2/opencv.hpp>

std::string KITTI_PATH;
int DETECT_FEATURE_NUM;
int MAX_ACTIVE_KEYFRAME_NUM;
double KEYFRAME_DISTANCE_TH;
double EDGE_CHI2_TH;
int OUTLIER_DETECT_ITERATIONS;
int OPTIMIZE_ITERATIONS;
int TRACK_GOOD_TH;
int TRACK_BAD_TH;
int MIN_MAPPOINTS_NUM_FOR_TRACK;
float MAPPOINT_SIZE;
float LINE_WIDTH;
float CURR_FRAME_COLOR[3];
float MAPPOINT_COLOR[3];
float MAP_FRAME_COLOR[3];
cv::Scalar FEATURE_COLOR;

int main() {
    std::ifstream inJson(R"(/home/rookie-lu/Project/mlk-graph-slam/config/config.json)");
    std::string error;
    Json::Value root;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    Json::parseFromStream(builder, inJson, &root, &error);

    KITTI_PATH = root["KITTI_PATH"].asString();
    DETECT_FEATURE_NUM = root["DETECT_FEATURE_NUM"].asInt();
    MAX_ACTIVE_KEYFRAME_NUM = root["MAX_ACTIVE_KEYFRAME_NUM"].asInt();
    KEYFRAME_DISTANCE_TH = root["KEYFRAME_DISTANCE_TH"].asDouble();
    EDGE_CHI2_TH = root["EDGE_CHI2_TH"].asDouble();
    OUTLIER_DETECT_ITERATIONS = root["OUTLIER_DETECT_ITERATIONS"].asInt();
    OPTIMIZE_ITERATIONS = root["OPTIMIZE_ITERATIONS"].asInt();
    TRACK_GOOD_TH = root["TRACK_GOOD_TH"].asInt();
    TRACK_BAD_TH = root["TRACK_BAD_TH"].asInt();
    MIN_MAPPOINTS_NUM_FOR_TRACK = root["MIN_MAPPOINTS_NUM_FOR_TRACK"].asInt();
    MAPPOINT_SIZE = root["MAPPOINT_SIZE"].asFloat();
    LINE_WIDTH = root["LINE_WIDTH"].asFloat();
    for (int idx = 0; idx < 3; ++idx)
        CURR_FRAME_COLOR[idx] = root["CURR_FRAME_COLOR"][idx].asFloat();

    for (int idx = 0; idx < 3; ++idx)
        MAPPOINT_COLOR[idx] = root["MAPPOINT_COLOR"][idx].asFloat();

    for (int idx = 0; idx < 3; ++idx)
        MAP_FRAME_COLOR[idx] = root["MAP_FRAME_COLOR"][idx].asFloat();

    for (int idx = 0; idx < 3; ++idx)
        FEATURE_COLOR[idx] = root["FEATURE_COLOR"][idx].asUInt();

    std::cout << KITTI_PATH << std::endl;
    std::cout << DETECT_FEATURE_NUM << std::endl;
    std::cout << MAX_ACTIVE_KEYFRAME_NUM << std::endl;
    std::cout << KEYFRAME_DISTANCE_TH << std::endl;
    std::cout << EDGE_CHI2_TH << std::endl;
    std::cout << OUTLIER_DETECT_ITERATIONS << std::endl;
    std::cout << OPTIMIZE_ITERATIONS << std::endl;
    std::cout << TRACK_GOOD_TH << std::endl;
    std::cout << TRACK_BAD_TH << std::endl;
    std::cout << MIN_MAPPOINTS_NUM_FOR_TRACK << std::endl;
    std::cout << MAPPOINT_SIZE << std::endl;
    std::cout << LINE_WIDTH << std::endl;
    std::cout << CURR_FRAME_COLOR[0] << CURR_FRAME_COLOR[1] << CURR_FRAME_COLOR[2] << std::endl;
    std::cout << MAPPOINT_COLOR[0] << MAPPOINT_COLOR[1] << MAPPOINT_COLOR[2] << std::endl;
    std::cout << MAP_FRAME_COLOR[0] << MAP_FRAME_COLOR[1] << MAP_FRAME_COLOR[2] << std::endl;
    std::cout << FEATURE_COLOR << std::endl;

    return 0;
}