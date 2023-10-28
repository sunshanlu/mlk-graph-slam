#include "config.h"

NAMESPACE_BEGIN

void Config::loadConfig() {
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
}

std::string Config::KITTI_PATH;
int Config::DETECT_FEATURE_NUM;
int Config::MAX_ACTIVE_KEYFRAME_NUM;
double Config::KEYFRAME_DISTANCE_TH;
double Config::EDGE_CHI2_TH;
int Config::OUTLIER_DETECT_ITERATIONS;
int Config::OPTIMIZE_ITERATIONS;
int Config::TRACK_GOOD_TH;
int Config::TRACK_BAD_TH;
int Config::MIN_MAPPOINTS_NUM_FOR_TRACK;
float Config::MAPPOINT_SIZE;
float Config::LINE_WIDTH;
float Config::CURR_FRAME_COLOR[3];
float Config::MAPPOINT_COLOR[3];
float Config::MAP_FRAME_COLOR[3];
cv::Scalar Config::FEATURE_COLOR;

NAMESPACE_END
