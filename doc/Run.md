# Run


配置文件config.json中的变量含义如下：
```shell
# config/config.json
KITTI_PATH                    KITTI数据集地址
DETECT_FEATURE_NUM            征检测数目
MAX_ACTIVE_KEYFRAME_NUM       Map中m_activeFrames的数目
KEYFRAME_DISTANCE_TH          保证Map::m_activeFrames的距离大于0.5
EDGE_CHI2_TH                  优化边的阈值
OUTLIER_DETECT_ITERATIONS     进行异常点检测的优化迭代次数
OPTIMIZE_ITERATIONS           单次优化迭代次数
TRACK_GOOD_TH                 track good 阈值
TRACK_BAD_TH                  track bad 阈值
MIN_MAPPOINTS_NUM_FOR_TRACK   track可执行的最少特征数目
MAPPOINT_SIZE                 定义绘制地图点的大小
LINE_WIDTH                    定义绘制线断的大小
CURR_FRAME_COLOR              定义绘制当前帧的颜色
MAPPOINT_COLOR                定义绘制地图点的颜色
MAP_FRAME_COLOR               定义绘制激活关键帧的颜色
FEATURE_COLOR                 定义绘制特征点的颜色 
```
- 项目编译完成后记得将`KITTI_PATH`的默认值修改到你电脑上的路径
- 其他参数可以保持不变，也可以根据你的理解自行修改
- 运行`build/mlk-slam`可执行文件即可
