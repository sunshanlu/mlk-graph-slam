# MLK-SLAM

![C++](https://img.shields.io/badge/c++-std14-blue)
![ubuntu](https://img.shields.io/badge/platform-ubuntu20.04-orange)
![SLAM](https://img.shields.io/badge/slam-stereo--visual-green)
![LK](https://img.shields.io/badge/opticalFlow-LK-brightgreen)
![BA](https://img.shields.io/badge/BA-SildingWindow-yellowgreen)
![License](https://img.shields.io/badge/license-Apache-blue)
![version](https://img.shields.io/badge/version-v1.0.0-orange)

该项目构建了一种基于多层光流法的前端和滑动窗口后端BA的简单slam系统，主要借鉴了《视觉slam十四讲》的内容。

系统的运行效果如下所示：

<div align="center">
	<a href="https://www.youtube.com/watch?v=6GFBdjZPgTg" title="youtube:MLK-SLAM">
		<img src="https://raw.githubusercontent.com/sunshanlu/mlk-graph-slam/main/doc/imgs/mlk-slam.png" alt="mlk-slam">
	</a>
</div>

## 特征

- 前端采用多层光流法，用于特征匹配
- 后端采用了基于滑动窗口的BA
- 与《视觉slam十四讲》不同的是
  - 前端线程并不会等待后端线程运行结束
  - 使用Pangolin渲染图片，而不是使用OpenCV
  - 使用更常用的json作为配置文件，而不是yml
  - 使用更轻量的spdlog作为日志输出库，而不是glog

## 依赖
- [vcpkg install](https://github.com/microsoft/vcpkg)
- [cmake 3.16+](https://cmake.org/)

## 文档
文档地址：https://sunshanlu.github.io/mlk-graph-slam/annotated.html

## LICENSE

该项目的代码是根据[MIT License](./LICENSE)许可证授权的。如需引用，请说明出处。