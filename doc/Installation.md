# Installation

文档提供两种安装项目依赖的方式
- 通过vcpkg包管理工具安装（推荐）
- 通过手动下载依赖包进行安装

## 安装依赖
方式一：采用[vcpkg包管理工具](https://github.com/microsoft/vcpkg)安装依赖

需要去vcpkg的github主页进行vcpkg管理工具的安装
```shell
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="[vcpkg root]/scripts/buildsystems/vcpkg.cmake"
cmake --build . -j10
```

方式二：手动下载依赖包进行安装
- [OpenCV – 3.4.16](https://opencv.org/releases/)
- [fmt - 8.1.0](https://github.com/fmtlib/fmt/releases/tag/8.1.0)
- [spdlog - 1.10.0](https://github.com/gabime/spdlog/releases/tag/v1.10.0)
- [jsoncpp - 1.9.5](https://github.com/open-source-parsers/jsoncpp/releases/tag/1.9.5)
- [g20 - 2023-02-23](https://github.com/RainerKuemmerle/g2o/releases/tag/20230223_git)
- [eigen - 3.4.0](https://gitlab.com/libeigen/eigen/-/releases)

注意，采用第二种方式进行项目依赖库的安装的话，需要修改`cmake/FindG2O.cmake`文件的内容。将 https://github.com/gaoxiang12/slambook2/blob/master/ch13/cmake_modules/FindG2O.cmake 的`FindG2O.cmake`内容替换到`cmake/FindG2O.cmake`中即可。

```shell
mkdir build
cd build
cmake ..
buildsystems/vcpkg.cmake"
cmake --build . -j10
```