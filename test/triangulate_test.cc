#include "common_include.h"
#include "frontend.h"
#include "config.h"

typedef Sophus::SE3d SE3;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 3, 4> Mat34;
using namespace myslam;

// 高翔三角化代码（用作对比）
bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXd A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}

int main() {
    Config::loadConfig();
    std::vector<SE3> poses;
    std::vector<Vec3> normPoints;
    std::vector<Vec2d> pixelPoints;

    auto frontend = Frontend::getInstance();
    auto dataset = KITTISet::getInstance();
    auto camera = dataset->createCamera();
    auto frame = dataset->createFrame();
    frontend->setCamera(camera);

    frame->initKeyFrame(camera);
    Vec2d leftPoint = frame->getLeftFeatures()[0]->m_position;
    Vec2d rightPoint = frame->getRightFeatures()[0]->m_position;

    Vec3d leftNormPoint = camera->m_K.inverse() * Vec3d(leftPoint.x(), leftPoint.y(), 1);
    Vec3d rightNormPoint = camera->m_K.inverse() * Vec3d(rightPoint.x(), rightPoint.y(), 1);

    pixelPoints.push_back(leftPoint);
    pixelPoints.push_back(rightPoint);

    normPoints.push_back(leftNormPoint);
    normPoints.push_back(rightNormPoint);

    poses.push_back(SE3());
    poses.emplace_back(SO3d(), Vec3d(-camera->m_baseline, 0, 0));

    Vec3d myTri;
    Vec3d gaoxiangTri;
    frontend->triangulate(poses, pixelPoints, myTri);
    triangulation(poses, normPoints, gaoxiangTri);

    std::cout << "高翔三角化\t" << gaoxiangTri.transpose() << std::endl;
    std::cout << "我的三角化\t" << myTri.transpose() << std::endl;
}