/* 外部依赖 */
#include <iostream>
#include <fstream>

/* 内部依赖 */
#include <include/backend.hpp>
using namespace ESKF_VIO_BACKEND;
using Scalar = ESKF_VIO_BACKEND::Scalar;

/* 测试用相关定义 */
std::string simPath = "../simulate/";
double maxTimeStamp = 10;

/* 载入 IMU 数据 */
void LoadIMUData(const std::shared_ptr<Backend> &backend) {
    std::string imu_file = simPath + "imu_pose.txt";
    std::cout << ">> Load imu data from " << imu_file << std::endl;
    std::ifstream fsIMU;
    fsIMU.open(imu_file.c_str());
    if (!fsIMU.is_open()) {
        std::cout << "   failed." << std::endl;
        return;
    }
    std::string oneLine;
    double timeStamp;
    Eigen::Matrix<Scalar, 3, 1> acc, gyr, pos;
    Eigen::Quaternion<Scalar> q;
    uint32_t cnt = 0;
    while (std::getline(fsIMU, oneLine) && !oneLine.empty()) {
        std::istringstream imuData(oneLine);
        imuData >> timeStamp >> q.w() >> q.x() >> q.y() >> q.z() >> pos.x() >> pos.y() >> pos.z()
			>> gyr.x() >> gyr.y() >> gyr.z() >> acc.x() >> acc.y() >> acc.z();

        std::shared_ptr<IMUMessage> imuMsg(new IMUMessage(gyr, acc, timeStamp));
        backend->GetIMUMessage(imuMsg);
        ++cnt;
    }
    std::cout << "   " << cnt << " imu raw data loaded.\n";
}

/* 载入特征点追踪数据 */
void LoadFeaturesData(const std::shared_ptr<Backend> &backend) {
    /* 读取所有特征点 */
    std::vector<Eigen::Matrix<Scalar, 3, 1>> allPoints;
    std::string pts_file = simPath + "all_points.txt";
    std::cout << ">> Load pts data from " << pts_file << std::endl;
    std::ifstream fsPts;
    fsPts.open(pts_file.c_str());
    if (!fsPts.is_open()) {
        std::cout << "   failed." << std::endl;
        return;
    }
    std::string oneLine;
    Eigen::Matrix<Scalar, 3, 1> pos;
    while (std::getline(fsPts, oneLine) && !oneLine.empty()) {
        std::istringstream ptsData(oneLine);
        Scalar unused;
        ptsData >> pos.x() >> pos.y() >> pos.z() >> unused;
        allPoints.emplace_back(pos);
    }
    fsPts.close();

    /* 读取相机位姿 */
	std::string camPose = simPath + "cam_pose.txt";
    std::cout << ">> Load camera and features data from " << camPose << std::endl;
	std::ifstream fsCam;
	fsCam.open(camPose.c_str());
	if (!fsCam.is_open()) {
        std::cout << "   failed." << std::endl;
        return;
	}
	Eigen::Quaternion<Scalar> q_wc;
	Eigen::Matrix<Scalar, 3, 1> p_wc;
	double timeStamp;
    uint32_t cnt = 0;

	// 提取对应时间戳的相机位姿，计算此时的观测
	while (std::getline(fsCam, oneLine) && !oneLine.empty()){
		std::vector<Eigen::Matrix<Scalar, 2, 1>> features;
		std::istringstream camData(oneLine);
		camData >> timeStamp >> q_wc.w() >> q_wc.x() >> q_wc.y() >> q_wc.z() >> p_wc.x() >> p_wc.y() >> p_wc.z();

		// 将世界坐标系的 points 投影到归一化平面
		for (unsigned long i = 0; i < allPoints.size(); i++) {
			Eigen::Matrix<Scalar, 3, 1> pc = q_wc.inverse() * (allPoints[i] - p_wc);
			Eigen::Matrix<Scalar, 2, 1> feature = Eigen::Matrix<Scalar, 2, 1>(pc(0, 0) / pc(2, 0), pc(1, 0) / pc(2, 0));
			features.emplace_back(feature);
		}

        // 构造所需输入
        std::vector<uint32_t> ids;  // 特征点的 global ID
        std::vector<std::shared_ptr<FeatureObserve>> obs;
        std::vector<uint8_t> flag;
        for (uint32_t i = 0; i < allPoints.size(); ++i) {
            ids.emplace_back(i);

            std::unordered_map<uint32_t, Eigen::Matrix<Scalar, 2, 1>> norms;
            norms.insert(std::make_pair(0, features[i]));
            std::shared_ptr<FeatureObserve> ob(new FeatureObserve(norms));
            obs.emplace_back(ob);

            flag.emplace_back(0);
        }
        std::shared_ptr<FeaturesMessage> featuresMsg(new FeaturesMessage(ids, obs, flag, timeStamp));
        backend->GetFeaturesMessage(featuresMsg);
        ++cnt;
    }
    std::cout << "   " << cnt << " features track data loaded.\n";

}

int main() {
    // 配置 std::cout 打印到指定文件
    std::ofstream logFile("../test_log/20220917_test_load_config_file.txt");
    std::streambuf *buf = std::cout.rdbuf(logFile.rdbuf());

    std::cout << "This is a vio backend with filter estimator." << std::endl;
    std::shared_ptr<Backend> backend(new Backend());
    backend->Initialize("../config");

    // LoadIMUData(backend);
    // LoadFeaturesData(backend);

    for (uint32_t i = 0; i < 80; ++i) {
        // backend->RunOnce();
    }
    return 0;
}