/* 外部依赖 */
#include <fstream>
#include <iostream>

/* 内部依赖 */
#include <backend.hpp>
using namespace ESKF_VIO_BACKEND;
using Scalar = ESKF_VIO_BACKEND::Scalar;

/* 测试用相关定义 */
std::string simPath = "../simulate/";
std::string configPath = "../eskf_vio_backend/config";
double maxTimeStamp = 20;

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
    Vector3 acc, gyr, pos;
    Quaternion q;
    uint32_t cnt = 0;
    while (std::getline(fsIMU, oneLine) && !oneLine.empty()) {
        std::istringstream imuData(oneLine);
        imuData >> timeStamp >> q.w() >> q.x() >> q.y() >> q.z() >> pos.x() >> pos.y() >> pos.z()
			>> gyr.x() >> gyr.y() >> gyr.z() >> acc.x() >> acc.y() >> acc.z();

        std::shared_ptr<IMUMessage> imuMsg(new IMUMessage(gyr, acc, timeStamp));
        backend->GetIMUMessage(imuMsg);
        ++cnt;
        if (imuMsg->timeStamp > maxTimeStamp) {
            break;
        }
    }
    std::cout << "   " << cnt << " imu raw data loaded." << std::endl;
}

/* 载入特征点追踪数据 */
void LoadFeaturesData(const std::shared_ptr<Backend> &backend) {
    /* 读取所有特征点 */
    std::vector<Vector3> allPoints;
    std::string pts_file = simPath + "all_points.txt";
    std::cout << ">> Load pts data from " << pts_file << std::endl;
    std::ifstream fsPts;
    fsPts.open(pts_file.c_str());
    if (!fsPts.is_open()) {
        std::cout << "   failed." << std::endl;
        return;
    }
    std::string oneLine;
    Vector3 pos;
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
	Quaternion q_wc;
	Vector3 p_wc;
	double timeStamp;
    uint32_t cnt = 0;

	// 提取对应时间戳的相机位姿，计算此时的观测
	while (std::getline(fsCam, oneLine) && !oneLine.empty()) {
		std::vector<Eigen::Matrix<Scalar, 2, 1>> features_0;
		std::vector<Eigen::Matrix<Scalar, 2, 1>> features_1;    // p_c0c1 = [0.1, 0, 0]
		std::istringstream camData_0(oneLine);
		camData_0 >> timeStamp >> q_wc.w() >> q_wc.x() >> q_wc.y() >> q_wc.z() >> p_wc.x() >> p_wc.y() >> p_wc.z();

		// 将世界坐标系的 points 投影到归一化平面
		for (unsigned long i = 0; i < allPoints.size(); i++) {
			Vector3 pc_0 = q_wc.inverse() * (allPoints[i] - p_wc);
			Eigen::Matrix<Scalar, 2, 1> feature_0 = Eigen::Matrix<Scalar, 2, 1>(pc_0(0, 0) / pc_0(2, 0), pc_0(1, 0) / pc_0(2, 0));
			features_0.emplace_back(feature_0);

            Vector3 pc_1 = q_wc.inverse() * (allPoints[i] - p_wc - q_wc * Vector3(0.1, 0, 0));
			Eigen::Matrix<Scalar, 2, 1> feature_1 = Eigen::Matrix<Scalar, 2, 1>(pc_1(0, 0) / pc_1(2, 0), pc_1(1, 0) / pc_1(2, 0));
			features_1.emplace_back(feature_1);
		}

        // 构造所需输入
        std::vector<uint32_t> ids;  // 特征点的 global ID
        std::vector<std::shared_ptr<FeatureObserve>> obs;
        std::vector<uint8_t> flag;
        for (uint32_t i = 0; i < allPoints.size(); ++i) {
            ids.emplace_back(i);

            std::unordered_map<uint32_t, Eigen::Matrix<Scalar, 2, 1>> norms;
            norms.insert(std::make_pair(0, features_0[i]));
            norms.insert(std::make_pair(1, features_1[i]));
            std::shared_ptr<FeatureObserve> ob(new FeatureObserve(norms));
            obs.emplace_back(ob);

            flag.emplace_back(0);
        }
        std::shared_ptr<FeaturesMessage> featuresMsg(new FeaturesMessage(ids, obs, flag, timeStamp));
        backend->GetFeaturesMessage(featuresMsg);
        ++cnt;
        if (featuresMsg->timeStamp > maxTimeStamp) {
            break;
        }
    }
    std::cout << "   " << cnt << " features track data loaded." << std::endl;

}

int main(int argc, char **argv) {
    // 处理输入的配置参数路径和数据路径
    if (argc != 3) {
        std::cerr << "Data path and Config path are needed." << std::endl;
        return -1;
    }
    simPath = argv[1];
    configPath = argv[2];

    // 配置 std::cout 打印到指定文件
    // std::ofstream logFile("../test_log/20221006_msckf_init_and_expand_state_cov_size.txt");
    // std::streambuf *buf = std::cout.rdbuf(logFile.rdbuf());

    // 初始化配置 vio backend，并载入数据
    std::cout << "This is a vio backend with filter estimator." << std::endl;
    std::shared_ptr<Backend> backend(new Backend());
    backend->Initialize(configPath);
    LoadIMUData(backend);
    LoadFeaturesData(backend);

    // 运行测试
    for (uint32_t i = 0; i < 20; ++i) {
        std::cout << "\n --- \n";
        backend->RunOnce();
        ESKF_VIO_BACKEND::IMUFullState state;
        bool res = backend->PublishPropagateState(state);
        if (res == true) {
            // Vector3 pitch_roll_yaw = ESKF_VIO_BACKEND::Utility::QuaternionToEuler(state.q_wb);
            // std::cout << pitch_roll_yaw.x() << " " << pitch_roll_yaw.y() << " " << pitch_roll_yaw.z() << " ";
            std::cout << state.p_wb.transpose() << std::endl;
        } else {
            std::cout << "backend is not ready." << std::endl;
        }
    }
    return 0;
}