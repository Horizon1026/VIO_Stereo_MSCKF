/* 外部依赖 */
#include <fstream>
#include <iostream>

/* 内部依赖 */
#include <backend.hpp>
#include <log_api.hpp>
using namespace ESKF_VIO_BACKEND;
using Scalar = ESKF_VIO_BACKEND::Scalar;

/* 测试用相关定义 */
std::string simPath = "/home/horizon_msi_wsl2/code_space/VIO_Stereo_MSCKF/simulate/longtime_scenes/";
std::string configPath = "/home/horizon_msi_wsl2/code_space/VIO_Stereo_MSCKF/eskf_vio_backend/config/";
std::string savePath = "/home/horizon_msi_wsl2/code_space/VIO_Stereo_MSCKF/saved_pose/";
double maxTimeStamp = 100;

/* 载入 IMU 数据 */
uint32_t LoadIMUData(const std::shared_ptr<Backend> &backend) {
    std::string imu_file = simPath + "imu_pose_noise.txt";
    std::cout << ">> Load imu data from " << imu_file << std::endl;
    std::ifstream fsIMU;
    fsIMU.open(imu_file.c_str());
    if (!fsIMU.is_open()) {
        std::cout << "   failed." << std::endl;
        return 0;
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
    return cnt;
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
	Quaternion q_wc0, q_wc1;
	Vector3 p_wc0, p_wc1;
	double timeStamp;
    uint32_t cnt = 0;
    Vector3 p_bc0 = Vector3(0.05, 0.04, 0.03);
    Vector3 p_bc1 = Vector3(0.05, -0.14, 0.03);
    Quaternion q_bc0 = Quaternion(0.5, 0.5, -0.5, -0.5);
    Quaternion q_bc1 = Quaternion(0.5, 0.5, -0.5, -0.5);

	// 提取对应时间戳的相机位姿，计算此时的观测
	while (std::getline(fsCam, oneLine) && !oneLine.empty()) {
		std::vector<Eigen::Matrix<Scalar, 2, 1>> features_0;
		std::vector<Eigen::Matrix<Scalar, 2, 1>> features_1;    // p_c0c1 = [0.1, 0, 0]
		std::istringstream camData_0(oneLine);
		camData_0 >> timeStamp >> q_wc0.w() >> q_wc0.x() >> q_wc0.y() >> q_wc0.z() >> p_wc0.x() >> p_wc0.y() >> p_wc0.z();
        /* T_wb = T_wc * T_bc.inv */
        /* [R_wb  t_wb] = [R_wc  t_wc]  *  [R_bc.t  - R_bc.t * t_bc] = [ R_wc * R_bc.t  - R_wc * R_bc.t * t_bc + t_wc]
           [  0    1  ]   [  0     1 ]     [  0             1      ]   [      0                       1              ] */
        Quaternion q_wb = q_wc0 * q_bc0.inverse();
        Vector3 p_wb = - (q_wb * p_bc0) + p_wc0;
        /* T_wc = T_wb * T_bc */
        /* [R_wc  t_wc] = [R_wb  t_wb]  *  [R_bc  t_bc] = [ R_wb * R_bc  R_wb * t_bc + t_wb]
           [  0    1  ]   [  0     1 ]     [  0     1 ]   [      0                1        ] */
        q_wc1 = q_wb * q_bc1;
        p_wc1 = q_wb * p_bc1 + p_wb;

		// 将世界坐标系的 points 投影到归一化平面
		for (unsigned long i = 0; i < allPoints.size(); i++) {
			Vector3 pc_0 = q_wc0.inverse() * (allPoints[i] - p_wc0);
			Eigen::Matrix<Scalar, 2, 1> feature_0 = Eigen::Matrix<Scalar, 2, 1>(pc_0(0, 0) / pc_0(2, 0), pc_0(1, 0) / pc_0(2, 0));
			features_0.emplace_back(feature_0);

            Vector3 pc_1 = q_wc1.inverse() * (allPoints[i] - p_wc1);
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

void SavePose(const std::vector<fp64> &timeStamps,
              const std::vector<Quaternion> &q_wbs,
              const std::vector<Vector3> &p_wbs,
              const std::vector<Vector3> &v_wbs,
              const std::vector<Vector3> &bias_as,
              const std::vector<Vector3> &bias_gs,
              const std::string &savePath) {
    std::ofstream outFile;
    outFile.open(savePath + "estimated_poses.txt");
    if (!outFile) {
        std::cout << "CANNOT save results into " << savePath << "estimated_poses.txt" << std::endl;
        return;
    }
    for (uint32_t i = 0; i < timeStamps.size(); ++i) {
        outFile << timeStamps[i] << " "
                << q_wbs[i].w() << " " << q_wbs[i].x() << " " << q_wbs[i].y() << " " << q_wbs[i].z() << " "
                << p_wbs[i].transpose() << " "
                << v_wbs[i].transpose() << " "
                << bias_as[i].transpose() << " "
                << bias_gs[i].transpose() << " "
                << std::endl;
    }
    outFile.close();
}


int main(int argc, char **argv) {
    // 处理输入的配置参数路径和数据路径
    if (argc == 3) {
        simPath = argv[1];
        configPath = argv[2];
    }

    // 配置 std::cout 打印到指定文件
    // std::ofstream logFile("../test_log/20221030_test_msckf_backend_in_static_scene.txt");
    // std::streambuf *buf = std::cout.rdbuf(logFile.rdbuf());

    // 初始化配置 vio backend，并载入数据
    std::cout << "This is a vio backend with filter estimator." << std::endl;
    std::shared_ptr<Backend> backend(new Backend());
    backend->ConfigParams(configPath);
    uint32_t cnt = LoadIMUData(backend);
    LoadFeaturesData(backend);

    // 准备记录结果
    std::vector<fp64> timeStamps;
    std::vector<Quaternion> q_wbs;
    std::vector<Vector3> p_wbs;
    std::vector<Vector3> v_wbs;
    std::vector<Vector3> bias_as;
    std::vector<Vector3> bias_gs;
    fp64 lastTimeStamp = NAN;

    // 运行测试
    for (uint32_t i = 0; i < cnt; ++i) {
        std::cout << "\n --- \n";
        backend->RunOnce();
        ESKF_VIO_BACKEND::IMUFullState state;
        fp64 timeStamp;
        bool res = backend->PublishPropagateState(state, timeStamp);
        if (res == true) {
            // Vector3 pitch_roll_yaw = ESKF_VIO_BACKEND::Utility::QuaternionToEuler(state.q_wb);
            // std::cout << pitch_roll_yaw.x() << " " << pitch_roll_yaw.y() << " " << pitch_roll_yaw.z() << " ";
            std::cout << state.p_wb.transpose() << std::endl;
            if (std::isnan(state.p_wb.x())) {
                break;
            }

            if (lastTimeStamp == timeStamp) {
                break;
            }
            lastTimeStamp = timeStamp;

            timeStamps.emplace_back(timeStamp);
            q_wbs.emplace_back(state.q_wb);
            p_wbs.emplace_back(state.p_wb);
            v_wbs.emplace_back(state.v_wb);
            bias_as.emplace_back(state.bias_a);
            bias_gs.emplace_back(state.bias_g);
        } else {
            std::cout << "backend is not ready." << std::endl;
        }
    }
    // 保存运行结果
    SavePose(timeStamps, q_wbs, p_wbs, v_wbs, bias_as, bias_gs, savePath);
    return 0;
}