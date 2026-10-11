//=====================================================================================
// RobotServer.cpp
//
// B/S 架构中的后端：把运动学计算封装成 HTTP / WebSocket 服务。
//
//   * 静态资源：把 web/ 目录下的前端页面与 Three.js 提供给浏览器
//   * 命令通道：POST /api/command  —— 前端把用户操作发给后端，后端解算后返回新状态
//   * 状态通道：WebSocket /ws     —— 后端把最新状态推给所有已连接的前端
//
// 后端不包含任何渲染代码，渲染完全交给前端（Three.js / WebGL）。
//=====================================================================================

#include "RobotCore.h"
#include "RobotUtils.h"
#include "RobotScenario.h"
#include "Json.h"
#include "Console.h"

//需要 SetConsoleOutputCP 修正控制台编码，以及 CommandLineToArgvW 读取宽字符命令行。
//windows.h 会定义 min/max 宏，与标准库冲突，这里用完立刻取消。
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
//CommandLineToArgvW 声明在 shellapi.h（shell32.lib）；
//PathCombineW 声明在 shlwapi.h（shlwapi.lib）
#include <shellapi.h>
#include <shlwapi.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include "httplib.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>	//_wtoi
#include <fstream>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace RobotTest {
	bool runAll();
}

namespace {

	//=================================================================================
	// 服务状态
	//=================================================================================
	class RobotService {
	public:
		RobotService() {
			//---------- 摆两台机械臂：传送带两侧各一台，分别用完备解与旧特解 ----------
			const float beltX = scenario_.layout().beltX;
			const int left = addArmAt(0.0f, 0.0f, "左臂（完备解析解）", Robot::IKMode::Complete);
			const int right = addArmAt(2.0f * beltX, 0.0f, "右臂（旧特解）", Robot::IKMode::Legacy);
			manualArmId_ = (left >= 0) ? left : right;

			//初始目标：以点云包围盒给出的位姿作为第一个目标
			loadOBBTarget(400.0f, 400.0f, 400.0f, false);
			anchor_ = Eigen::Vector3f(0, 0, 0);
			recomputeAnchor();
			version_ = 1;
		}

		~RobotService() {
			stopSimulation();
		}

		//-----------------------------------------------------------------------------
		// 机械臂集合：机械臂是独立实体，可以随时加/删/挪位置
		//-----------------------------------------------------------------------------
		/// <summary>新增一台机械臂（世界坐标 x/y），返回它的编号</summary>
		int addArmAt(float x, float y, const std::string& name, Robot::IKMode mode, bool registerScenario = true) {
			std::unique_ptr<Robot> robot(new Robot(RobotUtils::createDefaultRobot()));
			robot->ID = nextArmId_++;
			robot->Name = name.empty() ? ("机械臂 #" + std::to_string(robot->ID)) : name;
			robot->setTrackSize(scenario_.trackSize());
			robot->setTrackEnabled(true);
			robot->setIKMode(mode);
			robot->BaseTransform = Eigen::Matrix4f::Identity();
			robot->BaseTransform(0, 3) = x;
			robot->BaseTransform(1, 3) = y;

			const int id = robot->ID;
			robotsById_[id] = robot.get();
			robots_.push_back(std::move(robot));
			manualTargets_[id] = robotsById_[id]->FK4();
			if (registerScenario) {
				scenario_.addArm(*robotsById_[id]);
				scenario_.setArmBase(id, x, y);		//按新位置预解算工位
			}
			version_++;
			return id;
		}

		/// <summary>移除一台机械臂</summary>
		bool removeArm(int id) {
			if (robotsById_.find(id) == robotsById_.end()) {
				return false;
			}
			scenario_.removeArm(id);
			robotsById_.erase(id);
			manualTargets_.erase(id);
			robots_.erase(std::remove_if(robots_.begin(), robots_.end(),
				[id](const std::unique_ptr<Robot>& robot) { return robot->ID == id; }), robots_.end());
			if (manualArmId_ == id) {
				manualArmId_ = robots_.empty() ? -1 : robots_.front()->ID;
			}
			version_++;
			return true;
		}

		Robot* findRobot(int id) {
			const auto it = robotsById_.find(id);
			return (it == robotsById_.end()) ? nullptr : it->second;
		}

		/// <summary>当前「手动操作」的那台臂；没有臂时返回 nullptr</summary>
		Robot* manualRobot() {
			Robot* robot = findRobot(manualArmId_);
			if (!robot && !robots_.empty()) {
				manualArmId_ = robots_.front()->ID;
				robot = robots_.front().get();
			}
			return robot;
		}

		/// <summary>当前选中机械臂的目标位姿（每台臂各存一份，切臂不会串）</summary>
		Eigen::Matrix4f& targetPose() {
			Robot* robot = manualRobot();
			return manualTargets_[robot ? robot->ID : -1];
		}

		/// <summary>包围盒/微调滑块的基准点：以当前选中臂的目标点为参考</summary>
		void recomputeAnchor() {
			anchor_ = Eigen::Vector3f(targetPose()(0, 3), targetPose()(1, 3), targetPose()(2, 3));
		}

		//-----------------------------------------------------------------------------
		// 场景仿真时钟
		//-----------------------------------------------------------------------------
		/// <summary>启动传送带场景的仿真线程（约 50Hz）</summary>
		void startSimulation() {
			if (simulationThread_.joinable()) {
				return;
			}
			simulationRunning_ = true;
			simulationThread_ = std::thread([this]() { simulationLoop(); });
		}

		/// <summary>停止仿真线程</summary>
		void stopSimulation() {
			simulationRunning_ = false;
			if (simulationThread_.joinable()) {
				simulationThread_.join();
			}
		}

		/// <summary>
		/// 处理一条前端指令，返回是否产生了状态变化
		/// </summary>
		bool handleCommand(const std::string& action, const json::Value& payload) {
			std::lock_guard<std::mutex> guard(mutex_);
			pauseScenarioForManual(action);
			if (action == "setTargetPose") {
				std::vector<double> values = payload.numbers("matrix");
				if (values.size() < 16) {
					return false;
				}
				std::vector<float> floats(values.begin(), values.end());
				Eigen::Matrix4f pose = RobotUtils::fromRowMajor(floats);
				return applyTarget(pose);
			}
			if (action == "setTargetPosition") {
				std::vector<double> values = payload.numbers("position");
				if (values.size() < 3) {
					return false;
				}
				Eigen::Vector3f position(static_cast<float>(values[0]),
					static_cast<float>(values[1]), static_cast<float>(values[2]));
				//保持当前姿态，只改变位置
				Eigen::Vector3f zAxis = targetPose().block<3, 1>(0, 2);
				Eigen::Vector3f xAxis = targetPose().block<3, 1>(0, 0);
				return applyTarget(RobotUtils::makePose(position, zAxis, xAxis));
			}
			if (action == "nudgePosition") {
				std::vector<double> values = payload.numbers("delta");
				if (values.size() < 3) {
					return false;
				}
				Eigen::Vector3f position(targetPose()(0, 3), targetPose()(1, 3), targetPose()(2, 3));
				position += Eigen::Vector3f(static_cast<float>(values[0]),
					static_cast<float>(values[1]), static_cast<float>(values[2]));
				Eigen::Vector3f zAxis = targetPose().block<3, 1>(0, 2);
				Eigen::Vector3f xAxis = targetPose().block<3, 1>(0, 0);
				return applyTarget(RobotUtils::makePose(position, zAxis, xAxis));
			}
			if (action == "setOrientation") {
				std::vector<double> values = payload.numbers("zAxis");
				if (values.size() < 2) {
					return false;
				}
				//滑块给出目标坐标系 Z 轴方向的 X/Y 分量，剩余分量自动补齐
				float zx = static_cast<float>(values[0]);
				float zy = static_cast<float>(values[1]);
				float zz = std::sqrt(std::max(0.0f, 1.0f - zx * zx - zy * zy));
				Eigen::Vector3f position(targetPose()(0, 3), targetPose()(1, 3), targetPose()(2, 3));
				Eigen::Vector3f zAxis(zx, zy, zz);
				Eigen::Vector3f xAxis = targetPose().block<3, 1>(0, 0);
				return applyTarget(RobotUtils::makePose(position, zAxis, xAxis));
			}
			if (action == "setJointAngles") {
				std::vector<double> values = payload.numbers("degrees");
				Robot* manual = manualRobot();
				if (!manual || values.size() < static_cast<size_t>(manual->JointCount)) {
					return false;
				}
				for (int i = 0; i < manual->JointCount; i++) {
					//用户角度 -> 内部弧度（与 DH 参数表的角度符号约定保持一致）
					manual->joints[i].updateH(RobotUtils::toRadian(static_cast<float>(-values[i])));
				}
				manual->refreshTrack();
				syncTargetFromRobot("已按关节角更新");
				return true;
			}
			if (action == "goHome") {
				Robot* manual = manualRobot();
				if (!manual) {
					return false;
				}
				manual->goHome();
				manual->refreshTrack();
				syncTargetFromRobot("已回到零位");
				return true;
			}
			if (action == "loadOBBTarget") {
				loadOBBTarget(400.0f, -400.0f, 400.0f, true);
				return true;
			}
			if (action == "randomOBBTarget") {
				//随机生成一片点云（随机位置 + 随机姿态），以包围盒位姿为目标
				randomOBBTarget();
				return true;
			}
			if (action == "setTargetVisible") {
				//一键显隐「目标点云 + 包围盒」
				showCloud_ = payload.flag("visible", true);
				showObb_ = showCloud_;
				message_ = showCloud_ ? "已显示目标点云与包围盒" : "已隐藏目标点云与包围盒";
				warning_ = false;
				version_++;
				return true;
			}
			if (action == "setIKMode") {
				const std::string mode = payload.str("mode", "complete");
				const Robot::IKMode target = (mode == "legacy") ? Robot::IKMode::Legacy : Robot::IKMode::Complete;
				const int arm = static_cast<int>(payload.num("arm", -1));
				Robot* robot = findRobot(arm);
				if (!robot) {
					//不带 arm（或编号不存在）就所有臂一起切
					for (const auto& entry : robotsById_) {
						entry.second->setIKMode(target);
					}
					message_ = (target == Robot::IKMode::Legacy)
						? "所有机械臂都切到「旧特解」"
						: "所有机械臂都切到「完备解析解」";
				}
				else {
					robot->setIKMode(target);
					message_ = robot->Name +
						((target == Robot::IKMode::Legacy) ? " → 旧特解" : " → 完备解析解");
				}
				warning_ = false;
				version_++;
				return true;
			}
			if (action == "toggleCloud") {
				showCloud_ = !showCloud_;
				message_ = showCloud_ ? "已显示点云" : "已隐藏点云";
				warning_ = false;
				return true;
			}
			if (action == "setTrajectoryVisible") {
				showTrajectory_ = payload.flag("visible", true);
				if (Robot* manual = manualRobot()) {
					manual->setTrackEnabled(true);
					if (showTrajectory_) {
						manual->refreshTrack();
					}
				}
				return true;
			}
			if (action == "setTrackSize") {
				//轨迹采样点数：带 arm 就只改那一台，不带就所有臂一起改
				const int size = static_cast<int>(payload.num("size", 24));
				const int arm = static_cast<int>(payload.num("arm", -2));
				if (arm == -2) {
					scenario_.setTrackSize(size);			//-1 = 全部（并记住默认值）
				}
				else {
					scenario_.setTrackSize(size, arm);
				}
				for (const auto& entry : robotsById_) {
					if (arm == -2 || arm < 0 || entry.first == arm) {
						entry.second->refreshTrack();
					}
				}
				version_++;
				return true;
			}
			if (action == "setRobotSpeed") {
				//机械臂速度系数：作用在码垛场景每段运动的关节角速度上
				const float speed = static_cast<float>(payload.num("speed", 1.0));
				const int arm = static_cast<int>(payload.num("arm", -1));
				scenario_.setArmSpeed(arm, speed);
				const Robot* robot = findRobot(arm);
				message_ = (robot ? robot->Name : std::string("所有机械臂")) + " 速度已设为 " +
					std::to_string(static_cast<int>(std::lround(speed * 100.0f))) + "%";
				warning_ = false;
				version_++;
				return true;
			}
			if (action == "setArmTrackingSpeed") {
				scenario_.setArmTrackingSpeed(static_cast<int>(payload.num("arm", -1)),
					static_cast<float>(payload.num("speed", 1.6)));
				version_++;
				return true;
			}
			//---------- 机械臂的增删、选择与位置 ----------
			if (action == "addArm") {
				const float x = static_cast<float>(payload.num("x", 1200.0));
				const float y = static_cast<float>(payload.num("y", 0.0));
				const std::string name = payload.str("name", "");
				const std::string mode = payload.str("mode", "complete");
				const int id = addArmAt(x, y, name,
					mode == "legacy" ? Robot::IKMode::Legacy : Robot::IKMode::Complete);
				scenarioPrepared_ = false;			//新臂需要重新预解算工位
				if (scenario_.running()) {
					scenario_.reset();
					scenario_.start();
				}
				message_ = "已添加机械臂 #" + std::to_string(id) + "（" +
					std::to_string(static_cast<int>(std::lround(x))) + ", " +
					std::to_string(static_cast<int>(std::lround(y))) + "）";
				warning_ = false;
				return true;
			}
			if (action == "removeArm") {
				const int arm = static_cast<int>(payload.num("arm", -1));
				const Robot* robot = findRobot(arm);
				if (!robot) {
					return false;
				}
				const std::string name = robot->Name;
				removeArm(arm);
				if (scenario_.running()) {
					scenario_.reset();
					scenario_.start();
				}
				message_ = "已移除 " + name;
				warning_ = false;
				return true;
			}
			if (action == "selectArm") {
				const int arm = static_cast<int>(payload.num("arm", -1));
				if (!findRobot(arm)) {
					return false;
				}
				manualArmId_ = arm;
				Robot* manual = manualRobot();
				if (manual) {
					scenario_.pause();				//切到手动操作就先把场景停下来
					recomputeAnchor();
					manual->refreshTrack();
					message_ = "手动操作已切换到 " + manual->Name;
					warning_ = false;
				}
				version_++;
				return true;
			}
			if (action == "setArmBase") {
				const int arm = static_cast<int>(payload.num("arm", -1));
				const float x = static_cast<float>(payload.num("x", 0.0));
				const float y = static_cast<float>(payload.num("y", 0.0));
				const Robot* robot = findRobot(arm);
				if (!robot) {
					return false;
				}
				const bool ok = scenario_.setArmBase(arm, x, y);
				message_ = ok
					? (robot->Name + " 基座已移动到 (" +
						std::to_string(static_cast<int>(std::lround(x))) + ", " +
						std::to_string(static_cast<int>(std::lround(y))) + ")")
					: (robot->Name + " 在新位置解不出工位，已停止作业");
				warning_ = !ok;
				version_++;
				return true;
			}
			if (action == "setArmPickY") {
				//设置这台臂自己的抓取工位（传送带上的 Y）：货物路过这里时它才会申请抓取
				const int arm = static_cast<int>(payload.num("arm", -1));
				const float y = static_cast<float>(payload.num("y", 0.0));
				const Robot* robot = findRobot(arm);
				if (!robot) {
					return false;
				}
				const bool ok = scenario_.setArmPickY(arm, y);
				const float actual = scenario_.armById(arm) ? scenario_.armById(arm)->pickY : y;
				message_ = ok
					? (robot->Name + " 抓取工位已设为 y = " +
						std::to_string(static_cast<int>(std::lround(actual))) + " mm")
					: (robot->Name + " 在新工位解不出位姿，请换个位置");
				warning_ = !ok;
				version_++;
				return true;
			}
			if (action == "setArmName") {
				Robot* robot = findRobot(static_cast<int>(payload.num("arm", -1)));
				if (!robot) {
					return false;
				}
				robot->Name = payload.str("name", robot->Name);
				if (robotscenario::ConveyorScenario::Arm* arm = scenario_.armById(robot->ID)) {
					arm->name = robot->Name;
				}
				version_++;
				return true;
			}
			//---------- 传送带取件场景 ----------
			if (action == "startScenario") {
				if (!scenarioPrepared_) {
					//首次启动：预解算工位、清空箱子，并把两台臂摆到待命位
					scenario_.reset();
					scenarioPrepared_ = true;
				}
				scenario_.start();
				syncStatusFromScenario();
				version_++;
				return true;
			}
			if (action == "pauseScenario") {
				scenario_.pause();
				syncStatusFromScenario();
				version_++;
				return true;
			}
			if (action == "resetScenario") {
				scenario_.reset();
				scenarioPrepared_ = true;
				syncStatusFromScenario();
				version_++;
				return true;
			}
			if (action == "setBeltSpeed") {
				scenario_.setBeltSpeed(static_cast<float>(payload.num("speed", 160.0)));
				version_++;
				return true;
			}
			if (action == "setSpawnInterval") {
				scenario_.setSpawnRange(static_cast<float>(payload.num("min", 2.2)),
					static_cast<float>(payload.num("max", 4.0)));
				version_++;
				return true;
			}
			if (action == "setAutoClear") {
				scenario_.setAutoClear(payload.flag("enabled", true));
				version_++;
				return true;
			}
			if (action == "setDynamicPick") {
				//true = 抓取移动中的货物（不停机，全程跟踪）；false = 货物停稳后再抓
				scenario_.setDynamicPick(payload.flag("enabled", true));
				message_ = scenario_.dynamicPick()
					? "已切换为动态抓取：货物不停，机械臂预测并跟踪"
					: "已切换为定点抓取：货物停在挡停线上";
				warning_ = false;
				version_++;
				return true;
			}
			if (action == "setTrackingSpeed") {
				scenario_.setTrackingSpeed(static_cast<float>(payload.num("speed", 1.6)));
				version_++;
				return true;
			}
			//未知指令：不改状态
			return false;
		}

		/// <summary>当前状态版本号（用于前端推送判断）</summary>
		unsigned long long currentVersion() {
			std::lock_guard<std::mutex> guard(mutex_);
			return version_;
		}

		/// <summary>把当前状态序列化成前端需要的 JSON</summary>
		std::string stateJson() {
			std::lock_guard<std::mutex> guard(mutex_);

			Robot* manual = manualRobot();
			if (!manual) {
				return "{}";
			}
			Eigen::Matrix4f tcpFrame = manual->FK4();
			Eigen::Vector4f tcpPoint = tcpFrame * manual->BasePosition;
			Eigen::Vector3f tcpPosition(tcpPoint(0), tcpPoint(1), tcpPoint(2));
			Eigen::Vector3f targetPosition(targetPose()(0, 3), targetPose()(1, 3), targetPose()(2, 3));

			json::Writer writer;
			writer.beginObject();
			writer.key("type"); writer.value("state");
			writer.key("version"); writer.value(static_cast<double>(version_));

			//---------- 机械臂模型（前端据此绘制连杆与关节） ----------
			writer.key("model");
			writer.beginObject();
			writer.key("jointCount"); writer.value(static_cast<double>(manual->JointCount));
			writer.key("ikMode");
			writer.value(manual->ikMode() == Robot::IKMode::Legacy ? "legacy" : "complete");
			writer.key("base"); writer.value(std::vector<double>{
				manual->BasePosition(0), manual->BasePosition(1), manual->BasePosition(2) });
			writer.key("reachMax"); writer.value(static_cast<double>(manual->getReachSpaceMaxLength()));
			writer.key("reachMin"); writer.value(static_cast<double>(manual->getReachSpaceMinLength()));

			//关节角（用户角度制）
			writer.key("jointDegrees");
			writer.beginArray();
			for (int i = 0; i < manual->JointCount; i++) {
				writer.value(static_cast<double>(RobotUtils::toDegree(-manual->joints[i].theta)));
			}
			writer.endArray();

			//关节原点与关节坐标系姿态
			writer.key("joints");
			writer.beginArray();
			//基座
			writer.beginObject();
			writer.key("position"); writer.value(std::vector<double>{
				manual->BasePosition(0), manual->BasePosition(1), manual->BasePosition(2) });
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(Eigen::Matrix4f::Identity()));
			writer.key("showAxes"); writer.value(false);
			writer.endObject();
			for (int i = 0; i < manual->JointCount; i++) {
				Eigen::Matrix4f jointFrame = manual->getJointH(i + 1);
				Eigen::Vector4f jointPoint = jointFrame * manual->BasePosition;
				writer.beginObject();
				writer.key("position"); writer.value(std::vector<double>{
					jointPoint(0), jointPoint(1), jointPoint(2) });
				writer.key("matrix"); writer.value(RobotUtils::toRowMajor(jointFrame));
				//只在轴 4、轴 5 处显示坐标系，避免画面过于杂乱
				writer.key("showAxes"); writer.value(i == 3 || i == 4);
				writer.endObject();
			}
			//工具末端
			writer.beginObject();
			writer.key("position"); writer.value(std::vector<double>{
				tcpPoint(0), tcpPoint(1), tcpPoint(2) });
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(tcpFrame));
			writer.key("showAxes"); writer.value(true);
			writer.endObject();
			writer.endArray();

			writer.key("linkColors");
			writer.beginArray();
			for (int i = 0; i < manual->JointCount; i++) {
				float hue = static_cast<float>(i) / static_cast<float>(manual->JointCount);
				writer.value(std::vector<double>{ 0.35 + 0.65 * hue, 0.95 - 0.45 * hue, 1.0 - 0.55 * hue });
			}
			writer.endArray();
			writer.endObject();

			//---------- 目标位姿与误差 ----------
			writer.key("target");
			writer.beginObject();
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(targetPose()));
			writer.key("position"); writer.value(std::vector<double>{
				targetPosition(0), targetPosition(1), targetPosition(2) });
			writer.endObject();

			writer.key("tcp");
			writer.beginObject();
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(tcpFrame));
			writer.key("position"); writer.value(std::vector<double>{
				tcpPosition(0), tcpPosition(1), tcpPosition(2) });
			writer.key("error"); writer.value(static_cast<double>((targetPosition - tcpPosition).norm()));
			writer.endObject();

			//---------- 轨迹 ----------
			writer.key("trajectory");
			writer.beginObject();
			writer.key("visible"); writer.value(showTrajectory_);
			writer.key("size"); writer.value(static_cast<double>(manual->getTrackSize()));
			writer.key("points");
			writer.beginArray();
			if (showTrajectory_) {
				for (const Eigen::Vector3f& point : manual->getTrackPoints()) {
					writer.value(std::vector<double>{ point(0), point(1), point(2) });
				}
			}
			writer.endArray();
			writer.endObject();

			//---------- 点云与包围盒 ----------
			writer.key("cloud");
			writer.beginObject();
			writer.key("visible"); writer.value(showCloud_);
			writer.key("count"); writer.value(static_cast<double>(showCloud_ ? cloud_.size() : 0));
			//点云数据量大且很少变化，用一个版本号让前端只在真正变化时重建几何
			writer.key("revision"); writer.value(static_cast<double>(cloudRevision_));
			//点云数据量较大，先用紧凑格式拼好再作为整段值写入，避免逐点调用 Writer
			writer.key("points"); writer.rawValue(buildCloudJson());
			writer.endObject();

			//---------- 传送带取件场景 ----------
			writer.key("scenario");
			writeScenario(writer);

			//---------- 两台机械臂（对比用）----------
			writer.key("arms");
			writer.beginArray();
			for (const robotscenario::ConveyorScenario::Arm& arm : scenario_.arms()) {
				writeArm(writer, arm);
			}
			writer.endArray();

			writer.key("obb");
			writer.beginObject();
			writer.key("valid"); writer.value(obb_.valid);
			writer.key("visible"); writer.value(showObb_);
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(obb_.pose));
			writer.key("center"); writer.value(std::vector<double>{
				obb_.pose(0, 3), obb_.pose(1, 3), obb_.pose(2, 3) });
			writer.key("extents"); writer.value(std::vector<double>{
				obb_.extents(0), obb_.extents(1), obb_.extents(2) });
			writer.endObject();

			//---------- 提示信息 ----------
			writer.key("status");
			writer.beginObject();
			writer.key("message"); writer.value(message_);
			writer.key("warning"); writer.value(warning_);
			writer.endObject();

			writer.endObject();
			return writer.str();
		}

	private:
		std::vector<std::unique_ptr<Robot>> robots_;			//机械臂实体（可以随时增删）
		std::map<int, Robot*> robotsById_;						//编号 → 机械臂
		std::map<int, Eigen::Matrix4f> manualTargets_;			//每台臂各自的手动目标
		int nextArmId_ = 1;
		int manualArmId_ = -1;									//当前手动操作哪台臂
		robotscenario::ConveyorScenario scenario_;				//传送带取件场景（带任意台臂）
		bool scenarioPrepared_ = false;					//工位是否已预解算
		std::mutex mutex_;
		Eigen::Vector3f anchor_ = Eigen::Vector3f::Zero();		//位置微调的基准点
		RobotUtils::OBB obb_;
		std::vector<RobotUtils::Point3> cloud_;					//当前点云
		int cloudRevision_ = 1;									//点云版本号（前端据此决定是否重建几何）
		std::string message_ = "拖动滑块或直接输入数值以控制末端位姿";
		bool warning_ = false;
		bool showCloud_ = true;
		bool showObb_ = true;									//包围盒显隐（与点云一起由「隐藏目标」控制）
		bool showTrajectory_ = true;
		unsigned long long version_ = 0;
		std::mt19937 rng_{ 20240607u };							//随机目标用的随机数
		std::atomic<bool> simulationRunning_{ false };
		std::thread simulationThread_;

		//-----------------------------------------------------------------------------
		// 场景相关
		//-----------------------------------------------------------------------------
		/// <summary>仿真循环：固定步长推进场景，有运动就推送新状态</summary>
		void simulationLoop() {
			auto last = std::chrono::steady_clock::now();
			while (simulationRunning_) {
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
				const auto now = std::chrono::steady_clock::now();
				float dt = std::chrono::duration<float>(now - last).count();
				last = now;
				if (dt > 0.25f) {
					dt = 0.25f;		//系统卡顿后不要让货物瞬移
				}

				std::lock_guard<std::mutex> guard(mutex_);
				if (!scenario_.running()) {
					continue;
				}
				scenario_.tick(dt);
				//场景运行时「目标」就是当前选中臂的工具位姿，误差读数自然为 0
				if (Robot* manual = manualRobot()) {
					targetPose() = manual->FK4();
				}
				syncStatusFromScenario();
				version_++;			//场景一直在动，每帧都推给前端
			}
		}

		/// <summary>把场景的提示信息同步到状态栏</summary>
		void syncStatusFromScenario() {
			message_ = scenario_.message();
			warning_ = scenario_.warning();
			//两台臂谁报警就把谁的提示顶到状态栏
			for (const robotscenario::ConveyorScenario::Arm& arm : scenario_.arms()) {
				if (arm.warning) {
					message_ = arm.name + "：" + arm.message;
					warning_ = true;
					break;
				}
			}
		}

		/// <summary>写一台机械臂的状态（模型 + 末端 + 场景统计 + 本周期轨迹）</summary>
		void writeArm(json::Writer& writer, const robotscenario::ConveyorScenario::Arm& arm) {
			const Robot& robot = *arm.robot;
			const Eigen::Matrix4f tcpFrame = robot.FK4();
			const Eigen::Vector4f tcpPoint = tcpFrame * robot.BasePosition;

			writer.beginObject();
			writer.key("id"); writer.value(static_cast<double>(arm.id));
			writer.key("name"); writer.value(arm.name);
			writer.key("ikMode");
			writer.value(robot.ikMode() == Robot::IKMode::Legacy ? "legacy" : "complete");
			writer.key("side"); writer.value(static_cast<double>(arm.side));
			writer.key("base"); writer.value(std::vector<double>{
				static_cast<double>(arm.baseX), static_cast<double>(arm.baseY), 0.0 });
			//每台臂独立的运行参数
			writer.key("speedFactor"); writer.value(static_cast<double>(arm.speedFactor));
			writer.key("trackingSpeed"); writer.value(static_cast<double>(arm.trackingSpeed));
			writer.key("trackSize"); writer.value(static_cast<double>(robot.getTrackSize()));
			writer.key("dynamicPick"); writer.value(arm.dynamicPick);
			writer.key("selected"); writer.value(arm.id == manualArmId_);
			//抓取工位与货物锁
			writer.key("pickY"); writer.value(static_cast<double>(arm.pickY));
			writer.key("claimed"); writer.value(static_cast<double>(arm.claimed));
			writer.key("contended"); writer.value(static_cast<double>(arm.contended));
			writer.key("lockedBox");
			writer.beginArray();
			for (const robotscenario::Box& box : scenario_.boxes()) {
				if (box.state == robotscenario::Box::State::Belt && box.claimedBy == arm.id) {
					writer.value(static_cast<double>(box.id));
				}
			}
			writer.endArray();

			//---------- 机械臂模型 ----------
			writer.key("model");
			writer.beginObject();
			writer.key("jointCount"); writer.value(static_cast<double>(robot.JointCount));
			writer.key("base"); writer.value(std::vector<double>{
				robot.BaseTransform(0, 3), robot.BaseTransform(1, 3), robot.BaseTransform(2, 3) });
			writer.key("jointDegrees");
			writer.beginArray();
			for (int i = 0; i < robot.JointCount; i++) {
				writer.value(static_cast<double>(RobotUtils::toDegree(-robot.joints[i].theta)));
			}
			writer.endArray();
			writer.key("joints");
			writer.beginArray();
			//基座
			writer.beginObject();
			writer.key("position"); writer.value(std::vector<double>{
				robot.BaseTransform(0, 3), robot.BaseTransform(1, 3), robot.BaseTransform(2, 3) });
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(robot.BaseTransform));
			writer.key("showAxes"); writer.value(false);
			writer.endObject();
			for (int i = 0; i < robot.JointCount; i++) {
				const Eigen::Matrix4f jointFrame = robot.getJointH(i + 1);
				const Eigen::Vector4f jointPoint = jointFrame * robot.BasePosition;
				writer.beginObject();
				writer.key("position"); writer.value(std::vector<double>{
					jointPoint(0), jointPoint(1), jointPoint(2) });
				writer.key("matrix"); writer.value(RobotUtils::toRowMajor(jointFrame));
				writer.key("showAxes"); writer.value(i == 3 || i == 4);
				writer.endObject();
			}
			//工具末端
			writer.beginObject();
			writer.key("position"); writer.value(std::vector<double>{
				tcpPoint(0), tcpPoint(1), tcpPoint(2) });
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(tcpFrame));
			writer.key("showAxes"); writer.value(true);
			writer.endObject();
			writer.endArray();
			writer.key("linkColors");
			writer.beginArray();
			for (int i = 0; i < robot.JointCount; i++) {
				const float hue = static_cast<float>(i) / static_cast<float>(robot.JointCount);
				writer.value(std::vector<double>{ 0.35 + 0.65 * hue, 0.95 - 0.45 * hue, 1.0 - 0.55 * hue });
			}
			writer.endArray();
			writer.endObject();

			//---------- 末端 ----------
			writer.key("tcp");
			writer.beginObject();
			writer.key("matrix"); writer.value(RobotUtils::toRowMajor(tcpFrame));
			writer.key("position"); writer.value(std::vector<double>{
				tcpPoint(0), tcpPoint(1), tcpPoint(2) });
			writer.endObject();

			//---------- 场景统计 ----------
			writer.key("phase"); writer.value(robotscenario::phaseName(arm.phase));
			writer.key("picked"); writer.value(static_cast<double>(arm.picked));
			writer.key("stacked"); writer.value(static_cast<double>(arm.stacked));
			writer.key("missed"); writer.value(static_cast<double>(arm.missed));
			writer.key("cleared"); writer.value(static_cast<double>(arm.cleared));
			writer.key("gripper"); writer.value(static_cast<double>(arm.gripperOpening));
			writer.key("lastPickOffsetX"); writer.value(static_cast<double>(arm.lastPickOffsetX));
			writer.key("trackError"); writer.value(static_cast<double>(arm.trackError));
			writer.key("message"); writer.value(arm.message);
			writer.key("warning"); writer.value(arm.warning);
			writer.key("holding");
			writer.beginArray();
			for (const robotscenario::Box& box : scenario_.boxes()) {
				if (box.state == robotscenario::Box::State::Held && box.claimedBy == arm.id) {
					writer.value(static_cast<double>(box.id));
				}
			}
			writer.endArray();
			writer.key("slots");
			writer.beginArray();
			for (int i = 0; i < scenario_.layout().slotCount; i++) {
				writer.value(std::vector<double>{
					static_cast<double>(arm.slotX[i]), static_cast<double>(arm.slotY[i]) });
			}
			writer.endArray();

			//---------- 轨迹规划方式与规划结果 ----------
			//旧结算模式的臂由 Robot::getTrack 规划（场景逐点执行），其余臂用场景内置插值
			writer.key("planMode"); writer.value(arm.usingTrackPlan ? "getTrack" : "builtin");
			writer.key("planSegments"); writer.value(static_cast<double>(arm.plannedSegments));
			writer.key("planTotal"); writer.value(static_cast<double>(arm.planTotal));
			writer.key("planned");
			writer.beginArray();
			for (const robotscenario::Vec3& point : arm.planned) {
				writer.value(std::vector<double>{ point(0), point(1), point(2) });
			}
			writer.endArray();

			//---------- 本周期末端轨迹（前端画运动轨迹用）----------
			//顺带算出轨迹总长与相邻点最大跳变，作为「运动品质」的量化指标
			float pathLength = 0.0f;
			float maxJump = 0.0f;
			for (size_t i = 1; i < arm.path.size(); i++) {
				const float step = (arm.path[i] - arm.path[i - 1]).norm();
				pathLength += step;
				maxJump = std::max(maxJump, step);
			}
			writer.key("pathRevision"); writer.value(static_cast<double>(arm.pathRevision));
			writer.key("pathLength"); writer.value(static_cast<double>(pathLength));
			writer.key("pathMaxJump"); writer.value(static_cast<double>(maxJump));
			writer.key("path");
			writer.beginArray();
			for (const robotscenario::Vec3& point : arm.path) {
				writer.value(std::vector<double>{ point(0), point(1), point(2) });
			}
			writer.endArray();

			writer.endObject();
		}

		/// <summary>把传送带场景写进状态帧（前端只负责画，不做任何计算）</summary>
		void writeScenario(json::Writer& writer) {
			const robotscenario::Layout& layout = scenario_.layout();
			writer.beginObject();
			writer.key("running"); writer.value(scenario_.running());
			writer.key("beltSpeed"); writer.value(static_cast<double>(scenario_.beltSpeed()));
			writer.key("spawnMin"); writer.value(static_cast<double>(scenario_.spawnMin()));
			writer.key("spawnMax"); writer.value(static_cast<double>(scenario_.spawnMax()));
			writer.key("autoClear"); writer.value(scenario_.autoClear());
			writer.key("dynamicPick"); writer.value(scenario_.dynamicPick());
			writer.key("defaultTrackSize"); writer.value(static_cast<double>(scenario_.trackSize()));
			writer.key("manualArm"); writer.value(static_cast<double>(manualArmId_));
			writer.key("onBelt"); writer.value(static_cast<double>(scenario_.onBeltCount()));
			writer.key("capacity"); writer.value(static_cast<double>(scenario_.capacity()));
			writer.key("armCount"); writer.value(static_cast<double>(scenario_.armCount()));
			writer.key("message"); writer.value(scenario_.message());
			writer.key("warning"); writer.value(scenario_.warning());

			//布局：前端据此绘制传送带与收集台，避免两边各写一份常量
			writer.key("layout");
			writer.beginObject();
			writer.key("beltX"); writer.value(static_cast<double>(layout.beltX));
			writer.key("beltHalfWidth"); writer.value(static_cast<double>(layout.beltHalfWidth));
			writer.key("beltTopZ"); writer.value(static_cast<double>(layout.beltTopZ));
			writer.key("beltBeginY"); writer.value(static_cast<double>(layout.beltBeginY));
			writer.key("beltEndY"); writer.value(static_cast<double>(layout.beltEndY));
			writer.key("pickY"); writer.value(static_cast<double>(layout.pickY));
			writer.key("boxHeight"); writer.value(static_cast<double>(layout.boxHeight));
			writer.key("palletTopZ"); writer.value(static_cast<double>(layout.palletTopZ));
			writer.key("layerHeight"); writer.value(static_cast<double>(layout.layerHeight));
			writer.key("layers"); writer.value(static_cast<double>(layout.layers));
			writer.key("slotCount"); writer.value(static_cast<double>(layout.slotCount));
			writer.endObject();

			//传送带上的箱子、被夹持的箱子与已码放的箱子
			writer.key("boxes");
			writer.beginArray();
			for (const robotscenario::Box& box : scenario_.boxes()) {
				writer.beginObject();
				writer.key("id"); writer.value(static_cast<double>(box.id));
				writer.key("p"); writer.value(std::vector<double>{
					box.position.x(), box.position.y(), box.position.z() });
				writer.key("s"); writer.value(std::vector<double>{
					box.size.x(), box.size.y(), box.size.z() });
				writer.key("yaw"); writer.value(static_cast<double>(box.yaw));
				writer.key("c"); writer.value(std::vector<double>{
					box.color.x(), box.color.y(), box.color.z() });
				writer.key("state");
				if (box.state == robotscenario::Box::State::Held) {
					writer.value("held");
				}
				else if (box.state == robotscenario::Box::State::Stacked) {
					writer.value("stacked");
				}
				else {
					writer.value("belt");
				}
				writer.endObject();
			}
			writer.endArray();
			writer.endObject();
		}

		/// <summary>手动指令会接管机械臂，因此自动暂停场景</summary>
		void pauseScenarioForManual(const std::string& action) {
			static const std::set<std::string> manualActions = {
				"setTargetPose", "setTargetPosition", "nudgePosition",
				"setOrientation", "setJointAngles", "goHome", "loadOBBTarget"
			};
			if (manualActions.find(action) == manualActions.end()) {
				return;
			}
			if (scenario_.running()) {
				scenario_.pause();
				message_ = "手动操作已接管，传送带场景暂停";
			}
		}

		/// <summary>把当前机械臂位姿同步为目标位姿（用于手动改关节角后）</summary>
		void syncTargetFromRobot(const std::string& note) {
			Robot* manual = manualRobot();
			if (!manual) {
				return;
			}
			targetPose() = manual->FK4();
			message_ = note;
			warning_ = false;
			version_++;
		}

		/// <summary>应用目标位姿：先修正朝向，再做逆解</summary>
		bool applyTarget(Eigen::Matrix4f pose) {
			Robot* manual = manualRobot();
			if (!manual) {
				return false;
			}
			//我们希望机械臂的末端从机械臂本体朝向目标，而不是越过它再折返回来
			RobotUtils::flipZAxisIfFacingRobot(pose);
			//目标点始终记录用户的输入：不可达时目标框仍然停在用户指定的位置，便于对照
			targetPose() = pose;
			warning_ = false;
			message_.clear();

			bool solved = false;
			try {
				solved = manual->IK(targetPose(), showTrajectory_);
			}
			catch (const char* error) {
				warning_ = true;
				message_ = std::string("逆解异常：") + error;
			}
			catch (...) {
				warning_ = true;
				message_ = "逆解异常：目标位姿超出可达范围";
			}

			//逆解无解时 IK 不会写回关节角，机械臂停在原位等待下一个目标；
			//这里只给出提示，不影响后续任何一次逆解（目标点与提示都变了，需要推送）
			if (!solved) {
				warning_ = true;
				if (message_.empty()) {
					message_ = "目标超出可达范围，逆解无解，机械臂保持原位";
				}
				version_++;
				return false;
			}
			recomputeAnchor();
			version_++;
			return true;
		}

		/// <summary>生成点云、计算包围盒，并以包围盒位姿作为目标</summary>
		void loadOBBTarget(float x, float y, float z, bool resetRobot) {
			if (Robot* manual = manualRobot()) {
				if (resetRobot) {
					manual->goHome();
				}
			}
			//生成测试点云并做一次已知位姿的变换，再从变换后的点云求包围盒
			std::vector<RobotUtils::Point3> cloud = RobotUtils::makeTestCloud();
			Eigen::Matrix4f transform = RobotUtils::makePoseFromEuler(
				static_cast<float>(M_PI) / 4, static_cast<float>(M_PI) / 3, 0, x, y, z);
			cloud_ = RobotUtils::transformCloud(cloud, transform);
			obb_ = RobotUtils::computeOBB(cloud_);
			cloudRevision_++;
			showCloud_ = true;
			if (obb_.valid) {
				applyTarget(obb_.pose);
				message_ = "已载入点云包围盒目标";
			}
			else {
				warning_ = true;
				message_ = "点云包围盒计算失败";
			}
			version_++;
		}

		/// <summary>
		/// 随机生成一片点云并以其包围盒位姿作为目标：
		/// 位置在可达环带内随机、姿态用随机欧拉角，解不出来就换一组重试。
		/// </summary>
		void randomOBBTarget() {
			std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * static_cast<float>(M_PI));
			std::uniform_real_distribution<float> radiusDist(300.0f, 620.0f);
			std::uniform_real_distribution<float> heightDist(120.0f, 620.0f);
			std::uniform_real_distribution<float> tiltDist(-1.0f, 1.0f);
			std::uniform_real_distribution<float> spinDist(0.0f, 2.0f * static_cast<float>(M_PI));

			for (int attempt = 0; attempt < 12; attempt++) {
				const float angle = angleDist(rng_);
				const float radius = radiusDist(rng_);
				const float x = radius * std::cos(angle);
				const float y = radius * std::sin(angle);
				const float z = heightDist(rng_);
				const float roll = angleDist(rng_);
				const float pitch = tiltDist(rng_) * static_cast<float>(M_PI) / 3.0f;
				const float yaw = spinDist(rng_);

				std::vector<RobotUtils::Point3> cloud = RobotUtils::makeTestCloud();
				Eigen::Matrix4f transform = RobotUtils::makePoseFromEuler(roll, pitch, yaw, x, y, z);
				std::vector<RobotUtils::Point3> transformed = RobotUtils::transformCloud(cloud, transform);
				RobotUtils::OBB box = RobotUtils::computeOBB(transformed);
				if (!box.valid) {
					continue;
				}

				//先接受这批点云，再尝试解算；解算失败就换一组
				cloud_ = transformed;
				obb_ = box;
				if (applyTarget(box.pose)) {
					cloudRevision_++;
					showCloud_ = true;
					showObb_ = true;
					message_ = "已生成随机点云目标（位置随机、姿态随机）";
					version_++;
					return;
				}
			}
			warning_ = true;
			message_ = "随机目标连续 12 次都不可达，请再点一次";
			version_++;
		}

		/// <summary>拼接点云 JSON（数值保留两位小数以减小传输量）</summary>
		std::string buildCloudJson() {
			if (!showCloud_ || cloud_.empty()) {
				return "[]";
			}
			std::string out;
			out.reserve(cloud_.size() * 26 + 4);
			out += '[';
			char item[64];
			for (size_t i = 0; i < cloud_.size(); i++) {
				if (i) {
					out += ',';
				}
				std::snprintf(item, sizeof(item), "[%.2f,%.2f,%.2f]",
					cloud_[i].x, cloud_[i].y, cloud_[i].z);
				out += item;
			}
			out += ']';
			return out;
		}
	};

	//=================================================================================
	// 路径工具（Server 构造函数会用到，因此先声明）
	//=================================================================================
	std::wstring utf8ToWide(const std::string& text);
	bool directoryExists(const std::string& path);
	bool fileExists(const std::string& path);
	bool canServeAsWebRoot(const std::string& path);
	std::string resolveWebRoot(const std::string& requested);
	std::string absolutePath(const std::string& path);

	//=================================================================================
	// HTTP / WebSocket 服务器
	//=================================================================================
	class Server {
	public:
		Server(RobotService& service, const std::string& webRoot, int port)
			: service_(service), webRoot_(webRoot), port_(port) {
			//---------- REST 接口 ----------
			server_.Get("/api/state", [this](const httplib::Request&, httplib::Response& res) {
				res.set_content(service_.stateJson(), "application/json; charset=utf-8");
			});

			server_.Post("/api/command", [this](const httplib::Request& req, httplib::Response& res) {
				json::Value body = json::parse(req.body);
				std::string action = body.str("action");
				if (action.empty()) {
					res.status = 400;
					res.set_content("{\"ok\":false,\"error\":\"missing action\"}",
						"application/json; charset=utf-8");
					return;
				}
				service_.handleCommand(action, body);
				res.set_content(service_.stateJson(), "application/json; charset=utf-8");
			});

			server_.Get("/api/health", [](const httplib::Request&, httplib::Response& res) {
				res.set_content("{\"ok\":true}", "application/json; charset=utf-8");
			});

			//---------- 静态资源 ----------
			//把 --web 规范化：依次尝试原样 / 相对工作目录 / 相对 exe 目录，
			//并用 httplib 的拼接方式实际验证能取到 index.html
			resolvedWebRoot_ = resolveWebRoot(webRoot_);
			if (!canServeAsWebRoot(resolvedWebRoot_)) {
				console::writeErrorFormat("警告：静态目录不可用：%s", resolvedWebRoot_.c_str());
				console::writeError("      前端页面将返回 404，请用 --web 指定包含 index.html 的正确目录。");
				server_.set_mount_point("/", resolvedWebRoot_.c_str());
			}
			else {
				//把静态目录转成绝对路径并切换工作目录，然后以相对路径 "." 挂载。
				//
				//原因：httplib 用 stat()/_access_s() 判断路径，而这两个函数在 Windows 上
				//按 ANSI 代码页解释路径。本项目的目录名含中文，传入 UTF-8 字节时它们会
				//判定失败，导致 set_mount_point 静默不生效、整站返回 404。
				//切换工作目录后 httplib 只需要处理 "web/index.html" 这类纯 ASCII 相对路径，
				//中文只经过 Windows 宽字符 API（SetCurrentDirectoryW），不受代码页影响。
				std::string absolute = absolutePath(resolvedWebRoot_);
				if (SetCurrentDirectoryW(utf8ToWide(absolute).c_str())) {
					resolvedWebRoot_ = ".";
					filesystemNotice_ = "（已切换工作目录到静态目录）";
				}
				server_.set_mount_point("/", resolvedWebRoot_.c_str());
			}

			server_.set_error_handler([](const httplib::Request&, httplib::Response& res) {
				if (res.status == 404) {
					res.set_content("404 Not Found", "text/plain; charset=utf-8");
				}
			});

			//---------- 状态推送（Server-Sent Events） ----------
			//每个连接独占一个工作线程：状态版本变化就推一帧，否则发心跳。
			//SSE 只走普通 HTTP，不需要 WebSocket / OpenSSL，浏览器 EventSource 会自动重连。
			server_.Get("/api/stream", [this](const httplib::Request&, httplib::Response& res) {
				res.set_header("Cache-Control", "no-cache");
				res.set_header("X-Accel-Buffering", "no");
				res.set_chunked_content_provider("text/event-stream",
					[this](size_t /*offset*/, httplib::DataSink& sink) {
						//逐条推送，客户端断开时 write 会失败，从而结束循环
						auto sendEvent = [&sink](const std::string& payload) {
							std::string frame = "data: " + payload + "\n\n";
							return sink.write(frame.c_str(), frame.size());
						};

						if (!sendEvent(service_.stateJson())) {
							return false;
						}
						unsigned long long lastSent = service_.currentVersion();

						while (true) {
							std::this_thread::sleep_for(std::chrono::milliseconds(30));
							unsigned long long current = service_.currentVersion();
							if (current != lastSent) {
								if (!sendEvent(service_.stateJson())) {
									return false;
								}
								lastSent = current;
							}
							else if (!sendEvent("{\"type\":\"ping\"}")) {
								return false;
							}
						}
					});
			});
		}

		bool listen() {
			return server_.listen("127.0.0.1", port_);
		}

		int port() const { return port_; }
		const std::string& mountedWebRoot() const { return resolvedWebRoot_; }
		const std::string& filesystemNotice() const { return filesystemNotice_; }

	private:
		RobotService& service_;
		std::string webRoot_;
		std::string resolvedWebRoot_;	//规范化后的静态目录（实际挂载的路径）
		std::string filesystemNotice_;	//静态目录处理的补充说明
		int port_ = 0;
		httplib::Server server_;
	};

	/// <summary>宽字符转 UTF-8</summary>
	std::string wideToUtf8(const std::wstring& text) {
		if (text.empty()) {
			return std::string();
		}
		int length = WideCharToMultiByte(CP_UTF8, 0, text.c_str(),
			static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
		if (length <= 0) {
			return std::string();
		}
		std::string utf8(static_cast<size_t>(length), '\0');
		WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()),
			&utf8[0], length, nullptr, nullptr);
		return utf8;
	}

	/// <summary>UTF-8 转宽字符</summary>
	std::wstring utf8ToWide(const std::string& text) {
		if (text.empty()) {
			return std::wstring();
		}
		int length = MultiByteToWideChar(CP_UTF8, 0, text.c_str(),
			static_cast<int>(text.size()), nullptr, 0);
		if (length <= 0) {
			return std::wstring();
		}
		std::wstring wide(static_cast<size_t>(length), L'\0');
		MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()),
			&wide[0], length);
		return wide;
	}

	/// <summary>判断目录是否存在</summary>
	bool directoryExists(const std::string& path) {
		std::wstring wide = utf8ToWide(path);
		if (wide.empty()) {
			return false;
		}
		DWORD attributes = GetFileAttributesW(wide.c_str());
		return attributes != INVALID_FILE_ATTRIBUTES &&
			(attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
	}

	/// <summary>把相对路径按基准目录拼成绝对路径</summary>
	std::string resolveAgainst(const std::string& base, const std::string& relative) {
		std::wstring baseWide = utf8ToWide(base);
		std::wstring relativeWide = utf8ToWide(relative);
		if (baseWide.empty() || relativeWide.empty()) {
			return relative;
		}
		std::vector<wchar_t> buffer(baseWide.size() + relativeWide.size() + 2);
		PWSTR result = nullptr;
		if (PathCombineW(buffer.data(), baseWide.c_str(), relativeWide.c_str())) {
			result = buffer.data();
		}
		return result ? wideToUtf8(result) : relative;
	}

	/// <summary>判断文件是否存在</summary>
	bool fileExists(const std::string& path) {
		std::wstring wide = utf8ToWide(path);
		if (wide.empty()) {
			return false;
		}
		DWORD attributes = GetFileAttributesW(wide.c_str());
		return attributes != INVALID_FILE_ATTRIBUTES &&
			(attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
	}

	/// <summary>把路径转成绝对路径（用宽字符 API，避免中文路径受代码页影响）</summary>
	std::string absolutePath(const std::string& path) {
		std::wstring wide = utf8ToWide(path);
		if (wide.empty()) {
			return path;
		}
		std::vector<wchar_t> buffer(MAX_PATH * 2);
		DWORD length = GetFullPathNameW(wide.c_str(), static_cast<DWORD>(buffer.size()),
			buffer.data(), nullptr);
		if (length == 0 || length >= buffer.size()) {
			return path;
		}
		return wideToUtf8(std::wstring(buffer.data(), length));
	}

	/// <summary>
	/// 探测某个路径能否被 httplib 正常当作静态目录使用。
	///
	/// httplib 取文件时用 base_dir + "/index.html" 拼接路径，
	/// 若 base_dir 是 "D:\a\b\web" 这种带反斜杠的绝对路径，拼出来会变成
	/// "D:\a\b\web/index.html"（混合分隔符），Windows 上判定文件不存在从而返回 404。
	/// 因此这里必须用同样的拼接方式实际验证，而不能只看目录是否存在。
	/// </summary>
	bool canServeAsWebRoot(const std::string& path) {
		if (!directoryExists(path)) {
			return false;
		}
		return fileExists(path + "/index.html");
	}

	/// <summary>
	/// 规范化静态目录：
	/// 依次尝试「原样」「相对当前工作目录」「相对后端 exe 目录」，
	/// 并用 httplib 的拼接方式验证确实能提供 index.html。
	/// 这样可以避免因工作目录不同、或使用绝对路径而导致的整站 404。
	/// </summary>
	std::string resolveWebRoot(const std::string& requested) {
		//1) 原样
		if (canServeAsWebRoot(requested)) {
			return requested;
		}
		//2) 相对当前工作目录（httplib 最稳的用法）
		if (canServeAsWebRoot("./" + requested)) {
			return "./" + requested;
		}
		//3) 相对后端 exe 所在目录
		std::vector<wchar_t> buffer(MAX_PATH);
		DWORD length = GetModuleFileNameW(nullptr, buffer.data(),
			static_cast<DWORD>(buffer.size()));
		if (length > 0 && length < buffer.size()) {
			std::wstring exePath(buffer.data(), length);
			size_t slash = exePath.find_last_of(L"\\/");
			if (slash != std::wstring::npos) {
				std::string exeDir = wideToUtf8(exePath.substr(0, slash));
				std::string candidate = resolveAgainst(exeDir, requested);
				if (canServeAsWebRoot(candidate)) {
					return candidate;
				}
			}
		}
		//都不行就原样返回，由构造函数给出警告
		return requested;
	}

	/// <summary>
	/// 解析命令行参数：--port / --web
	///
	/// 这里不使用 main 的 argc/argv：CRT 会把命令行按本地代码页（简体中文为 GBK）
	/// 转换成窄字符串，含中文的路径（例如 D:\项目\...）会因此变成乱码。
	/// 改为直接读取 GetCommandLineW，拿到原始宽字符再转成 UTF-8 使用。
	/// </summary>
	void parseArgs(int& port, std::string& webRoot) {
		int argc = 0;
		LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
		if (!argv) {
			return;
		}
		for (int i = 1; i < argc; i++) {
			std::wstring arg = argv[i];
			if (arg == L"--port" && i + 1 < argc) {
				port = _wtoi(argv[++i]);
			}
			else if (arg == L"--web" && i + 1 < argc) {
				webRoot = wideToUtf8(argv[++i]);
			}
		}
		LocalFree(argv);
	}
}

int main(int argc, char** argv) {
	//本程序不使用 argc/argv（见 parseArgs 的说明），这里显式忽略，避免误用
	(void)argc;
	(void)argv;

	//初始化控制台编码（见 Console.h）：控制台走宽字符输出，重定向时写 UTF-8，
	//这样中文在任何宿主里都不会变成乱码
	console::init();

	//自检：确认移植后的运动学与原实现数值一致
	RobotTest::runAll();

	int port = 8760;
	std::string webRoot = "web";
	parseArgs(port, webRoot);

	RobotService service;
	service.startSimulation();
	Server server(service, webRoot, port);

	console::write("六轴机械臂仿真后端已启动");
	console::writeFormat("  前端页面： http://127.0.0.1:%d/", port);
	console::writeFormat("  状态推送： http://127.0.0.1:%d/api/stream (SSE)", port);
	console::writeFormat("  静态目录： %s %s", server.mountedWebRoot().c_str(), server.filesystemNotice().c_str());
	console::write("按 Ctrl+C 退出");

	if (!server.listen()) {
		console::writeErrorFormat("端口 %d 监听失败（可能已被占用）", port);
		return 1;
	}
	return 0;
}
