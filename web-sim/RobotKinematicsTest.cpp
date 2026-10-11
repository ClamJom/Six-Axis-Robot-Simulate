//=====================================================================================
// RobotKinematicsTest.cpp
//
// 运动学自检：验证从原 VTK 版本迁移过来的算法数值行为没有改变。
// 这些测试在服务端启动时自动运行，结果打印到控制台。
//
// 注意：所有输出都走 console::* 而不是 printf，
// 否则中文字面量（UTF-8 字节）在 GBK 控制台上会显示成乱码。
//=====================================================================================

#include "RobotCore.h"
#include "RobotUtils.h"
#include "RobotScenario.h"
#include "Console.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace RobotTest {

	namespace {
		/// <summary>以 4x4 形式打印矩阵</summary>
		void printMatrix(const char* title, const Eigen::Matrix4f& matrix) {
			std::string text = std::string(title) + "\n";
			char line[128];
			for (int row = 0; row < 4; row++) {
				std::snprintf(line, sizeof(line), "  %10.4f %10.4f %10.4f %10.4f\n",
					matrix(row, 0), matrix(row, 1), matrix(row, 2), matrix(row, 3));
				text += line;
			}
			console::write(text);
		}

		/// <summary>由关节角做正解得到位姿：以它为目标一定是可达的</summary>
		Eigen::Matrix4f poseFromAngles(const Robot& robot, const Eigen::Vector<float, 6>& angles) {
			Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			for (int i = 0; i < robot.JointCount; i++) {
				pose *= robot.joints[i].getHFromTheta(angles(i));
			}
			pose *= robot.tool.H;
			return pose;
		}

		/// <summary>工具竖直向下、X 轴对齐世界 X 轴的位姿</summary>
		Eigen::Matrix4f makeTopDownPose(float x, float y, float z) {
			Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			pose.block<3, 1>(0, 0) = Eigen::Vector3f(1, 0, 0);
			pose.block<3, 1>(0, 1) = Eigen::Vector3f(0, -1, 0);
			pose.block<3, 1>(0, 2) = Eigen::Vector3f(0, 0, -1);
			pose.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
			return pose;
		}
	}

	/// <summary>
	/// 正逆运动学闭环：给定关节角做正解，再对结果做逆解，最后重新正解比较位置
	/// </summary>
	bool testForwardInverseKinematics() {
		console::write("=== 测试一：正逆运动学闭环 ===");
		Robot robot = RobotUtils::createDefaultRobot();

		Eigen::Vector<float, 6> pose;
		pose << 0.3f, -0.6f, 0.9f, 0.4f, 0.7f, -0.2f;
		Eigen::Matrix4f aim = Eigen::Matrix4f::Identity();
		for (int i = 0; i < robot.JointCount; i++) {
			aim *= robot.joints[i].getHFromTheta(pose(i));
		}
		aim *= robot.tool.H;
		printMatrix("目标位姿：", aim);

		bool solved = robot.IK(aim, false);
		Eigen::Matrix4f result = robot.FK4();

		Eigen::Vector4f aimPosition = aim * robot.BasePosition;
		Eigen::Vector4f resultPosition = result * robot.BasePosition;
		float error = (aimPosition - resultPosition).norm();

		console::writeFormat("逆解是否成功： %s", solved ? "是" : "否");
		console::writeFormat("目标位置： (%.4f, %.4f, %.4f)",
			aimPosition(0), aimPosition(1), aimPosition(2));
		console::writeFormat("逆解位置： (%.4f, %.4f, %.4f)",
			resultPosition(0), resultPosition(1), resultPosition(2));
		console::writeFormat("位置误差： %.6f mm", error);

		//旋转部分也应当吻合
		float rotationError = (aim.block<3, 3>(0, 0) - result.block<3, 3>(0, 0)).norm();
		console::writeFormat("姿态误差： %.6f", rotationError);

		bool ok = solved && error < 1e-2f && rotationError < 1e-3f;
		console::writeFormat("结果：%s", ok ? "通过" : "失败");
		console::write("");
		return ok;
	}

	/// <summary>
	/// 目标点位于点云包围盒位置时的逆解，以及轨迹记录是否可用
	/// </summary>
	bool testOBBTargetAndTrack() {
		console::write("=== 测试二：点云包围盒目标 + 轨迹记录 ===");

		//生成测试点云并做一次已知的位姿变换
		std::vector<RobotUtils::Point3> cloud = RobotUtils::makeTestCloud();
		Eigen::Matrix4f transform = RobotUtils::makePoseFromEuler(
			static_cast<float>(M_PI) / 4, static_cast<float>(M_PI) / 3, 0, 400, 400, 400);
		std::vector<RobotUtils::Point3> transformed = RobotUtils::transformCloud(cloud, transform);

		RobotUtils::OBB box = RobotUtils::computeOBB(transformed);
		if (!box.valid) {
			console::write("包围盒计算失败");
			console::write("结果：失败");
			console::write("");
			return false;
		}
		printMatrix("包围盒位姿：", box.pose);

		//旋转矩阵必须正交：R * R^T 应当为单位矩阵
		Eigen::Matrix3f rotation = box.pose.block<3, 3>(0, 0);
		float orthogonalError = (rotation * rotation.transpose() - Eigen::Matrix3f::Identity()).norm();
		console::writeFormat("旋转矩阵正交性误差： %.6f", orthogonalError);
		console::writeFormat("包围盒尺寸： %.2f x %.2f x %.2f",
			box.extents(0), box.extents(1), box.extents(2));

		//以包围盒位姿为目标做逆解，并检查轨迹
		Eigen::Matrix4f target = box.pose;
		RobotUtils::flipZAxisIfFacingRobot(target);

		Robot robot = RobotUtils::createDefaultRobot();
		robot.setTrackSize(24);
		robot.goHome();
		robot.IK(target, true);

		Eigen::Vector4f reached = robot.FK();
		Eigen::Vector3f want(target(0, 3), target(1, 3), target(2, 3));
		Eigen::Vector3f got(reached(0), reached(1), reached(2));
		float error = (want - got).norm();

		std::vector<Eigen::Vector3f> track = robot.getTrackPoints();
		console::writeFormat("逆解末端： (%.2f, %.2f, %.2f)", got(0), got(1), got(2));
		console::writeFormat("目标位置： (%.2f, %.2f, %.2f)", want(0), want(1), want(2));
		console::writeFormat("位置误差： %.4f mm", error);
		console::writeFormat("轨迹点数： %d", static_cast<int>(track.size()));

		bool ok = error < 1.0f && orthogonalError < 1e-3f &&
			track.size() == static_cast<size_t>(robot.getTrackSize());
		console::writeFormat("结果：%s", ok ? "通过" : "失败");
		console::write("");
		return ok;
	}

	/// <summary>
	/// 不可达目标（逆解无解）不能破坏模型状态：
	/// 关节角必须保持原值且仍是有限值，之后的可达目标必须仍然能正常求解。
	/// </summary>
	bool testUnreachableTargetKeepsModelValid() {
		console::write("=== 测试三：不可达目标不破坏模型状态 ===");
		Robot robot = RobotUtils::createDefaultRobot();
		robot.setTrackSize(24);
		robot.goHome();

		//两个一定可达的目标：直接由关节角做正解得到
		Eigen::Vector<float, 6> anglesA, anglesB;
		anglesA << 0.3f, -0.6f, 0.9f, 0.4f, 0.7f, -0.2f;
		anglesB << -0.5f, -0.2f, 0.6f, -0.3f, 0.9f, 0.5f;
		Eigen::Matrix4f aimA = poseFromAngles(robot, anglesA);
		Eigen::Matrix4f aimB = poseFromAngles(robot, anglesB);

		bool firstSolved = robot.IK(aimA, true);

		//记录成功解算之后的关节角与末端位置，作为「不可达目标不得改动模型」的基准
		Eigen::Vector<float, 6> before;
		for (int i = 0; i < 6; i++) {
			before(i) = robot.joints[i].theta;
		}
		Eigen::Vector4f beforeTcp = robot.FK();

		//远超工作空间的目标：逆解应当明确失败
		Eigen::Matrix4f unreachable = aimA;
		unreachable(0, 3) += 5000.0f;
		unreachable(1, 3) += 5000.0f;
		unreachable(2, 3) += 5000.0f;
		bool unreachableSolved = robot.IK(unreachable, true);

		//关节角既不能变，也不能变成 NaN
		float maxDelta = 0.0f;
		for (int i = 0; i < 6; i++) {
			maxDelta = std::max(maxDelta, std::fabs(robot.joints[i].theta - before(i)));
		}
		Eigen::Vector4f afterTcp = robot.FK();
		float tcpShift = (afterTcp - beforeTcp).norm();
		bool tcpFinite = afterTcp.allFinite();

		//轨迹同样不能被污染（失败时不应该重新记录轨迹）
		bool trackFinite = true;
		std::vector<Eigen::Vector3f> track = robot.getTrackPoints();
		for (const Eigen::Vector3f& point : track) {
			trackFinite = trackFinite && point.allFinite();
		}

		//关键点：不可达目标之后，可达目标必须仍然解得出来
		bool secondSolved = robot.IK(aimB, true);
		Eigen::Vector4f reached = robot.FK();
		Eigen::Vector3f want(aimB(0, 3), aimB(1, 3), aimB(2, 3));
		Eigen::Vector3f got(reached(0), reached(1), reached(2));
		float error = (want - got).norm();

		console::writeFormat("可达目标一是否解出： %s", firstSolved ? "是" : "否");
		console::writeFormat("不可达目标是否判定为无解： %s", unreachableSolved ? "否" : "是");
		console::writeFormat("关节角最大变化量： %.6f rad", maxDelta);
		console::writeFormat("失败前后末端位置变化： %.6f mm", tcpShift);
		console::writeFormat("末端位置是否有限： %s", tcpFinite ? "是" : "否");
		console::writeFormat("轨迹点是否有限： %s", trackFinite ? "是" : "否");
		console::writeFormat("其后的可达目标位置误差： %.4f mm", error);

		bool ok = firstSolved && !unreachableSolved && tcpFinite && trackFinite &&
			maxDelta == 0.0f && tcpShift == 0.0f &&
			secondSolved && error < 1e-2f;
		console::writeFormat("结果：%s", ok ? "通过" : "失败");
		console::write("");
		return ok;
	}

	/// <summary>
	/// 传送带取件场景（传送带两侧各一台机械臂）：
	///   左臂跑完备解析解、右臂跑旧特解，检查两台都能抓取/码放，
	///   并统计各自的轨迹点数与抓取偏差，用来对比两种算法。
	/// </summary>
	bool testConveyorScenario() {
		console::write("=== 测试四：传送带两侧双机械臂取件 ===");
		const float dt = 1.0f / 50.0f;
		const int steps = static_cast<int>(120.0f / dt);

		//---------- 左臂：完备解析解 ----------
		Robot left = RobotUtils::createDefaultRobot();
		left.ID = 1;
		left.Name = "左臂（完备）";
		left.setTrackSize(24);

		//---------- 右臂：旧特解，基座在传送带另一侧 ----------
		Robot right = RobotUtils::createDefaultRobot();
		right.ID = 2;
		right.Name = "右臂（旧特解）";
		right.setTrackSize(24);
		right.BaseTransform = Eigen::Matrix4f::Identity();
		right.BaseTransform(0, 3) = 1200.0f;
		right.setIKMode(Robot::IKMode::Legacy);

		robotscenario::ConveyorScenario scenario;
		scenario.addArm(left);
		scenario.addArm(right);

		scenario.setSpawnRange(1.4f, 2.2f);
		scenario.setBeltSpeed(200.0f);
		scenario.reset();
		scenario.start();

		for (int i = 0; i < steps; i++) {
			scenario.tick(dt);
			const robotscenario::ConveyorScenario::Arm& leftArm = scenario.arms()[0];
			const robotscenario::ConveyorScenario::Arm& rightArm = scenario.arms()[1];
			if ((leftArm.stacked >= 5 && rightArm.stacked >= 5) ||
				leftArm.phase == robotscenario::Phase::Fault) {
				break;
			}
		}

		const robotscenario::Layout& layout = scenario.layout();
		int pathPoints[2] = { 0, 0 };
		float worstOffset = 0.0f;
		bool stackedPlaced = true;
		bool finite = true;

		for (size_t index = 0; index < scenario.arms().size(); index++) {
			const robotscenario::ConveyorScenario::Arm& arm = scenario.arms()[index];
			pathPoints[index] = static_cast<int>(arm.path.size());
			if (!arm.robot->FK().allFinite()) {
				finite = false;
			}
		}

		//每只码放好的箱子都必须落在**它自己那台臂**的垛位坐标上（按臂编号查，不能用下标）
		for (const robotscenario::Box& box : scenario.boxes()) {
			if (box.state != robotscenario::Box::State::Stacked) {
				continue;
			}
			const robotscenario::ConveyorScenario::Arm* owner = scenario.armById(box.claimedBy);
			if (!owner) {
				continue;
			}
			Eigen::Vector3f want(owner->slotX[box.slot], owner->slotY[box.slot],
				layout.palletTopZ + layout.layerHeight * (box.layer + 0.5f));
			worstOffset = std::max(worstOffset, (box.position - want).norm());
			if ((box.position - want).norm() > 1.0f) {
				stackedPlaced = false;
			}
		}

		bool ok = finite && stackedPlaced;
		int totalStacked = 0;
		for (const robotscenario::ConveyorScenario::Arm& arm : scenario.arms()) {
			console::writeFormat("%s： 工位 y=%+.0f，抓取 %d，码放 %d，申请成功 %d，被拒 %d，溜走 %d，阶段 %s",
				arm.name.c_str(), arm.pickY, arm.picked, arm.stacked,
				arm.claimed, arm.contended, arm.missed,
				robotscenario::phaseName(arm.phase));
			if (arm.phase == robotscenario::Phase::Fault) {
				console::writeFormat("   ⚠ 故障：%s", arm.message.c_str());
			}
			totalStacked += arm.stacked;
		}
		console::writeFormat("末端位置是否有限： %s", finite ? "是" : "否");
		console::writeFormat("码放位置最大偏差： %.3f mm", worstOffset);
		console::writeFormat("本周期轨迹点数： 左 %d / 右 %d", pathPoints[0], pathPoints[1]);
		console::writeFormat("两台臂合计码放： %d 件", totalStacked);

		//要求：流水线整体能干活（下游臂接上游漏掉的），且都记录了轨迹
		ok = ok && totalStacked >= 4 && pathPoints[0] > 50 && pathPoints[1] > 50;

		//---------- 货物锁：工位重叠时必须出现「申请被拒」，且绝不允许两台臂抓同一件 ----------
		console::write("--- 货物锁（工位重叠时的抢锁）---");
		Robot lockA = RobotUtils::createDefaultRobot();
		lockA.ID = 21;
		Robot lockB = RobotUtils::createDefaultRobot();
		lockB.ID = 22;
		lockB.BaseTransform = Eigen::Matrix4f::Identity();
		lockB.BaseTransform(0, 3) = 1200.0f;
		robotscenario::ConveyorScenario lockScene;
		lockScene.addArm(lockA);
		lockScene.addArm(lockB);
		lockScene.setSpawnRange(7.0f, 7.0f);			//一次只来一件货，逼出争抢
		lockScene.setBeltSpeed(160.0f);
		lockScene.reset();
		lockScene.setArmPickY(lockA.ID, 0.0f);
		lockScene.setArmPickY(lockB.ID, 0.0f);			//两台臂工位完全重叠
		lockScene.start();

		int doubleClaimFrames = 0;
		for (int i = 0; i < steps; i++) {
			lockScene.tick(dt);
			//互斥不变量：同一件货物不能同时被两台臂排队去抓
			for (const robotscenario::Box& box : lockScene.boxes()) {
				int owners = 0;
				for (const robotscenario::ConveyorScenario::Arm& arm : lockScene.arms()) {
					if (arm.pickBoxId == box.id && arm.phase != robotscenario::Phase::Idle) {
						owners++;
					}
				}
				if (owners > 1) {
					doubleClaimFrames++;
				}
			}
			if (lockScene.arms()[0].stacked >= 4 || lockScene.arms()[1].stacked >= 4) {
				break;
			}
		}
		int contended = 0;
		int lockedTotal = 0;
		for (const robotscenario::ConveyorScenario::Arm& arm : lockScene.arms()) {
			contended += arm.contended;
			lockedTotal += arm.stacked;
			console::writeFormat("  %s： 抓取 %d，码放 %d，申请成功 %d，被拒 %d",
				arm.name.c_str(), arm.picked, arm.stacked, arm.claimed, arm.contended);
		}
		console::writeFormat("抢锁被拒总次数： %d（>0 说明「申请—被拒—放弃」协议真的在起作用）", contended);
		console::writeFormat("同一件货物被两台臂同时锁定的帧数： %d（必须为 0）", doubleClaimFrames);
		console::writeFormat("两台臂合计码放： %d 件", lockedTotal);

		const bool lockOk = contended > 0 && doubleClaimFrames == 0 && lockedTotal >= 3;
		ok = ok && lockOk;

		//---------- 托盘满后自动清空：必须真的把台上的箱子清掉，否则会一直往上堆 ----------
		console::write("--- 托盘满后自动清空 ---");
		Robot clearA = RobotUtils::createDefaultRobot();
		clearA.ID = 31;
		Robot clearB = RobotUtils::createDefaultRobot();
		clearB.ID = 32;
		clearB.BaseTransform = Eigen::Matrix4f::Identity();
		clearB.BaseTransform(0, 3) = 1200.0f;
		robotscenario::ConveyorScenario clearScene;
		clearScene.addArm(clearA);
		clearScene.addArm(clearB);
		clearScene.setAutoClear(true);
		clearScene.setSpawnRange(1.2f, 1.8f);			//进料快，逼出「托盘满」
		clearScene.setBeltSpeed(200.0f);
		clearScene.reset();
		clearScene.start();

		const int capacity = clearScene.capacity();
		int maxStackedInScene = 0;
		bool overCapacity = false;
		for (int i = 0; i < steps; i++) {
			clearScene.tick(dt);
			int stackedInScene = 0;
			for (const robotscenario::Box& box : clearScene.boxes()) {
				if (box.state == robotscenario::Box::State::Stacked) {
					stackedInScene++;
				}
			}
			maxStackedInScene = std::max(maxStackedInScene, stackedInScene);
			if (stackedInScene > 2 * capacity) {
				overCapacity = true;		//两台臂合计也不该超过 2 倍容量
			}
			int cleared = 0;
			for (const robotscenario::ConveyorScenario::Arm& arm : clearScene.arms()) {
				cleared += arm.cleared;
			}
			if (cleared >= 2 && clearScene.arms()[0].stacked >= capacity + 2) {
				break;
			}
		}
		int clearedTotal = 0;
		int stackedTotal = 0;
		for (const robotscenario::ConveyorScenario::Arm& arm : clearScene.arms()) {
			clearedTotal += arm.cleared;
			stackedTotal += arm.stacked;
			console::writeFormat("  %s： 码放 %d（容量 %d），自动清空 %d 次",
				arm.name.c_str(), arm.stacked, capacity, arm.cleared);
		}
		console::writeFormat("场景里同时存在的已码放货物峰值： %d（不得超过 2×容量 = %d）",
			maxStackedInScene, 2 * capacity);
		console::writeFormat("自动清空总次数： %d（>0 才算没失效）", clearedTotal);

		const bool clearOk = clearedTotal > 0 && !overCapacity && stackedTotal > capacity;
		ok = ok && clearOk;

		//---------- 每台臂独立参数 + 增删臂 + 挪位置 ----------
		console::write("--- 机械臂实体化（每臂独立设置 / 增删 / 挪位置）---");
		Robot third = RobotUtils::createDefaultRobot();
		third.ID = 3;
		third.Name = "三号臂";
		third.BaseTransform = Eigen::Matrix4f::Identity();
		third.BaseTransform(0, 3) = 1200.0f;		//摆在右臂后方 400mm
		third.BaseTransform(1, 3) = -400.0f;
		scenario.addArm(third);
		scenario.setArmSpeed(3, 2.0f);				//只提速三号臂
		scenario.setArmTrackingSpeed(3, 2.4f);
		scenario.setTrackSize(64, 3);				//只改三号臂的轨迹点数

		const robotscenario::ConveyorScenario::Arm* thirdArm = scenario.armById(3);
		const bool independent = thirdArm && std::fabs(thirdArm->speedFactor - 2.0f) < 1e-6f &&
			std::fabs(thirdArm->trackingSpeed - 2.4f) < 1e-6f &&
			thirdArm->robot->getTrackSize() == 64 &&
			scenario.armById(1)->robot->getTrackSize() == 24 &&	//一号臂不受影响
			thirdArm->slotX[0] == 1050.0f &&					//垛位按基座推导：1200-150
			thirdArm->slotY[0] == -880.0f;						//纵向也跟着基座：-400-480

		//挪动右臂基座：往传送带方向靠 100mm（垛位随之重算）
		const bool moved = scenario.setArmBase(2, 1100.0f, 0.0f) &&
			std::fabs(scenario.armById(2)->baseX - 1100.0f) < 1e-6f &&
			scenario.armById(2)->slotX[0] == 950.0f &&
			scenario.setArmBase(2, 1200.0f, 0.0f);				//再挪回去

		const bool removed = scenario.removeArm(3) && scenario.armById(3) == nullptr &&
			scenario.armCount() == 2;

		console::writeFormat("三号臂独立参数（速度 2.0 / 跟踪 2.4 / 轨迹点 64）： %s",
			independent ? "是" : "否");
		console::writeFormat("挪动右臂基座到 (1100, 0) 并重算垛位： %s", moved ? "成功" : "失败");
		console::writeFormat("移除三号臂后剩 %d 台： %s", scenario.armCount(), removed ? "成功" : "失败");
		ok = ok && independent && moved && removed;

		//---------- 动态抓取：货物不停，机械臂预测 + 跟踪 ----------
		Robot trackingRobot = RobotUtils::createDefaultRobot();
		trackingRobot.ID = 11;
		Robot trackingRobot2 = RobotUtils::createDefaultRobot();
		trackingRobot2.ID = 12;
		trackingRobot2.BaseTransform = Eigen::Matrix4f::Identity();
		trackingRobot2.BaseTransform(0, 3) = 1200.0f;
		robotscenario::ConveyorScenario tracking;
		tracking.addArm(trackingRobot);
		tracking.addArm(trackingRobot2);
		tracking.setDynamicPick(true);
		tracking.setSpawnRange(2.5f, 4.0f);
		tracking.setBeltSpeed(200.0f);
		tracking.reset();
		tracking.start();

		float worstTrackError = 0.0f;
		float worstGraspError = 0.0f;
		float sumTrackError = 0.0f;
		int trackSamples = 0;
		for (int i = 0; i < steps; i++) {
			tracking.tick(dt);
			for (const robotscenario::ConveyorScenario::Arm& arm : tracking.arms()) {
				if (arm.phase == robotscenario::Phase::Descend) {
					worstTrackError = std::max(worstTrackError, arm.trackError);
					sumTrackError += arm.trackError;
					trackSamples++;
				}
				if (arm.phase == robotscenario::Phase::Grip) {
					worstGraspError = std::max(worstGraspError, arm.graspError);
				}
			}
			if (tracking.arms()[0].stacked >= 3) {
				break;
			}
		}
		const float averageTrackError = (trackSamples > 0) ? sumTrackError / trackSamples : 0.0f;

		console::write("--- 动态抓取（货物不停机）---");
		console::writeFormat("左臂： 抓取 %d，码放 %d，漏抓 %d",
			tracking.arms()[0].picked, tracking.arms()[0].stacked, tracking.arms()[0].missed);
		console::writeFormat("跟踪偏差： 平均 %.1f mm，追赶瞬态最大 %.1f mm（传送带 %.0f mm/s）",
			averageTrackError, worstTrackError, tracking.beltSpeed());
		console::writeFormat("合爪瞬间偏差： 最大 %.1f mm（这才是抓得准不准）", worstGraspError);

		const bool trackingOk = tracking.arms()[0].stacked >= 2 && worstGraspError < 20.0f;
		ok = ok && trackingOk;
		console::writeFormat("结果：%s", ok ? "通过" : "失败");
		console::write("");
		return ok;
	}

	/// <summary>
	/// 逆解完备性与连续性：
	///   1. 随机构型做正解再逆解，必须能回到原位姿（旧实现只有一组特解，很多构型解不出来）；
	///   2. 沿竖直方向逐点求解（只改 Z），相邻两点之间不允许出现大的关节跳变
	///      —— 旧实现在 ±180° 附近会翻转符号，表现为「姿态突然转 360°」。
	/// </summary>
	bool testInverseKinematicsCompleteness() {
		console::write("=== 测试五：逆解完备性与连续性 ===");
		Robot robot = RobotUtils::createDefaultRobot();

		//---------- 1) 随机构型的正解 → 逆解 ----------
		std::mt19937 engine(12345u);
		std::uniform_real_distribution<float> angleDist(-2.6f, 2.6f);
		int tried = 0;
		int reached = 0;
		int multipleSolutions = 0;
		int maxSolutions = 0;
		float worstPosition = 0.0f;
		float worstRotation = 0.0f;

		for (int sample = 0; sample < 400; sample++) {
			Eigen::Vector<float, 6> pose;
			for (int i = 0; i < 6; i++) {
				pose(i) = angleDist(engine);
			}
			//目标位姿由正解得到，因此一定是可达的
			const Eigen::Matrix4f aim = robot.forwardFor(pose);

			std::vector<Eigen::Vector<float, 6>> solutions;
			int count = robot.solveIKAll(aim, solutions);
			maxSolutions = std::max(maxSolutions, count);
			tried++;
			if (count > 1) {
				multipleSolutions++;
			}
			if (count == 0) {
				continue;
			}
			reached++;

			//每一组解都必须真的落到目标位姿上
			for (const Eigen::Vector<float, 6>& candidate : solutions) {
				const Eigen::Matrix4f got = robot.forwardFor(candidate);
				worstPosition = std::max(worstPosition,
					(got.block<3, 1>(0, 3) - aim.block<3, 1>(0, 3)).norm());
				worstRotation = std::max(worstRotation,
					(got.block<3, 3>(0, 0) - aim.block<3, 3>(0, 0)).norm());
			}
		}

		//---------- 2) 竖直下降的连续性 ----------
		//沿若干条竖直线从上往下走，每一步都以「上一步的解」为参考构型，
		//考察相邻两步之间最大的关节变化量。
		float worstJump = 0.0f;
		const char* worstColumn = "";
		int columnsTested = 0;
		int columnsOk = 0;
		const float columns[6][2] = {
			{ 600.0f, 0.0f }, { 500.0f, 120.0f }, { 350.0f, 150.0f },
			{ 420.0f, 0.0f }, { 250.0f, -450.0f }, { 700.0f, -150.0f }
		};

		for (const auto& column : columns) {
			const float x = column[0], y = column[1];
			Eigen::Vector<float, 6> reference;
			for (int i = 0; i < 6; i++) {
				reference(i) = robot.joints[i].theta;
			}
			bool havePrevious = false;
			float maxJump = 0.0f;
			int solvedSteps = 0;

			for (float z = 640.0f; z >= 260.0f; z -= 10.0f) {
				const Eigen::Matrix4f aim = makeTopDownPose(x, y, z);
				Eigen::Vector<float, 6> solution;
				if (!robot.solveIK(aim, reference, solution)) {
					havePrevious = false;		//这一步够不着，重新起头
					continue;
				}
				if (havePrevious) {
					float jump = 0.0f;
					for (int i = 0; i < 6; i++) {
						jump = std::max(jump, std::fabs(solution(i) - reference(i)));
					}
					maxJump = std::max(maxJump, jump);
				}
				reference = solution;
				havePrevious = true;
				solvedSteps++;
			}

			if (solvedSteps < 20) {
				continue;				//这条竖直线大部分不可达，不参与统计
			}
			columnsTested++;
			if (maxJump < 0.35f) {		//10mm 一步，关节变化超过 20° 就算异常
				columnsOk++;
			}
			else {
				console::writeFormat("  竖直列 (%.0f, %.0f) 出现 %.1f° 跳变",
					x, y, maxJump * 180.0f / static_cast<float>(M_PI));
			}
			if (maxJump > worstJump) {
				worstJump = maxJump;
			}
		}
		(void)worstColumn;

		console::writeFormat("随机构型样本： %d，解出： %d（%.1f%%）",
			tried, reached, 100.0f * static_cast<float>(reached) / static_cast<float>(tried));
		console::writeFormat("多解样本： %d 组，单目标最多解出 %d 组", multipleSolutions, maxSolutions);
		console::writeFormat("全部解的最大位置偏差： %.6f mm", worstPosition);
		console::writeFormat("全部解的最大姿态偏差： %.6f", worstRotation);
		console::writeFormat("竖直连续性： %d/%d 条竖直线无跳变，最大相邻关节变化 %.2f°",
			columnsOk, columnsTested, worstJump * 180.0f / static_cast<float>(M_PI));

		const float reachRate = static_cast<float>(reached) / static_cast<float>(tried);
		bool ok = reachRate > 0.98f && multipleSolutions > tried / 4 &&
			worstPosition < 0.1f && worstRotation < 2e-3f &&
			columnsTested >= 3 && columnsOk == columnsTested;
		console::writeFormat("结果：%s", ok ? "通过" : "失败");
		console::write("");
		return ok;
	}

	/// <summary>
	/// 依次运行全部自检
	/// </summary>
	bool runAll() {
		bool kinematics = testForwardInverseKinematics();
		bool obb = testOBBTargetAndTrack();
		bool unreachable = testUnreachableTargetKeepsModelValid();
		bool scenario = testConveyorScenario();
		bool complete = testInverseKinematicsCompleteness();
		console::writeFormat("=== 自检汇总：%s ===",
			(kinematics && obb && unreachable && scenario && complete) ? "全部通过" : "存在失败项");
		console::write("");
		return kinematics && obb && unreachable && scenario && complete;
	}
}
