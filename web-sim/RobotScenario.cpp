//=====================================================================================
// RobotScenario.cpp
//
// 「传送带两侧各一台机械臂」的取件码垛场景实现。设计说明见 RobotScenario.h。
//=====================================================================================

#include "RobotScenario.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace robotscenario {

	namespace {
		/// <summary>固定种子的随机数：演示效果上仍是随机的，自检结果却可复现</summary>
		std::mt19937& rng() {
			static std::mt19937 engine(20240607u);
			return engine;
		}

		float randomFloat(float low, float high) {
			std::uniform_real_distribution<float> dist(low, high);
			return dist(rng());
		}

		/// <summary>取整到 mm，让垛型更整齐</summary>
		float roundmm(float value) {
			return std::round(value);
		}

		/// <summary>本周期轨迹最多保留的点数（50Hz 下约 12 秒）</summary>
		constexpr size_t kMaxPathPoints = 600;
	}

	ConveyorScenario::ConveyorScenario() {
	}

	void ConveyorScenario::setDynamicPick(bool enabled) {
		dynamicPick_ = enabled;
		for (Arm& arm : arms_) {
			arm.dynamicPick = enabled;
		}
	}

	void ConveyorScenario::setTrackingSpeed(float speed) {
		for (Arm& arm : arms_) {
			arm.trackingSpeed = clampf(speed, 0.3f, 4.0f);
		}
	}

	void ConveyorScenario::setArmSpeed(int armId, float factor) {
		for (Arm& arm : arms_) {
			if (armId < 0 || arm.id == armId) {
				arm.speedFactor = clampf(factor, 0.15f, 4.0f);
			}
		}
	}

	void ConveyorScenario::setArmTrackingSpeed(int armId, float speed) {
		for (Arm& arm : arms_) {
			if (armId < 0 || arm.id == armId) {
				arm.trackingSpeed = clampf(speed, 0.3f, 4.0f);
			}
		}
	}

	void ConveyorScenario::setArmDynamicPick(int armId, bool enabled) {
		for (Arm& arm : arms_) {
			if (armId < 0 || arm.id == armId) {
				arm.dynamicPick = enabled;
			}
		}
	}

	void ConveyorScenario::setTrackSize(int size, int armId) {
		const int clamped = std::max(4, std::min(200, size));
		if (armId < 0) {
			trackSize_ = clamped;			//记住默认值，之后新加的臂也继承它
		}
		for (Arm& arm : arms_) {
			if (armId < 0 || arm.id == armId) {
				arm.robot->setTrackSize(clamped);
			}
		}
	}

	ConveyorScenario::Arm& ConveyorScenario::addArm(Robot& robot) {
		Arm arm;
		arm.id = robot.ID;
		arm.name = robot.Name;
		arm.robot = &robot;
		arm.baseX = robot.BaseTransform(0, 3);
		arm.baseY = robot.BaseTransform(1, 3);
		arm.side = (arm.baseX < layout_.beltX) ? -1.0f : 1.0f;
		//垛位按基座推导：横向朝带面靠 150mm，纵向相对基座偏 -480/-230（与出厂布局一致）
		arm.slotX[0] = arm.slotX[1] = arm.baseX - arm.side * 150.0f;
		arm.slotY[0] = arm.baseY - 480.0f;
		arm.slotY[1] = arm.baseY - 230.0f;
		//默认工位都落在同一条取件线上：所有臂一起竞争，由货物锁决定谁抓到；
		//想做成「上游没抓到、下游接着抓」的流水线，把各臂工位在界面上错开即可。
		arm.pickY = layout_.pickY;
		arm.speedFactor = 1.0f;
		arm.trackingSpeed = 1.6f;
		arm.dynamicPick = dynamicPick_;
		arm.picked = arm.stacked = arm.missed = arm.cleared = 0;
		arm.claimed = arm.contended = 0;
		robot.setTrackSize(trackSize_);
		arms_.push_back(arm);
		return arms_.back();
	}

	//=================================================================================
	// 货物锁：一件货物同一时刻只能被一台机械臂认领
	//=================================================================================
	bool ConveyorScenario::requestBox(int armId, int boxId) {
		Box* box = findBox(boxId);
		Arm* arm = armById(armId);
		if (!box || !arm || box->state != Box::State::Belt) {
			return false;
		}
		if (box->claimedBy == armId) {
			return true;						//已经是自己锁着的
		}
		if (box->claimedBy != 0) {
			arm->contended++;					//被别人锁了：记一次争抢失败，调用方放弃
			return false;
		}
		box->claimedBy = armId;
		arm->claimed++;
		return true;
	}

	void ConveyorScenario::releaseBox(int armId, int boxId) {
		Box* box = findBox(boxId);
		//只有还在传送带上的货物才谈得上「解锁」；已经抓在手里/码好的箱子，
		//claimedBy 表示归属，必须保留（自动清空、移动基座、删除臂都靠它认领自己的货）
		if (box && box->state == Box::State::Belt && box->claimedBy == armId) {
			box->claimedBy = 0;					//解锁，让下游的臂有机会接手
		}
	}

	void ConveyorScenario::releaseClaims(int armId) {
		for (Box& box : boxes_) {
			if (box.state == Box::State::Belt && box.claimedBy == armId) {
				box.claimedBy = 0;				//同样只释放传送带上的锁
			}
		}
	}

	bool ConveyorScenario::setArmPickY(int armId, float y) {
		Arm* arm = armById(armId);
		if (!arm) {
			return false;
		}
		arm->pickY = clampf(y, layout_.beltBeginY + 200.0f, layout_.beltEndY - 200.0f);
		//工位变了：正在进行的这一件放弃掉，重新预解算待命位
		releaseClaims(armId);
		arm->pickBoxId = 0;
		std::string error;
		arm->waypointsReady = prepareWaypoints(*arm, error);
		if (!arm->waypointsReady) {
			fail(*arm, "新工位预解算失败：" + error);
			return false;
		}
		arm->phase = Phase::Idle;
		arm->joints = arm->wpReady;
		arm->moveFrom = arm->moveTo = arm->joints;
		arm->moveT = 1.0f;
		arm->warning = false;
		arm->message = "抓取工位已设为 y = " +
			std::to_string(static_cast<int>(std::lround(arm->pickY))) + " mm";
		applyJoints(*arm);
		return true;
	}

	/// <summary>找一件「正经过我的工位、还没被别人锁定」的货物（最靠前的那件）</summary>
	/// <summary>
	/// 在工位窗口内找一件货物：
	///   mode = 0：只要还没被认领的（正常申请）
	///   mode = 1：只要被**别人**认领的（用来走一遍「申请 → 被拒 → 放弃」的流程）
	/// </summary>
	Box* ConveyorScenario::findBoxInWindow(const Arm& arm, int mode) {
		const float window = layout_.claimWindowY;
		Box* best = nullptr;
		for (Box& box : boxes_) {
			if (box.state != Box::State::Belt) {
				continue;
			}
			if (box.id == arm.skipBoxId) {
				continue;						//这件刚才解不出来，别再反复申请
			}
			const bool free = (box.claimedBy == 0);
			const bool takenByOther = (box.claimedBy != 0 && box.claimedBy != arm.id);
			if ((mode == 0 && !free) || (mode == 1 && !takenByOther)) {
				continue;
			}
			const float distance = box.position.y() - arm.pickY;
			if (distance > 20.0f || distance < -window) {
				continue;						//已经过站太多 / 还太远
			}
			if (!best || box.position.y() > best->position.y()) {
				best = &box;					//最靠前的优先（先到先得）
			}
		}
		return best;
	}

	Box* ConveyorScenario::findClaimableBox(const Arm& arm) {
		const float window = layout_.claimWindowY;
		Box* best = nullptr;
		for (Box& box : boxes_) {
			if (box.state != Box::State::Belt || box.claimedBy != 0 || box.id == arm.skipBoxId) {
				continue;
			}
			const float distance = box.position.y() - arm.pickY;
			if (distance > 20.0f || distance < -window) {
				continue;						//已经过站太多 / 还太远
			}
			if (betterSuited(arm, box)) {
				continue;						//有更合适的空闲臂，让给它（避免一台臂把活全包了）
			}
			if (!best || box.position.y() > best->position.y()) {
				best = &box;					//最靠前的优先（先到先得）
			}
		}
		return best;
	}

	/// <summary>
	/// 是否还有另一台「空闲且横向上更靠近这件货」的臂：有就让给它。
	/// 两台臂工位重合时靠这条规则自然按左右分工；那台臂忙不过来时，
	/// 另一台照样能申请到（锁仍然管用，只是优先权有个次序）。
	/// </summary>
	bool ConveyorScenario::betterSuited(const Arm& arm, const Box& box) const {
		const float mine = std::fabs(arm.baseX - box.position.x());
		for (const Arm& other : arms_) {
			if (other.id == arm.id || other.phase != Phase::Idle || other.skipBoxId == box.id) {
				continue;
			}
			const float theirs = std::fabs(other.baseX - box.position.x());
			if (theirs < mine - 1.0f) {
				return true;
			}
			if (std::fabs(theirs - mine) <= 1.0f && other.id < arm.id) {
				return true;				//一样近时让编号小的先，保证只会有一个人去申请
			}
		}
		return false;
	}

	bool ConveyorScenario::removeArm(int armId) {
		const auto it = std::find_if(arms_.begin(), arms_.end(),
			[armId](const Arm& arm) { return arm.id == armId; });
		if (it == arms_.end()) {
			return false;
		}
		//它抓着的、它码好的货物一并清掉（那台臂的收集台已经不存在了）
		boxes_.erase(std::remove_if(boxes_.begin(), boxes_.end(), [armId](const Box& box) {
			return (box.state != Box::State::Belt) && box.claimedBy == armId;
			}), boxes_.end());
		releaseClaims(armId);
		arms_.erase(it);
		return true;
	}

	ConveyorScenario::Arm* ConveyorScenario::armById(int armId) {
		for (Arm& arm : arms_) {
			if (arm.id == armId) {
				return &arm;
			}
		}
		return nullptr;
	}

	const ConveyorScenario::Arm* ConveyorScenario::armById(int armId) const {
		for (const Arm& arm : arms_) {
			if (arm.id == armId) {
				return &arm;
			}
		}
		return nullptr;
	}

	bool ConveyorScenario::setArmBase(int armId, float x, float y) {
		Arm* arm = armById(armId);
		if (!arm) {
			return false;
		}
		Robot& robot = *arm->robot;
		robot.BaseTransform(0, 3) = x;
		robot.BaseTransform(1, 3) = y;
		arm->baseX = x;
		arm->baseY = y;
		arm->side = (x < layout_.beltX) ? -1.0f : 1.0f;
		arm->slotX[0] = arm->slotX[1] = x - arm->side * 150.0f;
		arm->slotY[0] = y - 480.0f;
		arm->slotY[1] = y - 230.0f;

		//收集台跟着搬走了：清掉它自己码好的货物与占位
		boxes_.erase(std::remove_if(boxes_.begin(), boxes_.end(), [armId](const Box& box) {
			return box.state == Box::State::Stacked && box.claimedBy == armId;
			}), boxes_.end());
		for (auto& row : arm->occupied) {
			for (auto& cell : row) {
				cell = false;
			}
		}
		//换了位置：它原来锁着的货物交还给产线，重新由各工位竞争
		releaseClaims(armId);

		arm->path.clear();
		arm->planned.clear();
		arm->plannedPoses.clear();
		arm->usingTrackPlan = false;
		arm->plannedSegments = 0;
		arm->gripperOpening = arm->gripperTarget = 240.0f;
		arm->pickBoxId = 0;

		std::string error;
		arm->waypointsReady = prepareWaypoints(*arm, error);
		if (!arm->waypointsReady) {
			fail(*arm, "新位置工位预解算失败：" + error);
			return false;
		}
		arm->phase = Phase::Idle;
		arm->joints = arm->wpReady;
		arm->moveFrom = arm->moveTo = arm->joints;
		arm->moveT = 1.0f;
		arm->warning = false;
		arm->message = "基座已移动到 (" + std::to_string(static_cast<int>(std::lround(x))) +
			", " + std::to_string(static_cast<int>(std::lround(y))) + ")";
		applyJoints(*arm);
		return true;
	}

	//=================================================================================
	// 工位预解算
	//=================================================================================
	Eigen::Matrix4f ConveyorScenario::topDownPose(const Vec3& position) {
		//工具 Z 轴竖直向下，X 轴对齐世界 X 轴（两指夹爪沿 X 方向合拢），保证箱子不歪
		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		pose.block<3, 1>(0, 0) = Eigen::Vector3f(1, 0, 0);
		pose.block<3, 1>(0, 1) = Eigen::Vector3f(0, -1, 0);
		pose.block<3, 1>(0, 2) = Eigen::Vector3f(0, 0, -1);
		pose.block<3, 1>(0, 3) = position;
		return pose;
	}

	bool ConveyorScenario::prepareWaypoints(Arm& arm, std::string& error) {
		const Layout& L = layout_;
		Robot& robot = *arm.robot;

		//---------- 待命位（自己那一侧、取件线上方）----------
		const float readyX = L.beltX + arm.side * (L.beltHalfWidth * 0.5f);
		const float approachCandidates[] = { L.pickApproachZ, 480.0f, 440.0f, 400.0f };
		bool readyOk = false;
		for (float z : approachCandidates) {
			if (robot.solveIK(topDownPose(Vec3(readyX, arm.pickY, z)), arm.wpReady)) {
				readyOk = true;
				break;
			}
		}
		if (!readyOk) {
			error = "取件线太远（基座离取件线 (600, 0) 超过约 800mm）";
			return false;
		}

		//---------- 码垛工位 ----------
		//不同逆解算法的可达性差别很大（旧特解尤其挑），因此在标称位置附近自动搜一组能解的垛位
		const float transferCandidates[] = { 700.0f, 680.0f, 660.0f, 640.0f, 620.0f, 600.0f };
		const float slotDx[] = { 0.0f, 0.0f, -40.0f, 40.0f, 0.0f, 0.0f, -80.0f, 80.0f, -40.0f, 40.0f, -40.0f, 40.0f };
		const float slotDy[] = { 0.0f, -40.0f, 0.0f, 0.0f, 40.0f, 40.0f, -40.0f, -40.0f, -80.0f, -80.0f, 80.0f, 80.0f };
		const int slotTries = 12;

		for (int slot = 0; slot < L.slotCount; slot++) {
			const float nominalX = arm.slotX[slot];
			const float nominalY = arm.slotY[slot];
			bool found = false;

			for (int attempt = 0; attempt < slotTries && !found; attempt++) {
				const float sx = nominalX + slotDx[attempt];
				const float sy = nominalY + slotDy[attempt];
				Arm::SlotWaypoints candidate;
				candidate.ok = false;

				for (float z : transferCandidates) {
					if (robot.solveIK(topDownPose(Vec3(sx, sy, z)), arm.wpReady, candidate.up)) {
						candidate.ok = true;
						break;
					}
				}
				if (!candidate.ok) {
					continue;
				}

				bool layersOk = true;
				for (int layer = 0; layer < L.layers && layersOk; layer++) {
					const float z = L.palletTopZ + (layer + 1) * L.layerHeight;
					layersOk = robot.solveIK(topDownPose(Vec3(sx, sy, z)), candidate.up, candidate.down[layer]);
					if (!layersOk) {
						for (float offset : { 12.0f, 24.0f, 36.0f }) {
							if (robot.solveIK(topDownPose(Vec3(sx, sy, z + offset)),
								candidate.up, candidate.down[layer])) {
								layersOk = true;
								break;
							}
						}
					}
				}
				if (!layersOk) {
					continue;
				}

				//这一组可以用了：把垛位挪到搜索到的位置
				arm.slots[slot] = candidate;
				arm.slotX[slot] = sx;
				arm.slotY[slot] = sy;
				found = true;
			}

			if (!found) {
				error = "垛位 " + std::to_string(slot + 1) + " 附近找不到可解位置";
				return false;
			}
		}
		arm.waypointsReady = true;
		return true;
	}

	//=================================================================================
	// 按货物实际坐标现解取件位
	//=================================================================================
	bool ConveyorScenario::solveTopDownAt(Arm& arm, float x, float y, float z,
		const Eigen::Vector<float, 6>& reference, Eigen::Vector<float, 6>& out) const {
		const float offsets[][2] = {
			{ 0.0f, 0.0f }, { 12.0f, 0.0f }, { -12.0f, 0.0f },
			{ 0.0f, 10.0f }, { 0.0f, -10.0f }, { 20.0f, 0.0f }, { -20.0f, 0.0f }
		};
		const float heights[] = { 0.0f, 8.0f, 16.0f };
		for (float height : heights) {
			for (const auto& offset : offsets) {
				const Eigen::Matrix4f aim = topDownPose(Vec3(x + offset[0], y + offset[1], z + height));
				if (arm.robot->solveIK(aim, reference, out)) {
					return true;
				}
			}
		}
		return false;
	}

	bool ConveyorScenario::preparePickWaypoints(Arm& arm, const Box& box, std::string& error) {
		const Layout& L = layout_;
		Robot& robot = *arm.robot;

		Eigen::Vector<float, 6> reference = arm.joints;
		const float graspZ = box.position.z() + L.boxHeight * 0.5f;
		if (!solveTopDownAt(arm, box.position.x(), box.position.y(), graspZ,
			reference, arm.wpPickDown)) {
			error = "货物位置解不出取件位";
			return false;
		}

		const float approachCandidates[] = {
			L.pickApproachZ, L.pickApproachZ - 40.0f, L.pickApproachZ - 80.0f, 380.0f
		};
		bool approachOk = false;
		for (float z : approachCandidates) {
			if (z <= graspZ + 60.0f) {
				continue;
			}
			if (robot.solveIK(topDownPose(Vec3(box.position.x(), box.position.y(), z)),
				arm.wpPickDown, arm.wpPickUp)) {
				approachOk = true;
				break;
			}
		}
		if (!approachOk) {
			error = "货物位置解不出接近位";
			return false;
		}
		arm.lastPickOffsetX = box.position.x() - L.beltX;
		return true;
	}

	//=================================================================================
	// 运动
	//=================================================================================
	void ConveyorScenario::beginMove(Arm& arm, const Eigen::Vector<float, 6>& target, float speed) {
		arm.moveFrom = arm.joints;
		arm.moveTo = target;
		arm.usingTrackPlan = false;
		arm.plannedPoses.clear();
		arm.planned.clear();

		//机械臂速度系数作用在每段的关节角速度上
		speed *= arm.speedFactor;

		float maxDelta = 0.0f;
		for (int i = 0; i < 6; i++) {
			maxDelta = std::max(maxDelta, std::fabs(target(i) - arm.joints(i)));
		}
		if (maxDelta < 1e-4f) {
			arm.moveT = 1.0f;
			arm.moveDuration = 0.0f;
			return;
		}
		arm.moveDuration = clampf(maxDelta / std::max(0.05f, speed), 0.15f, 8.0f);
		arm.moveT = 0.0f;

		//旧结算模式：这一段交给 Robot::getTrack 规划，之后按时间逐点执行它给出的关节序列。
		//这样「规划出的轨迹」与「实际走出来的轨迹」是同一条，调试输出/回放都不会骗人。
		if (arm.robot->ikMode() == Robot::IKMode::Legacy) {
			arm.robot->planTrack(target);
			arm.plannedPoses = arm.robot->getTrackPoses();
			arm.planned = arm.robot->getTrackPoints();
			arm.usingTrackPlan = !arm.plannedPoses.empty();
			if (arm.usingTrackPlan) {
				arm.plannedSegments++;
				arm.planTotal++;
			}
		}
	}

	bool ConveyorScenario::advanceMove(Arm& arm, float dt) {
		if (arm.moveT >= 1.0f) {
			arm.joints = arm.moveTo;
			return true;
		}
		arm.moveT = clampf(arm.moveT + dt / std::max(1e-3f, arm.moveDuration), 0.0f, 1.0f);

		if (arm.usingTrackPlan && !arm.plannedPoses.empty()) {
			//执行 getTrack 规划出来的关节序列：按时间比例取第几个采样点
			const int count = static_cast<int>(arm.plannedPoses.size());
			int index = static_cast<int>(arm.moveT * static_cast<float>(count - 1) + 0.5f);
			index = std::min(count - 1, std::max(0, index));
			arm.joints = arm.plannedPoses[static_cast<size_t>(index)];
			if (arm.moveT >= 1.0f) {
				arm.joints = arm.moveTo;	//末点严格落在目标上（getTrack 的序列少走一步）
				return true;
			}
			return false;
		}

		const float s = arm.moveT * arm.moveT * (3.0f - 2.0f * arm.moveT);
		arm.joints = arm.moveFrom + (arm.moveTo - arm.moveFrom) * s;
		return arm.moveT >= 1.0f;
	}

	void ConveyorScenario::steerTo(Arm& arm, const Eigen::Vector<float, 6>& target,
		float maxJointSpeed, float dt) {
		const float step = maxJointSpeed * dt;
		for (int i = 0; i < 6; i++) {
			const float delta = target(i) - arm.joints(i);
			arm.joints(i) += clampf(delta, -step, step);
		}
	}

	void ConveyorScenario::applyJoints(Arm& arm) {
		for (int i = 0; i < arm.robot->JointCount && i < 6; i++) {
			arm.robot->joints[i].updateH(arm.joints(i));
		}
	}

	void ConveyorScenario::recordPath(Arm& arm) {
		//只在干活的时候记录：这样轨迹正好是「上一个状态到下一个状态」的一个完整周期
		if (arm.phase == Phase::Idle || arm.phase == Phase::Fault) {
			return;
		}
		const Eigen::Vector4f tcp = arm.robot->FK();
		arm.path.push_back(Vec3(tcp(0), tcp(1), tcp(2)));
		if (arm.path.size() > kMaxPathPoints) {
			arm.path.erase(arm.path.begin(), arm.path.begin() + (arm.path.size() - kMaxPathPoints));
		}
		arm.pathRevision++;
	}

	//=================================================================================
	// 夹爪
	//=================================================================================
	void ConveyorScenario::updateGripper(Arm& arm, float dt) {
		const float step = 900.0f * dt;
		if (arm.gripperOpening < arm.gripperTarget) {
			arm.gripperOpening = std::min(arm.gripperTarget, arm.gripperOpening + step);
		}
		else if (arm.gripperOpening > arm.gripperTarget) {
			arm.gripperOpening = std::max(arm.gripperTarget, arm.gripperOpening - step);
		}
	}

	//=================================================================================
	// 箱子与传送带
	//=================================================================================
	void ConveyorScenario::spawnBox() {
		const Layout& L = layout_;
		Box box;
		box.id = nextId_++;
		box.size.x() = roundmm(randomFloat(L.boxMinFootprint, L.boxMaxFootprint));
		box.size.y() = roundmm(randomFloat(L.boxMinFootprint, L.boxMaxFootprint));
		box.size.z() = L.boxHeight;
		const float maxSpread = std::max(0.0f, L.beltHalfWidth - box.size.x() * 0.5f - 8.0f);
		const float spread = std::min(L.spawnSpreadX, maxSpread);
		box.position = Vec3(L.beltX + randomFloat(-spread, spread),
			firstBox_ ? L.firstBoxY : L.beltBeginY,
			L.beltTopZ + L.boxHeight * 0.5f);
		box.yaw = 0.0f;
		box.stopJitter = randomFloat(-L.stopJitterY, L.stopJitterY);
		box.claimedBy = 0;					//新货物还没人认领
		firstBox_ = false;

		const Vec3 palette[5] = {
			Vec3(0.85f, 0.66f, 0.38f), Vec3(0.78f, 0.55f, 0.32f), Vec3(0.90f, 0.76f, 0.52f),
			Vec3(0.70f, 0.58f, 0.40f), Vec3(0.82f, 0.71f, 0.45f)
		};
		box.color = palette[static_cast<int>(randomFloat(0.0f, 4.99f))];
		box.state = Box::State::Belt;
		boxes_.push_back(box);
	}

	void ConveyorScenario::updateBelt(float dt) {
		const Layout& L = layout_;

		spawnTimer_ -= dt;
		if (spawnTimer_ <= 0.0f) {
			spawnTimer_ = randomFloat(spawnMin_, std::max(spawnMin_, spawnMax_));
			if (onBeltCount() < 8) {
				spawnBox();
			}
		}

		//---------- 前进 ----------
		//被认领的货物：减速停到「认领它的那台臂」的工位（认领即挡停）；
		//没被认领的：继续往前走，去下游工位碰运气；
		//同时按「前车」限制后车，避免把货停在自己工位时被后面撞上。
		std::vector<Box*> line;
		for (Box& box : boxes_) {
			if (box.state == Box::State::Belt) {
				line.push_back(&box);
			}
		}
		std::sort(line.begin(), line.end(), [](const Box* a, const Box* b) {
			return a->position.y() > b->position.y();
		});

		float leaderY = 1e9f;
		float leaderHalf = 0.0f;
		for (Box* box : line) {
			const float before = box->position.y();
			float target = L.beltEndY + 500.0f;				//没被认领就一直走
			if (const Arm* owner = armById(box->claimedBy)) {
				target = owner->pickY + box->stopJitter;	//认领者的工位（带一点停位抖动）
			}
			const float maxY = leaderY - (leaderHalf + L.queueGap + box->size.y() * 0.5f);
			const float goal = std::min(target, maxY);

			if (dynamicPick_ || box->claimedBy == 0) {
				box->position.y() += beltSpeed_ * dt;		//动态抓取 / 没人认领：不停
			}
			else if (box->position.y() < goal) {
				box->position.y() = std::min(goal, box->position.y() + beltSpeed_ * dt);
			}
			if (box->position.y() > goal) {
				box->position.y() = goal;					//被前车挤住时退到安全距离
			}

			//统计：越过某台臂的工位这么远还没人认领 → 记一次「从工位前溜走」
			for (Arm& arm : arms_) {
				const float limit = arm.pickY + L.pickMissWindowY;
				if (before <= limit && box->position.y() > limit && box->claimedBy == 0) {
					arm.missed++;
				}
			}
			leaderY = box->position.y();
			leaderHalf = box->size.y() * 0.5f;
		}

		boxes_.erase(std::remove_if(boxes_.begin(), boxes_.end(), [&L](const Box& box) {
			return box.state == Box::State::Belt && box.position.y() > L.beltEndY;
			}), boxes_.end());
	}

	int ConveyorScenario::onBeltCount() const {
		int count = 0;
		for (const Box& box : boxes_) {
			if (box.state == Box::State::Belt) {
				count++;
			}
		}
		return count;
	}

	Box* ConveyorScenario::findBox(int id) {
		for (Box& box : boxes_) {
			if (box.id == id) {
				return &box;
			}
		}
		return nullptr;
	}

	const Box* ConveyorScenario::findBox(int id) const {
		for (const Box& box : boxes_) {
			if (box.id == id) {
				return &box;
			}
		}
		return nullptr;
	}

	bool ConveyorScenario::findFreeSlot(Arm& arm, int& slot, int& layer) const {
		for (int l = 0; l < layout_.layers; l++) {
			for (int s = 0; s < layout_.slotCount; s++) {
				if (!arm.occupied[l][s]) {
					slot = s;
					layer = l;
					return true;
				}
			}
		}
		return false;
	}

	const Box* ConveyorScenario::leadBoxForTracking(const Arm& arm) const {
		//（保留给自检/外部查询：看这台臂当前锁着哪件货）
		const Layout& L = layout_;
		const Box* lead = nullptr;
		for (const Box& box : boxes_) {
			if (box.state != Box::State::Belt || box.claimedBy != arm.id) {
				continue;
			}
			if (box.position.y() > arm.pickY + L.pickWindowY) {
				continue;
			}
			if (!lead || box.position.y() > lead->position.y()) {
				lead = &box;
			}
		}
		return lead;
	}

	float ConveyorScenario::horizontalError(const Arm& arm, const Box& box) const {
		const Eigen::Vector4f tcp = arm.robot->FK();
		const float dx = tcp(0) - box.position.x();
		const float dy = tcp(1) - box.position.y();
		return std::sqrt(dx * dx + dy * dy);
	}

	void ConveyorScenario::updateHeldBoxes() {
		for (Arm& arm : arms_) {
			const Eigen::Matrix4f tcp = arm.robot->FK4();
			const Vec3 position(tcp(0, 3), tcp(1, 3), tcp(2, 3));
			const Vec3 toolZ(tcp(0, 2), tcp(1, 2), tcp(2, 2));
			for (Box& box : boxes_) {
				if (box.state == Box::State::Held && box.claimedBy == arm.id) {
					box.position = position + toolZ * (box.size.z() * 0.5f);
					box.yaw = 0.0f;
				}
			}
		}
	}

	void ConveyorScenario::fail(Arm& arm, const std::string& reason) {
		arm.warning = true;
		arm.phase = Phase::Fault;
		arm.message = reason;
	}

	void ConveyorScenario::setSpawnRange(float minSeconds, float maxSeconds) {
		spawnMin_ = clampf(minSeconds, 0.4f, 20.0f);
		spawnMax_ = clampf(std::max(minSeconds, maxSeconds), 0.4f, 30.0f);
	}

	//=================================================================================
	// 单台机械臂的状态机
	//=================================================================================
	void ConveyorScenario::updateArm(Arm& arm, float dt) {
		const Layout& L = layout_;
		Robot& robot = *arm.robot;
		arm.retryTimer = std::max(0.0f, arm.retryTimer - dt);	//申请被拒后的退避
		if (arm.skipBoxId != 0) {
			//那件货已经走远/被拿走了，清掉「跳过」标记
			const Box* skipped = findBox(arm.skipBoxId);
			if (!skipped || skipped->state != Box::State::Belt ||
				std::fabs(skipped->position.y() - arm.pickY) > layout_.claimWindowY + 60.0f) {
				arm.skipBoxId = 0;
			}
		}

		switch (arm.phase) {
		case Phase::Idle: {
			bool holding = false;
			for (const Box& box : boxes_) {
				if (box.state == Box::State::Held && box.claimedBy == arm.id) {
					holding = true;
					break;
				}
			}
			if (holding) {
				if (findFreeSlot(arm, arm.placeSlot, arm.placeLayer)) {
					beginMove(arm, arm.slots[arm.placeSlot].up, 2.0f);
					arm.phase = Phase::ToPlace;
					arm.message = "继续码放";
				}
				break;
			}

			if (arm.dynamicPick) {
				//---------- 动态抓取：先申请(加锁)，没抢到就放弃这一件 ----------
				Box* candidate = findClaimableBox(arm);
				if (!candidate) {
					//窗口里没有空闲货物：若有一件已被别的臂锁定，仍然按协议去申请一次，
					//拿不到锁就放弃（这样「申请—被拒」的过程是可见的）
					Box* taken = findBoxInWindow(arm, 1);
					if (taken && arm.retryTimer <= 0.0f) {
						arm.retryTimer = 0.5f;
						if (!requestBox(arm.id, taken->id)) {
							arm.message = "货物 #" + std::to_string(taken->id) +
								" 已被其它机械臂锁定，申请被拒，放弃";
						}
					}
					break;
				}
				if (!requestBox(arm.id, candidate->id)) {
					arm.message = "抓取申请被拒（货物已被其它机械臂锁定），放弃这一件";
					arm.retryTimer = 0.5f;
					break;
				}

				//预测到位时刻，决定什么时候出发
				const Box* box = candidate;
				float travelTime = 0.0f;
				Eigen::Vector<float, 6> approach;
				bool solved = false;
				for (int iteration = 0; iteration < 3; iteration++) {
					const float predictedY = box->position.y() + beltSpeed_ * travelTime;
					if (!solveTopDownAt(arm, box->position.x(), predictedY,
						L.pickApproachZ, arm.joints, approach)) {
						break;
					}
					float maxDelta = 0.0f;
					for (int i = 0; i < 6; i++) {
						maxDelta = std::max(maxDelta, std::fabs(approach(i) - arm.joints(i)));
					}
					travelTime = maxDelta / std::max(0.2f, arm.trackingSpeed * arm.speedFactor);
					solved = true;
				}
				if (!solved) {
					releaseBox(arm.id, candidate->id);		//抢到了但解不出来：赶紧放锁
					arm.skipBoxId = candidate->id;
					arm.missed++;
					break;
				}
				//预测：现在出发的话，到达时货物在哪。
				//只要那时货物还没跑出取件窗口，就立刻出发（早出发才能跟得上），
				//至于「到得比货早」完全没关系 —— 剩下的距离由跟踪阶段补。
				//以前这里还有一个「不能早于工位 120mm」的下界，结果机械臂要等到货
				//离工位只剩 160mm 才肯动，之前每帧都在申请-放弃之间空转，出发太晚根本追不上。
				const float arrivalY = box->position.y() + beltSpeed_ * travelTime;
				if (arrivalY > arm.pickY + L.pickWindowY) {
					//预测都来不及了：放开给下游工位，并记下这件别再反复申请
					releaseBox(arm.id, candidate->id);
					arm.skipBoxId = candidate->id;
					break;
				}
				arm.pickBoxId = box->id;
				arm.wpPickUp = approach;
				arm.phase = Phase::ToPick;
				arm.trackError = 0.0f;
				arm.warning = false;
				arm.message = "已锁定货物 #" + std::to_string(box->id) + "，预测到位时刻并开始跟踪";
				arm.path.clear();			//新周期：轨迹从头记
				arm.plannedSegments = 0;
				break;
			}

			//---------- 定点抓取：货物经过我的工位时发出抓取申请 ----------
			Box* candidate = findClaimableBox(arm);
			if (!candidate) {
				//窗口里没有空闲货物：若有一件已被别的臂锁定，仍然按协议申请一次，被拒就放弃
				Box* taken = findBoxInWindow(arm, 1);
				if (taken && arm.retryTimer <= 0.0f) {
					arm.retryTimer = 0.5f;
					if (!requestBox(arm.id, taken->id)) {
						if (!arm.warning) {
							arm.message = "货物 #" + std::to_string(taken->id) +
								" 已被其它机械臂锁定，申请被拒，放弃";
						}
					}
				}
				break;
			}
			if (!requestBox(arm.id, candidate->id)) {
				arm.message = "抓取申请被拒（货物已被其它机械臂锁定），放弃这一件";
				arm.retryTimer = 0.5f;
				break;
			}

			//锁到手了：让它停到我的工位，并按它将停下的位置解算抓取位
			candidate->stopY = arm.pickY + candidate->stopJitter;
			std::string error;
			const Vec3 aim(candidate->position.x(), candidate->stopY,
				layout_.beltTopZ + layout_.boxHeight * 0.5f);
			Box aimBox = *candidate;
			aimBox.position.y() = candidate->stopY;
			if (!preparePickWaypoints(arm, aimBox, error)) {
				//解不出来：解锁让它去下游工位碰运气（不是把它删掉），并记下不再反复申请
				arm.message = "货物位置不可达（" + error + "），解锁让下游工位接手";
				arm.warning = true;
				arm.missed++;
				arm.skipBoxId = candidate->id;
				releaseBox(arm.id, candidate->id);
				break;
			}
			arm.pickBoxId = candidate->id;
			arm.skipBoxId = 0;
			arm.warning = false;
			beginMove(arm, arm.wpPickUp, 2.0f);
			arm.phase = Phase::ToPick;
			arm.message = "已锁定货物 #" + std::to_string(candidate->id) + "（横向偏移 " +
				std::to_string(static_cast<int>(std::lround(candidate->position.x() - L.beltX))) +
				" mm），前往工位";
			arm.path.clear();				//新周期：轨迹从头记
			arm.plannedSegments = 0;
			break;
		}
		case Phase::ToPick: {
			if (!arm.dynamicPick) {
				//先到接近位等它：货物被锁住后会停到我的工位（位置含这台货自己的停位抖动）
				const Box* box = findBox(arm.pickBoxId);
				const bool arrived = box && std::fabs(box->position.y() - box->stopY) < 3.0f;
				const bool moveDone = advanceMove(arm, dt);
				if (moveDone) {
					if (!box) {
						arm.phase = Phase::Idle;		//货物没了，兜底
						break;
					}
					if (!arrived) {
						//等太久（例如被前车挡住）就放弃，避免整条线堵死
						arm.waitTimer += dt;
						arm.message = "已到接近位，等待货物就位（还差 " +
							std::to_string(static_cast<int>(std::lround(box->position.y() - box->stopY))) +
							" mm）";
						if (arm.waitTimer > 6.0f) {
							releaseBox(arm.id, arm.pickBoxId);
							arm.missed++;
							arm.warning = true;
							arm.message = "货物迟迟不到位（可能被前车挡住），解锁放弃";
							beginMove(arm, arm.wpReady, 2.0f);
							arm.phase = Phase::Idle;
							arm.waitTimer = 0.0f;
						}
						break;
					}
					arm.waitTimer = 0.0f;
					beginMove(arm, arm.wpPickDown, 1.3f);
					arm.phase = Phase::Descend;
					arm.message = "下降取件";
				}
				break;
			}
			const Box* box = findBox(arm.pickBoxId);
			if (!box || box->position.y() > arm.pickY + L.pickWindowY + 120.0f) {
				releaseBox(arm.id, arm.pickBoxId);		//放弃了就把锁还回去
				beginMove(arm, arm.wpReady, 2.0f);
				arm.phase = Phase::Idle;
				arm.message = "货物已越过取件窗口，放弃这一件";
				arm.warning = true;
				break;
			}
			Eigen::Vector<float, 6> approach;
			if (solveTopDownAt(arm, box->position.x(), box->position.y(),
				L.pickApproachZ, arm.joints, approach)) {
				arm.wpPickUp = approach;
				steerTo(arm, approach, arm.trackingSpeed * arm.speedFactor, dt);
			}
			arm.trackError = horizontalError(arm, *box);
			const Eigen::Vector4f tcp = robot.FK();
			if (arm.trackError < graspTolerance_ * 2.0f &&
				std::fabs(tcp(2) - L.pickApproachZ) < 60.0f) {
				arm.phase = Phase::Descend;
				arm.message = "跟踪到位，俯冲抓取";
			}
			break;
		}
		case Phase::Descend: {
			if (!arm.dynamicPick) {
				if (!advanceMove(arm, dt)) {
					break;
				}
			}
			else {
				const Box* box = findBox(arm.pickBoxId);
				if (!box) {
					break;
				}
				const float graspZ = box->position.z() + L.boxHeight * 0.5f;
				Eigen::Vector<float, 6> grasp;
				if (solveTopDownAt(arm, box->position.x(), box->position.y(),
					graspZ, arm.joints, grasp)) {
					arm.wpPickDown = grasp;
					arm.lastPickOffsetX = box->position.x() - L.beltX;
					steerTo(arm, grasp, arm.trackingSpeed * arm.speedFactor, dt);
				}
				arm.trackError = horizontalError(arm, *box);
				const Eigen::Vector4f tcp = robot.FK();
				if (arm.trackError > graspTolerance_ ||
					std::fabs(tcp(2) - graspZ) > graspTolerance_ + 10.0f) {
					if (box->position.y() > arm.pickY + L.pickWindowY + 120.0f) {
						releaseBox(arm.id, arm.pickBoxId);	//追不上：还锁，让它流走
						beginMove(arm, arm.wpReady, 2.0f);
						arm.phase = Phase::Idle;
						arm.message = "没追上货物，放弃这一件";
						arm.warning = true;
					}
					break;
				}
			}
			float width = 140.0f;
			if (const Box* box = findBox(arm.pickBoxId)) {
				width = box->size.x();
				arm.graspError = horizontalError(arm, *box);	//合爪这一刻的偏差
			}
			arm.gripperTarget = width + 6.0f;
			arm.phase = Phase::Grip;
			arm.message = arm.dynamicPick ? "跟上货物，合爪" : "夹取货物";
			break;
		}
		case Phase::Grip: {
			if (std::fabs(arm.gripperOpening - arm.gripperTarget) > 1.0f) {
				break;
			}
			Box* box = findBox(arm.pickBoxId);
			if (!box) {
				beginMove(arm, arm.wpPickUp, 1.4f);
				arm.phase = Phase::Lift;
				break;
			}
			box->state = Box::State::Held;
			arm.picked++;
			beginMove(arm, arm.wpPickUp, 1.4f);
			arm.phase = Phase::Lift;
			arm.message = "抓取完成，提升";
			break;
		}
		case Phase::Lift:
			if (!advanceMove(arm, dt)) {
				break;
			}
			if (!findFreeSlot(arm, arm.placeSlot, arm.placeLayer)) {
				if (autoClear_) {
					//清空这台臂自己的收集台：占位与台上的箱子一起清
					for (int i = 0; i < L.layers; i++) {
						for (int s = 0; s < L.slotCount; s++) {
							arm.occupied[i][s] = false;
						}
					}
					const size_t before = boxes_.size();
					boxes_.erase(std::remove_if(boxes_.begin(), boxes_.end(),
						[&arm](const Box& box) {
							//码好的箱子仍然归这台臂所有（claimedBy 不会被清掉）
							return box.state == Box::State::Stacked && box.claimedBy == arm.id;
						}), boxes_.end());
					arm.cleared++;
					arm.message = "托盘已满，自动清空 " +
						std::to_string(static_cast<int>(before - boxes_.size())) + " 件后继续";
					if (!findFreeSlot(arm, arm.placeSlot, arm.placeLayer)) {
						beginMove(arm, arm.wpReady, 2.0f);
						arm.phase = Phase::Idle;
						break;
					}
				}
				else {
					arm.warning = true;
					arm.message = "收集台已满（" + std::to_string(capacity()) + " 件），请复位或勾选自动清空";
					arm.phase = Phase::Idle;
					break;
				}
			}
			beginMove(arm, arm.slots[arm.placeSlot].up, 2.0f);
			arm.phase = Phase::ToPlace;
			arm.message = "搬运到收集台";
			break;
		case Phase::ToPlace:
			if (advanceMove(arm, dt)) {
				beginMove(arm, arm.slots[arm.placeSlot].down[arm.placeLayer], 1.3f);
				arm.phase = Phase::Lower;
				arm.message = "码放第 " + std::to_string(arm.placeLayer + 1) + " 层";
			}
			break;
		case Phase::Lower:
			if (advanceMove(arm, dt)) {
				arm.gripperTarget = 240.0f;
				arm.phase = Phase::Release;
				arm.message = "释放货物";
			}
			break;
		case Phase::Release: {
			if (std::fabs(arm.gripperOpening - arm.gripperTarget) > 1.0f) {
				break;
			}
			for (Box& box : boxes_) {
				if (box.state == Box::State::Held && box.claimedBy == arm.id) {
					box.state = Box::State::Stacked;
					//注意：这里**不能**清 claimedBy —— 码好的箱子仍然归这台臂所有，
					//「托盘满后自动清空」和「移除/移动这台臂」都要靠它认出自己的货
					box.slot = arm.placeSlot;
					box.layer = arm.placeLayer;
					box.position = Vec3(arm.slotX[arm.placeSlot], arm.slotY[arm.placeSlot],
						L.palletTopZ + L.layerHeight * (arm.placeLayer + 0.5f));
					box.yaw = 0.0f;
					arm.occupied[arm.placeLayer][arm.placeSlot] = true;
					arm.stacked++;
					break;
				}
			}
			beginMove(arm, arm.slots[arm.placeSlot].up, 1.4f);
			arm.phase = Phase::Retreat;
			arm.message = "码放完成，返回";
			break;
		}
		case Phase::Retreat:
			if (advanceMove(arm, dt)) {
				beginMove(arm, arm.wpReady, 2.0f);
				arm.phase = Phase::Idle;
				arm.message = "等待下一个货物";
				arm.warning = false;
			}
			break;
		case Phase::Fault:
			break;
		}
	}

	//=================================================================================
	// 主循环
	//=================================================================================
	void ConveyorScenario::start() {
		running_ = true;
		warning_ = false;
		message_ = "场景运行中";
		for (Arm& arm : arms_) {
			if (arm.phase == Phase::Fault) {
				continue;			//故障提示不要被覆盖
			}
			arm.warning = false;
			arm.message = "场景运行中";
		}
	}

	void ConveyorScenario::pause() {
		running_ = false;
		message_ = "场景已暂停";
		for (Arm& arm : arms_) {
			if (arm.phase != Phase::Fault) {
				arm.message = "场景已暂停";
			}
		}
	}

	void ConveyorScenario::reset() {
		boxes_.clear();
		nextId_ = 1;
		firstBox_ = true;
		time_ = 0.0f;
		spawnTimer_ = 0.8f;
		warning_ = false;
		message_ = running_ ? "场景运行中" : "场景已复位";

		for (Arm& arm : arms_) {
			for (auto& row : arm.occupied) {
				for (auto& cell : row) {
					cell = false;
				}
			}
			arm.picked = arm.stacked = arm.cleared = arm.missed = 0;
			arm.claimed = arm.contended = 0;
			arm.phase = Phase::Idle;
			arm.pickBoxId = 0;
			arm.placeSlot = arm.placeLayer = 0;
			arm.gripperOpening = arm.gripperTarget = 240.0f;
			arm.lastPickOffsetX = 0.0f;
			arm.trackError = 0.0f;
			arm.path.clear();
			arm.planned.clear();
			arm.plannedPoses.clear();
			arm.usingTrackPlan = false;
			arm.plannedSegments = 0;
			arm.planTotal = 0;
			arm.pathRevision++;
			arm.warning = false;
			arm.message = running_ ? "场景运行中" : "场景已复位";
			if (arm.phase == Phase::Fault) {
				arm.message = "场景运行中";
			}

			std::string error;
			arm.waypointsReady = prepareWaypoints(arm, error);
			if (!arm.waypointsReady) {
				fail(arm, "工位预解算失败：" + error);
				continue;
			}
			arm.joints = arm.wpReady;
			arm.moveFrom = arm.moveTo = arm.joints;
			arm.moveT = 1.0f;
			applyJoints(arm);
		}
	}

	void ConveyorScenario::tick(float dt) {
		if (dt <= 0.0f) {
			return;
		}
		time_ += dt;
		if (!running_) {
			return;			//暂停时把机械臂交还给手动控制
		}

		updateBelt(dt);
		for (Arm& arm : arms_) {
			if (arm.phase == Phase::Fault) {
				continue;
			}
			updateGripper(arm, dt);
			updateArm(arm, dt);
			applyJoints(arm);
			recordPath(arm);
		}
		updateHeldBoxes();
	}

}
