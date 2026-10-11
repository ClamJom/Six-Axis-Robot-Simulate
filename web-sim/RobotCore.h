#pragma once

//=====================================================================================
// RobotCore.h
//
// 六轴机械臂的运动学模型（B/S 架构中的“逻辑层”）。
// 本文件不依赖 PCL / VTK / 任何界面库，只依赖 Eigen，便于被后端服务复用。
//
// 说明：本文件由原 Robot.h / Joint.h 整理而来，DH 变换、正解、逆解的算法与
// 数值行为完全保持不变，只是把显示相关代码（PCL 画线、轨迹绘制）剥离出去。
// 原有的中文注释已译写，便于与前端代码对照。
//=====================================================================================

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <vector>
#include <iostream>

//数学常量：MSVC 默认不定义 M_PI / M_PI_2，这里显式给出，保证跨编译器一致
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace robotcore {
	constexpr float kPi = static_cast<float>(M_PI);
	constexpr float kHalfPi = static_cast<float>(M_PI_2);
}

/// <summary>
/// 单个关节（一组 DH 参数）。
/// alpha 与 theta 在内部一律使用弧度；构造函数为方便书写仍接受“度”。
/// </summary>
class Joint {
public:
	float a = 0.0f;			//连杆长度
	float alpha = 0.0f;		//连杆扭角
	float d = 0.0f;			//连杆偏距
	float theta = 0.0f;		//关节角
	Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f IH = Eigen::Matrix4f::Identity();

	Joint() = default;

	/// <summary>
	/// 使用角度制初始化，便于直接填写 DH 参数表
	/// </summary>
	Joint(float a, float alphaDegree, float d, float thetaDegree) {
		//PCL 的旋转以顺时针为正，这里统一取反，使逆时针为正
		this->a = a;
		this->alpha = -alphaDegree * 2.0f * robotcore::kPi / 360.0f;
		this->d = d;
		this->theta = -thetaDegree * 2.0f * robotcore::kPi / 360.0f;
		H = getHFromDH(this->a, this->alpha, this->d, this->theta);
		IH = getHInverseFromDH(this->a, this->alpha, this->d, this->theta);
	}

	/// <summary>由 DH 参数计算变换矩阵</summary>
	static Eigen::Matrix4f getHFromDH(float a, float alpha, float d, float theta) {
		Eigen::Matrix4f H;
		H << cos(theta), -sin(theta), 0, a,
			cos(alpha) * sin(theta), cos(alpha) * cos(theta), -sin(alpha), -d * sin(alpha),
			sin(alpha) * sin(theta), sin(alpha) * cos(theta), cos(alpha), d * cos(alpha),
			0, 0, 0, 1;
		return H;
	}

	/// <summary>由 DH 参数计算逆变换矩阵</summary>
	static Eigen::Matrix4f getHInverseFromDH(float a, float alpha, float d, float theta) {
		Eigen::Matrix3f R;
		Eigen::Vector3f T;
		R << cos(theta), -sin(theta), 0,
			cos(alpha) * sin(theta), cos(alpha) * cos(theta), -sin(alpha),
			sin(alpha) * sin(theta), sin(alpha) * cos(theta), cos(alpha);
		T << a, -d * sin(alpha), d * cos(alpha);
		Eigen::Matrix3f RT = R.transpose();
		T = -RT * T;

		Eigen::Matrix4f H;
		H << RT(0, 0), RT(0, 1), RT(0, 2), T(0),
			RT(1, 0), RT(1, 1), RT(1, 2), T(1),
			RT(2, 0), RT(2, 1), RT(2, 2), T(2),
			0, 0, 0, 1;
		return H;
	}

	Eigen::Matrix4f getHFromTheta(float newTheta) const {
		return getHFromDH(a, alpha, d, newTheta);
	}

	Eigen::Matrix4f getHInverseFromTheta(float newTheta) const {
		return getHInverseFromDH(a, alpha, d, newTheta);
	}

	/// <summary>更新关节角（弧度）并同步变换矩阵</summary>
	void updateH(float newTheta) {
		theta = newTheta;
		H = getHFromTheta(newTheta);
		IH = getHInverseFromTheta(newTheta);
	}
};

/// <summary>
/// 六轴机械臂（也支持三轴形式，与原实现一致）。
/// 正解由基座逐级相乘到工具末端；逆解先求前三轴位置，再求后三轴姿态。
/// </summary>
class Robot {
public:
	int ID = 0;							//机械臂编号：前端用它区分、操作各台臂
	std::string Name = "机械臂";		//显示名（可在前端改名）
	int JointCount = 0;
	float tolerance = 0.00005f;			//容忍度：绝对值小于该值即认为为 0
	std::vector<Joint> joints;
	Eigen::Vector4f BasePosition = Eigen::Vector4f(0, 0, 0, 1);	//基座原点
	/// <summary>
	/// 基座在世界中的位姿（默认单位阵＝基座就在世界原点）。
	/// 场景里可以把第二台机械臂摆到别处（例如传送带另一侧），
	/// 正解会把整条链乘上它，逆解会先把世界位姿换算回基座坐标再解。
	/// </summary>
	Eigen::Matrix4f BaseTransform = Eigen::Matrix4f::Identity();
	Joint tool;							//工具（末端）的 DH 参数

	Robot(const std::vector<Joint>& jointList, const Eigen::Vector3f& basePosition) {
		JointCount = static_cast<int>(jointList.size());
		joints = jointList;
		BasePosition << basePosition(0), basePosition(1), basePosition(2), 1;
		for (int i = 0; i < JointCount; i++) {
			reachSpaceMaxLength += std::sqrt(joints[i].a * joints[i].a + joints[i].d * joints[i].d);
		}
		reachSpaceMinLength = std::sqrt(joints[0].a * joints[0].a + joints[0].d * joints[0].d) +
			std::sqrt(joints[1].a * joints[1].a + joints[1].d * joints[1].d) -
			std::sqrt(joints[2].a * joints[2].a + joints[2].d * joints[2].d);
		track.resize(trackSize);
	}

	//---------- 可达范围 ----------
	float getReachSpaceMaxLength() const { return reachSpaceMaxLength; }
	float getReachSpaceMinLength() const { return reachSpaceMinLength; }

	//---------- 轨迹记录 ----------
	int getTrackSize() const { return trackSize; }
	void setTrackSize(int size) {
		trackSize = (size > 0) ? size : 1;
		track.resize(trackSize);
	}
	void setTrackEnabled(bool enabled) { trackEnabled = enabled; }
	bool isTrackEnabled() const { return trackEnabled; }

	/// <summary>取回轨迹点（由逆解插值得到）</summary>
	std::vector<Eigen::Vector3f> getTrackPoints() const {
		std::vector<Eigen::Vector3f> points;
		points.reserve(track.size());
		for (const Eigen::Vector4f& p : track) {
			points.push_back(Eigen::Vector3f(p(0), p(1), p(2)));
		}
		return points;
	}

	/// <summary>取回逆解过程中记录的关节角序列（用于前端回放动画）</summary>
	const std::vector<Eigen::Vector<float, 6>>& getTrackPoses() const { return trackPoses; }

	/// <summary>按当前关节角重新计算轨迹（不改变当前位姿）</summary>
	void refreshTrack() {
		if (JointCount != 6) {
			return;
		}
		Eigen::Vector<float, 6> currentTheta;
		for (int i = 0; i < 6; i++) {
			currentTheta(i) = joints[i].theta;
		}
		getTrack(currentTheta);
	}

	/// <summary>
	/// 只做轨迹规划、**不改变当前关节角**：把「当前关节角 → targetJoints」的插值路径
	/// 记进 track / trackPoses，供显示、回放或外部逐点执行。
	///
	/// 与 IK(aimPose, true) 的区别：后者会顺手把关节角提交到目标位姿，
	/// 而场景需要自己控制推进节奏，所以只取规划结果、由自己按时间逐点执行。
	/// </summary>
	void planTrack(const Eigen::Vector<float, 6>& targetJoints) {
		getTrack(targetJoints);
	}

	//---------- 正解 ----------
	/// <summary>基座到工具末端的变换矩阵（含基座在世界中的位姿）</summary>
	Eigen::Matrix4f FK4() const {
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		for (int i = 0; i < JointCount; i++) {
			tempH *= joints[i].H;
		}
		tempH *= tool.H;
		return BaseTransform * tempH;
	}

	/// <summary>工具末端的位置</summary>
	Eigen::Vector4f FK() const {
		return FK4() * BasePosition;
	}

	/// <summary>
	/// 基座到第 index 个关节坐标系的变换（含基座位姿）。
	/// 注意这里**不含**工具：index == JointCount 表示第 6 轴法兰（即工具安装面），
	/// 工具末端（TCP）请用 FK4()。前端据此把「法兰→TCP」这一段画成工具而不是连杆。
	/// </summary>
	Eigen::Matrix4f getJointH(int index) const {
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		int count = (index > JointCount) ? JointCount : index;
		for (int i = 0; i < count; i++) {
			tempH *= joints[i].H;
		}
		return BaseTransform * tempH;
	}

	//---------- 逆解 ----------
	//
	// 本模型用的是「改进 DH」：H_i = Rot_x(α_i)·Trans_x(a_i)·Rot_z(θ_i)·Trans_z(d_i)。
	// 轴 4、5、6 的平移量都是 0，所以这三根轴交于同一点（球形手腕），
	// 于是逆解可以解析地拆成「位置（θ1θ2θ3）+ 姿态（θ4θ5θ6）」两步，
	// 而且解是**有限的多组**：
	//
	//     肩部 ±1  ×  肘部 ±1  ×  腕部 ±1  =  最多 8 组
	//
	// 旧实现只给了其中一组「特解」（位置部分固定一侧的 acos 分支，姿态部分
	// 用 acos 加符号判据），于是有两个毛病：
	//   1. 大量几何上可达的目标被判成不可达（可达区域呈碎片状）；
	//   2. θ4/θ6 在 ±180° 附近符号判据会翻转，同一竖直路径上相邻两点会解出
	//      +180° 与 -180° 两种等价解，插值时就表现为「姿态突然转 360°」。
	//
	// 现在的做法：
	//   1. 由腕心位置解 θ1θ2θ3（纯 atan2，连续无跳变）；
	//   2. 由 R03^T · R_target 解 θ4θ5θ6（θ5 取双分支，θ4/θ6 用 atan2）；
	//   3. 每组候选都用正解回代校验，只有真正到位姿的解才返回（自校验）；
	//   4. 选解时把关节角就近折算到参考构型（±2π），等价解之间不会来回跳。

	/// <summary>把角度归一化到 (-π, π]</summary>
	static float wrapToPi(float angle) {
		while (angle > robotcore::kPi) {
			angle -= 2.0f * robotcore::kPi;
		}
		while (angle <= -robotcore::kPi) {
			angle += 2.0f * robotcore::kPi;
		}
		return angle;
	}

	/// <summary>用给定关节角做一次正解（不改变模型状态，含基座位姿）</summary>
	Eigen::Matrix4f forwardFor(const Eigen::Vector<float, 6>& angles) const {
		Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
		for (int i = 0; i < JointCount; i++) {
			result *= joints[i].getHFromTheta(angles(i));
		}
		result *= tool.H;
		return BaseTransform * result;
	}

	/// <summary>关节 index 的旋转部分 Rot_x(α)·Rot_z(θ)</summary>
	Eigen::Matrix3f jointRotation(int index, float theta) const {
		const Joint& joint = joints[index];
		const float ca = std::cos(joint.alpha), sa = std::sin(joint.alpha);
		const float ct = std::cos(theta), st = std::sin(theta);
		Eigen::Matrix3f rotation;
		rotation << ct, -st, 0.0f,
			ca * st, ca * ct, -sa,
			sa * st, sa * ct, ca;
		return rotation;
	}

	/// <summary>逆解算法：完备解析解（新）或单一特解（旧，仅供对比）</summary>
	enum class IKMode {
		Complete,		//最多 8 组解，按参考构型就近选解
		Legacy			//旧版单一特解（acos + 符号判据），保留用于对比
	};

	/// <summary>当前逆解算法</summary>
	IKMode ikMode() const { return ikMode_; }

	/// <summary>切换逆解算法</summary>
	void setIKMode(IKMode mode) { ikMode_ = mode; }

	/// <summary>
	/// 枚举全部逆解（最多 8 组），每组都用正解回代校验过。
	/// 返回解的数量；0 表示目标不可达。
	/// </summary>
	int solveIKAll(const Eigen::Matrix4f& worldPose, std::vector<Eigen::Vector<float, 6>>& solutions) const {
		solutions.clear();
		if (JointCount != 6) {
			//三轴形式无法用下面这套「球形手腕」推导；旧实现的该分支会越界访问 joints[3]，
			//实际上也是不可用的，因此这里直接拒绝而不是给出错误结果。
			throw "IK only supports 6 joints";
		}
		if (!worldPose.allFinite()) {
			return 0;
		}
		if (ikMode_ == IKMode::Legacy) {
			Eigen::Vector<float, 6> pose;
			if (solveIKLegacy(worldPose, pose)) {
				solutions.push_back(pose);
			}
			return static_cast<int>(solutions.size());
		}

		//下面这套推导都在「基座坐标系」里做，因此先把世界位姿换算回来
		const Eigen::Matrix4f aimPose = BaseTransform.inverse() * worldPose;

		//---------- 参数 ----------
		const float d1 = joints[0].d;						//基座高度
		const float a2 = joints[1].a, d2 = joints[1].d;		//肩部连杆与偏置
		const float a3 = joints[2].a;						//大臂
		const float a4 = joints[3].a, d4 = joints[3].d;		//小臂（含与轴 4 的偏置）
		const float r4 = std::sqrt(a4 * a4 + d4 * d4);		//小臂等效长度
		const float r4sq = r4 * r4;

		//---------- 腕心（轴 4/5/6 的交点）----------
		const Eigen::Matrix4f wristPose = aimPose * tool.IH;
		const Eigen::Vector4f wristPoint = wristPose * BasePosition;
		const float wx = wristPoint(0), wy = wristPoint(1), wz = wristPoint(2);

		const float radiusSq = wx * wx + wy * wy;			//腕心到基座轴线的水平距离²
		if (radiusSq < d2 * d2) {
			return 0;										//比肩部偏置还近，够不着
		}
		const float shoulderOffset = std::sqrt(radiusSq - d2 * d2);
		const float phi = std::atan2(wy, wx);
		const float height = wz - d1;						//等价于 p1 的 z 分量

		const Eigen::Matrix3f targetRotation = aimPose.block<3, 3>(0, 0);

		//---------- 肩部 ±1 × 肘部 ±1 ----------
		for (int shoulder = 0; shoulder < 2; shoulder++) {
			const float p1x = shoulder ? -shoulderOffset : shoulderOffset;
			//p1 = R1^T·(W - t1)，其中 p1 的 y 分量恰好等于肩部偏置 d2：
			//  p1y = -r·sin(θ1-φ) = d2  →  θ1 = φ - atan2(d2, p1x)
			const float theta1 = phi - std::atan2(d2, p1x);
			if (!std::isfinite(theta1)) {
				continue;
			}

			//平面二连杆：把腕心换算到「轴 2 原点」坐标系后解 θ2、θ3
			const float px = p1x - a2;
			const float q = height;
			const float reachSq = px * px + q * q;			//= (a3 + A3)² + B3²
			const float A3 = (reachSq - a3 * a3 - r4sq) / (2.0f * a3);
			const float B3sq = r4sq - A3 * A3;
			if (B3sq < 0.0f) {
				continue;									//这一侧肘部够不到
			}
			const float B3abs = std::sqrt(B3sq);

			for (int elbow = 0; elbow < 2; elbow++) {
				const float B3 = elbow ? -B3abs : B3abs;
				const float u = a3 + A3;
				const float v = B3;
				const float den = u * u + v * v;
				if (den < 1e-6f) {
					continue;
				}

				//[A3; B3] = [[a4, -d4], [d4, a4]]·[c3; s3]
				const float c3 = (a4 * A3 + d4 * B3) / r4sq;
				const float s3 = (-d4 * A3 + a4 * B3) / r4sq;
				const float theta3 = std::atan2(s3, c3);

				//[px; q] = [[u, -v], [-v, -u]]·[c2; s2]
				const float c2 = (u * px - v * q) / den;
				const float s2 = (-v * px - u * q) / den;
				const float theta2 = std::atan2(s2, c2);

				if (!std::isfinite(theta2) || !std::isfinite(theta3)) {
					continue;
				}

				//---------- 姿态：R36 = R03^T · R_target ----------
				const Eigen::Matrix3f R03 =
					jointRotation(0, theta1) * jointRotation(1, theta2) * jointRotation(2, theta3);
				const Eigen::Matrix3f R36 = R03.transpose() * targetRotation;
				const float c5 = clampf(R36(1, 2), -1.0f, 1.0f);
				const float acos5 = std::acos(c5);

				for (int wristFlip = 0; wristFlip < 2; wristFlip++) {
					const float theta5 = wristFlip ? -acos5 : acos5;
					const float s5 = std::sin(theta5);
					float theta4 = 0.0f;
					float theta6 = 0.0f;

					if (std::fabs(s5) < 1e-4f) {
						//奇异位形：θ4 与 θ6 只以「和/差」的形式出现，θ4 取参考值
						const float reference4 = (joints.size() > 3) ? joints[3].theta : 0.0f;
						if (c5 > 0.0f) {
							//θ5 = 0：R36(0,0)=cos(θ4+θ6)，R36(0,1)=-sin(θ4+θ6)
							const float sum = std::atan2(-R36(0, 1), R36(0, 0));
							theta4 = reference4;
							theta6 = sum - theta4;
						}
						else {
							//θ5 = π：R36(0,0)=-cos(θ4-θ6)，R36(0,1)=-sin(θ4-θ6)
							const float diff = std::atan2(-R36(0, 1), -R36(0, 0));
							theta4 = reference4;
							theta6 = theta4 - diff;
						}
					}
					else {
						theta4 = std::atan2(-R36(2, 2) / s5, R36(0, 2) / s5);
						theta6 = std::atan2(R36(1, 1) / s5, -R36(1, 0) / s5);
					}

					Eigen::Vector<float, 6> candidate;
					candidate << theta1, theta2, theta3, theta4, theta5, theta6;
					if (!candidate.allFinite()) {
						continue;
					}

					//---------- 正解回代校验：只有真到位的解才留下 ----------
					//注意 forwardFor 给的是世界位姿，因此与世界位姿比较
					const Eigen::Matrix4f reached = forwardFor(candidate);
					const float positionError =
						(reached.block<3, 1>(0, 3) - worldPose.block<3, 1>(0, 3)).norm();
					const float rotationError =
						(reached.block<3, 3>(0, 0) - worldPose.block<3, 3>(0, 0)).norm();
					if (positionError > 0.1f || rotationError > 2e-3f) {
						continue;
					}
					solutions.push_back(candidate);
				}
			}
		}
		return static_cast<int>(solutions.size());
	}

	/// <summary>
	/// 只解算、不写回：在全部解里挑「离参考构型最近」的一组（关节角按 ±2π 就近折算）。
	/// 参考构型决定了选哪一组分支，因此连续给点时会得到连续的解，不会突然转 360°。
	/// </summary>
	/// <returns>是否有解；false 表示目标不可达</returns>
	bool solveIK(const Eigen::Matrix4f& aimPose, const Eigen::Vector<float, 6>& reference,
		Eigen::Vector<float, 6>& out) const {
		if (ikMode_ == IKMode::Legacy) {
			//旧算法原样返回（不做就近折算、不做回代校验），这样对比才公平
			return solveIKLegacy(aimPose, out);
		}
		std::vector<Eigen::Vector<float, 6>> solutions;
		if (solveIKAll(aimPose, solutions) == 0) {
			return false;
		}

		float bestScore = 0.0f;
		Eigen::Vector<float, 6> best;
		bool found = false;
		for (const Eigen::Vector<float, 6>& candidate : solutions) {
			Eigen::Vector<float, 6> wrapped;
			float score = 0.0f;
			for (int i = 0; i < 6; i++) {
				//等价解之间可能相差 2π，先折算到参考角附近再比较
				const float delta = wrapToPi(candidate(i) - reference(i));
				wrapped(i) = reference(i) + delta;
				score += delta * delta;
			}
			if (!found || score < bestScore) {
				found = true;
				bestScore = score;
				best = wrapped;
			}
		}
		out = best;
		return true;
	}

	/// <summary>以当前关节角为参考构型解算</summary>
	bool solveIK(const Eigen::Matrix4f& aimPose, Eigen::Vector<float, 6>& out) const {
		Eigen::Vector<float, 6> reference;
		for (int i = 0; i < 6; i++) {
			reference(i) = (i < JointCount) ? joints[i].theta : 0.0f;
		}
		return solveIK(aimPose, reference, out);
	}

	/// <summary>
	/// 逆向运动学：把工具末端对齐到 aimPose，并在解算过程中记录末端轨迹。
	///
	/// 解算是「全有或全无」的：先解出关节角并校验，确认有效之后才写回关节。
	/// 目标不可达时模型状态保持不变（否则 NaN 会污染关节角，之后所有解算都会失效）。
	/// </summary>
	/// <returns>是否解算成功；返回 false 表示目标不可达，模型状态保持不变</returns>
	bool IK(const Eigen::Matrix4f& aimPose, bool recordTrack = true) {
		Eigen::Vector<float, 6> pose;
		if (!solveIK(aimPose, pose)) {
			return false;
		}

		//轨迹是从「当前关节角」插值到新解的，因此必须在写回新解之前记录
		if (recordTrack && trackEnabled) {
			getTrack(pose);
		}
		for (int i = 0; i < JointCount; i++) {
			joints[i].updateH(pose(i));
		}
		return true;
	}

	/// <summary>是否处于零位（所有关节角为 0）</summary>
	void goHome() {
		for (int i = 0; i < JointCount; i++) {
			joints[i].updateH(0.0f);
		}
	}

private:
	float reachSpaceMaxLength = 0.0f;
	float reachSpaceMinLength = 0.0f;
	int trackSize = 20;						//轨迹记录点数量
	bool trackEnabled = true;
	IKMode ikMode_ = IKMode::Complete;		//默认用完备解析解
	std::vector<Eigen::Vector4f> track;		//轨迹点
	std::vector<Eigen::Vector<float, 6>> trackPoses;	//轨迹上的关节角

	static float clampf(float value, float low, float high) {
		return (value < low) ? low : ((value > high) ? high : value);
	}

	//=================================================================================
	// 旧版单一特解（保留用于对比，逻辑与改动前完全一致）
	//
	// 已知问题：
	//   * 肩部、肘部各只取一个分支，大量可达目标会被判成不可达；
	//   * θ4/θ6 用 acos + 符号判据，在 ±180° 附近会翻转，竖直运动时相邻两点
	//     会解出 +180° 与 -180° 两种等价解，插值表现为「姿态突然转 360°」；
	//   * 解出后不做正解回代校验，姿态是否正确无从判断。
	//=================================================================================

	/// <summary>旧版：只解一组解（肩部 + 肘部 + 腕部各取一支）</summary>
	bool solveIKLegacy(const Eigen::Matrix4f& worldPose, Eigen::Vector<float, 6>& out) const {
		//同样先换算到基座坐标系
		const Eigen::Matrix4f aimPose = BaseTransform.inverse() * worldPose;
		Eigen::Vector3f frontPose = calculatePosThetaLegacy(aimPose * tool.IH);
		Eigen::Vector3f backPose = calculatePoseThetaLegacy(aimPose, frontPose);
		Eigen::Vector<float, 6> pose;
		pose << frontPose(0), frontPose(1), frontPose(2), backPose(0), backPose(1), backPose(2);
		if (!worldPose.allFinite() || !pose.allFinite()) {
			return false;
		}
		out = pose;
		return true;
	}

	/// <summary>旧版：由轴 4 原点位置计算前三轴的角度</summary>
	Eigen::Vector3f calculatePosThetaLegacy(const Eigen::Matrix4f& pose) const {
		Eigen::Vector4f position4;
		position4 << pose(0, 3), pose(1, 3), pose(2, 3), 1;

		//计算 theta1
		const float dx = position4(1) - BasePosition(0);
		const float dy = position4(0) - BasePosition(1);
		float alpha = std::acos(joints[1].d / std::sqrt(dx * dx + dy * dy));
		float theta = std::atan2(position4(1), position4(0));
		float theta1 = alpha + theta - robotcore::kHalfPi;

		//通过 theta1 获取轴 2 原点的位置
		Eigen::Matrix4f H0 = joints[0].getHFromTheta(theta1);
		Eigen::Matrix4f H1 = joints[1].H;
		Eigen::Matrix4f tempH = H0 * H1;
		Eigen::Vector4f position2 = tempH * BasePosition;
		Eigen::Vector4f distance = position4 - position2;

		//求模长
		float d24 = distance.norm();
		float l2 = joints[2].a;
		float l3 = std::sqrt(joints[3].a * joints[3].a + joints[3].d * joints[3].d);

		//余弦定理得到 theta2、theta3；
		//由于轴 3 与轴 4 之间存在角度偏差，通过消除该偏差得到旋转角
		float offsetTheta3 = std::acos((joints[3].d * joints[3].d + l3 * l3 - joints[3].a * joints[3].a) /
			(2.0f * joints[3].d * l3));
		float theta3 = -std::acos((l2 * l2 + l3 * l3 - d24 * d24) / (2.0f * l2 * l3)) +
			offsetTheta3 + robotcore::kHalfPi;

		theta = std::acos((d24 * d24 + l2 * l2 - l3 * l3) / (2.0f * d24 * l2));
		float l2v = distance(2);
		float l3h = std::sqrt(distance(0) * distance(0) + distance(1) * distance(1));
		alpha = std::atan2(l2v, l3h);
		float theta2 = -alpha - theta;

		return Eigen::Vector3f(theta1, theta2, theta3);
	}

	/// <summary>旧版：计算后三轴的角度，使工具末端姿态与目标一致</summary>
	Eigen::Vector3f calculatePoseThetaLegacy(const Eigen::Matrix4f& pose,
		const Eigen::Vector3f& frontPose) const {
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		for (int i = 0; i < 3; i++) {
			tempH *= joints[i].getHFromTheta(frontPose(i));
		}
		tempH *= joints[3].getHFromTheta(0);

		Eigen::Matrix4f tempAimH = pose * tool.IH;
		float theta4 = 0.0f, theta5 = 0.0f, theta6 = 0.0f;
		Eigen::Vector3f A, B;

		A << tempH(0, 2), tempH(1, 2), tempH(2, 2);
		B << tempAimH(0, 2), tempAimH(1, 2), tempAimH(2, 2);
		float tempTheta = std::acos(A.dot(B) / A.norm() / B.norm());
		if (tempTheta == 0) {
			theta4 = 0;
			theta5 = 0;
		}
		else {
			A = A.cross(B);
			B << tempH(0, 1), tempH(1, 1), tempH(2, 1);
			theta4 = std::acos(A.dot(B) / A.norm() / B.norm());
			tempH *= joints[3].getHInverseFromTheta(0);
			tempH *= joints[3].getHFromTheta(theta4);

			//用变换后坐标系的 X 轴是否与 Z4、ZA 的正交向量垂直来判断 theta4 的符号
			Eigen::Vector3f X4 = tempH.block<3, 1>(0, 0);
			float r = X4.dot(A);
			if (std::fabs(r) > tolerance) {
				tempH *= joints[3].getHInverseFromTheta(theta4);
				theta4 = -theta4;
				tempH *= joints[3].getHFromTheta(theta4);
			}
			theta5 = tempTheta;
		}

		tempH *= joints[4].getHFromTheta(theta5);
		Eigen::Matrix4f H5 = tempH;
		tempH *= joints[5].getHFromTheta(0);

		A << tempH(0, 0), tempH(1, 0), tempH(2, 0);
		B << tempAimH(0, 0), tempAimH(1, 0), tempAimH(2, 0);
		theta6 = std::acos(A.dot(B) / A.norm() / B.norm());

		//与 theta4 同理：用变换后的 X 轴是否与目标 Y 轴正交来判断 theta6 的符号
		A = (H5 * joints[5].getHFromTheta(theta6)).block<3, 1>(0, 0);
		B = tempAimH.block<3, 1>(0, 1);
		float r = A.dot(B);
		if (std::fabs(r) > tolerance) {
			theta6 = -theta6;
		}
		return Eigen::Vector3f(theta4, theta5, theta6);
	}

	/// <summary>
	/// 记录由当前位姿过渡到新位姿的末端轨迹
	/// （同时记录沿途的关节角，供前端做轨迹回放动画）
	/// </summary>
	void getTrack(const Eigen::Vector<float, 6>& newPose) {
		const int steps = (trackSize > 0) ? trackSize : 1;
		Eigen::Vector<float, 6> dtheta;
		Eigen::Vector<float, 6> currentTheta;
		dtheta << 0, 0, 0, 0, 0, 0;
		currentTheta << 0, 0, 0, 0, 0, 0;
		auto normolize = [](float a) {
			// 将角度归一化到[0, 2 * M_PI]
			return std::fmod(a + 2.0f * M_PI, 2.0f * M_PI);
		};
		for (int i = 0; i < JointCount && i < 6; i++) {
			currentTheta(i) = joints[i].theta;
			float cP = normolize(currentTheta(i));
			float aP = normolize(newPose(i));
			// 计算以何种方式旋转路径最短
			if (std::fabs(cP - aP) > M_PI) {
				if (cP > aP) {
					dtheta(i) = (aP + 2.0f * M_PI - cP) / static_cast<float>(steps);
				}
				else {
					dtheta(i) = (aP - 2.0f * M_PI - cP) / static_cast<float>(steps);
				}
			}
			else {
				dtheta(i) = (aP - cP) / static_cast<float>(steps);
			}
		}

		track.clear();
		trackPoses.clear();
		track.reserve(steps);
		trackPoses.reserve(steps);

		for (int i = 0; i < JointCount; i++) {
			std::cout << currentTheta(i) << "->" << newPose(i) << ":" << dtheta(i) * static_cast<float>(steps) << "|";
		}
		std::cout << std::endl;

		//从起点开始逐步插值推入，最终恰好得到 steps 个点
		for (int i = 0; i < steps; i++) {
			Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
			for (int j = 0; j < JointCount; j++) {
				tempH *= joints[j].getHFromTheta(currentTheta(j));
			}
			tempH *= tool.H;
			track.push_back(BaseTransform * tempH * BasePosition);
			trackPoses.push_back(currentTheta);
			currentTheta += dtheta;
		}
	}
};
