#pragma once

//=====================================================================================
// RobotUtils.h
//
// 机器人模型工厂与通用工具（B/S 架构中的逻辑层，不依赖任何界面库）。
//=====================================================================================

#include "RobotCore.h"

#include <string>
#include <vector>

namespace RobotUtils {

	/// <summary>
	/// 创建本项目使用的机械臂：博朗特 BRTIRUS0805A，附带 119 mm 的工具。
	/// 服务端与自检代码都通过它构造模型，保证两边完全一致。
	/// </summary>
	Robot createDefaultRobot();

	/// <summary>
	/// 由位置与两条轴向确定一个位姿矩阵（列向量为 X/Y/Z 基向量）。
	/// zAxis 为目标坐标系的 Z 轴方向，xReference 仅用于确定 X 轴所在的平面。
	/// </summary>
	Eigen::Matrix4f makePose(const Eigen::Vector3f& position, const Eigen::Vector3f& zAxis,
		const Eigen::Vector3f& xReference);

	/// <summary>
	/// 由欧拉角与平移构造位姿矩阵（roll 绕 X，pitch 绕 Y，yaw 绕 Z，单位为弧度）
	/// </summary>
	Eigen::Matrix4f makePoseFromEuler(float roll, float pitch, float yaw,
		float x, float y, float z);

	/// <summary>
	/// 当目标的 Z 轴指向机械臂本体时把它取反（同时取反 X 轴，保证仍是右手系）。
	/// 我们希望机械臂末端是“朝向目标”的，而不是越过目标再折返回来。
	/// </summary>
	/// <returns>是否发生了反向</returns>
	bool flipZAxisIfFacingRobot(Eigen::Matrix4f& pose);

	/// <summary>把位姿矩阵拆成前端需要的行主序 16 个元素</summary>
	std::vector<float> toRowMajor(const Eigen::Matrix4f& matrix);

	/// <summary>把行主序 16 个元素还原为位姿矩阵</summary>
	Eigen::Matrix4f fromRowMajor(const std::vector<float>& values);

	/// <summary>弧度转角度</summary>
	float toDegree(float radian);

	/// <summary>角度转弧度</summary>
	float toRadian(float degree);

	//---------- 测试点云与有向包围盒（OBB） ----------

	/// <summary>点云中的一个点</summary>
	struct Point3 {
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
	};

	/// <summary>
	/// 生成一个类似“蝴蝶结”的双锥面测试点云，用于演示 OBB
	/// </summary>
	std::vector<Point3> makeTestCloud(int firstCount = 200, int secondCount = 300);

	/// <summary>把点云按给定位姿变换</summary>
	std::vector<Point3> transformCloud(const std::vector<Point3>& cloud, const Eigen::Matrix4f& transform);

	/// <summary>有向包围盒结果</summary>
	struct OBB {
		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();	//位姿（平移为重心，旋转为主轴方向）
		Eigen::Vector3f minPoint = Eigen::Vector3f::Zero();	//包围盒自身坐标系下的最小点
		Eigen::Vector3f maxPoint = Eigen::Vector3f::Zero();	//包围盒自身坐标系下的最大点
		Eigen::Vector3f extents = Eigen::Vector3f::Zero();	//三边长度
		Eigen::Vector3f massCenter = Eigen::Vector3f::Zero();
		bool valid = false;
	};

	/// <summary>
	/// 计算点云的有向包围盒（基于惯性主轴，返回的旋转矩阵一定是正交的）
	/// </summary>
	OBB computeOBB(const std::vector<Point3>& cloud);
}
