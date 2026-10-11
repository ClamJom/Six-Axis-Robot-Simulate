//=====================================================================================
// RobotUtils.cpp
//=====================================================================================

#include "RobotUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace RobotUtils {

	Robot createDefaultRobot() {
		//此处使用的机械臂具体型号为博朗特 BRTIRUS0805A，内容为 DH 参数
		//为保证逆解正确，保证 2、3、4 轴坐标系原点形成的平面垂直于轴 1 的 X-Y 平面
		std::vector<Joint> links;
		links.push_back(Joint(0, 0, 155.5f, 0));
		links.push_back(Joint(75.95f, 90, 7.05f, 90));
		links.push_back(Joint(390, 0, 0, 0));
		links.push_back(Joint(117.5f, 90, 394, 0));
		links.push_back(Joint(0, 90, 0, 0));
		links.push_back(Joint(0, -90, 0, 0));

		Robot robot(links, Eigen::Vector3f(0, 0, 0));
		robot.tool = Joint(0, 0, 119, 0);
		robot.setTrackSize(24);
		return robot;
	}

	Eigen::Matrix4f makePose(const Eigen::Vector3f& position, const Eigen::Vector3f& zAxis,
		const Eigen::Vector3f& xReference) {
		Eigen::Vector3f z = zAxis.normalized();
		if (!z.allFinite() || z.norm() < 1e-6f) {
			z = Eigen::Vector3f(0, 0, 1);
		}
		Eigen::Vector3f x = xReference - z * xReference.dot(z);
		if (x.norm() < 1e-6f) {
			//参考轴与 Z 轴共线时换一个参考方向，保证 X 轴仍然可解
			Eigen::Vector3f fallback(1, 0, 0);
			if (std::fabs(z(0)) > 0.9f) {
				fallback = Eigen::Vector3f(0, 1, 0);
			}
			x = fallback - z * fallback.dot(z);
		}
		x.normalize();
		Eigen::Vector3f y = z.cross(x);

		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		pose.block<3, 1>(0, 0) = x;
		pose.block<3, 1>(0, 1) = y;
		pose.block<3, 1>(0, 2) = z;
		pose.block<3, 1>(0, 3) = position;
		return pose;
	}

	Eigen::Matrix4f makePoseFromEuler(float roll, float pitch, float yaw,
		float x, float y, float z) {
		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		Eigen::Quaternionf quaternion =
			Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
		pose.block<3, 3>(0, 0) = quaternion.matrix();
		pose(0, 3) = x;
		pose(1, 3) = y;
		pose(2, 3) = z;
		return pose;
	}

	bool flipZAxisIfFacingRobot(Eigen::Matrix4f& pose) {
		Eigen::Vector3f endPosition(pose(0, 3), pose(1, 3), pose(2, 3));
		Eigen::Vector3f zAxis(pose(0, 2), pose(1, 2), pose(2, 2));
		//只比较水平方向：Z 轴若指向机械臂中轴，说明末端是“背对”目标的
		float dot = zAxis.dot(endPosition) - zAxis(2) * endPosition(2);
		if (dot >= 0) {
			return false;
		}
		//绕 Y 轴转 180°：Z 与 X 同时取反。
		//注意不能只把 Z 取反 —— 那样旋转矩阵的行列式会变成 -1（左手系），
		//已经不是合法位姿了，逆解校验会（正确地）判为不可达。
		zAxis = -zAxis;
		pose.block<3, 1>(0, 2) = zAxis;
		pose.block<3, 1>(0, 0) = -pose.block<3, 1>(0, 0);
		return true;
	}

	std::vector<float> toRowMajor(const Eigen::Matrix4f& matrix) {
		std::vector<float> values(16, 0.0f);
		for (int row = 0; row < 4; row++) {
			for (int column = 0; column < 4; column++) {
				values[row * 4 + column] = matrix(row, column);
			}
		}
		return values;
	}

	Eigen::Matrix4f fromRowMajor(const std::vector<float>& values) {
		Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
		if (values.size() < 16) {
			return matrix;
		}
		for (int row = 0; row < 4; row++) {
			for (int column = 0; column < 4; column++) {
				matrix(row, column) = values[row * 4 + column];
			}
		}
		return matrix;
	}

	float toDegree(float radian) {
		return radian * 180.0f / static_cast<float>(M_PI);
	}

	float toRadian(float degree) {
		return degree * static_cast<float>(M_PI) / 180.0f;
	}

	//=================================================================================
	// 测试点云与有向包围盒
	//=================================================================================
	std::vector<Point3> makeTestCloud(int firstCount, int secondCount) {
		//构造一个形状不规则、三边尺度明显不同的点云：
		//细长螺旋 + 倾斜圆盘 + 立方体角点，使得惯性主轴明确、包围盒有意义。
		std::vector<Point3> cloud;
		cloud.reserve(static_cast<size_t>(firstCount) + secondCount + 200);

		//第一段：沿 Z 轴略微扩张的螺旋线（细长）
		for (int i = 0; i < firstCount; i++) {
			const float t = static_cast<float>(i) / static_cast<float>(firstCount);
			const float angle = t * 14.0f * static_cast<float>(M_PI);
			const float radius = 10.0f + 40.0f * t;
			Point3 point;
			point.x = radius * std::cos(angle);
			point.y = radius * std::sin(angle);
			point.z = -60.0f + 120.0f * t;
			cloud.push_back(point);
		}

		//第二段：倾斜的圆盘（扁平）
		for (int i = 0; i < secondCount; i++) {
			const float t = static_cast<float>(i) / static_cast<float>(secondCount);
			const float angle = t * 2.0f * static_cast<float>(M_PI) * 5.0f;
			const float radius = 95.0f * std::sqrt(t);
			Point3 point;
			point.x = radius * std::cos(angle);
			point.y = radius * std::sin(angle);
			point.z = 0.12f * point.x - 0.08f * point.y;	//略微倾斜
			cloud.push_back(point);
		}

		//第三段：立方体的顶点与棱上采样点（提供明确的三维轮廓）
		const float half = 80.0f;
		for (int i = 0; i <= 8; i++) {
			for (int j = 0; j <= 8; j++) {
				Point3 point;
				point.x = -half + (2.0f * half) * (i / 8.0f);
				point.y = -half + (2.0f * half) * (j / 8.0f);
				point.z = (i % 2 == 0) ? -half : half;
				cloud.push_back(point);
			}
		}
		return cloud;
	}

	std::vector<Point3> transformCloud(const std::vector<Point3>& cloud, const Eigen::Matrix4f& transform) {
		std::vector<Point3> result;
		result.reserve(cloud.size());
		for (const Point3& point : cloud) {
			Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
			Eigen::Vector4f q = transform * p;
			result.push_back(Point3{ q(0), q(1), q(2) });
		}
		return result;
	}

	OBB computeOBB(const std::vector<Point3>& cloud) {
		OBB result;
		if (cloud.size() < 4) {
			return result;
		}

		//---------- 重心 ----------
		Eigen::Vector3f center = Eigen::Vector3f::Zero();
		for (const Point3& point : cloud) {
			center += Eigen::Vector3f(point.x, point.y, point.z);
		}
		center /= static_cast<float>(cloud.size());
		result.massCenter = center;

		//---------- 协方差矩阵 ----------
		Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
		for (const Point3& point : cloud) {
			Eigen::Vector3f d = Eigen::Vector3f(point.x, point.y, point.z) - center;
			covariance += d * d.transpose();
		}
		covariance /= static_cast<float>(cloud.size());

		//---------- 惯性主轴 ----------
		//协方差矩阵是实对称矩阵，用自伴随特征分解可以得到正交的特征向量组，
		//因此构造出来的旋转矩阵一定是正交的（与原 PCL 实现的结论一致）
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
		if (solver.info() != Eigen::Success) {
			return result;
		}
		Eigen::Vector3f eigenvalues = solver.eigenvalues();		//升序
		Eigen::Matrix3f eigenvectors = solver.eigenvectors();	//每列一个特征向量

		//按方差从大到小排列，作为包围盒的 X(长) / Y(宽) / Z(高) 方向
		Eigen::Vector3f axisX = eigenvectors.col(2);
		Eigen::Vector3f axisY = eigenvectors.col(1);
		Eigen::Vector3f axisZ = eigenvectors.col(0);

		//保证是右手系
		if (axisX.cross(axisY).dot(axisZ) < 0) {
			axisZ = -axisZ;
		}

		//---------- 在主轴坐标系下求包围盒 ----------
		float minX = std::numeric_limits<float>::max(), maxX = -minX;
		float minY = minX, maxY = -minX;
		float minZ = minX, maxZ = -minX;
		for (const Point3& point : cloud) {
			Eigen::Vector3f d = Eigen::Vector3f(point.x, point.y, point.z) - center;
			float px = d.dot(axisX);
			float py = d.dot(axisY);
			float pz = d.dot(axisZ);
			minX = std::min(minX, px); maxX = std::max(maxX, px);
			minY = std::min(minY, py); maxY = std::max(maxY, py);
			minZ = std::min(minZ, pz); maxZ = std::max(maxZ, pz);
		}

		//把包围盒中心平移到真正的几何中心，而不是点云重心
		Eigen::Vector3f offset = center +
			axisX * ((minX + maxX) * 0.5f) +
			axisY * ((minY + maxY) * 0.5f) +
			axisZ * ((minZ + maxZ) * 0.5f);

		Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		pose.block<3, 1>(0, 0) = axisX;
		pose.block<3, 1>(0, 1) = axisY;
		pose.block<3, 1>(0, 2) = axisZ;
		pose.block<3, 1>(0, 3) = offset;

		result.pose = pose;
		result.minPoint = Eigen::Vector3f(minX, minY, minZ);
		result.maxPoint = Eigen::Vector3f(maxX, maxY, maxZ);
		result.extents = Eigen::Vector3f(maxX - minX, maxY - minY, maxZ - minZ);
		result.valid = true;
		return result;
	}
}
