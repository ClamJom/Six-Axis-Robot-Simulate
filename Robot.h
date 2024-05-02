#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>


#include "Joint.h"
#include "MyColor.h"

//六轴机械臂
class Robot {
public:
	int JointCount;
	float tolerance = 0.00005;		//容忍度，当绝对值小于该值是认为其为0
	std::vector<Joint>joints;
	Eigen::Vector4f BasePosition;	//机械臂基坐标原点
	Joint tool;
private:
	float reachSpaceMaxLength = 0;
	float reachSpaceMinLength = 0;
	int drawedTimes = 0;		//记录机械臂在视图中被绘制的次数，且防止ID冲突
	int trackSize = 100;		//轨迹记录点数量
	pcl::PointCloud<pcl::PointXYZ> track;	//轨迹点
public:
	Robot(std::vector<Joint> joints, Eigen::Vector3f BasePosition) {
		this->JointCount = joints.size();
		this->joints = joints;
		this->BasePosition << BasePosition(0), BasePosition(1), BasePosition(2), 1;
		for (int i = 0; i < JointCount; i++) {
			reachSpaceMaxLength += sqrt(pow(joints[i].a, 2) + pow(joints[i].d, 2));
		}
		reachSpaceMinLength = sqrt(pow(joints[0].a, 2) + pow(joints[0].d, 2)) +
			sqrt(pow(joints[1].a, 2) + pow(joints[1].d, 2)) - sqrt(pow(joints[2].a, 2) + pow(joints[2].d, 2));
		track.resize(trackSize);
	}

public:
	//获取轨迹记录点数量
	int getTrackSize() { return trackSize; }
	/// <summary>
	/// 重新设置轨迹记录点数量
	/// </summary>
	/// <param name="size"></param>
	void setTrackSize(int size) { trackSize = size; track.resize(size); }
	/// <summary>
	/// 画机械臂示意线
	/// </summary>
	void drawRobotLine(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, bool showTrack = true) {
		drawedTimes++;
		pcl::PointXYZ p1(BasePosition(0), BasePosition(1), BasePosition(2));
		pcl::PointXYZ p2;
		MyColor::CRGB rgb;
		MyColor::CHSV hsv(0, 100, 100);
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		Eigen::Vector4f tempP;
		std::string CurrentID = "T" + std::to_string(drawedTimes);
		for (int i = 0; i < JointCount; i++) {
			hsv.h = i * 360 / JointCount;
			MyColor::hsv2rgb(hsv, rgb);
			tempH *= joints[i].H;
			tempP = tempH * BasePosition;
			p2.x = tempP(0), p2.y = tempP(1), p2.z = tempP(2);
			viewer->addLine(p1, p2, rgb.r / 255, rgb.g / 255, rgb.b / 255, CurrentID + "A" + std::to_string(i));
			viewer->addSphere(p2, 10, CurrentID + "J" + std::to_string(i));
			/*if (i < 2) {
				drawCoordinateAtPointWithH(viewer, p2, tempH, CurrentID + "A" + std::to_string(i) + "C", 20.F);
			}*/
			if (i == 3) {
				drawCoordinateAtPointWithH(viewer, p2, tempH, CurrentID + "A" + std::to_string(i) + "C", 20.F);
			}
			if (i == 4) {
				drawCoordinateAtPointWithH(viewer, p2, tempH, CurrentID + "A" + std::to_string(i) + "C", 40.F);
			}
			p1 = p2;
		}
		tempH *= tool.H;
		tempP = tempH * BasePosition;
		p2.x = tempP(0), p2.y = tempP(1), p2.z = tempP(2);
		viewer->addLine(p1, p2, 1, 1, 1, CurrentID + "TA");
		viewer->addSphere(p2, 5, CurrentID + "TE");
		drawCoordinateAtPointWithH(viewer, p2, tempH, CurrentID + "TC");
		if (showTrack) {
			drawTrack(viewer, MyColor::CRGB(0, 255, 0), CurrentID + "Track");
		}
	}

	//从给定的三个点中获取姿态信息
	static Eigen::Matrix4f getHFrom3Points(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C) {
		Eigen::Vector3f AC = C - A;
		Eigen::Vector3f AB = B - A;
		Eigen::Vector3f Z = AB.cross(AC);
		Eigen::Vector3f X = AB;
		Eigen::Vector3f Y = Z.cross(X);
		//归一化
		X.normalize(); Y.normalize(); Z.normalize();
		Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
		//由旋转矩阵的性质得到变换矩阵，即：旋转矩阵的每一列都是相应的基底
		H.block<3, 3>(0, 0) << X, Y, Z;
		H(0, 3) = A(0); H(1, 3) = A(1); H(2, 3) = A(2);
		return H;
	}

	/// <summary>
	/// 正向运动学
	/// </summary>
	Eigen::Vector4f FK() {
		if (joints.size() == 0) {
			throw "Empty Joint List!!!";
		}
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		for (int i = 0; i < JointCount; i++) {
			tempH *= joints[i].H;
		}
		tempH *= tool.H;
		return tempH * BasePosition;
	}

	Eigen::Matrix4f FK(Eigen::Vector<float, 6> pose) {
		if (joints.size() == 0) {
			throw "Empty Joint List!!!";
		}
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		for (int i = 0; i < JointCount; i++) {
			tempH *= joints[i].getHFromTheta(pose(i));
		}
		tempH *= tool.H;
		return tempH;
	}

	/// <summary>
	/// 逆向运动学
	/// </summary>
	void IK(Eigen::Matrix4f aimPose, bool recordTrack = true) {
		if (joints.size() == 0) {
			throw "Empty Joint List!!!";
		}
		Eigen::Vector<float, 6> pose;
		if (JointCount == 3) {
			pose.resize(JointCount);
			Eigen::Vector3f newPose = calculatePosTheta(aimPose * tool.IH);
			pose << newPose(0), newPose(1), newPose(2);
		}
		else if (JointCount == 6) {
			pose.resize(JointCount);
			//从工具点转换至机械臂点
			Eigen::Vector3f fontPose = calculatePosTheta(aimPose * tool.IH);
			Eigen::Vector3f backPose = calculatePoseTheta(aimPose, fontPose);
			pose << fontPose(0), fontPose(1), fontPose(2), backPose(0), backPose(1), backPose(2);
		}
		if (recordTrack) {
			getTrack(pose);
		}
		for (int i = 0; i < JointCount; i++) {
			joints[i].updateH(pose(i));
		}
	}
private:
	//通过变换H画坐标系
	void drawCoordinateAtPointWithH(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointXYZ p, Eigen::Matrix4f H, std::string id = "coord", float scale = 10) {
		pcl::PointXYZ X(p.x + H(0, 0) * scale, p.y + H(1, 0) * scale, p.z + H(2, 0) * scale);
		pcl::PointXYZ Y(p.x + H(0, 1) * scale, p.y + H(1, 1) * scale, p.z + H(2, 1) * scale);
		pcl::PointXYZ Z(p.x + H(0, 2) * scale, p.y + H(1, 2) * scale, p.z + H(2, 2) * scale);
		viewer->addLine(p, X, 1, 0, 0, id + "X");
		viewer->addLine(p, Y, 0, 1, 0, id + "Y");
		viewer->addLine(p, Z, 0, 0, 1, id + "Z");
	}

	/// <summary>
	/// 获取机械臂末端运动轨迹信息
	/// </summary>
	///  <param name="newPose">新的角度</param>
	void getTrack(Eigen::Vector<float, 6> newPose) {
		if (newPose.size() < this->JointCount) {
			throw "Incorrect Pose Size";
		}
		Eigen::Vector<float, 6> dtheta;
		Eigen::Vector<float, 6> currentTheta;
		dtheta << 0, 0, 0, 0, 0, 0;
		for (int i = 0; i < 6; i++) {
			currentTheta(i) = this->joints[i].theta;
			dtheta(i) = (newPose(i) - this->joints[i].theta) / trackSize;
		}
		Eigen::Matrix4f tempH;
		Eigen::Vector4f tempP;
		tempH = FK(currentTheta);
		tempP = tempH * this->BasePosition;
		track.points[0] = pcl::PointXYZ(tempP(0), tempP(1), tempP(2));
		for (int i = 0; i < trackSize; i++) {
			currentTheta += dtheta;
			tempH = FK(currentTheta);
			tempP = tempH * this->BasePosition;
			track.points[i] = pcl::PointXYZ(tempP(0), tempP(1), tempP(2));
		}
	}

	void drawTrack(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, MyColor::CRGB color, std::string id = "Track") {
		for (int i = 1; i < track.size(); i++) {
			viewer->addLine(track.points[i - 1], track.points[i], color.r / 255, color.g / 255, color.b / 255, id + std::to_string(i));
		}
	}

	//通过轴4原点位置计算前三轴的姿态
	Eigen::Vector3f calculatePosTheta(Eigen::Matrix4f pose) {
		Eigen::Vector4f position4;
		position4 << pose(0, 3), pose(1, 3), pose(2, 3), 1;
		//计算theta1
		float theta1;
		float alpha = acos(joints[1].d / sqrt(pow(position4(1) - BasePosition(0), 2) + pow(position4(0) - BasePosition(1), 2)));
		float theta = atan2f(position4(1), position4(0));
		theta1 = alpha + theta - M_PI_2;
		//通过theta1获取轴2原点的位置
		Eigen::Matrix4f H0 = joints[0].getHFromTheta(theta1);
		//应用新的变换使P2、P3、P4共面
		Eigen::Matrix4f H1 = joints[1].H;
		Eigen::Matrix4f tempH = H0 * H1;
		Eigen::Vector4f position2 = tempH * this->BasePosition;
		Eigen::Vector4f distance = position4 - position2;
		//求模长
		float d24 = distance.norm();
		float l2 = joints[2].a, l3 = sqrt(pow(joints[3].a, 2) + pow(joints[3].d, 2));
		//余弦定理得到theta2，theta3
		//由于轴3与轴4之间存在角度偏差，通过消除偏差得到旋转角
		float offsetTheta3 = acos((pow(joints[3].d, 2) + pow(l3, 2) - pow(joints[3].a, 2)) / (2 * joints[3].d * l3));
		float theta3 = -acos((pow(l2, 2) + pow(l3, 2) - pow(d24, 2)) / (2 * l2 * l3)) + offsetTheta3 + M_PI_2;
		theta = acos((pow(d24, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * d24 * l2));
		float theta2;
		l2 = distance(2);
		l3 = sqrt(pow(distance(0), 2) + pow(distance(1), 2));
		alpha = atan2(l2, l3);
		theta2 = -alpha - theta;
		return Eigen::Vector3f(theta1, theta2, theta3);
	}

	//计算后三轴的姿态
	Eigen::Vector3f calculatePoseTheta(Eigen::Matrix4f pose, Eigen::Vector3f fontPose) {
		Eigen::Matrix4f tempH = Eigen::Matrix4f::Identity();
		for (int i = 0; i < 3; i++) {
			tempH *= joints[i].getHFromTheta(fontPose(i));
		}
		tempH *= joints[3].getHFromTheta(0);
		Eigen::Matrix4f tempAimH = pose * tool.IH;
		float theta4 = 0, theta5 = 0, theta6 = 0;
		//用于计算夹角
		Eigen::Vector3f A, B;

		A << tempH(0, 2), tempH(1, 2), tempH(2, 2);
		B << tempAimH(0, 2), tempAimH(1, 2), tempAimH(2, 2);
		float tempTheta = acos(A.dot(B) / A.norm() / B.norm());
		if (tempTheta == 0) {
			theta4 = 0;
			theta5 = 0;
		}
		else {
			A = A.cross(B);
			B << tempH(0, 1), tempH(1, 1), tempH(2, 1);
			theta4 = acos(A.dot(B) / A.norm() / B.norm());
			tempH *= joints[3].getHInverseFromTheta(0);
			tempH *= joints[3].getHFromTheta(theta4);
			Eigen::Vector3f X4 = tempH.block<3, 1>(0, 0);
			//通过判断通过Theta4变换得到的坐标系X轴是否与Z4、ZA的正交向量垂直来判断是否为正确的角度
			//若不正交则表示Theta4应当取反
			float r = X4.dot(A);
			if (abs(r) > tolerance) {
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
		theta6 = acos(A.dot(B) / A.norm() / B.norm());
		//与Theta4类似，由于无法判断acos得到的角度的正负，因此还需要判断变换后的坐标系X轴是否与目标Y轴正交来取符号
		A = (H5 * joints[5].getHFromTheta(theta6)).block<3, 1>(0, 0);
		B = tempAimH.block<3, 1>(0, 1);
		float r = A.dot(B);
		if (abs(r) > tolerance) {
			theta6 = -theta6;
		}
		return Eigen::Vector3f(theta4, theta5, theta6);
	}
};