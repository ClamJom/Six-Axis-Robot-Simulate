#pragma once
#include <cmath>
#include <pcl/io/pcd_io.h>
class Joint {
public:
	float a, alpha, d, theta;
	Eigen::Matrix4f H, IH;
	Joint() {
		a = 0; alpha = 0; d = 0; theta = 0;
		H = getHFromDH(this->a, this->alpha, this->d, this->theta);
		IH = getHInverseFromDH(this->a, this->alpha, this->d, this->theta);
	}
	/// <summary>
	/// 初始化时使用角度制（便于理解）
	/// </summary>
	/// <param name="a"></param>
	/// <param name="alpha"></param>
	/// <param name="d"></param>
	/// <param name="theta"></param>
	Joint(float a, float alpha, float d, float theta) {
		//由于PCl旋转为顺时针为正，故此处取反得到逆时针为正
		this->a = a;
		this->alpha = -alpha * 2 * M_PI / 360;
		this->d = d;
		this->theta = -theta * 2 * M_PI / 360;
		H = getHFromDH(this->a, this->alpha, this->d, this->theta);
		IH = getHInverseFromDH(this->a, this->alpha, this->d, this->theta);
	}

public:
	/// <summary>
	/// 通过DH参数获取变换矩阵
	/// </summary>
	/// <param name="a"></param>
	/// <param name="alpha"></param>
	/// <param name="d"></param>
	/// <param name="theta"></param>
	/// <returns></returns>
	static Eigen::Matrix4f getHFromDH(float a, float alpha, float d, float theta) {
		Eigen::Matrix4f H;
		H << cos(theta), -sin(theta), 0, a,
			cos(alpha)* sin(theta), cos(alpha)* cos(theta), -sin(alpha), -d * sin(alpha),
			sin(alpha)* sin(theta), sin(alpha)* cos(theta), cos(alpha), d* cos(alpha),
			0, 0, 0, 1;
		return H;
	}

	Eigen::Matrix4f getHFromTheta(float theta) {
		return this->getHFromDH(this->a, this->alpha, this->d, theta);
	}

	/// <summary>
	/// 通过DH参数获取逆变换矩阵
	/// </summary>
	/// <param name="a"></param>
	/// <param name="alpha"></param>
	/// <param name="d"></param>
	/// <param name="theta"></param>
	/// <returns></returns>
	static Eigen::Matrix4f getHInverseFromDH(float a, float alpha, float d, float theta) {
		Eigen::Matrix4f H;
		Eigen::Matrix3f R;
		Eigen::Matrix3f RT;
		Eigen::Vector3f T;
		R << cos(theta), -sin(theta), 0,
			cos(alpha)* sin(theta), cos(alpha)* cos(theta), -sin(alpha),
			sin(alpha)* sin(theta), sin(alpha)* cos(theta), cos(alpha);
		T << a, -d * sin(alpha), d* cos(alpha);
		RT = R.transpose();
		T = -RT * T;
		H << RT(0, 0), RT(0, 1), RT(0, 2), T(0),
			RT(1, 0), RT(1, 1), RT(1, 2), T(1),
			RT(2, 0), RT(2, 1), RT(2, 2), T(2),
			0, 0, 0, 1;
		return H;
	}

	Eigen::Matrix4f getHInverseFromTheta(float theta) {
		return this->getHInverseFromDH(this->a, this->alpha, this->d, theta);
	}

	/// <summary>
	/// 更新关节对应的角度（弧度制），
	/// </summary>
	/// <param name="theta">新的角度</param>
	void updateH(float theta) {
		this->theta = theta;
		this->H = getHFromTheta(theta);
		this->IH = getHInverseFromTheta(theta);
	}
};