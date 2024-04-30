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
	/// ��ʼ��ʱʹ�ýǶ��ƣ�������⣩
	/// </summary>
	/// <param name="a"></param>
	/// <param name="alpha"></param>
	/// <param name="d"></param>
	/// <param name="theta"></param>
	Joint(float a, float alpha, float d, float theta) {
		//����PCl��תΪ˳ʱ��Ϊ�����ʴ˴�ȡ���õ���ʱ��Ϊ��
		this->a = a;
		this->alpha = -alpha * 2 * M_PI / 360;
		this->d = d;
		this->theta = -theta * 2 * M_PI / 360;
		H = getHFromDH(this->a, this->alpha, this->d, this->theta);
		IH = getHInverseFromDH(this->a, this->alpha, this->d, this->theta);
	}

public:
	/// <summary>
	/// ͨ��DH������ȡ�任����
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
	/// ͨ��DH������ȡ��任����
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
	/// ���¹ؽڶ�Ӧ�ĽǶȣ������ƣ���
	/// </summary>
	/// <param name="theta">�µĽǶ�</param>
	void updateH(float theta) {
		this->theta = theta;
		this->H = getHFromTheta(theta);
		this->IH = getHInverseFromTheta(theta);
	}
};