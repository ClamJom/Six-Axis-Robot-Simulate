#include <iostream>

#include <vector>
#include "Robot.h"

//点云OBB有向包围盒
//来自：https://blog.csdn.net/u012010729/article/details/104115932
Eigen::Matrix4f obb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,std::string id="1")
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	//绘制OBB包围盒
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(10.0);

	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud);//设置随机颜色
	viewer->addPointCloud<pcl::PointXYZ>(cloud, RandomColor, "points"+id);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points"+id);

	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB" + id);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB" + id);

	major_vector *= 10, middle_vector *= 10, minor_vector *= 10;
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector" + id);
	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector" + id);
	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector" + id);

	Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
	H.block<3, 3>(0, 0) << rotational_matrix_OBB;
	H(0, 3) = mass_center(0), H(1, 3) = mass_center(1), H(2, 3) = mass_center(2);
	return H;
}

Eigen::Matrix4f getOBBH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
	H.block<3, 3>(0, 0) << rotational_matrix_OBB;
	H(0, 3) = mass_center(0), H(1, 3) = mass_center(1), H(2, 3) = mass_center(2);
	return H;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr initTestPointCloud(int cloudSize=500) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(cloudSize);
	float x, y, z;
	for (int i = 0; i < 200; i++) {
		z = i / 10;
		x = std::cos((i % 360) / M_PI / 2);
		y = std::sin((i % 360) / M_PI / 2);
		cloud->points[i] = pcl::PointXYZ(x, y, z);
	}
	for (int i = 0; i < 300; i++) {
		x = i / 10;
		y = std::cos((i % 360) / M_PI / 2);
		z = std::sin((i % 360) / M_PI / 2);
		cloud->points[i + 200] = pcl::PointXYZ(x, y, z);
	}
	return cloud;
}

/// <summary>
/// 从旋转角以及平移得到变换矩阵
/// </summary>
/// <param name="roll">滚角</param>
/// <param name="pitch">俯仰角</param>
/// <param name="yaw">偏航角</param>
/// <param name="x">X</param>
/// <param name="y">Y</param>
/// <param name="z">Z</param>
/// <returns>变换矩阵</returns>
Eigen::Matrix4f getH(float roll, float pitch, float yaw, float x, float y, float z) {
	Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
	Eigen::Quaternionf quaternion = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
	H.block<3, 3>(0, 0) = quaternion.matrix();
	H(0, 3) = x;
	H(1, 3) = y;
	H(2, 3) = z;
	return H;
}

int main()
{
	//初始化机械臂，Joint内的内容为DH参数
	//此处使用的机械臂具体型号为博朗特 BRTIRUS0805A
	//为保证逆解正确，保证2、3、4轴坐标系原点形成的平面垂直于轴1的X-Y平面
	std::vector<Joint> links;
	links.push_back(Joint(0, 0, 155.5, 0));
	links.push_back(Joint(75.95, 90, 7.05, 90));
	links.push_back(Joint(390, 0, 0, 0));
	links.push_back(Joint(117.5, 90, 394, 0));
	links.push_back(Joint(0, 90, 0, 0));
	links.push_back(Joint(0, -90, 0, 0));
	Robot robot(links, Eigen::Vector3f(0, 0, 0));
	robot.tool = Joint(0, 0, 119, 0);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->addCoordinateSystem(50);
	//生成测试点云并变换姿态
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = initTestPointCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr transedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f T = getH(M_PI / 4, M_PI / 3, 0, 400, 400, 400);
	pcl::transformPointCloud(*cloud, *transedCloud, T);
	//从变换后的姿态得到最小包围盒的姿态信息
	Eigen::Matrix4f H = obb(transedCloud, viewer);

	//我们希望机械臂的末端应当从机械臂本体朝向目标为佳，而不是越过其再折返回来，
	//因此当出现目标的Z轴指向机械臂时将其取反
	Eigen::Vector3f EndPos;
	Eigen::Vector3f EZ;
	EndPos << H(0, 3), H(1, 3), H(2, 3);
	EZ << H(0, 2), H(1, 2), H(2, 2);
	float d = EZ.dot(EndPos) - EZ(2) * EndPos(2);
	if (d < 0) {
		EZ = -EZ;
		H.block<3, 1>(0, 2) = EZ;
	}
	//逆运动学解算
	robot.IK(H);
	//生成机械臂线框
	robot.drawRobotLine(viewer);

	//重复上述过程解析另一个姿态
	T = getH(M_PI / 4, M_PI / 3, 0, 400, -400, 400);
	pcl::transformPointCloud(*cloud, *transedCloud, T);
	H = obb(transedCloud, viewer,"2");

	EndPos << H(0, 3), H(1, 3), H(2, 3);
	EZ << H(0, 2), H(1, 2), H(2, 2);
	d = EZ.dot(EndPos) - EZ(2) * EndPos(2);
	if (d < 0) {
		EZ = -EZ;
		H.block<3, 1>(0, 2) = EZ;
	}
	robot.IK(H);
	robot.drawRobotLine(viewer);

	//显示
	while (!viewer->wasStopped()) {
		viewer->spinOnce();
	}
	return 0;
}