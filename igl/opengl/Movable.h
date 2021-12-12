

#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);
	void SetCenterOfRotation(Eigen::Vector3d amt);
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);
	//Eigen::Matrix3d GetRotation() { return Tout.rotation(); }
	void MyRotate(const Eigen::Matrix3d &rot);
	void MyScale(Eigen::Vector3d amt);
	void MyRotateX(float angle);
	void MyRotateY(float angle);
	//Eigen::Matrix4f MakeTrans();

	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }

	virtual ~Movable() {}
private:
	Eigen::Affine3d Tout,Tin;
	//ass2
	Eigen::Matrix3f Rx; // Euler Angle - X
	Eigen::Matrix3f Ry; // Euler Angle - Y
};

