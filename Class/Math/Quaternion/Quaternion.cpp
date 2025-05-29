#include "Quaternion.h"
#include <cmath>

#include "../Vector/Vector3.h"

Quaternion::Quaternion() {
	q = { 0.0f,0.0f,0.0f,0.0f };
}

Vector3 Quaternion::ImaginaryPart() const {
	Vector3 result{};
	result.x = this->q.x;
	result.y = this->q.y;
	result.z = this->q.z;
	return result;
}

float Quaternion::RealPart() const {
	return this->q.w;
}

Quaternion Quaternion::Conjugation() const {
	Quaternion result{};

	result.q.x = -this->q.x;
	result.q.y = -this->q.y;
	result.q.z = -this->q.z;
	result.q.w = this->q.w;

	return result;
}

float Quaternion::Norm() const {
	return std::sqrtf(std::powf(this->q.w, 2.0f) + std::powf(this->q.x, 2.0f) + std::powf(this->q.y, 2.0f) + std::powf(this->q.z, 2.0f));
}

Quaternion Quaternion::Normalize() const {
	Quaternion result{};
	float norm = this->Norm();
	if (norm != 0.0f) {
		result.q.x = this->q.x / norm;
		result.q.y = this->q.y / norm;
		result.q.z = this->q.z / norm;
		result.q.w = this->q.w / norm;
	}
	return result;
}

Quaternion Quaternion::Multiply(const Quaternion& lhs, const Quaternion& rhs) {
	Quaternion result{};

	Vector3 lhsImaginary = lhs.ImaginaryPart();
	Vector3 rhsImaginary = rhs.ImaginaryPart();

	result.q.x = Vector3::Cross(lhsImaginary, rhsImaginary).x + lhsImaginary.x * rhs.q.w + rhsImaginary.x * lhs.q.w;
	result.q.y = Vector3::Cross(lhsImaginary, rhsImaginary).y + lhsImaginary.y * rhs.q.w + rhsImaginary.y * lhs.q.w;
	result.q.z = Vector3::Cross(lhsImaginary, rhsImaginary).z + lhsImaginary.z * rhs.q.w + rhsImaginary.z * lhs.q.w;
	result.q.w = lhs.q.w * rhs.q.w - Vector3::Dot(lhsImaginary, rhsImaginary);

	return result;
}

Quaternion Quaternion::IndentityQuaternion() {
	Quaternion result{};
	result.q.w = 1.0f;
	return result;
}

Quaternion Quaternion::ConjugationQuaternion(const Quaternion& quaternion) {
	Quaternion result{};

	result.q.x = -quaternion.q.x;
	result.q.y = -quaternion.q.y;
	result.q.z = -quaternion.q.z;
	result.q.w = quaternion.q.w;

	return result;
}

float Quaternion::Norm(const Quaternion& quaternion) {
	return std::sqrtf(std::powf(quaternion.q.w, 2.0f) + std::powf(quaternion.q.x, 2.0f) + std::powf(quaternion.q.y, 2.0f) + std::powf(quaternion.q.z, 2.0f));
}

Quaternion Quaternion::Normalize(const Quaternion& quaternion) {
	Quaternion result{};
	float norm = quaternion.Norm();
	if (norm != 0.0f) {
		result.q.x = quaternion.q.x / norm;
		result.q.y = quaternion.q.y / norm;
		result.q.z = quaternion.q.z / norm;
		result.q.w = quaternion.q.w / norm;
	}
	return result;
}

Quaternion Quaternion::Inverse(const Quaternion& quaternion) {
	Quaternion result{};
	float norm = quaternion.Norm();
	Quaternion imaginaryQuaternion = quaternion.Conjugation();

	float powNorm = std::powf(norm, 2.0f);
	if (powNorm != 0.0f) {
		result.q.x = imaginaryQuaternion.q.x / powNorm;
		result.q.y = imaginaryQuaternion.q.y / powNorm;
		result.q.z = imaginaryQuaternion.q.z / powNorm;
		result.q.w = imaginaryQuaternion.q.w / powNorm;
	}

	return result;
}
