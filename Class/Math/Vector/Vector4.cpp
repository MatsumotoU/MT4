#include "Vector4.h"
#include "Vector3.h"
#include <assert.h>
#include <cmath>

float Vector4::Length() const {
	return sqrt(x * x + y * y + z * z + w * w);
}

Vector4 Vector4::Normalize() const {
	Vector4 result = {};

	if (this->Length() == 0.0f) {
		assert(false);
	}

	result.x = x / this->Length();
	result.y = y / this->Length();
	result.z = z / this->Length();
	result.w = w / this->Length();

	return result;
}

Vector3 Vector4::xyz() const {
	Vector3 result{};
	result.x = this->x;
	result.y = this->y;
	result.z = this->z;
	return result;
}
