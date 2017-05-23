//Martin Kernjak

#pragma once

#include <cmath>
#include <cassert>

namespace mk {

	struct Vector3 {
		float x;
		float y;
		float z;

		Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
		Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

		Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

		void Set(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		inline void Divide(const float factor)
		{
			x /= factor;
			y /= factor;
			z /= factor;
		}

		inline void Multiply(const float factor)
		{
			x *= factor;
			y *= factor;
			z *= factor;
		}

		inline void Add(const Vector3& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
		}

		inline void Subtract(const Vector3& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
		}

		inline bool IsEqual(const Vector3& other, float epsilon)
		{
			bool result = true;

			result &= (x > other.x - epsilon && x < other.x + epsilon);
			result &= (y > other.y - epsilon && y < other.y + epsilon);
			result &= (z > other.z - epsilon && z < other.z + epsilon);

			return result;
		}

		inline Vector3 Cross(const Vector3& other) const
		{
			Vector3 result;

			result.x = y * other.z - z * other.y;
			result.y = z * other.x - x * other.z;
			result.z = x * other.y - y * other.x;

			return result;
		}

		inline float DistanceSquared(const Vector3& other) const
		{
			float tmp = (x - other.x) * (x - other.x);
			tmp += (y - other.y) * (y - other.y);
			tmp += (z - other.z) * (z - other.z);

			return tmp;
		}

		inline float Distance(const Vector3& other) const
		{
			float tmp = DistanceSquared(other);
			tmp = sqrtf(tmp);

			return tmp;
		}

		inline float Dot(const Vector3& other) const
		{
			float tmp = x * other.x;
			tmp += y * other.y;
			tmp += z * other.z;

			return tmp;
		}

		inline float Length() const
		{
			float tmp = Dot(*this);
			tmp = sqrtf(tmp);

			return tmp;
		}

		void Normalize()
		{
			float length = Length();
			Multiply(1.0f / length);
		}

		void Rotate(const Vector3& axis, float angle)
		{
			float L = axis.Dot(axis);
			float sqrtL = sqrtf(L);

			float factor = (-axis.x * x) - (-axis.y * y) - (-axis.z * z);

			x = -axis.x * factor * (1.0f - cosf(angle)) + L * x * cosf(angle) + sqrtL * ( -(axis.z * y) + (axis.y * z)) * sinf(angle);
			x /= L;

			y = -axis.y * factor * (1.0f - cosf(angle)) + L * y * cosf(angle) + sqrtL * (-(axis.z * x) + (axis.x * z)) * sinf(angle);
			y /= L;

			z = -axis.z * factor * (1.0f - cosf(angle)) + L * z * cosf(angle) + sqrtL * (-(axis.y * x) + (axis.x * y)) * sinf(angle);
			z /= L;
		}

		Vector3& operator+=(const Vector3& other)
		{
			Add(other);
			return *this;
		}

		Vector3& operator-=(const Vector3& other)
		{
			Subtract(other);
			return *this;
		}

		Vector3& operator*=(const float factor)
		{
			Multiply(factor);
			return *this;
		}

		Vector3& operator/=(const float factor)
		{
			Divide(factor);
			return *this;
		}

		Vector3 operator+(const Vector3& other) const
		{
			Vector3 temp(*this);
			temp += (other);
			return temp;
		}

		Vector3 operator-(const Vector3& other) const
		{
			Vector3 temp(*this);
			temp -= (other);
			return temp;
		}

		float& operator[](size_t index)
		{
			assert(index >= 0 && index <= 2);

			if (index == 0)
			{
				return x;
			}
			else if (index == 1)
			{
				return y;
			}
			else if (index == 2)
			{
				return z;
			}
			else
			{
				return x;
			}
		}

		float operator[](size_t index) const
		{
			assert(index >= 0 && index <= 2);

			if (index == 0)
			{
				return x;
			}
			else if (index == 1)
			{
				return y;
			}
			else if (index == 2)
			{
				return z;
			}
			else
			{
				return x;
			}
		}

		Vector3 operator*(const float factor) const
		{
			Vector3 temp(*this);
			temp *= factor;
			return temp;
		}

		Vector3& operator=(const Vector3& other)
		{
			if (this != &other)
			{
				this->x = other.x;
				this->y = other.y;
				this->z = other.z;
			}
			return *this;
		}

		static float Dot(const Vector3& input1, const Vector3& input2)
		{
			float result = input1.Dot(input2);
			return result;
		}

		static Vector3 Cross(const Vector3& input1, const Vector3& input2)
		{
			Vector3 result = input1;
			result.Cross(input2);

			return result;
		}

		static Vector3 Normalize(const Vector3& input)
		{
			Vector3 result = input;
			result.Normalize();

			return result;
		}

		static Vector3 Rotate(const Vector3& input, const Vector3& axis, float angle)
		{
			Vector3 result = input;
			result.Rotate(axis, angle);

			return result;
		}
	};
}