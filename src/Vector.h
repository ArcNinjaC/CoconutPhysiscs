#pragma once
#include <cmath>


struct Vector2 {
	float x, y;


	static Vector2 from_angle(float theta) 
	{
		return Vector2(cos(theta), sin(theta));
	}

	Vector2() : x(0), y(0) {}

	Vector2(float x, float y) : 
		x(x),
		y(y) {}

	float dot(const Vector2& other) 
	const {
		return x * other.x + y * other.y;
	}

	float length() 
	const {
		return sqrt(this->dot(*this));
	}

	Vector2 normalized() 
	const {
		return *this / this->length();
	}

	Vector2 flipX() {
		x *= -1;
		return *this;
	}

	Vector2 flipY() {
		y *= -1;
		return *this;
	}

	Vector2 operator+ (const Vector2& other)
	const {
		return Vector2(x + other.x, y + other.y);
	}

	Vector2& operator+= (const Vector2& other) 
	{
		return *this = *this + other;
	}

	Vector2 operator- (const Vector2& other)
	const {
		return Vector2(x - other.x, y - other.y);
	}

	Vector2& operator-= (const Vector2& other) 
	{
		return *this = *this - other;
	}

	Vector2 operator* (const float factor) 
	const {
		return Vector2(x * factor, y * factor);
	}

	Vector2& operator *= (const float factor) 
	{
		return *this = *this * factor;

	}

	Vector2 operator/ (const float factor)
	const {
		return Vector2(x / factor, y / factor);
	}

	Vector2& operator/= (const float factor)
	{
		return *this = *this / factor;
	}


};