#pragma once



class Vector3{

public:

	Vector3() : x(0.0), y(0.0), z(0.0) {}
	Vector3(double val) : x(val), y(val), z(val) {}
	Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
	~Vector3() {}

	double x;
	double y;
	double z;

	void setValue(double x, double y, double z){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double & operator[](int i){
		switch (i){
		case 0:
			return x;
			break;
		case 1:
			return y;
			break;

		case 2:
			return z;
			break;
		default:
			return z;
			break;
		}
	}

	
	bool operator==(Vector3 & rhs){
		return (x == rhs.x && y == rhs.y && z == rhs.z);
	}
	bool operator!=(Vector3 & rhs){
		return (x != rhs.x || y != rhs.y || z != rhs.z);
	}
	Vector3 operator+(Vector3 & rhs){
		return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
	}
	Vector3 operator+=(Vector3 & rhs){
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
	}
	Vector3 operator-(Vector3 & rhs){
		return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
	}
	Vector3 operator-=(Vector3 & rhs){
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
	}
	Vector3 operator*(Vector3 & rhs){
		return Vector3(x * rhs.x, y * rhs.y, z * rhs.z);
	}
	Vector3 operator*=(Vector3 & rhs){
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
	}


	Vector3 dot(Vector3 & rhs){
		return *this * rhs;
	}
	Vector3 cross(Vector3 & rhs){
		return Vector3(y*rhs.z - rhs.y*z, z * rhs.x - x* rhs.z, x*rhs.y - y*rhs.x);
	}

};