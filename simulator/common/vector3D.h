#pragma once
#include <cmath>

struct Vector3D {
    double x, y, z;

    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

    double module() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3D unit_vector() const {
        double mod = module();
        if (mod == 0) {
            return Vector3D(0.0, 0.0, 0.0);
        }
        return Vector3D(
            x / mod, 
            y / mod, 
            z / mod
        );
    }

    Vector3D operator+(const Vector3D& other) const {
        return {
            x + other.x, 
            y + other.y, 
            z + other.z
        };
    }

    Vector3D operator-(const Vector3D& other) const {
        return {
            x - other.x, 
            y - other.y, 
            z - other.z
        };
    }

    Vector3D operator*(double scalar) const {
        return {
            x * scalar, 
            y * scalar, 
            z * scalar
        };
    }
};

inline Vector3D operator*(double scalar, const Vector3D& v) {
    return v * scalar;
}
