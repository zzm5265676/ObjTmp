#pragma once
#include "vec3.hxx"
#include <cmath>
#include <stdexcept>

class Mat4 {
private:
	double m_[4][4];

	void setIdentity() {
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				m_[r][c] = (r == c) ? 1.0 : 0.0;
	}

	double minor3x3(int row, int col) const {
		double sub[3][3];
		int sr = 0;
		for (int r = 0; r < 4; ++r) {
			if (r == row) continue;
			int sc = 0;
			for (int c = 0; c < 4; ++c) {
				if (c == col) continue;
				sub[sr][sc] = m_[r][c];
				++sc;
			}
			++sr;
		}
		return sub[0][0] * (sub[1][1] * sub[2][2] - sub[1][2] * sub[2][1])
			- sub[0][1] * (sub[1][0] * sub[2][2] - sub[1][2] * sub[2][0])
			+ sub[0][2] * (sub[1][0] * sub[2][1] - sub[1][1] * sub[2][0]);
	}

	double cofactor(int row, int col) const {
		return ((row + col) % 2 == 0 ? 1.0 : -1.0) * minor3x3(row, col);
	}

	double determinant4x4() const {
		double det = 0;
		for (int c = 0; c < 4; ++c)
			det += m_[0][c] * cofactor(0, c);
		return det;
	}

public:
	// Default: identity matrix
	Mat4() {
		setIdentity();
	}

	Mat4(const double data[16]) {
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				m_[r][c] = data[r * 4 + c];
	}

	// ===== Factory methods =====

	static Mat4 identity() {
		return Mat4();
	}

	static Mat4 translation(double tx, double ty, double tz) {
		Mat4 mat;
		mat.m_[0][3] = tx;
		mat.m_[1][3] = ty;
		mat.m_[2][3] = tz;
		return mat;
	}

	static Mat4 translation(const Vec3& t) {
		return translation(t.x, t.y, t.z);
	}

	static Mat4 rotationX(double radians) {
		Mat4 mat;
		double c = std::cos(radians);
		double s = std::sin(radians);
		mat.m_[1][1] = c;  mat.m_[1][2] = -s;
		mat.m_[2][1] = s;  mat.m_[2][2] = c;
		return mat;
	}

	static Mat4 rotationY(double radians) {
		Mat4 mat;
		double c = std::cos(radians);
		double s = std::sin(radians);
		mat.m_[0][0] = c;  mat.m_[0][2] = s;
		mat.m_[2][0] = -s; mat.m_[2][2] = c;
		return mat;
	}

	static Mat4 rotationZ(double radians) {
		Mat4 mat;
		double c = std::cos(radians);
		double s = std::sin(radians);
		mat.m_[0][0] = c;  mat.m_[0][1] = -s;
		mat.m_[1][0] = s;  mat.m_[1][1] = c;
		return mat;
	}

	// Rotate around arbitrary axis (axis must be unit vector)
	static Mat4 rotationAxis(const Vec3& axis, double radians) {
		double c = std::cos(radians);
		double s = std::sin(radians);
		double t = 1.0 - c;
		double ax = axis.x, ay = axis.y, az = axis.z;

		Mat4 mat;
		mat.m_[0][0] = t * ax * ax + c;
		mat.m_[0][1] = t * ax * ay - s * az;
		mat.m_[0][2] = t * ax * az + s * ay;
		mat.m_[1][0] = t * ax * ay + s * az;
		mat.m_[1][1] = t * ay * ay + c;
		mat.m_[1][2] = t * ay * az - s * ax;
		mat.m_[2][0] = t * ax * az - s * ay;
		mat.m_[2][1] = t * ay * az + s * ax;
		mat.m_[2][2] = t * az * az + c;
		return mat;
	}

	static Mat4 scaling(double sx, double sy, double sz) {
		Mat4 mat;
		mat.m_[0][0] = sx;
		mat.m_[1][1] = sy;
		mat.m_[2][2] = sz;
		return mat;
	}

	static Mat4 scaling(double s) {
		return scaling(s, s, s);
	}

	// ===== Operators =====

	Mat4 operator*(const Mat4& rhs) const {
		Mat4 result;
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				result.m_[r][c] = 0;
				for (int k = 0; k < 4; ++k) {
					result.m_[r][c] += m_[r][k] * rhs.m_[k][c];
				}
			}
		}
		return result;
	}

	Mat4& operator*=(const Mat4& rhs) {
		*this = *this * rhs;
		return *this;
	}

	// Transform a point (with translation, homogeneous division)
	Vec3 transformPoint(const Vec3& p) const {
		double x = m_[0][0] * p.x + m_[0][1] * p.y + m_[0][2] * p.z + m_[0][3];
		double y = m_[1][0] * p.x + m_[1][1] * p.y + m_[1][2] * p.z + m_[1][3];
		double z = m_[2][0] * p.x + m_[2][1] * p.y + m_[2][2] * p.z + m_[2][3];
		double w = m_[3][0] * p.x + m_[3][1] * p.y + m_[3][2] * p.z + m_[3][3];
		if (std::abs(w) < 1e-15) return Vec3(x, y, z);
		return Vec3(x / w, y / w, z / w);
	}

	// Transform a direction (no translation)
	Vec3 transformDirection(const Vec3& d) const {
		return Vec3(
			m_[0][0] * d.x + m_[0][1] * d.y + m_[0][2] * d.z,
			m_[1][0] * d.x + m_[1][1] * d.y + m_[1][2] * d.z,
			m_[2][0] * d.x + m_[2][1] * d.y + m_[2][2] * d.z
		);
	}

	// Transpose
	Mat4 transpose() const {
		Mat4 result;
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				result.m_[r][c] = m_[c][r];
		return result;
	}

	// Inverse (fast path for rigid transforms, general fallback)
	Mat4 inverse() const {
		bool isRigid = std::abs(m_[3][0]) < 1e-12
			&& std::abs(m_[3][1]) < 1e-12
			&& std::abs(m_[3][2]) < 1e-12
			&& std::abs(m_[3][3] - 1.0) < 1e-12;

		if (isRigid) {
			Mat4 inv;
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					inv.m_[r][c] = m_[c][r];
			for (int r = 0; r < 3; ++r) {
				inv.m_[r][3] = 0;
				for (int c = 0; c < 3; ++c)
					inv.m_[r][3] -= m_[c][r] * m_[c][3];
			}
			return inv;
		}

		double det = determinant4x4();
		if (std::abs(det) < 1e-15) {
			throw std::runtime_error("Mat4::inverse(): singular matrix");
		}
		double invDet = 1.0 / det;
		Mat4 result;
		for (int r = 0; r < 4; ++r)
			for (int c = 0; c < 4; ++c)
				result.m_[r][c] = cofactor(c, r) * invDet;
		return result;
	}

	// ===== Element access =====

	double& at(int row, int col) {
		if (row < 0 || row > 3 || col < 0 || col > 3)
			throw std::out_of_range("Mat4 index out of range");
		return m_[row][col];
	}

	double at(int row, int col) const {
		if (row < 0 || row > 3 || col < 0 || col > 3)
			throw std::out_of_range("Mat4 index out of range");
		return m_[row][col];
	}

	const double* data() const noexcept {
		return &m_[0][0];
	}
};
