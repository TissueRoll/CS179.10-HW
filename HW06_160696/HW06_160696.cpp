#include <iostream>
#include <cmath>
#include <iomanip>

using namespace std;

struct Vector3 {
	double x, y, z;

	double operator*(const Vector3 &rhs) const {
		return x*rhs.x + y*rhs.y + z*rhs.z;
	}

	Vector3 operator%(const Vector3 &rhs) const {
		return Vector3{y*rhs.z-rhs.y*z, z*rhs.x-rhs.z*x, x*rhs.y-rhs.x*y};
	}

	Vector3 operator*(double s) const {
		return Vector3{x*s, y*s, z*s};
	}

	Vector3 operator/(double s) const {
		return Vector3{x/s, y/s, z/s};
	}

	Vector3 operator+(const Vector3 &rhs) const {
		return Vector3{x+rhs.x, y+rhs.y, z+rhs.z};
	}
	Vector3 operator-(const Vector3 &rhs) const {
		return Vector3{x-rhs.x, y-rhs.y, z-rhs.z};
	}


	//negation
	Vector3 operator-() const {
		return Vector3{-x, -y, -z};
	}

	double mag() const {
		return sqrt(x * x + y * y + z * z);
	}
};

struct Quaternion {
	double s;
	Vector3 v;

	Quaternion operator+(const Quaternion &rhs) const {
		return Quaternion{s+rhs.s, v+rhs.v};
	}

	Quaternion operator-(const Quaternion &rhs) const {
		return Quaternion{s-rhs.s, v-rhs.v};
	}

	Quaternion operator*(const Quaternion &rhs) const {
		return Quaternion{s*rhs.s-v*rhs.v, rhs.v*s + v*rhs.s + v%rhs.v};
	}

	Quaternion operator*(double c) const {
		return Quaternion{s * c, v * c};
	}

	Quaternion operator/(double c) const {
		return Quaternion{s/c, v/c};
	}

	Quaternion conj() const {
		return Quaternion{s, -v};
	}

	Quaternion operator!() const {
		return conj();
	}

	double mag() const {
		return sqrt(sqmag());
	}

	double sqmag() const {
		return s * s + v * v;
	}

	Quaternion inv() const {
		return Quaternion{conj()/sqmag()};
	}

	Quaternion operator/(const Quaternion &rhs) const {
		Quaternion r = rhs.inv();
		return Quaternion{s*r.s-v*r.v, r.v*s + v*r.s + v%r.v};
	}

	Quaternion operator^(const Quaternion &rhs) const {
		// q(this)^p(rhs)
		if (sqmag() == 0)
			return Quaternion{0,0,0,0};
		Vector3 qvn = (v.mag() <= 1e-9 ? v : v/v.mag());
		Quaternion lnq{log(mag()),qvn*atan2(v.mag(), s)};
		Quaternion omega = rhs*lnq;
		Vector3 ovn = (omega.v.mag() <= 1e-9 ? omega.v : omega.v/omega.v.mag());
		Quaternion result = Quaternion{cos(omega.v.mag())*exp(omega.s), ovn*sin(omega.v.mag())*exp(omega.s)};
		return result;
	}
};


ostream& operator<<(ostream &o, const Quaternion &q) {
	o << q.s << ' ' << q.v.x << ' ' << q.v.y << ' ' << q.v.z;
	return o;
}
istream& operator>>(istream &i, Quaternion &p) {
	i >> p.s >> p.v.x >> p.v.y >> p.v.z;
	return i;
}

int main() {
	size_t n;
	cin >> n;
	for ( size_t i = 0; i < n; ++i ) {
		Quaternion p, q;
		char oper;

		cin >> p >> oper >> q;
		cout << fixed << setprecision(2);
		switch (oper) {
			case '+':
				cout << p + q << endl;
			break;
			case '-':
				cout << p - q << endl;
			break;
			case '*':
				cout << p * q << endl;
			break;
			case '/':
				cout << p / q << endl;
			break;
			case '^':
				cout << (p ^ q) << endl;
			break;
		}
	}
}