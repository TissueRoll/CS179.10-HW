#include <iostream>
#include<cmath>
using namespace std;

// chris computational geometry library (ccglib)
namespace ccglib {
	struct vector3D {
		double x, y, z;
		vector3D(): x(0), y(0), z(0) {}
		vector3D(int _x, int _y, int _z): x(_x), y(_y), z(_z) {}
		vector3D operator-() const {
			return vector3D(-x, -y, -z);
		}
		vector3D operator+(const vector3D& r) const {
			return vector3D(x+r.x,y+r.y,z+r.z);
		}
		vector3D operator-(const vector3D& r) const {
			return vector3D(x-r.x,y-r.y,z-r.z);
		}
		vector3D operator*(double r) const {
			return vector3D(x*r,y*r,z*r);
		}
		friend vector3D operator*(double l, const vector3D& r) {
			return vector3D(l*r.x,l*r.y,l*r.z);
		}
		vector3D operator/(double r) const {
			return vector3D(x/r,y/r,z/r);
		}
		bool operator==(const vector3D& r) const {
			return (x == r.x) && (y == r.y) && (z == r.z);
		}
		bool operator!=(const vector3D& r) const {
			return (x != r.x) || (y != r.y) || (z != r.z);
		}
		void print() const {
			printf("%lf %lf %lf\n",x,y,z);
		}
	};
	double dot(const vector3D& u, const vector3D& v) {
		return u.x*v.x+u.y*v.y+u.z*v.z;
	}
	double norm(const vector3D& u) {
		return sqrt(dot(u,u));
	}
	vector3D cross(const vector3D& u, const vector3D& v) {
		return vector3D(u.y*v.z-v.y*u.z, v.z*u.x-u.z*v.x, u.x*v.y-v.x*u.y);
	}
	vector3D proj(const vector3D& u, const vector3D& v) {
		return (dot(u,v)/dot(v,v))*v;
	}
	vector3D perp(const vector3D& u, const vector3D& v) {
		return u-proj(u,v);
	}
}

int main() {
	cout.precision(4);
	int n; cin >> n;
	while (n--) {
		ccglib::vector3D input[3];
		for (int i = 0; i < 3; i++)
			cin >> input[i].x >> input[i].y;
		double temp = ccglib::cross(input[1]-input[0], input[2]-input[0]).z;
		cout << fixed << abs(temp/2.0) << ' ' << (temp > 0 ? "CCW" : (temp == 0 ? "COLLINEAR" : "CW")) << endl;
	}

	return 0;
}