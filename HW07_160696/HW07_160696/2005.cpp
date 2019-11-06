#include <bits/stdc++.h>
using namespace std;

const double PI = acos(-1);
const double conversion = PI/180.0;

// chris computational geometry library (ccglib)
namespace ccglib {
	struct vector3D {
		double x, y, z;
		vector3D(double _x = 0, double _y = 0, double _z = 0): x(_x), y(_y), z(_z) {}
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
		double operator*(const vector3D& r) const { // dot product
			return x*r.x+y*r.y+z*r.z;
		}
		vector3D operator%(const vector3D& r) const { // cross product
			return vector3D(y*r.z-r.y*z, z*r.x-r.z*x, x*r.y-r.x*y);
		}
		bool operator==(const vector3D& r) const {
			return (x == r.x) && (y == r.y) && (z == r.z);
		}
		bool operator!=(const vector3D& r) const {
			return (x != r.x) || (y != r.y) || (z != r.z);
		}
		bool operator<(const vector3D& r) const {
			if (x != r.x) return x < r.x;
			if (y != r.y) return y < r.y;
			return z < r.z;
		}
		void print() const {
			printf("%lf %lf %lf\n",x,y,z);
		}
		void apply_eps(double eps = 1e-9) {
			x = (abs(x) < eps ? 0 : x);
			y = (abs(y) < eps ? 0 : y);
			z = (abs(z) < eps ? 0 : z);
		}
	};
	double norm(const vector3D& u) {
		return sqrt(u*u);
	}
	
	vector3D proj(const vector3D& u, const vector3D& v) {
		return ((u*v)/(v*v))*v;
	}
	vector3D perp(const vector3D& u, const vector3D& v) {
		return u-proj(u,v);
	}
	vector3D to_implicit(vector3D u, vector3D v) {
		return u%v;
	}
	pair<vector3D, vector3D> to_four_numbers(vector3D u, bool type) {
		double d = -u.z/(u*u-u.z*u.z);
		return make_pair(vector3D(d*u.x, d*u.y, 1), vector3D(d*u.x*type + u.y, d*u.y*type - u.x, type));
	}
}

ostream& operator<<(ostream &o, const ccglib::vector3D &v) {
	o << v.x << ' ' << v.y << ' ' << v.z;
	return o;
}
istream& operator>>(istream &i, ccglib::vector3D &v) {
	i >> v.x >> v.y >> v.z;
	return i;
}

int n; double HA, HB; 
double alpha, beta, gammma, delta;

int main () {
	cout.precision(0);
	cout << fixed;
	cin >> n >> HA >> HB;
	double theta = atan2(HB-HA, 100.0);
	for (int i = 1; i <= n; i++) {
		cin >> alpha >> beta >> gammma >> delta;
		alpha *= conversion; beta *= conversion; gammma *= conversion; delta *= conversion;
		ccglib::vector3D A(-50.0, 0, HA), B(50.0, 0, HB);
		ccglib::vector3D U(cos(gammma)*cos(alpha), sin(gammma)*cos(alpha), sin(alpha)), V(cos(delta)*cos(beta), sin(delta)*cos(beta), sin(beta));
		// L0(t) = A + tU
		// L1(s) = B + sV
		double t = ((B-A)*((U%V)%V))/(U*((U%V)%V));
		ccglib::vector3D result0 = A+U*t;
		double s = ((A-B)*((U%V)%U))/(V*((U%V)%U));
		ccglib::vector3D result1 = B+V*s;
		cout << i << ": " << (result0.z+result1.z)/2.0 << endl;
	}
	return 0;
}