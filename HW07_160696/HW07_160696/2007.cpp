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

int n; double D, L, HA, HB, ERRDIST; 
double alpha, beta, gammma, delta;

int main () {
	cout.precision(0);
	cout << fixed;
	cin >> n >> D >> L >> HA >> HB >> ERRDIST;
	for (int i = 1; i <= n; i++) {
		cout << i << ' ';
		cin >> alpha >> beta >> gammma >> delta;
		if ((alpha <= 0.0 or alpha >= 90.0) or (beta <= 0.0 or beta >= 90.0) or (gammma <= 0.0 or gammma >= 90.0) or (delta <= 90.0 or delta >= 180.0)) {
			cout << "DISQUALIFIED" << endl;
			continue;
		}
		alpha *= conversion; beta *= conversion; gammma *= conversion; delta *= conversion;
		ccglib::vector3D A(-D/2.0, 0, HA), B(D/2.0, 0, HB), Ap(-D/2.0,0,0), Bp(D/2.0,0,0);
		ccglib::vector3D U(cos(gammma)*cos(alpha), sin(gammma)*cos(alpha), sin(alpha)), V(cos(delta)*cos(beta), sin(delta)*cos(beta), sin(beta));
		ccglib::vector3D ApU = A+U; ccglib::vector3D BpV = B+V;
		// L0(t) = A + tU
		// L1(s) = B + sV
		ccglib::vector3D AN = (Ap-A)%(ApU-A);
		ccglib::vector3D BN = (Bp-B)%(BpV-B);
		ccglib::vector3D Pd = AN%BN;
		ccglib::vector3D Pm = BN*((-A)*AN)-AN*((-B)*BN);
		ccglib::vector3D pt = (Pd%Pm)/(Pd*Pd); 

		double t = ((pt-A)*((U%Pd)%Pd))/(U*((U%Pd)%Pd));
		ccglib::vector3D result0 = A+U*t;
		// cout << result0 << endl;
		double s = ((pt-B)*((Pd%V)%Pd))/(V*((Pd%V)%Pd));
		ccglib::vector3D result1 = B+V*s;
		// cout << result1 << endl;
		if (abs(ccglib::norm(result1-result0)) > ERRDIST)
			cout << "ERROR" << endl;
		else
			cout << (result0.z+result1.z)/2.0 << endl;
	}
	return 0;
}