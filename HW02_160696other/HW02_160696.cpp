#include <bits/stdc++.h>
using namespace std;

// chris computational geometry library (ccglib)
namespace ccglib {
	struct vector3D {
		double x, y, z;
		vector3D(): x(0), y(0), z(0) {}
		vector3D(double _x, double _y, double _z): x(_x), y(_y), z(_z) {}
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
		return vector3D(u.y*v.z-v.y*u.z, u.z*v.x-v.z*u.x, u.x*v.y-v.x*u.y);
	}
	vector3D proj(const vector3D& u, const vector3D& v) {
		return (dot(u,v)/dot(v,v))*v;
	}
	vector3D perp(const vector3D& u, const vector3D& v) {
		return u-proj(u,v);
	}
	int convex2D_tests(const vector<vector3D>& points, bool pt = 0, vector3D ref = vector3D()) {
		int n = points.size(), t = -2, dir = -2;
		if (n < 3) return -2;
		double res = 0; bool zr = 0;
		for (int i = 0; i < n; i++) {
			res = (pt ? cross(points[i]-ref,points[(i+1)%n]-ref) : cross(points[i]-points[(i-1+n)%n], points[(i+1)%n]-points[i])).z;
			t = (res > 0 ? 1 : (res < 0 ? -1: 0));
			dir = (i == 0 ? t : (t == 0 ? dir : (dir == 0 ? t : (dir == t ? dir : -2))));
			if (pt) zr = (zr ? 1 : (t == 0));
		}
		return (!pt ? dir : (!zr ? dir : (dir != -2 ? 0 : -2)));
	}
	double area2D(const vector<vector3D>& points) {
		int n = points.size();
		if (n < 3) return 0;
		double ans = 0;
		for (int i = 0, j = n-1; i < n; j = i++)
			ans += cross(points[i],points[j]).z;
		return abs(ans)/2.0;
	}
}

int main() {
	int T; cin >> T;
	while(T--) {
		char c; cin >> c;
		if (c == 'p') {
			int n; cin >> n;
			vector<ccglib::vector3D> polygon(n);
			ccglib::vector3D point;
			cin >> point.x >> point.y;
			for (int i = 0; i < n; i++) 
				cin >> polygon[i].x >> polygon[i].y;
			int ans = ccglib::convex2D_tests(polygon, 1, point);
			cout << (ans == 0 ? "ON" : (abs(ans) == 1 ? "INSIDE" : "OUTSIDE")) << endl;
		} else if (c == 'x') {
			int n; cin >> n;
			vector<ccglib::vector3D> polygon(n);
			for (int i = 0; i < n; i++) 
				cin >> polygon[i].x >> polygon[i].y;
			int ans = ccglib::convex2D_tests(polygon);
			cout << (ans != -2 ? "CONVEX" : "NON-CONVEX") << endl;
		} else if (c == 'a') {
			ccglib::vector3D obj[2], v_obj[2];
			for (int i = 0; i < 2; i++) 
				cin >> obj[i].x >> obj[i].y >> obj[i].z >> v_obj[i].x >> v_obj[i].y >> v_obj[i].z;
			double ans = ccglib::dot(obj[1]-obj[0], v_obj[1]-v_obj[0]);
			cout << (ans < 0 ? "APPROACH" : (ans == 0 ? "NEITHER" : "SEPARATE")) << endl;
		}
	}
	return 0;
}