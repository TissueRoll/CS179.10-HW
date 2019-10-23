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
		void print(int bitmask, double eps) const {
			printf("%lf %lf %lf\n",x,y,z);
		}
		void apply_eps(double eps = 1e-9) {
			x = (abs(x) < eps ? 0 : x);
			y = (abs(y) < eps ? 0 : y);
			z = (abs(z) < eps ? 0 : z);
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
	vector<vector3D> convex_hull(vector<vector3D> pts) {
		sort(pts.begin(), pts.end(), [](vector3D a, vector3D b){
			if (a.x != b.x) return a.x < b.x;
			if (a.y != b.y) return a.y < b.y;
			return a.z < b.z;
		});
		int top = 0;
		vector<vector3D> stk(2*pts.size());
		for (int i = 0; i < pts.size(); i++) {
			while (top >= 2 and cross(stk[top-1]-stk[top-2], pts[i]-stk[top-2]).z <= 0) top--;
			stk[top++] = pts[i];
		}
		for (int i = pts.size()-2, t = top+1; i >= 0; i--) {
			while (top >= t and cross(stk[top-1]-stk[top-2], pts[i]-stk[top-2]).z <= 0) top--;
			stk[top++] = pts[i];
		}
		stk.resize(top-1);
		return stk;
	}
	vector3D to_implicit(vector3D u, vector3D v) {
		return cross(u,v);
	}
	pair<vector3D, vector3D> to_four_numbers(vector3D u, bool type) {
		double d = -u.z/(dot(u,u)-u.z*u.z);
		return make_pair(vector3D(d*u.x, d*u.y, 1), vector3D(d*u.x*type + u.y, d*u.y*type - u.x, type));
	}
}

int main() {
	cout.precision(4);
	cout << fixed;
	double eps = 1e-5;
	int T; cin >> T;
	while(T--) {
		char s, d; cin >> s >> d;
		ccglib::vector3D u, v;
		if (s == 'i') {
			cin >> u.x >> u.y >> u.z;
			if (d == 'i') {
				u.apply_eps(eps);
				cout << u.x << ' ' << u.y << ' ' << u.z << endl;
			} else {
				pair<ccglib::vector3D,ccglib::vector3D> ans = ccglib::to_four_numbers(u, (d == 'p' ? 0 : 1));
				ans.first.apply_eps(eps); ans.second.apply_eps(eps);
				cout << ans.first.x << ' ' << ans.first.y << ' ' << ans.second.x << ' ' << ans.second.y << endl;
			}
		} else if (s == 'p') {
			cin >> u.x >> u.y >> v.x >> v.y;
			u.z = 1; v.z = 0;
			if (d == 'i') {
				ccglib::vector3D ans = ccglib::to_implicit(u,v);
				ans.apply_eps(eps);
				cout << ans.x << ' ' << ans.y << ' ' << ans.z << endl;
			} else if (d == 'p') {
				u.apply_eps(eps); v.apply_eps(eps);
				cout << u.x << ' ' << u.y << ' ' << v.x << ' ' << v.y << endl;
			} else if (d == '2') {
				u.apply_eps(eps); v.apply_eps(eps);
				cout << u.x << ' ' << u.y << ' ' << u.x+v.x << ' ' << u.y+v.y << endl;
			}
		} else if (s == '2') {
			cin >> u.x >> u.y >> v.x >> v.y;
			u.z = 1; v.z = 1;
			if (d == 'i') {
				ccglib::vector3D ans = ccglib::to_implicit(u,v);
				ans.apply_eps(eps);
				cout << ans.x << ' ' << ans.y << ' ' << ans.z << endl;
			} else if (d == 'p') {
				u.apply_eps(eps); v.apply_eps(eps);
				cout << u.x << ' ' << u.y << ' ' << v.x-u.x << ' ' << v.y-u.y << endl;
			} else if (d == '2') {
				u.apply_eps(eps); v.apply_eps(eps);
				cout << u.x << ' ' << u.y << ' ' << v.x << ' ' << v.y << endl;
			}
		}
	}
	return 0;
}