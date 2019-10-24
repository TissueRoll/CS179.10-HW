#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
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
		sort(pts.begin(), pts.end());
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
	int nProb; cin >> nProb;
	for (int T = 1; T <= nProb; T++) {
		int nPos; cin >> nPos;
		vector<ccglib::vector3D> prev, cur;
		for (int i = 0; i < nPos; i++) {
			double x; cin >> x;
			prev.push_back(ccglib::vector3D(x, 1, 0));
		}
		ccglib::vector3D ans;
		while (nPos > 1) {
			cur.clear();
			for (int j = 1; j < nPos; j++) {
				ccglib::vector3D u = (prev[j]-prev[j-1])/2.0;
				ccglib::vector3D v = sqrt((4-dot(u,u))/dot(u,u)) * ccglib::vector3D(-u.y, u.x, 0);
				cur.push_back(prev[j-1]+u+v);
			}
			prev = cur;
			nPos--;
		}
		cout << T << ": " << prev[0].x << ' ' << prev[0].y << endl;
	}
	return 0;
}