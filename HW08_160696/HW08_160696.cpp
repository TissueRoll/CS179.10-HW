#include <bits/stdc++.h>
using namespace std;

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
}

typedef pair<double,double> pdd;

struct line {
	ccglib::vector3D a, b; int ep, index;
	line(ccglib::vector3D _a, ccglib::vector3D _b, int _ep, int _index): a(_a), b(_b), ep(_ep), index(_index) {}
	line(pdd _a, pdd _b, int _ep, int _index): a(ccglib::vector3D(_a.first, _a.second, 1)), b(ccglib::vector3D(_b.first, _b.second, 1)), ep(_ep), index(_index) {}
	bool operator<(const line& rhs) {
		if (a.x == rhs.a.x) return ep < rhs.ep;
		return a.x < rhs.a.x;
	}
};


ostream& operator<<(ostream &o, const pdd &z) {
	o << z.first << ' ' << z.second;
	return o;
}

istream& operator>>(istream &i, pdd &z) {
	i >> z.first >> z.second;
	return i;
}

ostream& operator<<(ostream &o, const ccglib::vector3D &v) {
	o << v.x << ' ' << v.y << ' ' << v.z;
	return o;
}

istream& operator>>(istream &i, ccglib::vector3D &v) {
	i >> v.x >> v.y >> v.z;
	return i;
}

bool intersecting(line l0, line l1) {
	ccglib::vector3D u = l0.b-l0.a, v = l1.b-l1.a;
	ccglib::vector3D n = u%v;
	if (n.z == 0 and ((l1.a-l0.a)%u).z == 0) {
		double l1r0a = ((l1.a-l0.a)*u)/(u*u), l1r0b = ((l1.b-l0.a)*u)/(u*u);
		double c = (l1r0a+l1r0b)/2.0, r = abs(l1r0b-c);
		return (abs(abs(c)-0.5) <= r+0.5 ? 1 : 0);
	} else if (n.z == 0 and ((l1.a-l0.a)%u).z != 0) {
		return 0;
	} else {
		double t = ((l1.a-l0.a)%v).z/n.z, s = ((l0.a-l1.a)%u).z/-n.z;
		return ((t >= 0 and t <= 1 and s >= 0 and s <= 1) ? 1 : 0);
	}
	// double t = ((l1.a-l0.a)*(n%v))/(u*(n%v)), s = ((l0.a-l1.a)*(n%u))/(v*(n%u));
	// return (ccglib::norm(ccglib::proj(l0.a-l1.a,u%v)) > 1e-9 ? 1 : 0);
}

int main () {
	int T, n;
	pdd p1, p2, temp;
	cin >> T;
	while (T--) {
		cin >> n;
		vector<line> in;
		vector<line> pts;
		for (int i = 0; i < n; i++) {
			cin >> p1 >> p2;
			if (p2 < p1) 
				swap(p1,p2);
			in.push_back(line(p1,p2,0,i));
			pts.push_back(line(p1,p2,0,i));
			pts.push_back(line(p2,p1,1,i));
		}
		sort(pts.begin(), pts.end());
		set<pair<int,int>> ans;
		set<int> s;
		s.insert(pts[0].index);
		for (int i = 1; i < pts.size(); i++) {
			line x = pts[i];
			if (x.ep == 0) {
				for (int ind : s)
					if (intersecting(in[ind],in[x.index]))
						ans.insert({min(ind,x.index),max(ind,x.index)});
				s.insert(x.index);
			} else {
				s.erase(x.index);
			}
		}
		cout << ans.size() << ' ';
		for (pair<int,int> p : ans)
			cout << p.first+1 << '-' << p.second+1 << ' ';
		cout << endl;
	}
	return 0;
}