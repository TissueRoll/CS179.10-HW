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
	line(): a(ccglib::vector3D()), b(ccglib::vector3D()), ep(-1), index(-1) {}
	line(ccglib::vector3D _a, ccglib::vector3D _b, int _ep, int _index): a(_a), b(_b), ep(_ep), index(_index) {}
	line(pdd _a, pdd _b, int _ep, int _index): a(ccglib::vector3D(_a.first, _a.second, 1)), b(ccglib::vector3D(_b.first, _b.second, 1)), ep(_ep), index(_index) {}
	bool operator<(const line& rhs) {
		if (a.x == rhs.a.x) return ep < rhs.ep;
		return a.x < rhs.a.x;
	}
	bool operator==(const line& rhs) {
		return (a == rhs.a and b == rhs.b and ep == rhs.ep and index == rhs.index);
	}
	bool operator!=(const line& rhs) {
		return (a != rhs.a or b != rhs.b or ep != rhs.ep or index != rhs.index);
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

bool intersecting(line l0, line l1) {
	ccglib::vector3D u = l0.b-l0.a, v = l1.b-l1.a;
	ccglib::vector3D n = u%v;
	if (n.z == 0 and ((l1.a-l0.a)%u).z == 0) {
		double l1r0a = ((l1.a-l0.a)*u)/(u*u), l1r0b = ((l1.b-l0.a)*u)/(u*u);
		double c = (l1r0a+l1r0b)/2.0, r = abs(l1r0b-l1r0a)/2.0;
		return (abs(c-0.5) <= r+0.5 ? 1 : 0);
	} else if (n.z == 0 and ((l1.a-l0.a)%u).z != 0) {
		return 0;
	} else {
		double t = ((l1.a-l0.a)%v).z/n.z, s = ((l0.a-l1.a)%u).z/-n.z;
		return ((t >= 0 and t <= 1 and s >= 0 and s <= 1) ? 1 : 0);
	}
	// double t = ((l1.a-l0.a)*(n%v))/(u*(n%v)), s = ((l0.a-l1.a)*(n%u))/(v*(n%u));
	// return (ccglib::norm(ccglib::proj(l0.a-l1.a,u%v)) > 1e-9 ? 1 : 0);
}

void line_sweep(vector<line>& in, int& n) {
	vector<line> pts;
	for (int i = 0; i < n; i++) {
		pts.push_back(line(in[i].a,in[i].b,0,i));
		pts.push_back(line(in[i].b,in[i].a,1,i));
	}
	sort(pts.begin(), pts.end());
	set<pair<int,int>> ans;
	set<int> s;
	for (int i = 0; i < pts.size(); i++) {
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

struct comp {
	bool operator()(const line& u, const line& v) const {
		if (u.a.y == v.a.y and u.a.x == v.a.x) return u.ep < v.ep;
		if (u.a.y == v.a.y) return u.a.x < v.a.x;
		return u.a.y < v.a.y;
	}
};

// bentley ottman broken for non-distinct lines
void bentley_ottman(vector<line>& in, int& n) {
	vector<line> pts;
	for (int i = 0; i < n; i++) {
		pts.push_back(line(in[i].a,in[i].b,0,i));
		pts.push_back(line(in[i].b,in[i].a,1,i));
	}
	sort(pts.begin(), pts.end());
	set<pair<int,int>> ans;
	map<line,int,comp> m;
	map<line,int,comp>::iterator mi;
	bool prev_ok, next_ok;
	line prev_x, next_x;
	for (int i = 0; i < pts.size(); i++) { 
		line x = pts[i];
		cout << ":: " << x.index << endl;
		if (x.ep == 0) {
			// m.insert({x,1});
			m[x]++;
			auto mi0 = m.find(x);
			auto mi1 = mi0; auto mi2 = mi0;
			prev_ok = (mi0 != m.begin()); 
			next_ok = (mi0 != m.end() and (++mi2) != m.end());
			if (prev_ok) 
				prev_x = (--mi1)->first;
			if (next_ok)
				next_x = mi2->first;
			if (prev_ok and intersecting(x, prev_x))
				ans.insert({min(prev_x.index,x.index),max(prev_x.index,x.index)});
			if (next_ok and intersecting(x, next_x))
				ans.insert({min(next_x.index,x.index),max(next_x.index,x.index)});
		} else {
			cout << x.index << " close " << m.size() << endl;
			auto mi0 = m.find(in[x.index]);
			cout << "ded" << endl;
			auto mi1 = mi0; auto mi2 = mi0;
			prev_ok = (mi0 != m.begin()); 
			next_ok = (mi0 != m.end() and (++mi2) != m.end());
			if (prev_ok)
				prev_x = (--mi1)->first;
			if (next_ok)
				next_x = mi2->first;
			if (prev_ok and next_ok and intersecting(prev_x, next_x))
				ans.insert({min(prev_x.index,next_x.index),max(prev_x.index,next_x.index)});
			if (mi0->second <= 1)
				m.erase(mi0);
			else
				mi0->second--;
		}
	}
	cout << ans.size() << ' ';
	for (pair<int,int> p : ans)
		cout << p.first+1 << '-' << p.second+1 << ' ';
	cout << endl;
}

int main () {
	int T, n;
	pdd p1, p2, temp;
	cin >> T;
	while (T--) {
		cin >> n;
		vector<line> in;
		for (int i = 0; i < n; i++) {
			cin >> p1 >> p2;
			if (p2 < p1) 
				swap(p1,p2);
			in.push_back(line(p1,p2,0,i));
		}
		line_sweep(in, n); // O(n lg n + k) algorithm, where k is number of intersections. O(n^2) if k = n^2
		// bentley_ottman(in, n); // BROKEN; O(n lg n + k lg n), actually worse if k = n^2
	}
	return 0;
}