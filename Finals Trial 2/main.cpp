#include <bits/stdc++.h>
using namespace std;

const double PI = acos(-1);
const double DEGREES = 180/PI;
const double RADIANS = PI/180.0;

struct Vec3 {
	double x, y, z;
	Vec3(double _x = 0, double _y = 0, double _z = 0): x(_x), y(_y), z(_z) {}
	Vec3 operator-() const {
		return Vec3(-x,-y,-z);
	}
	Vec3 operator+(const Vec3& rhs) const {
		return Vec3(x+rhs.x, y+rhs.y, z+rhs.z);
	}
	Vec3 operator-(const Vec3& rhs) const {
		return Vec3(x-rhs.x, y-rhs.y, z-rhs.z);
	}
	friend Vec3 operator*(const double& lhs, const Vec3& rhs) {
		return Vec3(lhs*rhs.x, lhs*rhs.y, lhs*rhs.z);
	}
	Vec3 operator*(const double& rhs) const {
		return Vec3(x*rhs, y*rhs, z*rhs);
	}
	Vec3 operator/(const double& rhs) const {
		return Vec3(x/rhs, y/rhs, z/rhs);
	}
	double operator*(const Vec3& rhs) const {
		return x*rhs.x+y*rhs.y+z*rhs.z;
	}
	Vec3 operator%(const Vec3& rhs) const {
		return Vec3(y*rhs.z - rhs.y*z,
					z*rhs.x - rhs.z*x,
					x*rhs.y - rhs.x*y);
	}
	double SQmag() {
		return x*x+y*y+z*z;
	}
	double mag() {
		return sqrt(SQmag());
	}
};

/*
	--Spherical--
	x = cosTcosP	r = sqrt(x^2, y^2, z^2)
	y = cosTsinP	T = acos(z/r)
	z = sinT		P = atan2(y,x)

*/

// typedef vector<vector<double>> vvd;
// vvd multiply(vvd A, vvd B) {
// 	int p = A.size(), q = A[0].size(), r = B[0].size();
// 	// req: q == B.size()
// 	vvd AB(p, r); //pseudo
// 	for (int i = 0; i < p; i++)
// 	for (int j = 0; j < q; j++)
// 	for (int k = 0; k < r; k++)
// 		(AB[i][k] += A[i][j]*B[j][k]);
// 	return AB;
// }

// returns alpha, beta (gamma = 1 - alpha - beta)
Vec3 bary(Vec3 A, Vec3 B, Vec3 C, Vec3 P) {
	Vec3 v0 = B-A, v1 = C-A, v2 = P-A;
	double d00 = v0*v0, d01 = v0*v1, d11 = v1*v1, d20 = v2*v0, d21 = v2*v1;
	double dn = d00*d11-d01*d01;
	double b = (d11*d20-d01*d21)/dn, c = (d00*d21-d01*d20)/dn;
	double a = 1.0 - b - c;
	return Vec3(a,b,c);
}

Vec3 proj(Vec3 u, Vec3 v) {
	return v*((u*v)/(v*v));
}

Vec3 projLine(Vec3 p, Vec3 p0, Vec3 p1) {
	return p0+proj(p-p0,p1-p0);
}

Vec3 projSegment(Vec3 p, Vec3 p0, Vec3 p1) {
	double s = ((p-p0)*(p1-p0))/((p1-p0)*(p1-p0));
	return p0+min(1.0, max(0.0, s))*(p1-p0);
}

Vec3 projPlane(Vec3 p, Vec3 n, double d) {
	double k = -d/n.SQmag();
	Vec3 o = n*k; Vec3 v = p-o;
	double s = (v*n)/(n*n);
	return p-n*s;
}

struct Quaternion {
	double s;
	Vec3 v;
	Quaternion(double _s = 0, Vec3 _v = Vec3(0,0,0)): s(_s), v(_v) {}
	Quaternion operator+(const Quaternion &rhs) const {return Quaternion(s+rhs.s, v+rhs.v);}
	Quaternion operator-(const Quaternion &rhs) const {return Quaternion(s-rhs.s, v-rhs.v);}
	Quaternion operator*(const Quaternion &rhs) const {return Quaternion(s*rhs.s-v*rhs.v, rhs.v*s + v*rhs.s + v%rhs.v);}
	Quaternion operator*(double c) const {return Quaternion(s * c, v * c);}
	Quaternion operator/(double c) const {return Quaternion(s/c, v/c);}
	Quaternion conj() const {return Quaternion(s, -v);}
	Quaternion operator!() const {return conj();}
	double sqmag() const {return s*s + v*v;}
	double mag() const {return sqrt(sqmag());}
	Quaternion inv() const {return Quaternion{conj()/sqmag()};}
	Quaternion operator/(const Quaternion &rhs) const {
		Quaternion r = rhs.inv();
		return Quaternion{s*r.s-v*r.v, r.v*s + v*r.s + v%r.v};
	}
	Quaternion operator^(const Quaternion &rhs) const {
		// q(this)^p(rhs)
		if (sqmag() == 0) return Quaternion(0,Vec3(0,0,0));
		Vec3 tv = v;
		Vec3 qvn = (tv.mag() <= 1e-9 ? tv : tv/tv.mag());
		Quaternion lnq(log(mag()),qvn*atan2(tv.mag(), s));
		Quaternion omega = rhs*lnq;
		Vec3 ovn = (omega.v.mag() <= 1e-9 ? omega.v : omega.v/omega.v.mag());
		Quaternion result = Quaternion(cos(omega.v.mag())*exp(omega.s), ovn*sin(omega.v.mag())*exp(omega.s));
		return result;
	}
};
ostream& operator<<(ostream& o, const Vec3& v) {
	o << v.x << ' ' << v.y << ' ' << v.z;
	return o;
}
istream& operator>>(istream& i, Vec3& v) {
	i >> v.x >> v.y >> v.z;
	return i;
}
ostream& operator<<(ostream &o, const Quaternion &q) {
	o << q.s << ' ' << q.v.x << ' ' << q.v.y << ' ' << q.v.z;
	return o;
}
istream& operator>>(istream &i, Quaternion &p) {
	i >> p.s >> p.v.x >> p.v.y >> p.v.z;
	return i;
}


// 2D only
Vec3 to_implicit(Vec3 u, Vec3 v) {return u%v;} 
pair<Vec3, Vec3> to_four_numbers(Vec3 u, bool type) {
	double d = -u.z/(u*u-u.z*u.z);
	return make_pair(Vec3(d*u.x, d*u.y, 1), Vec3(d*u.x*type + u.y, d*u.y*type - u.x, type));
}

//distances
/*
	f0(t) = p0+tu
	f1(s) = p1+sv
	n = u%v
	s = (P0-P1)*(n%u)/v*(n%u)
	t = (P1-P0)*(n%v)/u*(n%v)
*/

int main () {

	cout.precision(6);
	cout << fixed;

	/* // 1 AC (6 pts)

		// The canonical (magnitude of the direction is 1) Plucker form of the line
		// (<1, -0.5, 0>:<0.5, 1, 1>) translated by <1, -1, 0> is:

		Vec3 d(1, -0.5, 0), m(0.5, 1, 1), t(1, -1, 0);
		Vec3 p0 = (d%m)/(d*d);
		p0 = p0+t;
		m = (p0%d)/d.mag();
		d = d/d.mag();
		cout << d << endl << m << endl;

	// */

	/* // 2 AC (6 pts)

		// The GJK algorithm is performed on polygons A and B with the Minkowski sum D computed as D = A - B.
		// Suppose the closest edge of D to the origin is {(3, 2), (2, -4)} with the corresponding edge on B being {(1, -1), (1, -1)}.
		// What's the point on A that's closest to B?

		Vec3 B(1,-1), D0(3,2), D1(2,-4);
		Vec3 A0 = D0+B, A1 = D1+B;
		Vec3 pt_on_A = projSegment(B, A0, A1);
		cout << pt_on_A << endl;

	// */

	/* // 3 AC (6 pts)

		// Given parallel lines (<1, -0.5, 2>:<-1, 2, 1>) and (<1, -0.5, 2>:<0, 4, 1>), find the plane containing both lines, with the normal facing the origin, and magnitude is 1.

		Vec3 d0(1, -0.5, 2), m0(-1, 2, 1), d1(1, -0.5, 2), m1(0, 4, 1);
		Vec3 p0 = (d0%m0)/(d0*d0), p1 = (d1%m1)/(d1*d1);
		Vec3 pn = p0+d0;
		Vec3 n = (pn-p0)%(p1-p0);
		if (p0*n > 0) n = -n;
		double d = (-p0*n)/n.mag();
		n = n/n.mag();
		cout << n << ' ' << d << endl;

	// */

	/* // 4 AC (6 pts)

		// Suppose the earth is a perfect sphere with a radius 6,300km (ha!), what is the distance along the surface of the earth (in km) between locations <14.5, 10> and <15, 30> (the coordinates are ordered longitude, latitude)?
		// Give the answer rounded to the nearest kilometer.
		// (hint: one way to do this is to convert longitude-latitude to 3D-vectors)

		double r = 6300;
		double az0 = 14.5*RADIANS, el0 = 10*RADIANS, az1 = 15*RADIANS, el1 = 30*RADIANS;
		Vec3 A(cos(el0)*cos(az0), sin(el0), cos(el0)*sin(az0)), B(cos(el1)*cos(az1), sin(el1), cos(el1)*sin(az1));
		double ans = acos((A*B)/A.mag()/B.mag());
		cout << ans*r << endl;

	// */

	/* // 5 AC (12 pts) 

		// There are 5 enemies:
		// (1, -1, 0.3)
		// (1.25, -0.8, 0.4)
		// (2.3, 0.5, 0.1)
		// (1.2, -1.3, 1.2)
		// (-0.5, -1.2, 1.1)
		// a) If a player at (-1, 2, 1) fires a cone (of cold) spell with a total spread of 30 degrees (so 15 degrees spread from the center) towards enemy A, who else gets hit? 
		// b) Suppose the entire field is projected to the x-y plane. If the player wants to cast Fireball (a circular AOE spell) to hit enemies A, B, and C, what's the minimum radius of the spell such that all three enemies are hit (assuming the spell can be targeted anywhere)?

		Vec3 A(1, -1, 0.3),
			 B(1.25, -0.8, 0.4),
			 C(2.3, 0.5, 0.1),
			 D(1.2, -1.3, 1.2),
			 E(-0.5, -1.2, 1.1);

		Vec3 P(-1,2,1);

		Vec3 v = A-P;
		v = v/v.mag();

		double aB = acos(v*(B-P)/(B-P).mag());
		double aC = acos(v*(C-P)/(C-P).mag());
		double aD = acos(v*(D-P)/(D-P).mag());
		double aE = acos(v*(E-P)/(E-P).mag());	

		if (aB*DEGREES <= 15) cout << 'B' << endl;
		if (aC*DEGREES <= 15) cout << 'C' << endl;
		if (aD*DEGREES <= 15) cout << 'D' << endl;
		if (aE*DEGREES <= 15) cout << 'E' << endl;

		// research why
		Vec3 pA(A.x, A.y), pB(B.x, B.y), pC(C.x, C.y);
		Vec3 v0(pA.x*pA.x+pA.y*pA.y, pB.x*pB.x+pB.y*pB.y, pC.x*pC.x+pC.y*pC.y), v1(A.x, B.x, C.x), v2(A.y, B.y, C.y), v3(1,1,1);
		double x2y2 = (v1%v2)*v3, x = (v0%v2)*v3, y = (v0%v1)*v3, r = (v0%v1)*v2;
		double x0 = 0.5*x/x2y2, y0 = -0.5*y/x2y2;
		cout << x0 << ' ' << y0 << endl;
		double radius = sqrt(x0*x0+y0*y0+r/x2y2);
		cout << radius << endl;

	// */

	/* // 6 AC (6 pts)

		// Suppose there's a plane 3x + y + z + 1 = 0.
		// At time T = 1, it is oriented by 30 degrees about the axis <1,0,1> and at time T = 11, it is oriented by 45 degrees about the axis <0, -1, 0>.
		// If Quaternions are used to interpolate the plane between two orientations, give the implicit form of the rotated plane at T = 3. Make sure that the magnitude and orientation of the normal of the plane is preserved (i.e. it does not "scale" or "flip").

		Vec3 plane(3,1,1); double d = 1;
		Vec3 closest_point = plane*(-d/plane.SQmag());
		Vec3 axis0(1,0,1), axis1(0,-1,0);
		axis0 = axis0/axis0.mag();
		axis1 = axis1/axis1.mag();
		double rot0 = 30*RADIANS/2.0, rot1 = 45*RADIANS/2.0;
		Quaternion a(cos(rot0), sin(rot0)*axis0), b(cos(rot1), sin(rot1)*axis1);
		Quaternion temp = ((b*(a.inv()))^Quaternion(2.0/10.0,Vec3(0,0,0)))*a;
		cout << temp << endl;
		Quaternion ansN = temp*Quaternion(0,plane)*(temp.inv());
		Quaternion ansP = temp*Quaternion(0,closest_point)*(temp.inv());

		double d1 = -ansP.v*ansN.v;

		cout << ansN.v << ' ' << d1 << endl;

	// */

	/* // 7 AC (18 pts)

		// A ball at (10, 20, 6) with radius 5 is moving at a velocity of <-1, -3, -4> per second. There's also a plane which contains the points A: (0, 0, 0), B: (-1, 1, 1), and C: (2, 3, -5).
		// a. How long will it take for the ball to hit the plane? 
		// b. What's the barycentric coordinates w.r.t. △ABC of the ball's point of contact with the plane? 
		// c. What's the resulting velocity of the ball when it bounces off the plane? 

		Vec3 ball_center(10,20,6), vel(-1,-3,-4);
		double radius = 5.0;
		Vec3 A(0,0,0), B(-1,1,1), C(2,3,-5);

		Vec3 plane = (B-A)%(C-A);
		plane = plane/plane.mag();
		int up_or_down = (plane*ball_center > 0 ? 1 : -1);
		double t = ((-ball_center*plane)+up_or_down*5*(plane*plane))/(vel*plane);

		cout << t << endl;

		cout << bary(A, B, C, ball_center+up_or_down*5*plane+t*vel) << endl;

		cout << vel-2*proj(vel, plane*up_or_down) << endl;

	// */

	/* // 8 AC (12 pts)

		// Given the sequence of points:
		// (2.09, 1.53, 2.63),
		// (-3.37, 0.71, 2.40),
		// (-2.93, -3.05, -2.89),
		// (-1.18, 0.24, -0.83),
		// (3.71, -2.06, -3.30),
		// (1.14, -0.02, 1.06)
		// project them to the plane x + 2y + 2 = 0:
		// a. If the projected points trace a trajectory of an object traveling at 2 units per second, where is the object after 4 seconds? (assume that the object starts at (2.09, 1.53, 2.63)): 
		// b. Compute the unsigned area of the polygon formed by the points projected to the plane 
		// * <-0.0305391, -0.98473, -1.55048> (distance traveled: 14 at segment 3)

		int n = 6;

		Vec3 pts[n] = {
			Vec3(2.09, 1.53, 2.63),
			Vec3(-3.37, 0.71, 2.40),
			Vec3(-2.93, -3.05, -2.89),
			Vec3(-1.18, 0.24, -0.83),
			Vec3(3.71, -2.06, -3.30),
			Vec3(1.14, -0.02, 1.06)
		};

		Vec3 plane(1,2,0); double d = 2;
		d /= plane.mag();
		plane = plane/plane.mag();

		Vec3 proj_pts[n];

		// project the points
		for (int i = 0; i < n; i++) 
			proj_pts[i] = projPlane(pts[i], plane, d);

		// projected distances
		double distances[n]; distances[0] = 0;
		for (int i = 1; i < n; i++)
			distances[i] = (proj_pts[i]-proj_pts[i-1]).mag();

		double prefix_distances[n]; prefix_distances[0] = 0;
		for (int i = 1; i < n; i++)
			prefix_distances[i] = prefix_distances[i-1]+distances[i];

		double dist_traveled = 2*7; 

		cout << dist_traveled << endl;
		
		int segment = 0;
		for (int i = 0; i < n; i++)
			if (dist_traveled <= prefix_distances[i]) {
				segment = i;
				break;
			}

		double t = (dist_traveled - prefix_distances[segment-1]) / distances[segment];
		Vec3 location = proj_pts[segment-1]+t*(proj_pts[segment]-proj_pts[segment-1]);
		cout << location << endl;

		Vec3 pt_on_plane = plane*(-d/plane.SQmag());
		Vec3 temp(0,0,0);
		double ans = 0;
		for (int i = 0, j = n-1; i < n; j = i++) {
			temp = temp + ((proj_pts[i]-pt_on_plane)%(proj_pts[j]-pt_on_plane));
		}
		ans = temp*plane;
		cout << ans/2.0 << endl;
	// */

	return 0;
}