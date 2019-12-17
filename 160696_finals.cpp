/*
	--Spherical--
	x = cosTcosP	r = sqrt(x^2, y^2, z^2)
	y = cosTsinP	T = acos(z/r)
	z = sinT		P = atan2(y,x)

*/

typedef vector<vector<double>> vvd;
vvd multiply(vvd A, vvd B) {
	int p = A.size(), q = A[0].size(), r = B[0].size();
	// req: q == B.size()
	vvd AB(p, r); //pseudo
	for (int i = 0; i < p; i++)
	for (int j = 0; j < q; j++)
	for (int k = 0; k < r; k++)
		(AB[i][k] += A[i][j]*B[j][k]);
	return AB;
}

// returns alpha, beta (gamma = 1 - alpha - beta)
pair<double,double> bary(Vec3 A, Vec3 B, Vec3 C, Vec3 P) {
	Vec3 v0 = B-A, v1 = C-A, v2 = P-A;
	double d00 = v0*v0, d01 = v0*v1, d11 = v1*v1, d20 = v2*v0, d21 = v2*v1;
	double dn = d00*d11-d01*d01;
	return {(d11*d20-d01*d21)/dn, (d00*d21-d01*d20)/dn};
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
	Vec3 o = n*k; Vec3 = p-o;
	double s = (v*n)/(n*n);
	return p+o+n*s;
}

struct Quaternion {
	double s;
	Vec3 v;
	Quaternion operator+(const Quaternion &rhs) const {return Quaternion{s+rhs.s, v+rhs.v};}
	Quaternion operator-(const Quaternion &rhs) const {return Quaternion{s-rhs.s, v-rhs.v};}
	Quaternion operator*(const Quaternion &rhs) const {return Quaternion{s*rhs.s-v*rhs.v, rhs.v*s + v*rhs.s + v%rhs.v};}
	Quaternion operator*(double c) const {return Quaternion{s * c, v * c};}
	Quaternion operator/(double c) const {return Quaternion{s/c, v/c};}
	Quaternion conj() const {return Quaternion{s, -v};}
	Quaternion operator!() const {return conj();}
	double mag() const {return sqrt(sqmag());}
	double sqmag() const {return s*s + v*v;}
	Quaternion inv() const {return Quaternion{conj()/sqmag()};}
	Quaternion operator/(const Quaternion &rhs) const {
		Quaternion r = rhs.inv();
		return Quaternion{s*r.s-v*r.v, r.v*s + v*r.s + v%r.v};
	}
	Quaternion operator^(const Quaternion &rhs) const {
		// q(this)^p(rhs)
		if (sqmag() == 0) return Quaternion{0,0,0,0};
		Vec3 qvn = (v.mag() <= 1e-9 ? v : v/v.mag());
		Quaternion lnq{log(mag()),qvn*atan2(v.mag(), s)};
		Quaternion omega = rhs*lnq;
		Vec3 ovn = (omega.v.mag() <= 1e-9 ? omega.v : omega.v/omega.v.mag());
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

// 2D only
Vec3 to_implicit(Vec3 u, Vec3 v) {return u%v;} 
pair<Vec3, Vec3> to_four_numbers(Vec3 u, bool type) {
	double d = -u.z/(u*u-u.z*u.z);
	return make_pair(Vec3(d*u.x, d*u.y, 1), Vec3(d*u.x*type + u.y, d*u.y*type - u.x, type));
}

//distances
