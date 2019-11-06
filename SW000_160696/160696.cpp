#include <bits/stdc++.h>
using namespace std;

struct Vector2D {
	double x, y;
	Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
};
struct Line {
	Vector2D p0, p1;
	Line() {};
	Line(const Vector2D &p0, const Vector2D &p1) : p0(p0), p1(p1) {};
};

double distance(const Vector2D &a, const Vector2D &b) { return 0; }
double distance(const Line &l, const Vector2D &p) { return 0; }
double proj(const Vector2D &base, const Vector2D &target) { return 0; }
Vector2D projVec(const Vector2D &base, const Vector2D &target) { return Vector2D(); }

vector<Vector2D> lineSimplify (const vector<Vector2D>& polyline, double epsilon) {
	if (polyline.size() <= 2) {
		return polyline;
	}

	Vector2D p0 = polyline.front(), pn = polyline.back();
	Line ref_line = Line(p0,pn);
	double max_distance = 0;
	int index = 0;
	for (int i = 1; i < polyline.size()-1; i++) {
		double dist_to_line = distance(ref_line, polyline[i]);
		if (max_distance < dist_to_line) {
			max_distance = dist_to_line;
			index = i;
		}
	}
	
	vector<Vector2D> result;
	if (max_distance >= epsilon) {
		vector<Vector2D> l, r;
		for (int i = 0; i <= index; i++)
			l.push_back(polyline[i]);
		for (int i = index; i < polyline.size(); i++)
			r.push_back(polyline[i]);
		vector<Vector2D> l_new = lineSimplify(l, epsilon);
		vector<Vector2D> r_new = lineSimplify(r, epsilon);
		for (int i = 0; i < l_new.size(); i++)
			result.push_back(l_new[i]);
		for (int i = 1; i < r_new.size(); i++) // no need for 0 index cuz its included
			result.push_back(r_new[i]);
	} else {
		result.push_back(p0);
		result.push_back(pn);
	}
	
	return result;
}

int main () {
	return 0;
}