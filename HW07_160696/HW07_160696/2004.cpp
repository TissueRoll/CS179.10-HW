#include <bits/stdc++.h>
using namespace std;

const double PI = acos(-1);
const double conversion = PI/180.0;
const double EPS = 1e-9;
const double wtf = 1e9;

int main() {
	cout.precision(0);
	cout << fixed;
	double D, H; cin >> D >> H;
	double a, b, g;
	while(1) {
		cin >> a >> b >> g;
		if (a == 0 || b == 0 || g == 0) break;
		double A = tan(a*conversion);
		double B = tan(b*conversion);
		double G = tan(g*conversion);
		A = 1.0/(A*A); B = 1.0/(B*B); G = 1.0/(G*G);
		double temp = A+G-2.0*B;
		cout << (H+D*sqrt(2.0/temp)) << endl;
	}
	return 0;
}