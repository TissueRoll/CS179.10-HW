#include <bits/stdc++.h>
using namespace std;

typedef pair<double,double> pdd;

struct line {
	pdd a, b; int ep, index;
	bool operator<(const line& rhs) {
		if (a.first == rhs.a.first) return ep < rhs.ep;
		return a.first < rhs.a.first;
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

bool intersecting(line a, line b) {
	bool ok = 1;
	
	return ok;
}

int main () {
	int T, n;
	pdd p1, p2;
	cin >> T;
	while (T--) {
		cin >> n;
		vector<line> v;
		vector<pdd> pts;
		for (int i = 0; i < n; i++) {
			cin >> p1 >> p2;
			if (p2 < p1)
				swap(p1,p2);
			v.push_back(line{p1,p2,0,i+1});
			pts.push_back(line{p1,p2,0,i+1});
			pts.push_back(line{p2,p1,1,i+1});
		}
		sort(pts.begin(), pts.end());
		set<pair<int,int>> ans;
		set<int> s;
		s.insert(pts[0].index)
		for (int i = 1; i < pts.size(); i++) {
			line x = pts[i];
			if (x.ep == 0) {
				for (int ind : pts) 
					if (intersecting(v[ind],v[x.index]))
						ans.insert({min(ind,x.index),max(ind,x.index)});
				s.insert(x.index);
			} else {
				s.erase(x.index);
			}
		}

	}
	return 0;
}