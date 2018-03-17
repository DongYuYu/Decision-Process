#include<vector>
//#include<unordered_map>
#include<set>
#include<utility>
#include<map>
#include<iostream>
#include<algorithm>
#include<limits>
#include<cstdlib>
#include<stack>
using namespace std;
enum Orientation {left,up, right, down};
//enum Orientation {right,up,left,down};
class Point{
  int x, y;

public:
  Point(int x, int y):x(x), y(y){
  }
  bool operator==(const Point &rhs) const {
    if (x==rhs.getX() && y ==rhs.getY()) return true;
    else return false;}
  int getX () const {return x;}
  int getY () const {return y;}
  Point add(Orientation action){
    int a = x;
    int b = y;
    switch(action){
    case ::left: b--; break;
    case ::up: a++; break;
    case ::right: b++; break;
    case ::down: a--; break;


    }
    return Point(a,b);
  }

};

Orientation turn_left(Orientation action){
  //cout<<sizeof(Orientation);
  return (Orientation)(action-1+sizeof(Orientation)%sizeof(Orientation));
}

Orientation turn_right(Orientation action){
  return (Orientation)(action+1+sizeof(Orientation)%sizeof(Orientation));
}  
struct PointCompare
{
   bool operator() (const Point& lhs, const Point& rhs) const
   {
     if (lhs.getX()==rhs.getX()) return lhs.getY()<rhs.getY();
     else return lhs.getX() < rhs.getX();
     // return lhs.getX() < rhs.getX();
   }
};

struct OrientationCompare
{
   bool operator() (const Orientation& lhs, const Orientation& rhs) const
   {
     //     if (lhs.getX()==rhs.getX()) return lhs.getY()<rhs.getY();
     // else return lhs.getX() < rhs.getX();
      return lhs < rhs;
   }
};
 
 

class UAV{

  set<Point, PointCompare> states;
  map<Point, double, PointCompare> reward;
  vector<Orientation> actlist;
  //  unordered_map<Point, unordered_map<Orientation, vector<pair<double, Point>>>> transitions;
  map<Point, map<Orientation, vector<pair<double, Point>>, OrientationCompare>, PointCompare> transitions;
  double gamma;
  map<Point, vector<Orientation>, PointCompare> actions;
  vector<Point> terminals;
  vector<Point> nstates;
  int row, coll;
public:
  UAV(vector<vector<double>> grid, vector<Point> terminals, Point init=Point(0,0), double gamma=0.99):gamma(gamma), terminals(terminals), row(grid.size()), coll(grid[0].size()){
    for (int i =0; i<grid.size(); i++){
      for (int j =0; j<grid[0].size(); j++){
	if (grid[i][j]) {
	  //Point * p= new Point(i,j);
	  Point p (i,j);
	  states.insert(p);
	  reward[p] = grid[i][j];
	}
	else {
	  Point p(i,j);
	  reward[p] = 0;
	  nstates.push_back(p);
	}
      }
    }
    
    actlist = {::left, ::up, ::right, ::down};

    for (Point state: states){
      auto it = find(terminals.begin(), terminals.end(), state); 
      if (it!=terminals.end()) {actions[state]=vector<Orientation>(); }//cout<<state.getX()<<","<<state.getY()<<endl;}
      else {actions[state]=actlist; }//for (int i =0; i<actions[state].size(); i++) cout<<actions[state][i];cout<<endl;}
      //actions[state]=actlist;
    }

    for (Point s: states){
      for (Orientation a: actions[s]){
	transitions[s][a]=calculateT(s,a);
	
      }
    }
    
  }

  auto getTransitions(){ return transitions;}
  auto getGamma(){return gamma;}
  auto getActions(){return actions;}
  auto getReward(){return reward;}
  auto getTerminal(){return terminals;}
  auto getStates(){return states;}
  auto getNstates(){return nstates;}
  auto getRow(){return row;}
  auto getColl(){return coll;}
  vector<pair<double, Point>> calculateT(Point state, Orientation action){
    // if (action){
    vector<pair<double, Point>> c={{0.85, takeAction(state, action)},
				   {0.075, takeAction(state, turn_right(action))},
				   {0.075, takeAction(state, turn_left(action))}}; 
    return c;

    //}else{
    // vector<pair<double, Point>> c ={{0, state}};
    // return c;
    //}
    
  }

  Point takeAction(Point state, Orientation action){
    Point state1 = state.add(action);
    return states.find(state1)!=states.end()?state1:state;
    //return state1;
  }
  vector<pair<double, Point>> T(Point state, Orientation action){
    return transitions[state][action];


  }

};

map<Point, double, PointCompare> value_iteration(UAV uav, double epsilon= 0.001){

  map<Point, double, PointCompare> U;
  auto T = uav.getTransitions();
  auto R = uav.getReward();
  double gamma = uav.getGamma();
  auto A = uav.getActions();
  auto S = uav.getStates();
  for (Point state: S){
    U[state]=0;


  }
  while (true){
    map<Point, double, PointCompare> U1 = U;
    double delta = 0;
    for (Point state: S){
     
      vector<double> EU;
      for (Orientation action: A[state]){
	
        double sum =0;
	for (auto p: T[state][action]){
	  sum+=p.first*U1[p.second];
	}
	EU.push_back(sum);
      }

      auto it = max_element(EU.begin(), EU.end());
      if (it != EU.end())
	U[state] = R[state] +gamma* (*it);
      else
	U[state] = R[state];
      
      delta = max(delta, abs(U[state] - U1[state]));

    }
    
    if (delta <epsilon*(1-gamma)/gamma) return U1;

  }


}
double EUY (Orientation action, Point state, map<Point, double, PointCompare> U, UAV uav){
   

  auto T = uav.getTransitions();
  
  double sum =0;
  for (auto p: T[state][action]){
    sum+=p.first*U[p.second];
  }
  return sum;




      
 

}


map<Point, Orientation, PointCompare> best_policy(UAV uav, map<Point, double, PointCompare> U){
  /*
  auto a = uav.getActions();
  for (auto state: uav.getStates()){
    for (auto action: a[state]) {
      //for (int i =0; i<action.size(); i++)
	cout<<action;

    }
    cout<<endl;

  }

  */
  map<Point, Orientation, PointCompare> policy;

  auto a = uav.getActions();
  for (auto state: uav.getStates()){
    //cout<<uav.getActions()[state][0];
    map <double, Orientation> table;
    double value=numeric_limits<double>::max()*(-1);
    //    cout<<"for"<<state.getX()<<","<<state.getY()<<":"<<endl;
    for (auto action: a[state]){
      //      cout<<(int)action<<endl;
      double euv=EUY(action, state, U, uav);
      //cout<<"hi"<<endl;
      //      cout<<euv<<","<<(int)action<<endl;
      //  cout<<state.getX()<<","<<state.getY()<<endl;
      table[euv]=action;

      //cout<<"v"<<value<<endl;
      //cout<<"e"<<euv<<endl;
            if ( value < euv ) {value=euv;}
    }

    //    for (auto c: table)
      //cout<<c.first<<","<<c.second<<endl;
      //cout<<"value "<<value<<endl;
    cout<<state.getX()<<","<<state.getY()<<endl;
    cout<<"EUV"<<value<<endl;
    policy[state]=table[value];
    //    cout<<(int)table[value]<<endl;
  }
  return policy;

}

map<Point, double, PointCompare> &policy_evaluation(map<Point, Orientation, PointCompare> p, map<Point, double, PointCompare> &U, UAV &uav, int k=230){
  //  int j;
  auto R = uav.getReward();
  //for (auto r: R) cout<<"R "<<r.second<<endl;
  //cin>>j;
  auto T = uav.getTransitions();
  auto gamma = uav.getGamma();
  auto S = uav.getStates();
  auto P = p;
  
  for (int i =0; i<k; i++){

    for (Point state : S){
      double sum =0;
      for (auto t: T[state][P[state]]){

	sum+= t.first*U[t.second];
	//cout<<"tf  "<<t.first<<endl;
	//cout<<"sum  "<<sum<<endl;
	//if (sum>20) cin>>j;
      }
      //      cout<<"sum "<<sum<<endl;
      //cout<<R[state]<<",,"<<gamma<<","<<sum<<endl;
      //if (sum>20)  cin>>j;
      U[state]= R[state] + gamma*sum;

   }

  }
  for (auto u: U) cout<<u.first.getX()<<","<<u.first.getY()<<":"<<u.second<<endl;
  return U;

}

map<Point, Orientation, PointCompare> policy_iteration(UAV uav){
  auto S = uav.getStates();
  auto a = uav.getActions();
  map <Point, double, PointCompare> U;
  
  map<Point, Orientation, PointCompare> p;
  for (Point state: S) {
    U[state] =0; 
    p[state] =(Orientation) (rand()%(sizeof(Orientation)));
    //   p[state]=(Orientation)0;
    //cout<<a[state].size();
  }
  //  for (auto u: U) cout<<"u  "<<u.second<<endl;

  int a1;
  //  cin>>a1;
  int unchanged;
  while (true){

    U = policy_evaluation(p, U, uav);
    unchanged ++;
    for (auto state: S){
    //cout<<uav.getActions()[state][0];
    map <double, Orientation> table;
    double value=numeric_limits<double>::max()*(-1);
    //cout<<"for"<<state.getX()<<","<<state.getY()<<":"<<endl;
      for (auto action: a[state]){
      //      cout<<(int)action<<endl;
	double euv=EUY(action, state, U, uav);
      //cout<<"hi"<<endl;
      //      cout<<euv<<","<<(int)action<<endl;
      //  cout<<state.getX()<<","<<state.getY()<<endl;
	table[euv]=action;

	//      cout<<"v"<<value<<endl;
	//cout<<"e"<<euv<<endl;
        if ( value < euv ) {value=euv;}
      }

    //    for (auto c: table)
      //cout<<c.first<<","<<c.second<<endl;
      //cout<<"value "<<value<<endl;
  
      if (table[value]!= p[state]){
	p[state]=table[value];
	unchanged = 0;
      }
  
    }
    if (unchanged>0) return p;
  }
}

void printPolicy(map<Point, Orientation, PointCompare> p, UAV uav){

  map<Point, char, PointCompare> prepare;
  


  stack<vector<pair<Point, char>>> s;
  vector<pair<Point, char>> v;
  auto nstate = uav.getNstates();
  auto terminal = uav.getTerminal();
  auto col = uav.getColl();
  for (auto policy: p){
      switch(policy.second){

      case ::left: prepare[policy.first]='<';break;
      case ::up: prepare[policy.first]='o';break;
      case ::right: prepare[policy.first]='>'; break;
      case ::down: prepare[policy.first]= 'l';break;


      }

    } 
  for (Point s: nstate) prepare[s]='.';
  for (Point s: terminal) prepare[s]='.';
  
  
  for (auto n: terminal) cout<<n.getX()<<", "<<n.getY()<<endl;
  int count=0;
  for (pair<Point, char> policy: prepare){
    
    v.push_back(policy);

    if(count==(col-1)){
      s.push(v);
      v.clear();
      count=-1;
    }
    count++;

  }
  while (!s.empty()){

    auto p = s.top();
    s.pop();
    for (auto policy:p){
      cout<<policy.second;
    }
      cout<<endl;
  }

}


int main(){
  
  vector<vector<double>> grid = {{-0.05, -0.05, -0.05, +1},
				 {-0.05, -0.05, -0.05, -0.05},
				 {-0.05, NULL,-0.05, -0.05},
				 {-1, -0.05,-0.05, +1}};

  vector<Point> terminal = {Point(3,0), Point(3,3), Point(0,3)};
  
  /*
  vector<vector<double>> grid = {{-0.04, -0.04, -0.04, -0.04},
				 {-0.04, NULL, -0.04, -1},
				 {-0.04, -0.04, -0.04, +1}};
  vector<Point> terminal = {Point(1,3), Point(2,3)};

  */
  UAV uav(grid, terminal);
  auto policy = policy_iteration(uav);
  //  auto policy=best_policy(uav, value_iteration(uav,0.001));
  // auto a = value_iteration(uav);
  // for (auto u: a){
    
  //cout<<"("<<u.first.getX()<<","<<u.first.getY()<<"),"<<u.second<<endl;

  //}
        for (auto p:policy)
      cout<<"("<<p.first.getX()<<","<<p.first.getY()<<"),"<<p.second<<endl;
	printPolicy(policy, uav);
  return 0;
}
								      
