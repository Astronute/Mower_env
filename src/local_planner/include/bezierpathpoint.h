#pragma once


class FrenetPoint{

public:

  FrenetPoint() : s(0), d(0), d_d(0), d_dd(0), width(0), length(0), t(0){}
  FrenetPoint(double _s, double _d, double _d_d, double _d_dd, double _width, double _length, double _t):
            s(_s), d(_d), d_d(_d_d), d_dd(_d_dd), width(_width), length(_length), t(_t) {}

  FrenetPoint(const FrenetPoint& frenet_point) {
    s = frenet_point.s;
    d = frenet_point.d;    
    d_d = frenet_point.d_d;  
    d_dd = frenet_point.d_dd;  
    width = frenet_point.width;  
    length = frenet_point.length;
    t =  frenet_point.t;        
  }   
  ~FrenetPoint(){};
  double d, d_d, d_dd;
  
  double width, length;   
  
  double s, t;

  void setS(double _s) {
    s = _s;
  }

  void setD(double _d) {
    d = _d;
  }

  void setD_d(double _d_d) {
    d_d = _d_d;
  }

  void setD_dd(double _d_dd) {
    d_dd = _d_dd;
  }
  
  void setWidth(double _width) {
    width = _width;
  }

  void setLength(double _length) {
    length = _length;
  }

  void setT(double _t) {
    t = _t;
  }

};

/*! A light-weight path point with fields x, y, yaw, kappa, s */
class PathPoint{

public:

  PathPoint() :  x(0), y(0), yaw(0), kappa(0), s(0){}
  PathPoint(double _x, double _y, double _yaw, double _kappa, double _s):
            x(_x), y(_y), yaw(_yaw), kappa(_kappa), s(_s) {}

  PathPoint(const PathPoint& path_point) {
    x = path_point.x;
    y = path_point.y;    
    yaw = path_point.yaw;  
    kappa = path_point.kappa;  
    s = path_point.s;
       
  }
  ~PathPoint(){}; 
  // x, y, yaw
  double x, y, yaw;
  // curvature on the x-y planning
  double kappa;   
  // accumulated distance from beginning of the path
  double s;


  void setX(double _x) {
    x = _x;
  }

  void setY(double _y) {
    y = _y;
  }

  void setYaw(double _yaw) {
    yaw = _yaw;
  }

  void setKappa(double _kappa) {
    kappa = _kappa;
  }
  
  void setS(double _s) {
    s = _s;
  }
};


/*! A light-weight traj point with fields path_point, v, w, t */
class TrajPoint{

public:

  TrajPoint() : path_point(0, 0, 0, 0, 0), frenet_point(0, 0, 0, 0, 0, 0, 0), v(0), w(0), t(0) {}
  TrajPoint(PathPoint _path_point, FrenetPoint _frenet_point, double _v, double _w, double _t):
                 path_point(_path_point), frenet_point(_frenet_point), v(_v), w(_w), t(_t) {}
  ~TrajPoint(){};
  // x, y, theta
  FrenetPoint frenet_point;  
  PathPoint path_point;
  // linear vel
  double v;
  // angular vel
  double w;
  // time
  double t;  

  void setPathPoint(PathPoint _path_point)
  {
    path_point.setX(_path_point.x);
    path_point.setY(_path_point.y);  
    path_point.setYaw(_path_point.yaw);
    path_point.setKappa(_path_point.kappa);
    path_point.setS(_path_point.s);  
  }

  void setFrenetPoint(FrenetPoint _frenet_point)
  {
    frenet_point.setS(_frenet_point.s);
    frenet_point.setD(_frenet_point.d);  
    frenet_point.setD_d(_frenet_point.d_d);
    frenet_point.setD_dd(_frenet_point.d_dd);
    frenet_point.setWidth(_frenet_point.width);  
    frenet_point.setLength(_frenet_point.length); 
    frenet_point.setT(_frenet_point.t); 
  }

  void setV(double _v) {
    v = _v;
  }

  void setW(double _w) {
    w = _w;
  }

  void setT(double _t) {
    t = _t;
  }

};