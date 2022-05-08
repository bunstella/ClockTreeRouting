#pragma once

#include "global.h"

namespace gr {

struct DirectEnum
{
    enum DirectType 
    {
        UP = 0, 
        DOWN = 1, 
        LEFT = 2,
        RIGHT = 3,
    };

    int x_off[4] = {0, 0, -1, 1};
    int y_off[4] = {1, -1, 0, 0}; 
};

const DirectEnum Direction;

class pin_dist {
public:
    double dist;
    int idx;

    pin_dist(double d, int i) : 
                dist(d), idx(i){}
};

class pin_assign_tap {
public:
    int _id;
    int _to;
    int _dist;
    pin_assign_tap(int i, int d, int dist) :
                    _id(i), _to(d), _dist(dist) {}
};

class GRPoint: public utils::PointT<int> {
public:
    // int x
    // int y
    GRPoint(int _x, int _y): utils::PointT<int>(_x, _y) {}
    GRPoint(db::Point pos): utils::PointT<int>(pos._x, pos._y) {}
    friend inline std::ostream& operator<<(std::ostream& os, const GRPoint& pt) {
        os << "(" << pt.x << ", " << pt.y << ")";
        return os;
    }
};

class GRNet {
private:
    int _id = -1;

public:
    GRNet() {}
    GRNet(int id) : _id(id) {}
    const int id() {return _id;}
    vector<GRPoint> pinPoints;
};

struct Edge {
public:
    db::Point from;
    db::Point to;

public:
    Edge(int x_1, int y_1, int x_2, int y_2) 
        {from = db::Point(x_1,y_1); to = db::Point(x_2,y_2);}
    Edge(db::Point p1, db::Point p2) : from(p1), to(p2) {}
    Edge(utils::PointT<int> p1, utils::PointT<int> p2) 
        {from = db::Point(p1.x,p1.y); to = db::Point(p2.x,p2.y);}

    friend inline std::ostream& operator<<(std::ostream& os, const Edge& eg) {
        os << eg.from._x << ' ' << eg.from._y << ' ' << 
                eg.to._x << ' ' << eg.to._y << ' ';
        return os;
    }
};

class TwoPinNet {
public:
    int _id = -1;
    int p_id = -1;
public:
    vector<db::Point> Pins;

    TwoPinNet(db::Point p0, db::Point p1){Pins.push_back(p0);Pins.push_back(p1);}
    TwoPinNet(int i, int p, db::Point p0, db::Point p1) : _id(i),p_id(p) {Pins.push_back(p0);Pins.push_back(p1);}
    TwoPinNet(int p, db::Point p0, db::Point p1) : p_id(p) {Pins.push_back(p0);Pins.push_back(p1);}
    TwoPinNet(int lhs_x,int lhs_y,int rhs_x,int rhs_y){Pins.emplace_back(lhs_x,lhs_y);Pins.emplace_back(rhs_x,rhs_y);}
    TwoPinNet(const int id) : _id(id) { }
    ~TwoPinNet() {Pins.clear();}
    
    int HPWL() {return (Pins[0] - Pins[1]);}

    friend ostream& operator<<(ostream& os, const TwoPinNet& c) {
        return os << c._id << "\n";
    }
};

class NetPrior {
public:
    int cost;
    int _id;

    NetPrior(int c, int i) : 
                cost(c), _id(i){}
};

class Vertex {
public:
    int cost;
    db::Point pos;
    std::shared_ptr<Vertex> prev;

    int direction;

    Vertex(int c, db::Point v, const std::shared_ptr<Vertex> &p, int d) : 
                cost(c), pos(v), prev(p), direction(d) {}
};

template <typename T>
struct CPoint
{   
    CPoint() {}
    CPoint(T x, T y)
        : _x(x), _y(y) {}
    T _x = 0;
    T _y = 0;
    int _c = -1;

    friend std::ostream& operator<<(std::ostream& os, const CPoint& c) {
        return os << "(" << c._x << ", " << c._y << ")";
    }
    T operator-(CPoint b) const{
        return (abs(_x - b._x) + abs(_y - b._y));
    }
    CPoint& operator+=(const CPoint& rhs) {
        _x += rhs._x;
        _y += rhs._y;
        return *this;
    }
    CPoint& operator/=(const double d) {
        _x /= d;
        _y /= d;
        return *this;
    }
};

}