#include "global.h"
#include "db/Database.h"

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

class pin_dist {
public:
    int dist;
    int idx;

    pin_dist(int d, int i) : 
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
    Edge(utils::PointT<int> p1, utils::PointT<int> p2) 
        {from = db::Point(p1.x,p1.y); to = db::Point(p2.x,p2.y);}

    friend inline std::ostream& operator<<(std::ostream& os, const Edge& eg) {
        os << eg.from._x << ' ' << eg.from._y << ' ' << 
                eg.to._x << ' ' << eg.to._y << ' ';
        return os;
    }
};

// struct Point {
//     Point() {}
//     Point(int x, int y)
//         : x_(x), y_(y) {}
//     int x_ = 0;
//     int y_ = 0;

//     bool operator==(Point a) const{
//         if(a.x_ == x_ && a.y_ == y_) return true;
//         else return false;
//     }

//     int operator-(Point b) const{
//         return (abs(x_ - b.x_) + abs(y_ - b.y_));
//     }
// };

// const DirectEnum Direction;

// class Vertex {
// public:
//     int cost;
//     Point pos;
//     std::shared_ptr<Vertex> prev;

//     int direction;

//     Vertex(int c, Point v, const std::shared_ptr<Vertex> &p, int d) : 
//                 cost(c), pos(v), prev(p), direction(d) {}
// };


}