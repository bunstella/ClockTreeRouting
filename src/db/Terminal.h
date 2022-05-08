#pragma once

namespace db {

struct Point
{   
    Point() {}
    Point(int x, int y)
        : _x(x), _y(y) {}
    int _x = 0;
    int _y = 0;

    friend ostream& operator<<(ostream& os, const Point& c) {
        return os << "(" << c._x << ", " << c._y << ")";
    }
    int operator-(Point b) const{
        return (abs(_x - b._x) + abs(_y - b._y));
    }
    bool operator==(Point a) const{
        if(a._x == _x && a._y == _y) return true;
        else return false;
    }
};

class Pin {
private:
    string _name;
    int _id = -1;
    // s: sink, t: tap
    char _type = 's';
    int max_load = -1;

public:
    Pin(const int id, const Point& point) : _id(id), pos(point) { }
    Pin(const int id, const char t, const Point& point) : _id(id), _type(t), pos(point) { }
    ~Pin();
    Point pos;
    int load = 0;

    void set_id(const int i) { _id = i; }
    void set_name(const string& name) { _name = name; }
    void set_load(const int load) { max_load = load; }

    const std::string& name() const { return _name; }
    const int id() const { return _id; }

    friend ostream& operator<<(ostream& os, const Pin& c) {
        return os << c._name << "\n";
    }
};

class Tap {
private:
    string _name;
    int _id = -1;
    int max_load = -1;

public:
    Tap(const int id, const Point& point) : _id(id), pos(point) { }
    ~Tap();
    Point pos;
    int load = 0;

    void set_id(const int i) { _id = i; }
    void set_name(const string& name) { _name = name; }
    void set_load(const int load) { max_load = load; }

    const std::string& name() const { return _name; }
    const int id() const { return _id; }

    friend ostream& operator<<(ostream& os, const Tap& c) {
        return os << c._name << "\n";
    }
};

}
