#pragma once

namespace db {

class Net {
private:
    int _id = -1;

public:
    vector<Pin*> Pins;

    Net(){}
    Net(const int id) : _id(id) { }
    ~Net();

    void set_id(const int i) { _id = i; }
    const int id() const { return _id; }

    void addPin(Pin* pin);

    friend ostream& operator<<(ostream& os, const Net& c) {
        return os << c._id << "\n";
    }
};

}
