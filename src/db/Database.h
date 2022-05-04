#pragma once
#include "global.h"

namespace db {
class GCell;
class GRGrid;
class Pin;
class Tap;
class Net;
}  // namespace db

// FIXME:
#include "Terminal.h"
#include "GCellGrid.h"
#include "Net.h"

namespace db {

class Database {
public:
    utils::logger* logger;
    string designName;

    // int nNets;
    // vector<Net*> nets;
    int nPins;
    int nTaps;
    int MaxLoad;

    unordered_map<int, Pin*> pinNames;
    unordered_map<int, Tap*> tapNames;

    vector<Pin*> pins;
    vector<Pin*> pin_taps;
    vector<Tap*> taps;

    GRGrid grGrid;  // global routing grid

private:
    static const size_t _bufferCapacity = 128 * 1024;
    size_t _bufferSize = 0;
    char* _buffer = nullptr;

    Pin* addPin(const int name, const Point& pos);
    Pin* addPin(const int name, const char t, const Point& pos);
    Tap* addTap(const int name, const Point& pos);

public:
    Database();
    ~Database();
    void clear();

public:
    /* defined in io/ */
    bool read(const std::string& inFile);
    void write(ofstream& ofs);

    bool readPin(const std::string& file);
};

} // namespace db