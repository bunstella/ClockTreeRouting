#include "Database.h"

using namespace db;

/***** Database *****/
Database::Database() {
    _buffer = new char[_bufferCapacity];
}

Database::~Database() {
    delete[] _buffer;
    _buffer = nullptr;
    // for regions.push_back(new Region("default"));
} //END MODULE

//---------------------------------------------------------------------

Pin* Database::addPin(const int name, const Point& pos) {
    // Net* net = getNet(name);
    // if (net) {
    //     printlog("Net re-defined: %s", name.c_str());
    //     return net;
    // }
    Pin* pin = new Pin(name, pos);
    pinNames[name] = pin;
    pins.push_back(pin);
    return pin;
} //END MODULE

//---------------------------------------------------------------------

Pin* Database::addPin(const int name, const char t, const Point& pos) {
    // Net* net = getNet(name);
    // if (net) {
    //     printlog("Net re-defined: %s", name.c_str());
    //     return net;
    // }
    Pin* pin = new Pin(name, t, pos);
    pinNames[name] = pin;
    pin_taps.push_back(pin);
    return pin;
} //END MODULE

//---------------------------------------------------------------------
Tap* Database::addTap(const int name, const Point& pos) {
    // Net* net = getNet(name);
    // if (net) {
    //     printlog("Net re-defined: %s", name.c_str());
    //     return net;
    // }
    Tap* tap = new Tap(name, pos);
    tapNames[name] = tap;
    taps.push_back(tap);
    return tap;
} //END MODULE

//---------------------------------------------------------------------

// Net* Database::getNet(const string& name) {
//     unordered_map<string, Net*>::iterator mi = net_names.find(name);
//     if (mi == net_names.end()) {
//         return nullptr;
//     }
//     return mi->second;
// } //END MODULE

// //---------------------------------------------------------------------