#pragma once

namespace db {

class GCell {
public:
    GCell() {}
    ~GCell() {}
    int X_cor = 0;
    int Y_cor = 0;
    int supplyH = 0;
    int supplyV = 0;
    int demandH = 0;
    int demandV = 0;
    vector<int> netsX;
    vector<int> netsY;
};

class GRGrid {
public:
    vector<vector<GCell>> gcells;
    int nx = 0;
    int ny = 0;
    int Hcap = 0;
    int Vcap = 0;
public:
    GRGrid() {}
    void init(const int x, const int y, const int hcap, const int vcap);
};


}