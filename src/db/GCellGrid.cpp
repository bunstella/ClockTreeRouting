
#include "Database.h"
using namespace db;

void GRGrid::init(const int x, const int y, const int hcap, const int vcap){
    nx = x;
    ny = y;
    Hcap = hcap;
    Vcap = vcap;
    gcells.resize(nx, vector<GCell>(ny)); // |G| + 1 = |E|
    // log() << gcells.size() << endl;
    // log() << gcells[0].size() << endl;

    for(int i = 0; i < nx; i++){
        for(int j = 0; j < ny; j++){
            gcells[i][j].X_cor = i;
            gcells[i][j].Y_cor = j;
            gcells[i][j].supplyH = Hcap;
            gcells[i][j].supplyV = Vcap;
        }
    }
}