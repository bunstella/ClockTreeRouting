#include "global.h"
#include "db/Database.h"

namespace gr {


class Rpath {
public:
    vector<db::GCell*> path;
    vector<bool> direction;
    void clear() {path.clear();direction.clear();}
};

}