#pragma once
#include "global.h"
#include "flute/flute.h"
#include "db/Database.h"

#include "Rpath.h"
#include "Vertex.h"

extern "C" {
void readLUT();
Tree flute(int d, DTYPE x[], DTYPE y[], int acc);
}

namespace gr {

// auto distComp = [](const shared_ptr<pin_dist> &lhs, const shared_ptr<pin_dist> &rhs) {
//     return rhs->dist > lhs->dist;
// };

struct distComp{
	bool operator()(const shared_ptr<pin_dist> &lhs, const shared_ptr<pin_dist> &rhs){
		return lhs->dist > rhs->dist;
	}
};

class Router {
public:
    db::Database* database;
    int gridX;
    int gridY;
    int capH = 0;
    int capV = 0;
    int npins;
    int ntaps;
    int MaxLoad;

public:
    /* Grid */
    vector<vector<int>> demH;
    vector<vector<int>> demV;
    db::GRGrid grGrid;
    vector<vector<db::GCell*>> gcells;

    // vector<vector<int>> pin_per_tap;
    // vector<vector<int>> pin_per_tap_dist;
    vector<int> load_per_tap;
    vector<int> pin_id_tap;
    vector<vector<int>> tap_id_pin;
    vector<shared_ptr<pin_assign_tap>> pin_assignment;

    vector<priority_queue<shared_ptr<pin_dist>, vector<shared_ptr<pin_dist>>, 
                        distComp>> tapsDistQueue;
    vector<vector<shared_ptr<pin_dist>>> tapsDist;

    vector<priority_queue<shared_ptr<pin_dist>, vector<shared_ptr<pin_dist>>, 
                        distComp>> pinsDistQueue;
    vector<vector<shared_ptr<pin_dist>>> pinsDist;
    
    /* Net */
    vector<vector<int>> Net_xs;
    vector<vector<int>> Net_ys;
    vector<GRNet> nets;

    /* Route info */
    vector<vector<db::Point>> rpoints;
    vector<vector<Edge>> rEdges;

    // vector<Rpath> rpaths;
    // vector<vector<Point>> rpoints;
    // vector<db::Net*> net_queue;
    // vector<bool> net_rflag;
    // vector<int> net_ovfl;
    // priority_queue<std::shared_ptr<net_prior>> net_queue_ovfl;
    

public:
    Router(db::Database* database_);
    utils::logger* logger;
    static void readFluteLUT() { readLUT(); };

    bool cluster();
    bool PatternRoute();
    bool constructSteinerTree(GRNet net);


    void write(const string& output_path);


    // bool single_net_pattern(db::Net* net);
    // bool patter_route();
    // bool single_net_maze(db::Net* net);
    // bool unroute_net(db::Net* net);
    // bool break_ovfl();
    // void print_demand();
    // void run();
    // void write(const string& output_path);
    // void ripup(const vector<int>& netsToRoute);
};

}