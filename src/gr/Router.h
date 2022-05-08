#pragma once
#include "global.h"
#include "db/Database.h"

#include <lemon/list_graph.h>
#include <lemon/matching.h>
#include <math.h>       /* log2 */
#include <random>

#include "flute/flute.h"
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

struct NegdistComp{
	bool operator()(const shared_ptr<pin_dist> &lhs, const shared_ptr<pin_dist> &rhs){
		return lhs->dist < rhs->dist;
	}
};

struct ovflComp{
	bool operator()(const shared_ptr<NetPrior> &lhs, const shared_ptr<NetPrior> &rhs){
		return lhs->cost > rhs->cost;
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

    /* Pin assign to Tap | Clusters*/
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
    vector<shared_ptr<TwoPinNet>> two_pin_nets;

    /* Route info */
    vector<vector<int>> net_2pnet_map;
    vector<vector<Edge>> rEdges;
    vector<vector<Edge>> r2pEdges;
    vector<int> net_ovfl;
    priority_queue<shared_ptr<NetPrior>, vector<shared_ptr<NetPrior>>, 
                        ovflComp> ovflQueue;
    priority_queue<shared_ptr<pin_dist>, vector<shared_ptr<pin_dist>>, 
                        distComp> netHPWLQueue;

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

    /* Routing */
    bool PatternRoute();
    bool constructSteinerTree(GRNet net);
    bool PatternRouteTwoPin(shared_ptr<TwoPinNet> net);
    bool addPath(shared_ptr<TwoPinNet> net);
    bool MazeRouteTwoPin(shared_ptr<TwoPinNet> net);

    /* UnRouting */
    bool ReRoute();
    bool unRouteNet(shared_ptr<TwoPinNet> net);
    bool deletePath(shared_ptr<TwoPinNet> net);

    /* Function */
    bool Cluster();
    bool KMeans();
    bool BKMeans();
    bool CKMeans();
    bool KMeansRefine();
    void write(const string& output_path);
    void print_demand();

    // bool single_net_pattern(db::Net* net);
    // bool patter_route();
    // bool single_net_maze(db::Net* net);
    // bool unroute_net(db::Net* net);
    // bool break_ovfl();
    // void run();
    // void write(const string& output_path);
    // void ripup(const vector<int>& netsToRoute);
};

}