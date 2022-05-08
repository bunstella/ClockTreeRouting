#include "Router.h"

typedef lemon::ListGraph UGraph;
typedef UGraph::EdgeMap<double> DistMap;
typedef lemon::MaxWeightedPerfectMatching<UGraph,DistMap> MWPM;
typedef lemon::MaxWeightedMatching<UGraph,DistMap> MWM;

namespace gr {

//---------------------------------------------------------------------

bool Router::Cluster(){
    /* Priority Queue */
    int MaxDist = 0;
    for (db::Pin* pin : database->pins) {
        for (db::Tap* tap : database->taps) {
            int dist = tap->pos - pin->pos;
            if (dist > MaxDist) MaxDist = dist;
            tapsDistQueue[pin->id()].push(
                std::make_shared<pin_dist>(dist, tap->id()));
            pinsDistQueue[tap->id()].push(
                std::make_shared<pin_dist>(dist, pin->id()));
        }
    }

    /* Cluster */
    for(int dist = 0; dist <= MaxDist; dist += 2) {
        for (db::Tap* tap : database->taps) {
            int id = tap->id();
            while(!pinsDistQueue[id].empty() && 
                    (pinsDistQueue[id].top()->dist <= dist)) {
                auto pin = pinsDistQueue[id].top();
                pinsDistQueue[tap->id()].pop();
                /* 
                    A pin is assigned or
                    The tap is full
                */
                if ((pin_id_tap[pin->idx] != -1) || 
                    (load_per_tap[id] >= MaxLoad)) {
                    continue;
                }
                else {
                    /* 
                        A pin is unassigned and
                        The tap is unfilled
                    */
                    load_per_tap[id]++;
                    pin_id_tap[pin->idx] = id;
                    tap_id_pin[id].push_back(pin->idx);
                    pin_assignment[pin->idx] = make_shared<pin_assign_tap>(pin->idx, id, dist);
                }
            }
        }
    }

    /* Init route net */
    for (int i = 0; i < tap_id_pin.size(); i++) {
        nets.emplace_back(i);
        nets[i].pinPoints.emplace_back(database->taps[i]->pos);
        vector<int> tap_pins = tap_id_pin[i];
        for (int pin_id : tap_pins) {
            // cout << pin_id << ' ';
            nets[i].pinPoints.emplace_back(database->pins[pin_id]->pos);
        }
        // cout << endl;
    }

    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Router::BKMeans(){
    log () << "Start Balanced K-Means Clustering..." << std::endl;
    /* Initialization */
    int N = npins;
    int K = ntaps;
    int S = MaxLoad;
    vector<vector<CPoint<double>>> clusters;
    clusters.resize(K);
    vector<CPoint<double>> centroids;
    vector<CPoint<double>> taps;
    for (int i = 0; i < K; i++) {
        db::Tap* tap = database->taps[i];
        centroids.emplace_back(tap->pos._x,tap->pos._y);
        centroids.back()._c = i;
        taps.emplace_back(tap->pos._x,tap->pos._y);
        taps.back()._c = i;
    }
    vector<CPoint<double>> points;
    for (int i = 0; i < N; i++) {
        db::Pin* pin = database->pins[i];
        points.emplace_back(pin->pos._x,pin->pos._y);
        points.back()._c = i;
    }

    /* Clustering */
    double cost = 0;
    double pre_cost = INT_MAX;
    for (int iter = 0; iter < 20; iter++) {
        log() << "Iteration " << iter << std:: endl;
        /* Maximum Distance */
        double max_dist = 0;
        for (db::Pin* pin : database->pins) {
            for (db::Tap* tap : database->taps) {
                double dist = double(tap->pos - pin->pos);
                if (dist > max_dist) max_dist = dist;
            }
        }
        /* Creating Graph */
        log () << "Creating Graph" << std::endl;
        UGraph g;
        DistMap distmap(g);
        vector<UGraph::Node> sites_node;
        for (int i = 0; i < N; i++) {
            UGraph::Node u = g.addNode();
            sites_node.push_back(u);
        }
        vector<UGraph::Node> centroids_dummy_node;
        for (int i = 0; i < K * S; i++) {
            UGraph::Node v = g.addNode();
            centroids_dummy_node.push_back(v);
        }
        /* Assign weight to edges; weight is Max_Distance - Distance */
        for (int i = 0; i < N; i++) {
            auto u = sites_node[i];
            for (int j = 0; j < K * S; j++) {
                auto v = centroids_dummy_node[j];
                int idx = j / S;
                UGraph::Edge e = g.addEdge(u,v);
                auto p = points[i];
                auto c = centroids[idx];
                double dist = max_dist - (p - c);
                distmap.set(e,dist);
            }
        }
        /* Run Matching */
        MWM Matching(g,distmap);
        Matching.run();

        /* Matching to cluster */
        for (auto u : sites_node) {
            if (Matching.mate(u) == lemon::INVALID) std::cout << "Error\n";
            else clusters[(g.id(Matching.mate(u)) - N) / S].push_back(points[g.id(u)]);
            cost += centroids[(g.id(Matching.mate(u)) - N) / S] - points[g.id(u)];
        }
         /* Cost */
        log() << "Last cost: " << pre_cost << " | " << "cost " << cost << '\n';
        
        /* Update centroids */
        centroids.clear();
        for (int i = 0; i < clusters.size(); i++) {
            CPoint<double> center(0,0);
            for (auto p : clusters[i])
                center += p;
            center /= double(clusters[i].size());
            centroids.push_back(center);
            centroids.back()._c = i;
        }

        if ((pre_cost - cost) < 30) break;
        /* reset */
        pre_cost = cost;
        cost = 0;
        for (int i = 0; i < clusters.size(); i++) clusters[i].clear();
    }

    /* Cluster */
    double max_dist = double(gridX + gridY + 10);
    UGraph g;
    DistMap distmap(g);
    vector<UGraph::Node> tap_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node u = g.addNode();
        tap_node.push_back(u);
    }
    vector<UGraph::Node> centroid_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node v = g.addNode();
        centroid_node.push_back(v);
    }
    /* Assign weight to edges; weight is Max_Distance - Distance */
    for (int i = 0; i < K; i++) {
        auto u = tap_node[i];
        for (int j = 0; j < K; j++) {
            auto v = centroid_node[j];
            UGraph::Edge e = g.addEdge(u,v);
            auto t = taps[i];
            auto c = centroids[j];
            double dist = max_dist - (t - c);
            distmap.set(e,dist);
        }
    }
    /* Run Matching */
    MWM Matching(g,distmap);
    Matching.run();

    /* Matching to cluster */
    vector<int> tap_id_cluster;
    tap_id_cluster.resize(K);
    for (auto u : tap_node) {
        if (Matching.mate(u) == lemon::INVALID) std::cout << "Error\n";
        else tap_id_cluster[g.id(Matching.mate(u)) - K] = g.id(u);
    }

    /* Init route net */
    for (int i = 0; i < tap_id_pin.size(); i++) {
        nets.emplace_back(i);
        nets[i].pinPoints.emplace_back(database->taps[i]->pos);
        for (auto pin : clusters[tap_id_cluster[i]]) {
            tap_id_pin[i].push_back(pin._c);
            nets[i].pinPoints.emplace_back(database->pins[pin._c]->pos);
        }
    }

    return true;
} //END MODULE

// ---------------------------------------------------------------------

bool Router::CKMeans(){
    log () << "Start Balanced K-Means Clustering..." << std::endl;
    /* Initialization */
    int N = npins;
    int K = ntaps;
    int S = MaxLoad;
    vector<int> loads;
    loads.resize(K,0);
    vector<vector<CPoint<double>>> clusters;
    clusters.resize(K);
    vector<CPoint<double>> centroids;
    vector<CPoint<double>> taps;
    for (int i = 0; i < K; i++) {
        db::Tap* tap = database->taps[i];
        centroids.emplace_back(tap->pos._x,tap->pos._y);
        centroids.back()._c = i;
        taps.emplace_back(tap->pos._x,tap->pos._y);
        taps.back()._c = i;
    }
    vector<CPoint<double>> points;
    for (int i = 0; i < N; i++) {
        db::Pin* pin = database->pins[i];
        points.emplace_back(pin->pos._x,pin->pos._y);
        points.back()._c = i;
    }
    
    /* Clustering */
    const double max_dist = double(gridX + gridY + 10);
    double cost = 0;
    double pre_cost = INT_MAX;
    for (int iter = 0; iter < 20; iter++) {
        log() << "Iteration " << iter << std:: endl;
        /* Assignment */
        for (CPoint<double> p : points) {
            double min_dist = double(INT_MAX);
            double max_score = 0;
            int cluster = -1;
            for (CPoint<double> c : centroids) {
                double dist = p - c;
                double score = (max_dist - dist) * (log2(S - loads[c._c]) + 1);
                if (score > max_score) {min_dist = dist; cluster = c._c; max_score = score;}
            }
            cost += min_dist;
            clusters[cluster].push_back(p);
            loads[cluster]++;
        }

        /* Cost */
        log() << "Last cost: " << pre_cost << " | " << "cost " << cost << '\n';
        
        /* Update centroids */
        centroids.clear();
        for (int i = 0; i < clusters.size(); i++) {
            CPoint<double> center(0,0);
            for (auto p : clusters[i])
                center += p;
            center /= double(clusters[i].size());
            centroids.push_back(center);
            centroids.back()._c = i;
        }

        if ((pre_cost - cost) < 30) break;
        /* reset */
        pre_cost = cost;
        cost = 0;
        for (int& l : loads) l = 0;
        for (int i = 0; i < clusters.size(); i++) clusters[i].clear();
    }

    /* Cluster */
    UGraph g;
    DistMap distmap(g);
    vector<UGraph::Node> tap_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node u = g.addNode();
        tap_node.push_back(u);
    }
    vector<UGraph::Node> centroid_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node v = g.addNode();
        centroid_node.push_back(v);
    }
    /* Assign weight to edges; weight is Max_Distance - Distance */
    for (int i = 0; i < K; i++) {
        auto u = tap_node[i];
        for (int j = 0; j < K; j++) {
            auto v = centroid_node[j];
            UGraph::Edge e = g.addEdge(u,v);
            auto t = taps[i];
            auto c = centroids[j];
            double dist = max_dist - (t - c);
            distmap.set(e,dist);
        }
    }
    /* Run Matching */
    MWM Matching(g,distmap);
    Matching.run();

    /* Matching to cluster */
    vector<int> tap_id_cluster;
    tap_id_cluster.resize(K);
    for (auto u : tap_node) {
        if (Matching.mate(u) == lemon::INVALID) std::cout << "Error\n";
        else tap_id_cluster[g.id(Matching.mate(u)) - K] = g.id(u);
    }

    /* Init route net */
    for (int i = 0; i < tap_id_pin.size(); i++) {
        nets.emplace_back(i);
        nets[i].pinPoints.emplace_back(database->taps[i]->pos);
        for (auto pin : clusters[tap_id_cluster[i]]) {
            tap_id_pin[i].push_back(pin._c);
            nets[i].pinPoints.emplace_back(database->pins[pin._c]->pos);
        }
    }

    return true;
} //END MODULE

// ---------------------------------------------------------------------

bool Router::KMeans(){
    log () << "Start Naive K-Means Clustering..." << std::endl;
    /* Initialization */
    int N = npins;
    int K = ntaps;
    int S = MaxLoad;
    vector<int> loads;
    loads.resize(K,0);
    vector<vector<CPoint<double>>> clusters;
    clusters.resize(K);
    vector<CPoint<double>> centroids;
    vector<CPoint<double>> taps;
    for (int i = 0; i < K; i++) {
        db::Tap* tap = database->taps[i];
        centroids.emplace_back(tap->pos._x,tap->pos._y);
        centroids.back()._c = i;
        taps.emplace_back(tap->pos._x,tap->pos._y);
        taps.back()._c = i;
    }
    vector<CPoint<double>> points;
    for (int i = 0; i < N; i++) {
        db::Pin* pin = database->pins[i];
        points.emplace_back(pin->pos._x,pin->pos._y);
        points.back()._c = i;
    }
    
    /* Clustering */
    const double max_dist = double(gridX + gridY + 10);
    double cost = 0;
    double pre_cost = INT_MAX;
    for (int iter = 0; iter < 50; iter++) {
        log() << "Iteration " << iter << std:: endl;
        /* Assignment */
        for (CPoint<double> p : points) {
            double min_dist = double(INT_MAX);
            double max_score = 0;
            int cluster = -1;
            for (CPoint<double> c : centroids) {
                double dist = p - c;
                if (dist < min_dist) {min_dist = dist; cluster = c._c;}
            }
            cost += min_dist;
            clusters[cluster].push_back(p);
            loads[cluster]++;
        }

        /* Cost */
        log() << "Last cost: " << pre_cost << " | " << "cost " << cost << '\n';
        
        /* Update centroids */
        centroids.clear();
        for (int i = 0; i < clusters.size(); i++) {
            CPoint<double> center(0,0);
            for (auto p : clusters[i])
                center += p;
            center /= double(clusters[i].size());
            centroids.push_back(center);
            centroids.back()._c = i;
        }

        if (((pre_cost - cost) < 30) || (iter == 30)) break;
        /* reset */
        pre_cost = cost;
        cost = 0;
        for (int& l : loads) l = 0;
        for (int i = 0; i < clusters.size(); i++) clusters[i].clear();
    }

    /* Cluster */
    UGraph g;
    DistMap distmap(g);
    vector<UGraph::Node> tap_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node u = g.addNode();
        tap_node.push_back(u);
    }
    vector<UGraph::Node> centroid_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node v = g.addNode();
        centroid_node.push_back(v);
    }
    /* Assign weight to edges; weight is Max_Distance - Distance */
    for (int i = 0; i < K; i++) {
        auto u = tap_node[i];
        for (int j = 0; j < K; j++) {
            auto v = centroid_node[j];
            UGraph::Edge e = g.addEdge(u,v);
            auto t = taps[i];
            auto c = centroids[j];
            double dist = max_dist - (t - c);
            distmap.set(e,dist);
        }
    }
    /* Run Matching */
    MWM Matching(g,distmap);
    Matching.run();

    /* Matching to cluster */
    vector<int> tap_id_cluster;
    tap_id_cluster.resize(K);
    for (auto u : tap_node) {
        if (Matching.mate(u) == lemon::INVALID) std::cout << "Error\n";
        else tap_id_cluster[g.id(Matching.mate(u)) - K] = g.id(u);
    }

    /* Init route net */
    for (int i = 0; i < tap_id_pin.size(); i++) {
        nets.emplace_back(i);
        nets[i].pinPoints.emplace_back(database->taps[i]->pos);
        cout << clusters[tap_id_cluster[i]].size() << endl;
        for (auto pin : clusters[tap_id_cluster[i]]) {
            tap_id_pin[i].push_back(pin._c);
            nets[i].pinPoints.emplace_back(database->pins[pin._c]->pos);
        }
    }

    return true;
} //END MODULE

// ---------------------------------------------------------------------

bool Router::KMeansRefine(){
    log () << "Start Refined K-Means Clustering..." << std::endl;
    /* Initialization */
    int N = npins;
    int K = ntaps;
    int S = MaxLoad;
    vector<int> loads;
    loads.resize(K,0);
    vector<vector<CPoint<double>>> clusters;
    clusters.resize(K);
    vector<CPoint<double>> centroids;
    vector<CPoint<double>> taps;
    for (int i = 0; i < K; i++) {
        db::Tap* tap = database->taps[i];
        centroids.emplace_back(tap->pos._x,tap->pos._y);
        centroids.back()._c = i;
        taps.emplace_back(tap->pos._x,tap->pos._y);
        taps.back()._c = i;
    }
    vector<CPoint<double>> points;
    for (int i = 0; i < N; i++) {
        db::Pin* pin = database->pins[i];
        points.emplace_back(pin->pos._x,pin->pos._y);
        points.back()._c = i;
    }
    
    /* Clustering */
    const double max_dist = double(gridX + gridY + 10);
    double cost = 0;
    double pre_cost = INT_MAX;
    for (int iter = 0; iter < 50; iter++) {
        log() << "Iteration " << iter << std:: endl;
        /* Assignment */
        for (CPoint<double> p : points) {
            double min_dist = double(INT_MAX);
            double max_score = 0;
            int cluster = -1;
            for (CPoint<double> c : centroids) {
                double dist = p - c;
                if (dist < min_dist) {min_dist = dist; cluster = c._c;}
            }
            cost += min_dist;
            clusters[cluster].push_back(p);
            loads[cluster]++;
        }

        /* Cost */
        log() << "Last cost: " << pre_cost << " | " << "cost " << cost << '\n';
        
        /* Update centroids */
        centroids.clear();
        for (int i = 0; i < clusters.size(); i++) {
            CPoint<double> center(0,0);
            for (auto p : clusters[i])
                center += p;
            center /= double(clusters[i].size());
            centroids.push_back(center);
            centroids.back()._c = i;
        }

        if (((pre_cost - cost) < 30) || (iter == 30)) break;
        /* reset */
        pre_cost = cost;
        cost = 0;
        for (int& l : loads) l = 0;
        for (int i = 0; i < clusters.size(); i++) clusters[i].clear();
    }

    /* Cluster */
    UGraph g;
    DistMap distmap(g);
    vector<UGraph::Node> tap_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node u = g.addNode();
        tap_node.push_back(u);
    }
    vector<UGraph::Node> centroid_node;
    for (int i = 0; i < K; i++) {
        UGraph::Node v = g.addNode();
        centroid_node.push_back(v);
    }
    /* Assign weight to edges; weight is Max_Distance - Distance */
    for (int i = 0; i < K; i++) {
        auto u = tap_node[i];
        for (int j = 0; j < K; j++) {
            auto v = centroid_node[j];
            UGraph::Edge e = g.addEdge(u,v);
            auto t = taps[i];
            auto c = centroids[j];
            double dist = max_dist - (t - c);
            distmap.set(e,dist);
        }
    }
    /* Run Matching */
    MWM Matching(g,distmap);
    Matching.run();

    /* Matching to cluster */
    vector<int> tap_id_cluster;
    tap_id_cluster.resize(K);
    for (auto u : tap_node) {
        if (Matching.mate(u) == lemon::INVALID) std::cout << "Error\n";
        else tap_id_cluster[g.id(Matching.mate(u)) - K] = g.id(u);
    }

    /* Per Tap */
    vector<priority_queue<shared_ptr<pin_dist>, vector<shared_ptr<pin_dist>>, 
                        NegdistComp>> pinsClusterQueue;
    vector<vector<CPoint<double>>> pinsClusterVector;
    pinsClusterQueue.resize(K);
    pinsClusterVector.resize(K);
    for (int i = 0; i < K; i++) {
        for (auto pin : clusters[tap_id_cluster[i]]) {
            pinsClusterQueue[i].push(
                std::make_shared<pin_dist>(pin - centroids[tap_id_cluster[i]], pin._c));
        }
    }
    /* Priority Queue to Vector */
    for (int i = 0; i < K; i++) {
        auto cluster = pinsClusterQueue[i];
        while (!cluster.empty()) {
            auto pin = cluster.top();
            cluster.pop();
            pinsClusterVector[i].push_back(points[pin->idx]);
            // cout << pin->dist << ' ';
        }
        // cout << '\n';
    }
    
    /* Refine */
    auto swapComp = [](const shared_ptr<Move_pair> &lhs, const shared_ptr<Move_pair> &rhs) {
        return rhs->dist < lhs->dist;
    };
    for (int i = 0; i < K; i++) {
        cout << i << " : " << pinsClusterVector[i].size() << " : " << MaxLoad << endl;
        if (pinsClusterVector[i].size() > MaxLoad) {
            priority_queue<std::shared_ptr<Move_pair>, vector<std::shared_ptr<Move_pair>>, 
                                decltype(swapComp)> swapQueue(swapComp);
            /* for each pin */
            for (int m = 0; m < pinsClusterVector[i].size(); m++) {
                for (int j = 0; j < K; j++) {
                    if (j != i && pinsClusterVector[j].size() < MaxLoad) {
                        for (auto pin : pinsClusterVector[j]) {
                            double dist_margin = pinsClusterVector[i][m] - pin;
                            swapQueue.push(make_shared<Move_pair>(dist_margin, m, j));
                        }
                    }
                }
            }
            for (int n = 0; n < pinsClusterVector[i].size() - MaxLoad; n++) {
                auto pair = swapQueue.top();
                swapQueue.pop();
                // cout << pair->dist << endl;
                if (pinsClusterVector[pair->to].size() == MaxLoad) {n--;continue;}
                if (pinsClusterVector[i][pair->from]._c == -1) {n--;continue;}
                pinsClusterVector[pair->to].push_back(points[pinsClusterVector[i][pair->from]._c]);
                pinsClusterVector[i][pair->from]._c = -1;
            }
        }
    }
    
    /* Init route net */
    for (int i = 0; i < tap_id_pin.size(); i++) {
        nets.emplace_back(i);
        nets[i].pinPoints.emplace_back(database->taps[i]->pos);
        for (auto pin : pinsClusterVector[i]) {
            if (pin._c != -1) {
                tap_id_pin[i].push_back(pin._c);
                nets[i].pinPoints.emplace_back(database->pins[pin._c]->pos);
            }
        }
    }

    return true;
} //END MODULE

// ---------------------------------------------------------------------

}