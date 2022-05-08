#include "Router.h"

typedef lemon::ListGraph UGraph;
typedef UGraph::EdgeMap<double> DistMap;
typedef lemon::MaxWeightedPerfectMatching<UGraph,DistMap> MWPM;
typedef lemon::MaxWeightedMatching<UGraph,DistMap> MWM;

namespace gr {

Router::Router(db::Database* database_) :   
    database(database_)
{
    grGrid = database->grGrid;
    logger = database->logger;

    /*
    Grgrid info
    */
    gridX = grGrid.nx;
    gridY = grGrid.ny;
    capH = grGrid.Hcap;
    capV = grGrid.Vcap;
    npins = database->nPins;
    ntaps = database->nTaps;
    MaxLoad = database->MaxLoad;
    printf("%d pins\n",npins);
    printf("%d taps\n",ntaps);
    printf("%d MaxLoad\n",MaxLoad);
    cout << (npins/ntaps) << " pins per tap\n";

    // // Queue of nets to route
    // for (db::Net* net: database->nets)
    //     net_queue.push_back(net);

    /* Gcells | Shape of a map */
    gcells.resize(gridX, vector<db::GCell*>(gridY));
    for(int i = 0; i < gridX; i++){
        for(int j = 0; j < gridY; j++){
            gcells[i][j] = new db::GCell();
            gcells[i][j]->X_cor = i;
            gcells[i][j]->Y_cor = j;
            gcells[i][j]->supplyH = capH;
            gcells[i][j]->supplyV = capV;
        }
    }

    /* Demand map | Two directions */
    demH.resize(gridY, vector<int>(gridX));     // H [Y][X]
    demV.resize(gridX, vector<int>(gridY));     // V [X][Y]

    /* Cluster ID */
    load_per_tap.resize(ntaps, 0);
    pin_assignment.resize(npins);
    pin_id_tap.resize(npins, -1);
    tap_id_pin.resize(ntaps);

    /* Cluster */
    tapsDistQueue.resize(npins);
    tapsDist.resize(npins);
    pinsDistQueue.resize(ntaps);
    pinsDist.resize(ntaps);
    
    /* Net */
    Net_xs.resize(ntaps);
    Net_ys.resize(ntaps);
    rEdges.resize(ntaps);
    net_2pnet_map.resize(ntaps);

    // // Route paths | net-wise
    // rpaths.resize(database->nNets);
    // rpoints.resize(database->nNets);
    // net_rflag.resize(database->nNets);
    // net_ovfl.resize(database->nNets, 0);

} //END MODULE

//---------------------------------------------------------------------

bool Router::PatternRoute() {
    log() << "Pattern Route\n";
    readLUT();  // read flute LUT

    /* Loop over Nets(Taps) */
    for (auto net : nets) {
        constructSteinerTree(net);
    }
    r2pEdges.resize(two_pin_nets.size());
    for (auto net : two_pin_nets) {
        PatternRouteTwoPin(net);
    }
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Router::constructSteinerTree(GRNet net) {
    // 2. Construct Steiner tree
    const int degree = net.pinPoints.size();
    if (degree == 1) return true;
    // log() << degree << endl;
    int xs[degree * 100];
    int ys[degree * 100];
    int i = 0;
    for (auto& accessPoint : net.pinPoints) {
        xs[i] = accessPoint.x;
        ys[i] = accessPoint.y;
        i++;
        // log() << accessPoint.x << " | " << accessPoint.y << endl;
    }
    /* Construct Steiner Points */
    Tree flutetree = flute(degree, xs, ys, ACCURACY);
    const int numBranches = degree + degree - 2;
    vector<utils::PointT<int>> steinerPoints;
    steinerPoints.reserve(numBranches);
    vector<vector<int>> adjacentList(numBranches);
    for (int branchIndex = 0; branchIndex < numBranches; branchIndex++) {
        const Branch& branch = flutetree.branch[branchIndex];
        steinerPoints.emplace_back(branch.x, branch.y);
        if (branchIndex == branch.n) continue;
        adjacentList[branchIndex].push_back(branch.n);
        adjacentList[branch.n].push_back(branchIndex);
    }
    /* Construct nets */
    for (int branchIndex = 0; branchIndex < numBranches; branchIndex++) {
        for (int n : adjacentList[branchIndex]) {
            if (n > branchIndex) {
                int lhs_x = steinerPoints[branchIndex].x;
                int lhs_y = steinerPoints[branchIndex].y;
                int rhs_x = steinerPoints[n].x;
                int rhs_y = steinerPoints[n].y;
                two_pin_nets.emplace_back(make_shared<TwoPinNet>(two_pin_nets.size(), 
                            net.id(), db::Point(lhs_x,lhs_y), db::Point(rhs_x,rhs_y)));
                net_2pnet_map[net.id()].push_back(two_pin_nets.size()-1);
            }
        }
        // cout << endl;
    }

    return true;
} //END MODULE

// ---------------------------------------------------------------------

bool Router::PatternRouteTwoPin(shared_ptr<TwoPinNet> net){
    // log() << "Routing two pin net\n";
    int x_1 = net->Pins[0]._x;int y_1 = net->Pins[0]._y;
    int x_2 = net->Pins[1]._x;int y_2 = net->Pins[1]._y;

    int x_l = min(x_1, x_2);int y_l = min(y_1, y_2);
    int x_h = max(x_1, x_2);int y_h = max(y_1, y_2);

    if (((x_1 - x_2) * (y_1 - y_2))) {
        // log() << "L shape: " << net->Pins[0] << " | " << net->Pins[1] << endl;
        int x_s_1; int x_s_2;
        int y_s_1; int y_s_2;
        if (((x_2 - x_1) * (y_2 - y_1)) < 0){
            x_s_1 = x_h; x_s_2 = x_l;
            y_s_1 = y_h; y_s_2 = y_l;
        } else{
            x_s_1 = x_l; x_s_2 = x_h;
            y_s_1 = y_l; y_s_2 = y_h;
        }

        int min_cost = INT_MAX;
        int cost;
        bool direction;     // true vertival | false horizontal
        int bend;           // Bending point

        // Vertical line
        for (int i = x_l; i <= x_h; i++){
            cost = 0;
            cost += accumulate(demH[y_s_1].begin() + x_l, demH[y_s_1].begin() + i, 0);
            cost += accumulate(demH[y_s_2].begin() + i, demH[y_s_2].begin() + x_h, 0);
            cost += accumulate(demV[i].begin() + y_l, demV[i].begin() + y_h, 0);
            if (cost < min_cost){
                min_cost = cost;
                direction = true;
                bend = i;
            }
        }
        // Horizontal line
        for (int j = y_l; j <= y_h; j++){
            cost = 0;
            cost += accumulate(demV[x_s_1].begin() + y_l, demV[x_s_1].begin() + j, 0);
            cost += accumulate(demV[x_s_2].begin() + j, demV[x_s_2].begin() + y_h, 0);
            cost += accumulate(demH[j].begin() + x_l, demH[j].begin() + x_h, 0);
            if (cost < min_cost){
                min_cost = cost;
                direction = false;
                bend = j;
            }
        }
        // Update demand matrix | Update route path
        int idx = net->_id;
        if (direction == true) {
            r2pEdges[idx].emplace_back(x_l,y_s_1,bend,y_s_1);
            r2pEdges[idx].emplace_back(bend,y_s_1,bend,y_s_2);
            r2pEdges[idx].emplace_back(bend,y_s_2,x_h,y_s_2);
        } else {
            r2pEdges[idx].emplace_back(x_s_1,y_l,x_s_1,bend);
            r2pEdges[idx].emplace_back(x_s_1,bend,x_s_2,bend);
            r2pEdges[idx].emplace_back(x_s_2,bend,x_s_2,y_h);
        }
    } else {
        // Update dem and matrix | Update route path
        // log() << "Line shape: " << net->Pins[0] << " | " << net->Pins[1] << endl;
        int idx = net->_id;
        r2pEdges[idx].emplace_back(x_1,y_1,x_2,y_2);
    }

    addPath(net);

    return true;
} //END MODULE

// ---------------------------------------------------------------------

bool Router::ReRoute(){
    log() << "Reroute\n";
    // Calculating overflow; Collect nets
    set<int> nets_to_unroute;
    for (int i = 0; i < gridX; i++) {
        for (int j = 0; j < gridY; j++) {
            if (demV[i][j] > capV){            // X = H | Y = V
                int ovfl = demV[i][j] - capV;
                for (int idx : gcells[i][j]->netsY) {
                    nets_to_unroute.insert(idx);
                }
            }
        }
    }
    for (int i = 0; i < gridX; i++) {
        for (int j = 0; j < gridY; j++) {
            if (demH[j][i] > capH){           // X = H | Y = V
                int ovfl = demH[j][i] - capH;
                for (int idx : gcells[i][j]->netsX) {
                    nets_to_unroute.insert(idx);
                }
            }
        }
    }
    for (int idx : nets_to_unroute) {
        netHPWLQueue.push(std::make_shared<pin_dist>(two_pin_nets[idx]->HPWL(), idx));
        unRouteNet(two_pin_nets[idx]);
        log() << two_pin_nets[idx]->Pins[0] << " | " << two_pin_nets[idx]->Pins[1] << 
                    " | " << two_pin_nets[idx]->HPWL() << endl;
    }
    while (!netHPWLQueue.empty()) {
        auto net = netHPWLQueue.top();
        netHPWLQueue.pop();
        MazeRouteTwoPin(two_pin_nets[net->idx]);
        addPath(two_pin_nets[net->idx]);
    }

    return true;
} //END MODULE

// ---------------------------------------------------------------------

bool Router::unRouteNet(shared_ptr<TwoPinNet> net){
    // log() << "Unrouting two pin net\n";
    deletePath(net);
    return true;
} //END MODULE

//---------------------------------------------------------------------
// TODO: thres blk

bool Router::MazeRouteTwoPin(shared_ptr<TwoPinNet> net){
    // logger->info() << net->name() << ' ' << net_queue.size() << " nets left\n";
    // Create maps
    vector<vector<int>> visited(gridX, vector<int>(gridY, false));
    vector<vector<int>> blkH(gridX, vector<int>(gridY, false));
    vector<vector<int>> blkV(gridX, vector<int>(gridY, false));
    for(int i = 0; i < gridX; i++){
        for(int j = 0; j < gridY; j++){
            blkH[i][j] = (demH[j][i] >= capH);
            blkV[i][j] = (demV[i][j] >= capV);
        }
    }
    int x_1 = net->Pins[0]._x;int y_1 = net->Pins[0]._y;
    int x_2 = net->Pins[1]._x;int y_2 = net->Pins[1]._y;

    // Prepare for queues
    auto solComp = [](const std::shared_ptr<Vertex> &lhs, const std::shared_ptr<Vertex> &rhs) {
        return rhs->cost < lhs->cost;
    };
    priority_queue<std::shared_ptr<Vertex>, vector<std::shared_ptr<Vertex>>, 
                        decltype(solComp)> solQueue(solComp);

    // Init vertex
    solQueue.push(std::make_shared<Vertex>(0, db::Point(x_1, y_1), nullptr, -1));
    db::Point dstPin(x_2, y_2);
    db::Point srcPin(x_1, y_1);
    std::shared_ptr<Vertex> dstVer = std::make_shared<Vertex>(0, db::Point(x_2, y_2), nullptr, -1);

    // Hadlock's 
    while (!solQueue.empty()) {
        auto curVer = solQueue.top();
        solQueue.pop();
        int x_v = curVer->pos._x;
        int y_v = curVer->pos._y;
        int cost = curVer->cost;
        visited[x_v][y_v] = true;

        // reach a pin?
        if (curVer->pos == dstPin) {
            dstVer = curVer;
            break;
        }

        // 4 directions
        for(int d = 0; d < 4; d++){
            // Next vertex to visit | remember to calc the bound
            int x_off = Direction.x_off[d];
            int y_off = Direction.y_off[d];
            int x_new = x_v + x_off;
            int y_new = y_v + y_off;
            if (x_new < 0 || y_new < 0 || x_new >= gridX || y_new >= gridY) continue;

            // This direction has block
            if (d == 0 && blkV[x_v][y_v]) continue;
            if (d == 1 && blkV[x_v][y_v - 1]) continue;
            if (d == 2 && blkH[x_v - 1][y_v]) continue;
            if (d == 3 && blkH[x_v][y_v]) continue;

            // If not visited
            if (!visited[x_new][y_new]) {
                db::Point newPin(x_new, y_new);
                int new_cost = cost;

                if ((newPin - dstPin) > (curVer->pos - dstPin)) new_cost++;
                solQueue.push(std::make_shared<Vertex>(new_cost, newPin, curVer, d));
                visited[x_new][y_new] = true;
            }
        }
    }

    int idx = net->_id;
    if (dstVer->prev == nullptr) return false;

    /* Add edges from solution */
    shared_ptr<Vertex> curVer = dstVer;
    int cur_dir;
    int next_dir = curVer->direction;
    db::Point last_bend = curVer->pos;
    while (curVer->prev != nullptr){
        shared_ptr<Vertex> preVer = curVer->prev;

        cur_dir = curVer->direction;
        if (cur_dir == 0) demV[preVer->pos._x][preVer->pos._y] += 1;
        if (cur_dir == 1) demV[curVer->pos._x][curVer->pos._y] += 1;
        if (cur_dir == 2) demH[curVer->pos._y][curVer->pos._x] += 1;
        if (cur_dir == 3) demH[preVer->pos._y][preVer->pos._x] += 1;

        if (cur_dir != next_dir) {
            r2pEdges[idx].emplace_back(last_bend,curVer->pos);
            last_bend = curVer->pos;
        }
        next_dir = cur_dir;
        curVer = preVer;
    }
    r2pEdges[idx].emplace_back(last_bend,srcPin);

    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Router::addPath(shared_ptr<TwoPinNet> net) {
    int idx = net->_id;
    auto edges = r2pEdges[idx];
    for (auto edge : edges) {
        db::Point from = edge.from;
        db::Point to = edge.to;
        if ((from._x - to._x) == 0) {
            // Vertical edge
            int x = from._x;
            int y_l = min(from._y,to._y);int y_h = max(from._y,to._y);
            for (int j = y_l; j != y_h; j++){
                demV[x][j] += 1;
                db::GCell* gcell = gcells[x][j];
                gcell->demandV += 1;
                gcell->netsY.push_back(idx);        // X = H | Y = V
            }
        } else {
            // Hotizontal edge
            int y = from._y;
            int x_l = min(from._x,to._x);int x_h = max(from._x,to._x);
            for (int i = x_l; i != x_h; i++){
                demH[y][i] += 1;
                db::GCell* gcell = gcells[i][y];
                gcell->demandH += 1;
                gcell->netsX.push_back(idx);        // X = H | Y = V
            }
        }
    }
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Router::deletePath(shared_ptr<TwoPinNet> net) {
    int idx = net->_id;
    auto edges = r2pEdges[idx];
    for (auto edge : edges) {
        db::Point from = edge.from;
        db::Point to = edge.to;
        if ((from._x - to._x) == 0) {
            // Vertical edge
            int x = from._x;
            int y_l = min(from._y,to._y);int y_h = max(from._y,to._y);
            for (int j = y_l; j != y_h; j++){
                demV[x][j] -= 1;
                db::GCell* gcell = gcells[x][j];
                gcell->demandV -= 1;
                vector<int>::iterator pos = find(gcell->netsY.begin(), gcell->netsY.end(), idx);
                if (pos != gcell->netsY.end()) // == myVector.end() means the element was not found
                    gcell->netsY.erase(pos);
                else{
                    logger->info() << "Error! Path not found!\n";
                    exit(1);
                }
            }
        } else {
            // Hotizontal edge
            int y = from._y;
            int x_l = min(from._x,to._x);int x_h = max(from._x,to._x);
            for (int i = x_l; i != x_h; i++){
                demH[y][i] -= 1;
                db::GCell* gcell = gcells[i][y];
                gcell->demandH -= 1;
                vector<int>::iterator pos = find(gcell->netsX.begin(), gcell->netsX.end(), idx);
                if (pos != gcell->netsX.end()) // == myVector.end() means the element was not found
                    gcell->netsX.erase(pos);
                else{
                    logger->info() << "Error! Path not found!\n";
                    exit(1);
                }
            }
        }
    }
    r2pEdges[idx].clear();
    return true;
} //END MODULE

//---------------------------------------------------------------------

void Router::print_demand(){
    for(int j = 0; j < gridY; j++){
        for(int i = 0; i < gridX; i++){
            logger->info() << setw(3) << demV[i][j];
        }
        logger->info() << endl;
    }
    logger->info() << endl;
    logger->info() << endl;
    for(int j = 0; j < gridY; j++){
        for(int i = 0; i < gridX; i++){
            logger->info() << setw(3) << demH[j][i];
        }
        logger->info() << endl;
    }
    logger->info() << endl;
    logger->info() << endl;
    logger->info() << endl;
    logger->info() << endl;
} //END MODULE

//---------------------------------------------------------------------

void Router::write(const string& output_path) {
    ofstream outfile;
    outfile.open(output_path, ios::out);
    for (int idx = 0; idx < ntaps; idx++){
        outfile << "TAP " << database->taps[idx]->id() << "\n";
        outfile << "PINS " << tap_id_pin[idx].size() << "\n";
        for (int p : tap_id_pin[idx]) outfile << "PIN " << p << "\n";
        int num_route = 0;
        for (auto edges_idx : net_2pnet_map[idx]) {
            num_route += r2pEdges[edges_idx].size();
        }
        outfile << "ROUTING " << num_route << "\n";
        for (auto edges_idx : net_2pnet_map[idx]) {
            for (Edge edge : r2pEdges[edges_idx]){
                outfile << "EDGE " << edge << "\n";
            }
        }
        // outfile << "ROUTING " << rEdges[idx].size() << "\n";
        // for (Edge edge : rEdges[idx]){
        //     outfile << "EDGE " << edge << "\n";
        // }
    }
    outfile.close();
} //END MODULE

//---------------------------------------------------------------------

// bool Router::SingleNetPatternRoute(db::Net* net){
//     int x_1 = net->Pins[0]->pos_x;int y_1 = net->Pins[0]->pos_y;
//     int x_2 = net->Pins[1]->pos_x;int y_2 = net->Pins[1]->pos_y;

//     int x_l = min(x_1, x_2);int y_l = min(y_1, y_2);
//     int x_h = max(x_1, x_2);int y_h = max(y_1, y_2);

//     int x_s_1; int x_s_2;
//     int y_s_1; int y_s_2;
//     if (((x_2 - x_1) * (y_2 - y_1)) < 0){
//         x_s_1 = x_h; x_s_2 = x_l;
//         y_s_1 = y_h; y_s_2 = y_l;
//     } else{
//         x_s_1 = x_l; x_s_2 = x_h;
//         y_s_1 = y_l; y_s_2 = y_h;
//     }

//     int min_cost = INT_MAX;
//     int cost;
//     bool direction;     // true vertival | false horizontal
//     int bend;           // Bending point

//     // Vertical line
//     for (int i = x_l; i <= x_h; i++){
//         cost = 0;
//         cost += accumulate(demH[y_s_1].begin() + x_l, demH[y_s_1].begin() + i, 0);
//         cost += accumulate(demH[y_s_2].begin() + i, demH[y_s_2].begin() + x_h, 0);
//         cost += accumulate(demV[i].begin() + y_l, demV[i].begin() + y_h, 0);
//         if (cost < min_cost){
//             min_cost = cost;
//             direction = true;
//             bend = i;
//         }
//     }
    
//     // Horizontal line
//     for (int j = y_l; j <= y_h; j++){
//         cost = 0;
//         cost += accumulate(demV[x_s_1].begin() + y_l, demV[x_s_1].begin() + j, 0);
//         cost += accumulate(demV[x_s_2].begin() + j, demV[x_s_2].begin() + y_h, 0);
//         cost += accumulate(demH[j].begin() + x_l, demH[j].begin() + x_h, 0);
//         if (cost < min_cost){
//             min_cost = cost;
//             direction = false;
//             bend = j;
//         }
//     }

//     // Update demand matrix | Update route path
//     int idx = net->id();
//     if (direction == true){
//         for (int i = x_l; i != bend; i++){
//             demH[y_s_1][i] += 1;
//             db::GCell* gcell = gcells[i][y_s_1];
//             rpaths[idx].path.push_back(gcell);
//             rpaths[idx].direction.push_back(false);       // true vertival | false horizontal
//             gcell->demandH += 1;
//             gcell->netsX.push_back(idx);                  // X = H | Y = V
//         }
//         for (int j = y_l; j != y_h; j++){
//             demV[bend][j] += 1;
//             db::GCell* gcell = gcells[bend][j];
//             rpaths[idx].path.push_back(gcell);
//             rpaths[idx].direction.push_back(true);
//             gcell->demandV += 1;
//             gcell->netsY.push_back(idx);
//         }
//         for (int i = bend; i != x_h; i++){
//             demH[y_s_2][i] += 1;
//             db::GCell* gcell = gcells[i][y_s_2];
//             rpaths[idx].path.push_back(gcell);
//             rpaths[idx].direction.push_back(false);
//             gcell->demandH += 1;
//             gcell->netsX.push_back(idx);
//         }
//     } else{
//         for (int j = y_l; j != bend; j++){
//             demV[x_s_1][j] += 1;
//             db::GCell* gcell = gcells[x_s_1][j];
//             rpaths[idx].path.push_back(gcell);
//             rpaths[idx].direction.push_back(true);
//             gcell->demandV += 1;
//             gcell->netsY.push_back(idx);
//         }
//         for (int i = x_l; i != x_h; i++){
//             demH[bend][i] += 1;
//             db::GCell* gcell = gcells[i][bend];
//             rpaths[idx].path.push_back(gcell);
//             rpaths[idx].direction.push_back(false);
//             gcell->demandH += 1;
//             gcell->netsX.push_back(idx);
//         }
//         for (int j = bend; j != y_h; j++){
//             demV[x_s_2][j] += 1;
//             db::GCell* gcell = gcells[x_s_2][j];
//             rpaths[idx].path.push_back(gcell);
//             rpaths[idx].direction.push_back(true);
//             gcell->demandV += 1;
//             gcell->netsY.push_back(idx);
//         }
//     }

//     // Writer format
//     if (direction == true){
//         rpoints[idx].push_back(Point(x_l, y_s_1));
//         rpoints[idx].push_back(Point(bend, y_s_1));
//         rpoints[idx].push_back(Point(bend, y_s_2));
//         rpoints[idx].push_back(Point(x_h, y_s_2));
//     } else{
//         rpoints[idx].push_back(Point(x_s_1, y_l));
//         rpoints[idx].push_back(Point(x_s_1, bend));
//         rpoints[idx].push_back(Point(x_s_2, bend));
//         rpoints[idx].push_back(Point(x_s_2, y_h));
//     }

//     // Update queues
//     net_rflag[idx] = true;
//     // net_queue.clear();

//     return true;
// } //END MODULE

// //---------------------------------------------------------------------

// bool Router::patter_route(){
//     log() << "Start pattern route\n";
//     for(db::Net* net : net_queue) {
//         single_net_pattern(net);
//     }
//     return true;
// } //END MODULE

// //---------------------------------------------------------------------

// bool Router::unroute_net(db::Net* net){
//     int idx = net->id();
//     net_rflag[idx] = false;

//     for (int i = 0; i != rpaths[idx].direction.size(); i++){
//         bool direction = rpaths[idx].direction[i];
//         db::GCell* gcell = rpaths[idx].path[i];
//         if(direction){
//             demV[gcell->X_cor][gcell->Y_cor] -= 1;
//             gcell->demandV -= 1;
//             vector<int>::iterator pos = find(gcell->netsY.begin(), gcell->netsY.end(), idx);
//             if (pos != gcell->netsY.end()) // == myVector.end() means the element was not found
//                 gcell->netsY.erase(pos);
//             else{
//                 logger->info() << "Error! Path not found!\n";
//                 exit(1);
//             }
//         } else{
//             demH[gcell->Y_cor][gcell->X_cor] -= 1;
//             gcell->demandH -= 1;
//             vector<int>::iterator pos = find(gcell->netsX.begin(), gcell->netsX.end(), idx);
//             if (pos != gcell->netsX.end()) // == myVector.end() means the element was not found
//                 gcell->netsX.erase(pos);
//             else{
//                 logger->info() << "Error! Path not found!\n";
//                 exit(1);
//             }
//         }
//     }

//     rpaths[idx].clear();
//     // rpoints[idx].clear();    //FIXME: clear ovfl path
    
//     return true;
// } //END MODULE

// //---------------------------------------------------------------------
// // TODO: thres blk

// bool Router::break_ovfl(){
//     log() << "Start maze route\n";
//     // Calc ovfl
//     for(int i = 0; i < gridX; i++){
//         for(int j = 0; j < gridY; j++){
//             if (demV[i][j] > capV){            // X = H | Y = V
//                 int ovfl = demV[i][j] - capV;
//                 for(int idx : gcells[i][j]->netsY){
//                     net_ovfl[idx] += ovfl;
//                 }
//             }
//         }
//     }
//     for(int i = 0; i < gridX; i++){
//         for(int j = 0; j < gridY; j++){
//             if (demH[j][i] > capH){           // X = H | Y = V
//                 int ovfl = demH[j][i] - capH;
//                 db::Net* net = database->nets[gcells[i][j]->netsX.back()];
//                 for(int idx : gcells[i][j]->netsX){
//                     net_ovfl[idx] += ovfl;
//                 }
//             }
//         }
//     }

//     // Prepare for queues
//     auto ovflComp = [](const std::shared_ptr<net_prior> &lhs, const std::shared_ptr<net_prior> &rhs) {
//         return rhs->cost > lhs->cost;
//     };
//     priority_queue<std::shared_ptr<net_prior>, vector<std::shared_ptr<net_prior>>, 
//                         decltype(ovflComp)> ovflQueue(ovflComp);

//     // #1. Unroute till no ovfl
//     net_queue.clear();
//     for(int i = 0; i < gridX; i++){
//         for(int j = 0; j < gridY; j++){
//             while(demV[i][j] > capV){            // X = H | Y = V
//                 db::Net* net = database->nets[gcells[i][j]->netsY.back()];
//                 unroute_net(net);
//                 net_queue.push_back(net);
//                 net_rflag[net->id()] = false;
//                 ovflQueue.push(std::make_shared<net_prior>(net_ovfl[net->id()], net->id()));
//             }
//         }
//     }
//     for(int i = 0; i < gridX; i++){
//         for(int j = 0; j < gridY; j++){
//             while(demH[j][i] > capH){           // X = H | Y = V
//                 db::Net* net = database->nets[gcells[i][j]->netsX.back()];
//                 unroute_net(net);
//                 net_queue.push_back(net);
//                 net_rflag[net->id()] = false;
//                 ovflQueue.push(std::make_shared<net_prior>(net_ovfl[net->id()], net->id()));
//             }
//         }
//     }

//     // // #2. Unroute all ovfl nets
//     // for(db::Net* net : database->nets) {
//     //     if (net_ovfl[net->id()] > 0){
//     //         unroute_net(net);
//     //         ovflQueue.push(std::make_shared<net_prior>(net_ovfl[net->id()], net->id()));
//     //     }
//     // }

//     // Maze route nets with resp. # of ovfl
//     while (!ovflQueue.empty()) {
//         auto net_prior = ovflQueue.top();
//         ovflQueue.pop();
//         db::Net* net = database->nets[net_prior->idx];
//         // cout << net_prior->cost << endl;
//         single_net_maze(net);
//     }

//     return true;
// } //END MODULE

// //---------------------------------------------------------------------
// // TODO: thres blk

// bool Router::single_net_maze(db::Net* net){
//     // logger->info() << net->name() << ' ' << net_queue.size() << " nets left\n";
//     // Create maps
//     vector<vector<int>> visited(gridX, vector<int>(gridY, false));
//     vector<vector<int>> blkH(gridX, vector<int>(gridY, false));
//     vector<vector<int>> blkV(gridX, vector<int>(gridY, false));
//     for(int i = 0; i < gridX; i++){
//         for(int j = 0; j < gridY; j++){
//             blkH[i][j] = (demH[j][i] >= capH);
//             blkV[i][j] = (demV[i][j] >= capV);
//         }
//     }
//     int x_1 = net->Pins[0]->pos_x;int y_1 = net->Pins[0]->pos_y;
//     int x_2 = net->Pins[1]->pos_x;int y_2 = net->Pins[1]->pos_y;

//     // Prepare for queues
//     auto solComp = [](const std::shared_ptr<Vertex> &lhs, const std::shared_ptr<Vertex> &rhs) {
//         return rhs->cost < lhs->cost;
//     };
//     priority_queue<std::shared_ptr<Vertex>, vector<std::shared_ptr<Vertex>>, 
//                         decltype(solComp)> solQueue(solComp);

//     // Init vertex
//     solQueue.push(std::make_shared<Vertex>(0, Point(x_1, y_1), nullptr, -1));
//     Point dstPin(x_2, y_2);
//     Point srcPin(x_1, y_1);
//     std::shared_ptr<Vertex> dstVer = std::make_shared<Vertex>(0, Point(x_2, y_2), nullptr, -1);

//     // Hadlock's 
//     while (!solQueue.empty()) {
//         auto curVer = solQueue.top();
//         solQueue.pop();
//         int x_v = curVer->pos.x_;
//         int y_v = curVer->pos.y_;
//         int cost = curVer->cost;
//         visited[x_v][y_v] = true;

//         // reach a pin?
//         if (curVer->pos == dstPin) {
//             dstVer = curVer;
//             break;
//         }

//         // 4 directions
//         for(int d = 0; d < 4; d++){
//             // Next vertex to visit | remember to calc the bound
//             int x_off = Direction.x_off[d];
//             int y_off = Direction.y_off[d];
//             int x_new = x_v + x_off;
//             int y_new = y_v + y_off;
//             if (x_new < 0 || y_new < 0 || x_new >= gridX || y_new >= gridY) continue;

//             // This direction has block
//             if (d == 0 && blkV[x_v][y_v]) continue;
//             if (d == 1 && blkV[x_v][y_v - 1]) continue;
//             if (d == 2 && blkH[x_v - 1][y_v]) continue;
//             if (d == 3 && blkH[x_v][y_v]) continue;

//             // If not visited
//             if (!visited[x_new][y_new]) {
//                 Point newPin(x_new, y_new);
//                 int new_cost = cost;

//                 if ((newPin - dstPin) > (curVer->pos - dstPin)) new_cost++;
//                 solQueue.push(std::make_shared<Vertex>(new_cost, newPin, curVer, d));
//                 visited[x_new][y_new] = true;
//             }
//         }
//     }

//     int idx = net->id();
//     if (dstVer->prev == nullptr) return false;
//     rpoints[idx].clear();

//     shared_ptr<Vertex> curVer = dstVer;
//     rpoints[idx].push_back(curVer->pos);    // Start point
//     int cur_dir;
//     int next_dir = curVer->direction;
//     while (curVer->prev != nullptr){
//         shared_ptr<Vertex> preVer = curVer->prev;

//         cur_dir = curVer->direction;
//         if (cur_dir == 0) demV[preVer->pos.x_][preVer->pos.y_] += 1;
//         if (cur_dir == 1) demV[curVer->pos.x_][curVer->pos.y_] += 1;
//         if (cur_dir == 2) demH[curVer->pos.y_][curVer->pos.x_] += 1;
//         if (cur_dir == 3) demH[preVer->pos.y_][preVer->pos.x_] += 1;

//         if (cur_dir != next_dir) rpoints[idx].push_back(curVer->pos);    // Bend point

//         next_dir = cur_dir;
//         curVer = preVer;
//     }

//     if (!(srcPin == rpoints[idx].back()))  rpoints[idx].push_back(srcPin);

//     return true;
// } //END MODULE

// //---------------------------------------------------------------------

// void Router::print_demand(){
//     for(int j = 0; j < gridY; j++){
//         for(int i = 0; i < gridX; i++){
//             logger->info() << setw(3) << demV[i][j];
//         }
//         logger->info() << endl;
//     }
//     logger->info() << endl;
//     logger->info() << endl;
//     for(int j = 0; j < gridY; j++){
//         for(int i = 0; i < gridX; i++){
//             logger->info() << setw(3) << demH[j][i];
//         }
//         logger->info() << endl;
//     }
//     logger->info() << endl;
//     logger->info() << endl;
//     logger->info() << endl;
//     logger->info() << endl;
// } //END MODULE

// //---------------------------------------------------------------------

// void Router::write(const string& output_path) {
//     ofstream outfile;
//     outfile.open(output_path, ios::out);
    
//     for(int idx = 0; idx < database->nNets; idx++){
//         outfile << database->nets[idx]->name() << ' ' 
//                     << idx << "\n";
//         for(int j = 0; j < rpoints[idx].size() - 1; j++){
//             outfile << '(' << rpoints[idx][j].x_ << ", "
//                             << rpoints[idx][j].y_ << ", 1)-("
//                             << rpoints[idx][j+1].x_ << ", "
//                             << rpoints[idx][j+1].y_ << ", 1)\n";
//         }
//         outfile << "!\n";
//     }
//     outfile.close();
// } //END MODULE

// //---------------------------------------------------------------------

}