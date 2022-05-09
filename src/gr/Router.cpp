#include "Router.h"

typedef lemon::ListGraph UGraph;
typedef UGraph::EdgeMap<double> DistMap;
typedef lemon::MaxWeightedPerfectMatching<UGraph,DistMap> MWPM;
typedef lemon::MaxWeightedMatching<UGraph,DistMap> MWM;

namespace gr {

Router::Router(db::Database* database_) :   
    database(database_) {
    grGrid = database->grGrid;
    logger = database->logger;

    /* Grgrid info */
    gridX = grGrid.nx;
    gridY = grGrid.ny;
    capH = grGrid.Hcap;
    capV = grGrid.Vcap;
    npins = database->nPins;
    ntaps = database->nTaps;
    MaxLoad = database->MaxLoad;
    printlog(npins, " pins");
    printlog(ntaps, " taps");
    printlog(MaxLoad, " MaxLoad");
    log() << (npins/ntaps) << " pins per tap\n";

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
    // Construct Steiner tree
    const int degree = net.pinPoints.size();
    if (degree == 1) return true;
    int xs[degree * 100];
    int ys[degree * 100];
    int i = 0;
    for (auto& accessPoint : net.pinPoints) {
        xs[i] = accessPoint.x;
        ys[i] = accessPoint.y;
        i++;
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
                    logger->info() << "Error! Path not found!\nPlease contact administrator\n";
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
                    logger->info() << "Error! Path not found!\nPlease contact administrator\n";
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
    }
    outfile.close();
} //END MODULE

//---------------------------------------------------------------------

}