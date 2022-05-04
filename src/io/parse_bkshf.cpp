#include "global.h"
#include "db/Database.h"

using namespace db;

class BookshelfData {
public:
    int nCells;
    int nNets;
    int nMacros;
    int nPins;
    int nTaps;
    int nTerminals;
    int nGridX;
    int nGridY;
    int Vcap;
    int Hcap;
    int MaxLoad;
    
    std::string format;
    unordered_map<string, int> cellMap;

    vector<string> cellName;
    vector<int> cellSize;
    vector<char> cellType;
    vector<int> cellX;
    vector<int> cellY;

    vector<Point> pins;
    vector<Point> taps;


    void clearData() {
        cellMap.clear();
        cellName.clear();
    }
};

BookshelfData bsData;

//---------------------------------------------------------------------

bool isBookshelfSymbol(unsigned char c) {
    static char symbols[256] = {0};
    static bool inited = false;
    if (!inited) {
        symbols[(int)'('] = 1;
        symbols[(int)')'] = 1;
        // symbols[(int)'['] = 1;
        // symbols[(int)']'] = 1;
        symbols[(int)','] = 1;
        // symbols[(int)'.'] = 1;
        symbols[(int)':'] = 1;
        symbols[(int)';'] = 1;
        // symbols[(int)'/'] = 1;
        symbols[(int)'#'] = 1;
        symbols[(int)'{'] = 1;
        symbols[(int)'}'] = 1;
        symbols[(int)'*'] = 1;
        symbols[(int)'\"'] = 1;
        symbols[(int)'\\'] = 1;

        symbols[(int)' '] = 2;
        symbols[(int)'\t'] = 2;
        symbols[(int)'\n'] = 2;
        symbols[(int)'\r'] = 2;
        inited = true;
    }
    return symbols[(int)c] != 0;
} //END MODULE

//---------------------------------------------------------------------

bool readBSLine(istream& is, vector<string>& tokens) {
    tokens.clear();
    string line;
    while (is && tokens.empty()) {
        // read next line in
        getline(is, line);

        char token[1024] = {0};
        int lineLen = (int)line.size();
        int tokenLen = 0;
        for (int i = 0; i < lineLen; i++) {
            char c = line[i];
            if (c == '#') {
                break;
            }
            if (isBookshelfSymbol(c)) {
                if (tokenLen > 0) {
                    token[tokenLen] = (char)0;
                    tokens.push_back(string(token));
                    token[0] = (char)0;
                    tokenLen = 0;
                }
            } else {
                token[tokenLen++] = c;
                if (tokenLen > 1024) {
                    // TODO: unhandled error
                    tokens.clear();
                    return false;
                }
            }
        }
        // line finished, something else in token
        if (tokenLen > 0) {
            token[tokenLen] = (char)0;
            tokens.push_back(string(token));
            tokenLen = 0;
        }
    }
    return !tokens.empty();
} //END MODULE

//---------------------------------------------------------------------

bool Database::readPin(const std::string& file) {
    log() << "reading pins" << std::endl;
    ifstream fs(file.c_str());
    if (!fs.good()) {
        printlog("cannot open file: %s", file.c_str());
        return false;
    }

    vector<string> tokens;
    while (readBSLine(fs, tokens)) {
        if (tokens[0] == "CAPACITY") {
            bsData.Hcap = atoi(tokens[1].c_str());
            bsData.Vcap = atoi(tokens[1].c_str());
        } else if (tokens[0] == "GRID_SIZE") {
            bsData.nGridX = atoi(tokens[1].c_str());
            bsData.nGridY = atoi(tokens[1].c_str());
        } else if (tokens[0] == "MAX_LOAD") {
            bsData.MaxLoad = atoi(tokens[1].c_str());
        } else if (tokens[0] == "PINS") {
            bsData.nPins = atoi(tokens[1].c_str());
        } else if (tokens[0] == "TAPS") {
            bsData.nTaps = atoi(tokens[1].c_str());
        } else if (tokens[0] == "PIN") {
            int id = atoi(tokens[1].c_str());
            int x = atoi(tokens[2].c_str());
            int y = atoi(tokens[3].c_str());
            bsData.pins.emplace_back(x,y);
        } else if (tokens[0] == "TAP") {
            int id = atoi(tokens[1].c_str());
            int x = atoi(tokens[2].c_str());
            int y = atoi(tokens[3].c_str());
            bsData.taps.emplace_back(x,y);
        } else{
            printlog("Warning: Invalid token!");
        }
    }
    if(bsData.pins.size() != bsData.nPins) printlog("Error: Wrong number of pins!\n");
    if(bsData.taps.size() != bsData.nTaps) printlog("Error: Wrong number of pins!\n");
    
    fs.close();
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Database::read(const std::string& inFile) {
    readPin(inFile);

    this->nPins = bsData.nPins;
    this->nTaps = bsData.nTaps;
    this->MaxLoad = bsData.MaxLoad;

    // log() << this->nPins << " | " << this->nTaps << endl;

    // pins
    for (int i = 0; i != bsData.nPins; ++i) {
        int ID = i;
        string name = to_string(i);
        const Point pos = bsData.pins[i];
        
        Pin* pin = this->addPin(ID, pos);
        pin->set_name(name);

        // log() << pos;
    }

    // taps
    for (int i = 0; i != bsData.nTaps; ++i) {
        int ID = i;
        string name = to_string(i);
        const Point pos = bsData.taps[i];
        
        Tap* tap = this->addTap(ID, pos);
        tap->set_name(name);
        tap->set_load(bsData.MaxLoad);

        Pin* pin = this->addPin(ID, 't', pos);
        pin->set_name(name);
        pin->set_load(bsData.MaxLoad);

        // log() << pos;
    }

    grGrid.init(bsData.nGridX, bsData.nGridY, bsData.Hcap, bsData.Vcap);

    return true;
} //END MODULE

//---------------------------------------------------------------------