#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include "module.h"
#include <map>
#include <deque>
using namespace std;

class Floorplanner{
public:
    // constructor and deconstructor
    Floorplanner(fstream& input_blk, fstream& input_net, const double alpha):
    _alpha(alpha)
    {
        parseBlock(input_blk);
        parseNet(input_net);
    }
    ~Floorplanner(){
        clear();
    }

    // basic access methods
    double getArea() { setArea(); return _area; }
    double getWL()   { setWL(); return _wl; }
    double getDeadspaceRate() { return 1-_usedArea/_area; }
    double getAdaptiveCost();
    double getCost(){ return _alpha*_area/_areaNorm + (1-_alpha)*_wl/_wireNorm;}

    // modify method
    void parseBlock(fstream& inBlock);
    void parseNet(fstream& inNet);
    void setArea() {_area = _ctrx*_ctry;}
    void setWL() {
        _wl = 0;
        for(const auto &item: _netList){
            _wl += item->calcHPWL();
        }
    }
    void floorplan();

    // Contour
    void initContour(Block* root);
    void updateContour(const double xCor1,const double yCor1, const double xCor2,const double yCor2);
    double contourY(const double x_start, const double x_end);
    void updateRad();
    void clearContour();

    // B*-tree
    void initialize(bool init);
    void initBuild(const bool init);
    void initPacking(Block* const parent,const int opblk, const bool loosen);
    void packing(const int opblk,const bool build);  
    void eraseVec(Block* const blk, const size_t idx);
    void reconstuctVec(Block* const blk, const size_t idx);

    // Perturb
    pair<int, int> perturb(const int move_choice);
    void perturbRecover(const int move_choice, const int choice1, const int choice2);
    void rotate(Block* const block) {block->rotate();}
    void delIns(Block* const blk1, Block* const blk2);
    void swap(const size_t blk1,const bool side1,const size_t blk2,const bool side2, const bool isparent);

    // SA
    void record(const bool best);
    void recover(const bool best);
    double TempUpdate(const double iter, const double delta_cost);
    void recordFeas(const bool feas);

    // Reports
    void writeResult(ofstream& outFile, clock_t start);
    void printBlks();
    void printTmls();
    void printNets();
    void plotSVG(string* filename);
    void printResult(clock_t start);
    
private:
    // basic data
    double _alpha;
    double _widthConstraint;
    double _heightConstraint;
    double _outlineArea;
    double _usedArea;
    double _outlineAspect;
    size_t _blkNum;
    size_t _tmlNum;
    size_t _netNum;


    // members
    unordered_map<string, Block*> _blkMap;      // blk name to blk
    unordered_map<size_t, Block*> _blkVec;      // record blk relation with idx
    vector<Tempblk*> _bestVec;                  // record the best step
    vector<Tempblk*> _lastVec;                  // record the last step
    vector<Block*> _blkList;                    // list all the blks
    unordered_map<string, Terminal*> _tmlMap;   // terminal name to terminal
    vector<Net*> _netList;                      // record the nests
    Block* _bigBlk;                             // bigest block
    double _area;                               // result area
    double _wl;                                 // result wire length

    // contour
    map<bool, Contour*> _ctrHead;               // 0 for horizon contour
    double _ctrx;                               // the widest edge
    double _ctry;                               // the highest edge
    bool _init;                                 // record if heuristic init succeeded

    // SA
    double _areaNorm;                           // average area
    double _wireNorm;                           // average wire length
    double _temp1;                              // init temperature
    deque<bool> _feas_queue;                    // feasible step record
    size_t _n_feas;                             // feasible steps in n steps
    

    void clear();
};

#endif  // FLOORPLANNER.H 