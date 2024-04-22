#include "floorplanner.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cmath>
#include <iomanip>
using namespace std;

void Floorplanner::clear(){
    // tml
    vector<string> keysToDelete;
    for(auto& item : _tmlMap){
        keysToDelete.push_back(item.first);
    }
    for (string key : keysToDelete) {
        delete _tmlMap[key];
        _tmlMap.erase(key);
    }

    // best
    keysToDelete.clear();
    for(auto& item : _bestVec){
        keysToDelete.push_back(item.first);
    }
    for (Tempblk key : keysToDelete){
        delete _bestVec[key];
        _bestVec.erase();
    }

    // last
    keysToDelete.clear();
    for(auto& item : _lastVec){
        keysToDelete.push_back(item.first);
    }
    for (Tempblk key : keysToDelete){
        delete _lastVec[key];
        _lastVec.erase();
    }

    // block
    for(auto& item : _blkVec){
        _blkVec.erase(item.first);
    }

    for(auto& item : _blkMap){
        _blkMap.erase(item.first);
    }

    for(auto& item : _blkList){
        delete item;
    }

    // net
    for(auto& item: _netList){
        delete item;
    }

    clearContour();

    return;
}

void Floorplanner::initialize(bool init){
    // Define root
    Block* root;
    root = _bigBlk;

    // first tree in whole program
    if(init){
        _blkList.emplace_back(_bigBlk);
        
        if(_bigBlk->getWidth() >= _bigBlk->getHeight()){
            _bigBlk->rotate();
        }
    }
    // tree latter
    else{
        root = _blkVec[0];
    }

    root->setPos(0, 0, root->getWidth(0), root->getHeight(0));
    root->setPin();
    
    // initiate contour
    initContour(root);
};

bool compareHeight(Block* const a, Block* const b) {
    return a->getHeight() > b->getHeight();
}

void Floorplanner::initBuild(const bool init){
    // initiate root and contour
    initialize(init);

    // init build tree
    if(init){
        for(const auto& item : _blkMap){
            Block* const blk = item.second;
            if(blk->getWidth() >= blk->getHeight()){
                blk->rotate();
            }

            if(blk->getName() == _bigBlk->getName()){
                continue;
            }

            _blkList.emplace_back(blk);
        }

        sort(_blkList.begin()+1, _blkList.end(), compareHeight);
    }
    // biuld tree with _blkVec
    else{
        for(const auto& item : _blkMap){
            Block* const blk = item.second;
            blk->setRight(NULL);
            blk->setLeft(NULL);
        }
    }

    initPacking(_bigBlk, 1, !init);
}

void Floorplanner::initPacking(Block* const parent, const int opblk, const bool loosen){
    // all nodes are iterated
    if(opblk == _blkList.size()){
        return;
    }

    // set constraint
    double loosen_rate = 1;
    if(loosen){
        loosen_rate = 1.065;
    }
    const double height = _heightConstraint*loosen_rate;
    const double width = _widthConstraint*loosen_rate;
    
    // blk under insert operation
    Block* blk = _blkList[opblk];

    // DFS placing order
    double x_start=0, x_end=0, max_yCor=0;
    
    // check if the tree is valid

    if(parent->getName() == _bigBlk->getName() && opblk > 1){
        // root is filled
        if( _bigBlk->getLeft()!=NULL && _bigBlk->getRight()!=NULL){
            // cout << "Root is filled, failed building tree" << endl;
            _init = 0;
            return;
        }
        // check space
        else{
            x_start = parent->getX1();
            x_end = x_start + blk->getWidth(0);
            max_yCor = contourY(x_start, x_end);

            // try rotating
            if(x_end > width){
                blk->rotate();
                x_end = x_start + blk->getWidth(0);
                max_yCor = contourY(x_start, x_end);
            }

            // check space
            max_yCor = max(parent->getY2(), max_yCor);
            if( parent->getRight()==NULL
                && x_end > width
                && blk->getHeight(0) + max_yCor > height
            ){
                // cout << "Space is full, failed building tree" << endl;
                _init = 0;
                return;
            }
        }
    } 

    // condition1: leftchild
    x_start = parent->getX2();
    x_end = x_start + blk->getWidth(0);
    max_yCor = contourY(x_start, x_end);

    // check space
    if( parent->getLeft()==NULL
        && x_end <= width
        && blk->getHeight(0) + max_yCor <= height
    ){
        // palce in leftchild
        parent->setLeft(blk);
        blk->setParent(parent);

        // set side
        blk->setSide(0);
    }
    else{
        // condition2: rightchild
        x_start = parent->getX1();
        x_end = x_start + blk->getWidth(0);
        max_yCor = contourY(x_start, x_end);

        // check space
        max_yCor = max(parent->getY2(), max_yCor);
        if( parent->getRight()==NULL
            && x_end <= width
            && blk->getHeight(0) + max_yCor <= height
        ){
            // palce in rightchild
            parent->setRight(blk);
            blk->setParent(parent);

            // set side
            blk->setSide(1);
        }
        else{
            // right and left are both unavailable
            if(loosen){
                initPacking(parent->getParent(), opblk, 1);
            }else{
                initPacking(parent->getParent(), opblk, 0);
            }
            return;
        }
    }
  
    // update child(x1, y1, x2, y2)
    blk->setPos(x_start, max_yCor, x_end, max_yCor+blk->getHeight());
    blk->setPin();

    // update ctr
    updateContour(x_start, max_yCor, x_end, max_yCor+blk->getHeight());

    // update raduis
    updateRad();

    // recursion
    if(loosen){
        initPacking(blk, opblk+1, 1);
    }else{
        initPacking(blk, opblk+1, 0);
    }

    return; 
}

void Floorplanner::eraseVec(Block* const blk, const size_t idx){
    const size_t left = idx*2+1;
    const size_t right = (idx+1)*2;
    Block* const lchild = blk->getLeft();
    Block* const rchild = blk->getRight();

    if(lchild==NULL && rchild==NULL){
        _blkVec.erase(idx);
        return;
    }
    else{
        if(blk->getLeft()!= NULL){
            eraseVec(lchild, left);
        }

        if(blk->getRight()!= NULL){
            eraseVec(rchild, right);
        }
    }

    _blkVec.erase(idx);
    return;
}

void Floorplanner::reconstuctVec(Block* const blk, const size_t i){
    size_t idx = i;
    Block* const lchild = blk->getLeft();
    Block* const rchild = blk->getRight();

    if(_blkVec[idx]==NULL){
        _blkVec[idx] = blk;
    }

// cout << "["<< idx << "] " << blk->getName() << endl;
    // recursive

    // cout << " left" << endl;
    if(lchild!= NULL){
        idx = idx*2+1;
        _blkVec[idx] = lchild;
        lchild->setIdx(idx);
        reconstuctVec(lchild, idx);
    }

    idx = i;
// cout << "["<< idx << "] " << blk->getName() << endl;

    // cout << " right" << endl;
    if(rchild!= NULL){
        idx = (idx+1)*2;
        _blkVec[idx] = rchild;
        rchild->setIdx(idx);
        reconstuctVec(rchild, idx);
    }
    return;
}

void Floorplanner::record(const bool best){
    // init _bestVec
    if(best){
        _bestVec.clear();
    }
    // init _lastVec
    else{
        _lastVec.clear();
    }

    // build recover list
    for(const auto& item:_blkMap){
        string name = item.second->getName();
        Tempblk* blk = new Tempblk(name);

        // choose vec
        if(best){
            _bestVec.emplace_back(blk);
        }
        else{
            _lastVec.emplace_back(blk);
        }

        blk->setIdx(item.second->getIdx());
        blk->setPos(item.second->getWidth(), item.second->getHeight());
        blk->setParent(item.second->getParent());
        blk->setLeft(item.second->getLeft());
        blk->setRight(item.second->getRight());
        blk->setSide(item.second->getSide());
    }

    return;
}

void Floorplanner::recover(const bool best){
    // clear _blkVec
    eraseVec(_blkVec[0], 0);

    // choose vec
    vector<Tempblk*> vec; 
    if(best){
        vec = _bestVec;
    }else{
        vec = _lastVec;
    } 
    for(const auto& item : vec){
        string name = item->getName();
        const size_t idx = item->getIdx();
        Block* const blk = _blkMap[name];

        blk->setIdx(item->getIdx());
        blk->setWidth(item->getPos().first);
        blk->setHeight(item->getPos().second);
        blk->setParent(item->getParent());
        blk->setLeft(item->getLeft());
        blk->setRight(item->getRight());
        blk->setSide(item->getSide());

        if(idx == 0){
            _blkVec[0] = blk;
        }
    }

    // recover _blkList
    reconstuctVec(_blkVec[0], 0);

    // recover contour line
    initialize(0);
    packing(0, 0);

    return;
}

void Floorplanner::packing(const int opblk, const bool build){
    // cout << opblk << "      ";
    Block* const blk = _blkVec[opblk];
    Block* const parent = _blkVec[opblk]->getParent();

    double x_start=0, x_end=0, max_yCor = 0;
    const bool side = blk->getSide();
    

    if(build == 1){
        // leftchild
        if(opblk%2==1){
            // decide x, y value
            x_start = parent->getX2();
            x_end = x_start + blk->getWidth(0);
            max_yCor = contourY(x_start, x_end);
        }
        else{
            // decide x, y value
            x_start = parent->getX1();
            x_end = x_start + blk->getWidth(0);
            max_yCor = contourY(x_start, x_end);
        }

        // set x, y value
        blk->setPos(x_start, max_yCor, x_end, max_yCor+blk->getHeight());
        blk->setPin();

        // update contour line
        updateContour(x_start, max_yCor, x_end, max_yCor+blk->getHeight());

        // update raduis
        updateRad();
    }

    // recursive
    if(blk->getLeft()!= NULL){
        // cout << " pack left" << "      ";
        packing(opblk*2+1, 1);
    }
    if(blk->getRight()!= NULL){
        // cout << " pack right" << "      ";
        packing((opblk+1)*2, 1);
    }

    return;  
}

void Floorplanner::floorplan(){
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    srand(seed);

// cout << "--------- initiate tree with heuristic or random method ---------" << endl;

    _init = 1;

    initBuild(1);
    // init solution is legal
    if(!_init){
        _blkVec[0] = _bigBlk;
        initBuild(0);
    }
    // biuld _blkVec with init result
    _blkVec[0] = _bigBlk;
    // reconstruct _blkVec
    reconstuctVec(_bigBlk, 0);

    // double init_area = getArea();
    // double init_wire = getWL();

// cout << "init cost: " << getCost() << "\n" << endl;


// cout << "--------- Calculate norm_factor ---------" << endl;

    // decide iter count
    size_t iter = (rand()%10+1)*100;  // total iterations

    // setup variables for norm_factor(area_norm, wl_norm)
    _areaNorm = 0;
    _wireNorm = 0;

    // random perturbute
    int move_choice;
    for(size_t n=0; n<iter; ++n){
        move_choice = rand() % 3;

        // perturb
        perturb(move_choice);

        // pack
        initialize(0);
        packing(0, 0);

        // get avg_area, avg_wl
        _areaNorm += getArea();
        _wireNorm += getWL();
    }

    _areaNorm = _areaNorm/(double)iter;
    _wireNorm = _wireNorm/(double)iter;

// cout << "_areaNorm " << _areaNorm << endl;
// cout << "_wireNorm " <<_wireNorm << "\n" << endl;


// cout << "--------- Calculate avg_uphill_cost ---------" << endl;

    // setup variables for average_uphill_cost
    size_t up = 0; // total uphill moves
    double avg_uphill_cost = 0; // record uphill cost and calculate the average
    double cost = getAdaptiveCost();
    double prev_cost = cost;

    // random perturbute
    while(up < 10){
        iter = (rand()%5+1)*100;
        for(size_t n=0; n<iter; ++n){
            move_choice = rand() % 3;

            // perturb
            perturb(move_choice);

            // pack
            initialize(0);
            packing(0, 0);

            // evaluate cost
            cost = getAdaptiveCost();

            // record uphills
            if(prev_cost < cost){
                ++up;
                avg_uphill_cost += (cost - prev_cost);
                prev_cost = cost;
            }
        }
    }

    avg_uphill_cost = avg_uphill_cost/(double)up;

//     cout << "avg_uphill_cost " << avg_uphill_cost << "\n" << endl;


// cout << "--------- Fast SA ---------" << endl;

    // init record
    record(1);

    // setup init temperature
    double avg_delta_cost = 0;
    double P = 0.85; // probability to be feasible
    _temp1 = (-1)*avg_uphill_cost/log(P);
    double temp = _temp1;

    // init cost
    if(_ctrx <= _widthConstraint && _ctry <= _heightConstraint){
        recordFeas(true);
    }else{
        recordFeas(false);
    }
    cost = getAdaptiveCost();
    double best_cost = cost;
    prev_cost = cost;
    
    // record the state of each iteration
    double move;
    double reject;
    size_t max_up = 20*_blkNum; // k=20 is self defined
    size_t max_move = 2*max_up; // k=2 is self defined

    iter = 0;
    while(temp >= 0.0003){
        // stage 3: temperature is raised to facilitate hill climbing to search for better solutions
        if(iter>7){
                // cout << "--------- stage III ---------" << endl;
                temp = TempUpdate(iter, avg_delta_cost);
        }else{
            // stage 1: temperature is set to a very large to avoid getting trapped in a local optimal
            if(iter < 1){
                // cout << "--------- stage I ---------" << endl;  
                temp = _temp1;
            }
            // stage 2: temperature approach zero to accept only a small number of inferior solutions
            else{ 
                // cout << "--------- stage II ---------" << endl;   
                temp = TempUpdate(iter, avg_delta_cost);
            }
        }
        cout << "temperture: "<< temp << endl;

        // setup init cost for each iteration
        avg_delta_cost = 0;

        // init state for each iteration
        up = 0;
        move = 0;
        reject = 0;

        // start iteration
        while(up<=max_up && move<=max_move){ 
            move_choice = rand() % 3;
            // record if del and insert is choosen
            if(move_choice == 1){
                record(0);
            }

            // perturb
            pair<int, int> choice;
            choice = perturb(move_choice);
            ++move;

            // pack
            initialize(0);
            packing(0, 0);

            // evaluate cost
            cost = getAdaptiveCost();
            double delta_cost = cost - prev_cost;
            avg_delta_cost += delta_cost;
            
            // decide acceptance
            double random = (rand()%10000)/(double)10000;
            P = exp((-1)*delta_cost/temp);
            P = min((double)1, P);

            // record Best and uphills
            if(cost < prev_cost || random <= P){
                // feasible
                recordFeas(true);

                // check if its a uphill
                if(delta_cost > 0){
                    ++up;
                }

                // record best
                if(cost < best_cost){
                    best_cost = cost;
                    record(1);
                }

                // update previous cost
                prev_cost = cost;          
            }else{
                // rejected
                recordFeas(false);
                ++reject;
                ++up;

                // recover to the last move
                if(move_choice == 1){
                    recover(0);
                }else{
                    perturbRecover(move_choice, choice.first, choice.second);
                }
                
                // recover total_delta_cost
                // total_delta_cost -= delta_cost;
            }
        }

        avg_delta_cost = avg_delta_cost/move;
        ++iter;

        if(reject/move > 0.98){
            if(iter < 7 || temp > 0.005){
                continue;
            }else{
                break;
            }
        }
    }

cout << "\n==========   RESULT   ==========" << endl;

// double init_cost = _alpha*init_area/_areaNorm + (1-_alpha)*init_wire/_wireNorm;

cout << "iterations: "<< iter << endl;

// if(init_cost < getCost()){
//     cout << "bad result" << endl;
// }else{
//     cout << "good result" << endl;
// }

// cout << init_cost <<" vs " << getCost() << endl;

    recover(1);

    if(_ctrx <= _widthConstraint && _ctry <= _heightConstraint ){
        cout << "valid result" << endl;
    }else{
        cout << "invalid result" << endl;
    }

cout << "========== ========== ==========" << endl;
    return;
}

double Floorplanner::TempUpdate(const double iter, const double delta_cost){
    // K = 7 is self defined
    if(iter <= 7){
        // c = 100 is self defined
        return _temp1 * delta_cost/iter/100;
    }
    else{
        return _temp1 * delta_cost/iter;
    }
    
}

double Floorplanner::getAdaptiveCost(){
    // get cost
    getArea();
    getWL();
    double cost = getCost();

    // calculate outflow
    double outflow = 0;
    if(_ctrx > _widthConstraint || _ctry > _heightConstraint){
        double max_x = max(_ctrx, _widthConstraint);
        double max_y = max(_ctry, _heightConstraint);
        outflow = max_x * max_y / _outlineArea;
    }

    // calculate whitespace
    double white = _area - _usedArea;
    white = white / _area;
    if(white < 0.15){
        white = 0 ;
    }

    // self defined fixed outline
    // let maximum percent of dead space Γ be 10%
    // expected aspect R* set to 1
    // _fixedwidth = sqrt((1.1)*_outlineArea);
    // _fixedheight = sqrt((1.1)*_outlineArea);

    // calculate aspect
    // double R = _ctry/_ctrx;
    // R = _outlineAspect - R;

    // penalty = (1-alpha-(1-_alpha))*(_outlineAspect - 1)*(_outlineAspect - 1);
    double feas_rate; // feasible solutions in n most recent floorplan solutions
    if(_feas_queue.size() == 0){
        feas_rate = 1;
    }else{
        feas_rate = _n_feas/_feas_queue.size();
    }
    double panalty_rate = 0.5 + (0.5)*(feas_rate);

    // calculate adaptive cost
    double penalty = (panalty_rate)*((outflow)*(outflow) + (white)*(white));
    cost = (1 - panalty_rate) * cost;

    // return cost + penalty;
    return cost + penalty;
}

void Floorplanner::recordFeas(const bool feas){
    // add recent feas
    _feas_queue.push_front(feas);
    if(feas){
        ++_n_feas;
    }

    // recent count is self defined as _blkNum*20
    if(_feas_queue.size() >  500){
        if(_feas_queue.back()){
            --_n_feas;
        }
        _feas_queue.pop_back();
    }

    return;
}