#include "floorplanner.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

void Floorplanner::parseBlock(fstream& input_blk){
    string head, key, val1, val2;
    _usedArea = 0;

    //Outline: <outline width, outline height>
	input_blk >> head >> val1 >> val2;
    _widthConstraint = stod(val1);
    _heightConstraint = stod(val2);
    _outlineAspect = _heightConstraint/_widthConstraint;
    _outlineArea = _widthConstraint*_heightConstraint;

    input_blk.ignore();
    
    //NumBlocks: <# of blocks>
	input_blk >> head >> val1;
    _blkNum = (size_t)stoi(val1);
    input_blk.ignore();
	
    //NumTerminals: <# of terminals>
	input_blk >> head >> val1;
    _tmlNum = (size_t)stoi(val1);
    input_blk.ignore();
	
    //<macro name> <macro width> <macro height>
    double maxSize = 0;
	for(int i=0; i<_blkNum; ++i){
		string name;
		double w;
		double h;

		input_blk >> head >> val1 >> val2;	
        name = head;
        w = stod(val1);
        h = stod(val2);

		_blkMap[name] = new Block(name, w, h);
        // _blkMap[name]->setMaxX(_width);
        // _blkMap[name]->setMaxY(_height);
        _usedArea += w*h;

        const double size = w + h;
        if(_bigBlk == NULL){
            _bigBlk = _blkMap[name];
            maxSize = size;
        }else{
            if(size > maxSize){
                _bigBlk = _blkMap[name];
                maxSize = size;
            }
        }

        input_blk.ignore();
	}

    //<terminal name> terminal <terminal x coordinate> <terminal y coordinate>
	for(size_t i=0; i<_tmlNum; ++i){
		string name;
		double x;
		double y;

		input_blk >> head >> key >> val1 >>val2;
        name = head;
        x = stod(val1);
        y = stod(val2);

        _tmlMap[name] = new Terminal(name, x, y);
        _tmlMap[name]->setPin();

        input_blk.ignore();
	}

    // printBlks();
    // printTmls();

	return; 
}

void Floorplanner::parseNet(fstream& input_net){
    string head, val;

    // NumNets: <# of nets>
    input_net >> head >> val;
    _netNum = stoi(val);
    input_net.ignore();

    
    for(size_t i=0; i<_netNum; ++i){
        // NetDegree: <# of terminals in this net>
        input_net >> head >> val;
        const size_t netDeg = (size_t)stoi(val);
        input_net.ignore();

        _netList.emplace_back(new Net());

        for(size_t j=0; j<netDeg; ++j){
            input_net >> head;
            // block || tml
            if(_tmlMap[head] == NULL){
                _netList[i]->addTerm(_blkMap[head]);
            }else{
                _netList[i]->addTerm(_tmlMap[head]);
            }
            input_net.ignore();
        }
    }
    // printNets();
    return;
}