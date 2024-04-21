// #pragma once

#ifndef MODULE_H
#define MODULE_H

#include <vector>
#include <string>
#include <unordered_map>
using namespace std;

//added
class Contour;

class Terminal
{
public:
    // constructor and destructor
    Terminal(string& name, double x, double y) :
        _name(name), _x1(x), _y1(y), _x2(x), _y2(y) { }
    ~Terminal()  { }

    // basic access methods
    const string getName()  { return _name; }
    const double getX1()    { return _x1; }
    const double getX2()    { return _x2; }
    const double getY1()    { return _y1; }
    const double getY2()    { return _y2; }
    pair<double, double> getPin()        { setPin(); return _pin; }

    // set functions
    void setName(string& name) { _name = name; }
    void setPos(double x1, double y1, double x2, double y2) {
        _x1 = x1;   _y1 = y1;
        _x2 = x2;   _y2 = y2;
    }
    void setPin(){ 
        _pin.first = (_x2+_x1)/2;
        _pin.second = (_y2+_y1)/2;
    }

protected:
    string      _name;      // module name
    double      _x1;        // min x coordinate of the terminal
    double      _y1;        // min y coordinate of the terminal
    double      _x2;        // max x coordinate of the terminal
    double      _y2;        // max y coordinate of the terminal
    pair<double, double>   _pin;       // pin of the blk
};


class Block : public Terminal
{
public:
    // constructor and destructor
    Block(string& name, double w, double h) :
        Terminal(name, 0, 0), _w(w), _h(h), _area(w*h) { }
    ~Block() { }

    // basic access methods
    const double getWidth(bool rotate = false)  { return rotate? _h: _w; }
    const double getHeight(bool rotate = false) { return rotate? _w: _h; }
    const double getArea()  { return _area; }
    // static double getMaxX() { return _maxX; }
    // static double getMaxY() { return _maxY; }
    double getAspect()                          { return _aspect; }

    // set functions
    void setWidth(double w)         { _w = w; }
    void setHeight(double h)        { _h = h; }
    void setAspect()                { _aspect = _h/_w; }
    void rotate()                   {double temp=_w; _w = _h; _h = temp; }
    // static void setMaxX(double x)   { _maxX = x; }
    // static void setMaxY(double y)   { _maxY = y; }

    // added: tree access methods
    Block* getParent(){return _parent;}
    Block* getRight(){return _rchild;}
    Block* getLeft(){return _lchild;}
    bool getSide() { return _side;} 
    size_t getIdx() { return _idx;}

    // added: tree set operation
    void setParent(Block* const parent)  {_parent = parent;}
    void setLeft(Block* const lchild)    {_lchild = lchild;}
    void setRight(Block* const rchild)   {_rchild = rchild;}
    void setSide(const bool side) { _side = side;} 
    void setIdx(const size_t idx) {_idx = idx;}


    // added: deep copy
    // void deepCopy(Tempblk* blk);

private:
    double          _w;         // width of the block
    double          _h;         // height of the block
    double          _aspect;
    double          _area;      
    // static double   _maxX;      // maximum x coordinate for all blocks
    // static double   _maxY;      // maximum y coordinate for all blocks


    // added: tree members
    Block* _parent;
    Block* _rchild;
    Block* _lchild;
    size_t _idx;                // map to _blkVec
    bool _side;                 // record where the block is (0 for leftchild(rightBlk), 1 for rchild(topBlk))
};

class Net
{
public:
    // constructor and destructor
    Net()   { }
    ~Net()  { }

    // basic access methods
    const vector<Terminal*> getTermList()   { return _tmlList; }

    // modify methods
    void addTerm(Terminal* tml) { _tmlList.push_back(tml); }

    // other member functions
    double calcHPWL();

private:
    vector<Terminal*>   _tmlList;  // list of terminals the net is connected to
};

// added
class Contour
{
public:
    // constructor and destructor
    Contour(double x, double y):   
        _xCor(x),
        _yCor(y),
        _next(NULL),
        _prev(NULL)
    { }
    ~Contour()  { }

    // basic access methods
    const double getxCor() {return _xCor;}
    const double getyCor() {return _yCor;}
    Contour* getNext() {return _next;}
    Contour* getPrev() {return _prev;}

    // modify methods
    void setNext(Contour* const node) {_next = node;}
    void setPrev(Contour* const node) {_prev = node;}
    void setState(const double x, const double y)
    {
        _xCor = x;
        _yCor = y; 
    }

private:
    double _xCor;
    double _yCor;
    Contour* _next;
    Contour* _prev;
};

class Tempblk
{
public:
    // constructor and destructor
    Tempblk(string& name):  _name(name), _parent(NULL), _left(NULL), _right(NULL)  { }
    ~Tempblk()  { }
    
    // basic access methods
    string getName(){ return _name; }    
    Block* getParent(){ return _parent; }
    Block* getLeft(){ return _left; }
    Block* getRight(){ return _right; }
    size_t getIdx() { return _idx;}
    pair<double, double> getPos() { return _widthandheight; }
    bool getSide() { return _side;}


    // modify methods
    void setIdx(size_t idx){ _idx = idx; }  
    void setParent(Block* parent){ _parent = parent; }
    void setLeft(Block* left){ _left = left; }
    void setRight(Block* right){  _right = right; }
    void setPos(double w, double h){
        _widthandheight.first = w;
        _widthandheight.second = h;
    }
    void setSide(bool side){ _side = side; } 

private:
    size_t _idx;
    string _name; 
    Block* _parent;
    Block* _left;
    Block* _right;
    pair<double, double> _widthandheight;
    bool _side;
};


#endif  // MODULE_H
