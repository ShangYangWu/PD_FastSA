#include "floorplanner.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
using namespace std;

void Floorplanner::printBlks(){
    cout << "\n---------------------------------" << endl;
    cout << "[ printBlks ]\n" << endl;
    for(const auto& item : _blkMap){
        cout << item.first << " : " << item.second->getWidth(0) << " x " << item.second->getHeight(0) << endl;  
    }
    cout << "---------------------------------" << endl;
    return;
}
void Floorplanner::printTmls(){
    cout << "\n---------------------------------" << endl;
    cout << "[ printTmls ]\n" << endl;
    for(const auto& item : _tmlMap){
        cout << item.first << "- x: " << item.second->getX1()<< " / y: " << item.second->getY1() << endl;  
    }
    cout << "---------------------------------" << endl;
    return;
}
void Floorplanner::printNets(){
    cout << "\n---------------------------------" << endl;
    cout << "[ printNets ]\n" << endl;
    size_t i=0;
    for(const auto& item : _netList){
        cout << "Net " << i << endl;
        for(const auto&it : item->getTermList()){
            cout << it->getName() << " ";
        }
        cout << endl;
        ++i;
    }
    cout << "---------------------------------" << endl;
    return;
}

void Floorplanner::writeResult(ofstream& outFile, clock_t start){
    setArea();
    setWL();
    
    if(!outFile.is_open()){
        cerr << "SVG printing error" << endl;
    }else{
        streambuf *coutbuf = cout.rdbuf();
        streambuf *filebuf = outFile.rdbuf();
        cout.rdbuf(filebuf);

        // cout << "<final cost>" << endl;
        // Cost = αA+ (1-α)W
        cout << fixed << setprecision(2) << _area*_alpha + _wl*(1-_alpha) << endl;
        // cout << endl;

        // cout << "<total wirelength>" <<  endl;
        // W = sum(HPWL)
        cout << fixed << setprecision(1) << getWL() << endl;
        // cout << endl;

        // cout << "<chip_area>" << endl;
        // area = (chip_width) * (chip_height)
        cout << fixed << setprecision(0) << getArea() << endl;
        // cout << endl;

        // cout << "<chip_width> <chip_height>" << endl;
        cout << _ctrx << "  " << _ctry << endl;
        // cout << endl;

        // cout << "<white space ratio>" << endl;
        // cout << fixed << setprecision(2) << 1-_usedArea/_area << endl;
        // cout << endl;

        // cout << "<program_runtime>" << endl;
        // cout << fixed << setprecision(6) << ((double)(clock() - start))/CLOCKS_PER_SEC << endl;
        cout << fixed << setprecision(6) << ((double)(clock() - start))/CLOCKS_PER_SEC << endl;
        // cout << endl;

        for(const auto& blk : _blkMap){
            cout << fixed << setprecision(0) << left << setw(6) << blk.first ;
            cout << right << setw(6)  << blk.second->getX1();
            cout << setw(6)  << blk.second->getY1();
            cout << setw(6)  << blk.second->getX2();
            cout << setw(6)  << blk.second->getY2();
            cout << endl;
        }

        cout.rdbuf(coutbuf);
        outFile.close();
    }
    
    return;
}

void Floorplanner::printResult(clock_t start){
    
    cout << "<final cost>" << endl;
    // Cost = αA+ (1-α)W
    cout << fixed << setprecision(2) << _area*_alpha + _wl*(1-_alpha) << endl;
    cout << endl;

    cout << "<total wirelength>" <<  endl;
    // W = sum(HPWL)
    cout << fixed << setprecision(1) << getWL() << endl;
    cout << endl;

    cout << "<chip_area>" << endl;
    // area = (chip_width) * (chip_height)
    cout << fixed << setprecision(0) << getArea() << endl;
    cout << endl;

    cout << "<chip_width> <chip_height>" << endl;
    cout << _ctrx << "  " << _ctry << endl;
    cout << endl;

    cout << "<white space ratio>" << endl;
    cout << fixed << setprecision(2) << 1-_usedArea/_area << endl;
    cout << endl;

    cout << "<program_runtime>" << endl;
    cout << fixed << setprecision(6) << ((double)(clock() - start))/CLOCKS_PER_SEC << endl;
    cout << endl;

    for(const auto& blk : _blkMap){
        cout << fixed << setprecision(0) << left << setw(6) << blk.first ;
        cout << right << setw(6)  << blk.second->getX1();
        cout << setw(6)  << blk.second->getY1();
        cout << setw(6)  << blk.second->getX2();
        cout << setw(6)  << blk.second->getY2();
        cout << endl;
    }  
}

void Floorplanner::plotSVG(string* filename){
    double dividor=1;
    if(_widthConstraint >10000){
        dividor = 20;
    }else{
        if(_widthConstraint>5000){
            dividor = 20;
        }else{
            if(_widthConstraint>1000){
                dividor = 5;
            }
        }
    }
    ofstream file(*filename);
    
    if(!file.is_open()){
        cerr << "SVG printing error" << endl;
    }else{
        streambuf *coutbuf = cout.rdbuf();
        streambuf *filebuf = file.rdbuf();
        cout.rdbuf(filebuf);

        cout << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
        cout << "<svg xmlns=\"http://www.w3.org/2000/svg\">" << endl;


        cout << "<rect " 
        << "x=\"" << 0 << "\" "
        << "y=\"" << 0 << "\" "
        << "width=\"" << _widthConstraint/dividor << "\" "
        << "height=\"" << _heightConstraint/dividor << "\" "
        << "stroke=\"red\" "
        << "stroke-width=\"3\" "
        << "fill-opacity=\"0\"/>"
        << endl;


        for(const auto& item : _blkMap){
            cout << "<rect " 
            << "x=\"" << item.second->getX1()/dividor << "\" "
            << "y=\"" << (_heightConstraint-item.second->getY2())/dividor << "\" "
            // << "y=\"" << item.second->getY1()/dividor << "\" "
            << "width=\"" << item.second->getWidth()/dividor << "\" "
            << "height=\"" << item.second->getHeight()/dividor << "\" "
            << "stroke=\"blue\" "
            << "stroke-width=\"2\" "
            << "fill-opacity=\"0.3\"/>"
            << endl;

            // cout << "<text " 
            // << "x=\"" << item.second->getPin().first/dividor << "\" "
            // // << "y=\"" << (_heightConstraint-item.second->getY2())/dividor << "\" "
            // << "y=\"" << item.second->getPin().second/dividor << "\" "
            // << "> " << item.second->getName() << " </text>"
            // << endl;
        }

        // Contour* contour = _ctrHead[0]->getNext();
        // while(contour!=NULL){
        //     Contour* prev = contour->getPrev();
        //     cout << "<line " 
        //     << "x1=\"" << prev->getxCor()/dividor << "\" "
        //     << "y1=\"" << prev->getyCor()/dividor << "\" "
        //     // << "y1=\"" << (_heightConstraint-prev->getyCor())/dividor << "\" "
        //     << "x2=\"" << contour->getxCor()/dividor << "\" "
        //     // << "y2=\"" << (_heightConstraint-contour->getyCor())/dividor << "\" "
        //     << "y2=\"" << contour->getyCor()/dividor << "\" "
        //     << "stroke=\"black\" "
        //     << "stroke-width=\"5\"/> "
        //     << endl;

        //     if(contour->getNext()==NULL){
        //         break;
        //     }
        //     contour = contour->getNext();
        // }

        // // outline
        // cout << "<line " 
        // << "x1=\"" << 0 << "\" "
        // << "y1=\"" << 0 << "\" "
        // // << "y1=\"" << (_heightConstraint-prev->getyCor())/dividor << "\" "
        // << "x2=\"" << _ctrx/dividor << "\" "
        // // << "y2=\"" << (_heightConstraint-contour->getyCor())/dividor << "\" "
        // << "y2=\"" << _ctry/dividor << "\" "
        // << "stroke=\"green\" "
        // << "stroke-width=\"2\"/> "
        // << endl;

        cout << "</svg>" << endl;

        std::cout.rdbuf(coutbuf);
        file.close();
    }
}

