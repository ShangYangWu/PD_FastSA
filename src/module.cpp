#include "module.h"
#include <iostream>
#include <climits>

using namespace std;

double Net::calcHPWL(){
    double x1=INT_MAX, x2=INT_MIN, y1=INT_MAX, y2=INT_MIN;
    for(const auto &item : this->getTermList()){
        // cout << item->getName() << endl;
        double xmid = item->getPin().first;
        double ymid = item->getPin().second;
        

        //decide x1, x2, y1, y2
        // x1 = min(x)
        if(x1 > xmid){
             x1 = xmid;
        }
        // x2 = max(x)
        if(x2 < xmid){
            x2 = xmid;
        }
        // y1 = min(y)
        if(y1 > ymid){
             y1 = ymid;
        }
        // y2 = max(y)
        if(y2 < ymid){
             y2 = ymid;
        }
    }

    return (x2-x1)+(y2-y1);
}

