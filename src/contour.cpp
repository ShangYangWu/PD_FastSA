#include "floorplanner.h"
#include <iostream>
using namespace std;

void Floorplanner::updateContour(const double xCor1, const double yCor1, const double xCor2, const double yCor2){
    Contour* first = new Contour(xCor1, yCor2);
    Contour* second = new Contour(xCor2, yCor2);
    Contour* third = new Contour(xCor2, yCor1);
    first->setNext(second);
    second->setNext(third);
    third->setPrev(second);
    second->setPrev(first);

    Contour* current = _ctrHead[0];
    double y = 0;
    double y_last = 0;
    const double h = yCor2 - yCor1;

// while(current!=NULL){
//     cout << "(" <<current->getxCor() << "," << current->getyCor() << ") ";
//     if(current->getNext()!=NULL){
//         current = current->getNext();
//     }else{
//         break;
//     }
// }
// current = _ctrHead[0];
// cout << "tar: " << xCor1 << " " << yCor1 << " " << xCor2 << " " << yCor2 <<" ";

    // find y_last to update third
    while(current->getxCor()<xCor2){
        y_last = current->getyCor();
        if(current->getNext()==NULL){
            break;
        }
        current = current->getNext();

    }
    if(current->getNext() != NULL){
        if(current->getNext()->getxCor() == xCor1){
            current = current->getNext();
            y_last = current->getyCor();
        }
    }
    current = _ctrHead[0];

    // find the node closest to xCor1
    while(current->getxCor()<xCor1){
        if(current->getNext()==NULL){
            break;
        }
        current = current->getNext();

    }
    if(current->getNext() != NULL){
        if(current->getNext()->getxCor() == xCor1){
            current = current->getNext();

        }
    }

    // current.x == xCor1
    if(current->getxCor() == xCor1){
        // delete the reduncdant insert new nodes
        Contour* temp = current->getNext();
        Contour* PrevCtr = current->getPrev();

        // check if current should be removed
        bool deleteCurrent = 0;
        // condition 1: current overlaps first
        if(current->getyCor() == yCor2){
            deleteCurrent = 1;
        }
        // condition 2: current is reduncdant
        if(PrevCtr!=NULL && PrevCtr->getxCor() == xCor1){
            deleteCurrent = 1;
        }

        // check if PrevCtr overlaps first
        if(PrevCtr!=NULL && PrevCtr->getyCor() == yCor2){
            Contour* remove = PrevCtr;
            PrevCtr = PrevCtr->getPrev();
            current->setPrev(PrevCtr);
            PrevCtr->setNext(current);
            delete remove;
        }

        // condition 1: temp == NULL
        if(temp==NULL){
            if(deleteCurrent){
                if(PrevCtr!=NULL){        
                    PrevCtr->setNext(first);
                    first->setPrev(PrevCtr);
                }
                else{
                    _ctrHead[0] = first;
                }
                delete current;
            }else{
                current->setNext(first);
                first->setPrev(current);
                third->setState(xCor1, y_last);
            }
            return;
        }
        // condition 2: temp is not reduncdant
        else{
            if(temp->getxCor() > xCor2){
                // insert new nodes
                if(deleteCurrent){
                    if(PrevCtr!=NULL){        
                        PrevCtr->setNext(first);
                        first->setPrev(PrevCtr);
                    }
                    else{
                        _ctrHead[0] = first;
                    }
                    delete current;
                }else{
                    current->setNext(first);
                    first->setPrev(current);
                }

                third->setNext(temp);
                temp->setPrev(third);
                return;
            }
        // condition 3: temp is reduncdant
            else{
                y = temp->getyCor();
                while(temp->getxCor()<xCor2){
                    if(temp->getyCor()>y){
                        y = temp->getyCor();
                    }        
                    if(temp->getNext() == NULL){
                        break;
                    }
                    temp = temp->getNext();
                }

                // condition 1: temp ---> NULL -x-> xCor2 (temp didn't arrive xCor2)
                if(temp->getNext() == NULL){
                    // reset y for the new nodes
                    first->setState(xCor1, y+h);
                    second->setState(xCor2, y+h);
                    third->setState(xCor2, y_last);

                    // insert the new nodes
                    if(deleteCurrent){
                        if(PrevCtr!=NULL){        
                            PrevCtr->setNext(first);
                            first->setPrev(PrevCtr);
                        }
                        else{
                            _ctrHead[0] = first;
                        }

                        // reset temp for elimination before current deleted
                        temp = current->getNext();
                        delete current;
                    }
                    else{
                        // reset temp for elimination
                        temp = current->getNext();

                        current->setNext(first);
                        first->setPrev(current);
                    }

                    // use temp for elimination
                    while(1){
                        Contour* next = temp->getNext();
                        if(next == NULL){
                            delete temp;
                            break;
                        }
                        else{
                            delete temp;
                            temp = next;
                        }
                    }
                    return;
                }
                // condition 2: temp ---xCor2---> temp (temp arrived xCor2)
                else{
                    // condition 1: temp ----> Contour.x = xCor2 (temp is on xCor2)
                    if(temp->getxCor() == xCor2){
                        // reset y for the new nodes
                        first->setState(xCor1, y+h);
                        second->setState(xCor2, y+h);
                        third->setState(xCor2, y_last);

                        // set remove for elimination before current deleted
                        Contour* remove = current->getNext();

                        // insert the new nodes
                        if(deleteCurrent){
                            if(PrevCtr!=NULL){        
                                PrevCtr->setNext(first);
                                first->setPrev(PrevCtr);
                            }
                            else{
                                _ctrHead[0] = first;
                            }
                            delete current;
                        }
                        else{
                            current->setNext(first);
                            first->setPrev(current);
                        }

                        // check if temp is at or not at a cliff
                        bool cliff = 0;
                        if(temp->getNext()!=NULL){
                            if(temp->getNext()->getxCor() == xCor2){
                                cliff = 1;
                            }
                        }

                        // condition 1: temp is not at a cliff
                        if(!cliff){
                            if(temp->getNext()==NULL){
                                y = temp->getyCor();
                                third->setState(xCor2, y);
                            }
                            else{
                                third->setNext(temp->getNext());
                                temp->getNext()->setPrev(third);
                                temp->setNext(NULL);
                            }

                            // use remove for elimination
                            while(1){
                                Contour* next = remove->getNext();
                                if(next == NULL){
                                    delete remove;
                                    break;
                                }
                                else{
                                    delete remove;
                                    remove = next;
                                }
                            } 
                            return; 
                        }
                        // condition 2: temp is at a cliff
                        else{
                            // second is reduncdant
                            if(temp->getNext()->getyCor()==second->getyCor()){
                                first->setNext(temp->getNext());
                                temp->getNext()->setPrev(first);
                                delete second;
                                delete third;
                                temp->setNext(NULL);
                            }
                            // only third is reduncdant
                            else{
                                second->setNext(temp->getNext());
                                temp->getNext()->setPrev(second);
                                delete third;
                                temp->setNext(NULL);
                            }
                            // use remove for elimination
                            while(1){
                                Contour* next = remove->getNext();
                                if(next == NULL){
                                    delete remove;
                                    break;
                                }
                                else{
                                    delete remove;
                                    remove = next;
                                }
                            } 
                            return; 
                        }                     
                    }
                    // condition 2: temp ---- xCor2 ---> temp (temp exceeded xCor2)
                    else{
                        // reset y for the new nodes
                        first->setState(xCor1, y+h);
                        second->setState(xCor2, y+h);
                        third->setState(xCor2, y_last);

                        // set remove for elimination before current deleted
                        Contour* remove = current->getNext();

                        // insert the new nodes
                        if(deleteCurrent){
                            if(PrevCtr!=NULL){        
                                PrevCtr->setNext(first);
                                first->setPrev(PrevCtr);
                            }
                            else{
                                _ctrHead[0] = first;
                            }
                            delete current;
                        }
                        else{
                            current->setNext(first);
                            first->setPrev(current);
                        }
                        temp->getPrev()->setNext(NULL);
                        third->setNext(temp);
                        temp->setPrev(third);

                        // use temp for elimination
                        while(1){
                            Contour* next = remove->getNext();
                            if(next == NULL){
                                delete remove;
                                break;
                            }
                            else{
                                delete remove;
                                remove = next;
                            }
                        }  
                        return;                         
                    }
                }
            }
        }
    }
    // current.x < xCor1
    else{
        // delete the reduncdant insert new nodes
        Contour* temp = current->getNext();

        // condition 1: temp == NULL
        if(temp==NULL){      
            current->setNext(first);
            first->setPrev(current);
            third->setState(xCor1, y_last);
            return;
        }
        // condition 2: temp is not reduncdant
        else{
            if(temp->getxCor() > xCor2){
                current->setNext(first);
                first->setPrev(current);

                third->setNext(temp);
                temp->setPrev(third);

                return;
            }
        // condition 3: temp is reduncdant
            else{
                y = temp->getyCor();
                while(temp->getxCor()<xCor2){
                    if(temp->getyCor()>y){
                        y = temp->getyCor();
                    }
        
                    if(temp->getNext() == NULL){
                        break;
                    }
                    temp = temp->getNext();
                }
                // condition 1: temp ---> NULL -X-> xCor2 (temp didn't arrive xCor2)
                if(temp->getNext() == NULL){
                    // reset y for the new nodes
                    first->setState(xCor1, y+h);
                    second->setState(xCor2, y+h);
                    third->setState(xCor2, y_last);

                    // insert the new nodes
                    current->setNext(first);
                    first->setPrev(current);

                    // reset temp for elimination
                    temp = current->getNext();

                    // use temp for elimination
                    while(1){
                        Contour* next = temp->getNext();
                        if(next == NULL){
                            delete temp;
                            break;
                        }
                        else{
                            delete temp;
                            temp = next;
                        }
                    }
                    return;
                }
                // condition 2: temp ---xCor2---> temp (temp arrived xCor2)
                else{
                    // condition 1: temp ----> Contour.x = xCor2 (temp is on xCor2)
                    if(temp->getxCor() == xCor2){
                        // reset y for the new nodes
                        first->setState(xCor1, y+h);
                        second->setState(xCor2, y+h);
                        third->setState(xCor2, y_last);

                        // set remove for elimination before current deleted
                        Contour* remove = current->getNext();

                        // insert the new nodes
                        current->setNext(first);
                        first->setPrev(current);

                        // check if temp is at or not at a cliff
                        bool cliff = 0;
                        if(temp->getNext()!=NULL){
                            if(temp->getNext()->getxCor() == xCor2){
                                cliff = 1;
                            }
                        }

                        // condition 1: temp is not at a cliff
                        if(!cliff){
                            if(temp->getNext()==NULL){
                                y = temp->getyCor();
                                third->setState(xCor2, y);
                            }
                            else{
                                third->setNext(temp->getNext());
                                temp->getNext()->setPrev(third);
                                temp->setNext(NULL);
                            }

                            // use remove for elimination
                            while(1){
                                Contour* next = remove->getNext();
                                if(next == NULL){
                                    delete remove;
                                    break;
                                }
                                else{
                                    delete remove;
                                    remove = next;
                                }
                            } 
                            return; 
                        }
                        // condition 2: temp is at a cliff
                        else{
                            // second is reduncdant
                            if(temp->getNext()->getyCor()==second->getyCor()){
                                first->setNext(temp->getNext());
                                temp->getNext()->setPrev(first);
                                delete second;
                                delete third;
                                temp->setNext(NULL);
                            }
                            // only third is reduncdant
                            else{
                                second->setNext(temp->getNext());
                                temp->getNext()->setPrev(second);
                                delete third;
                                temp->setNext(NULL);
                            }
                            // use remove for elimination
                            while(1){
                                Contour* next = remove->getNext();
                                if(next == NULL){
                                    delete remove;
                                    break;
                                }
                                else{
                                    delete remove;
                                    remove = next;
                                }
                            } 
                            return; 
                        }                     
                    }
                    // condition 2: temp ---- xCor2 ---> temp (temp exceeded xCor2)
                    else{
                        // set remove for elimination
                        Contour* remove = current->getNext();

                        // reset y for the new nodes
                        first->setState(xCor1, y+h);
                        second->setState(xCor2, y+h);
                        third->setState(xCor2, y_last);
                        
                        // insert the new nodes
                        current->setNext(first);
                        first->setPrev(current);
                        temp->getPrev()->setNext(NULL);
                        third->setNext(temp);
                        temp->setPrev(third);

                        // use temp for elimination
                        while(1){
                            Contour* next = remove->getNext();
                            if(next == NULL){
                                delete remove;
                                break;
                            }
                            else{
                                delete remove;
                                remove = next;
                            }
                        }  
                        return;                         
                    }
                }
            }
        }
    }
}


double Floorplanner::contourY(const double x_start, const double x_end){
    double max_yCor = 0;
    Contour* contour = _ctrHead[0];
    while (contour->getxCor() < x_start) {
        if(contour->getNext() == NULL){
            break;
        }
        contour = contour->getNext();
    }
    if(contour->getNext() != NULL){
        if(contour->getNext()->getxCor() == x_start){
            contour = contour->getNext();
        }
    }
    max_yCor = contour->getyCor();

    if(contour->getNext()!=NULL){
        while (contour->getxCor() < x_end) {
            if(contour->getyCor() > max_yCor){
                max_yCor = contour->getyCor();
            }

            if(contour->getNext() == NULL){
                break;
            }
            contour = contour->getNext();
        }
    }
    return max_yCor;
}

void Floorplanner::updateRad(){
    Contour* contour = _ctrHead[0];
    _ctrx = 0;
    _ctry = 0;

    while (contour!=NULL) {
        _ctrx = max(_ctrx, contour->getxCor());
        _ctry = max(_ctry, contour->getyCor());

        if(contour->getNext() == NULL){
            break;
        }
        contour = contour->getNext();
    }
    return;
}

void Floorplanner::initContour(Block* root){
    clearContour();
    _ctrHead[0] = new Contour(0, root->getHeight(0));
    _ctrHead[0]->setNext(new Contour(root->getWidth(0), root->getHeight(0)));
    _ctrHead[0]->getNext()->setPrev(_ctrHead[0]);
    _ctrHead[0]->getNext()->setNext(new Contour(root->getWidth(0), 0));
    _ctrHead[0]->getNext()->getNext()->setPrev(_ctrHead[0]->getNext());
}

void Floorplanner::clearContour(){
    Contour* current = _ctrHead[0];
    if(_ctrHead[0] != NULL){
        while(current != NULL){
            Contour* next = current->getNext();
            if(next == NULL){
                delete current;
                break;
            }
            delete current;
            current = next;
        }
    }

    return;
}