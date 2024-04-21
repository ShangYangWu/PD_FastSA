#include "floorplanner.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <ctime>
using namespace std;

void Floorplanner::perturbRecover(const int move_choice, const int choice1, const int choice2){
    bool isparent = 0;
    // coriginally blk 2
    Block* const blk1 = _blkList[choice1];
    const size_t blk_choice1 = blk1->getIdx();
    bool side1 = 0;
    if(blk_choice1%2 == 0){
        side1 = 1;
    }

    // originally blk 1
    Block* const blk2 = _blkList[choice2];
    const size_t blk_choice2 = _blkList[choice2]->getIdx();
    bool side2 = 0;
    if(blk_choice2 %2 == 0){
        side2 = 1;
    }

    if(blk_choice1%2 == 0){
        side1 = 1;
    }

    switch(move_choice){
        case 0:
            rotate(blk1);
            break;
        case 2:
            if(blk1->getParent() != NULL){
                if(blk1->getParent() == blk2){
                    isparent = 1;
                }
            }
            
            if(blk2->getParent() != NULL){
                if(blk2->getParent() == blk1){
                    isparent = 1;
                }
            } 
            swap(blk_choice1, side1, blk_choice2, side2, isparent);
            break;
        default:
            break;            
    }
}

pair<int, int> Floorplanner::perturb(const int move_choice){
    // choose blk1
    int choice1 = rand() % _blkNum;
    Block* blk1 = _blkList[choice1];
    bool side1 = blk1->getSide();
    size_t blk_choice1 = blk1->getIdx();

    // choose blk2
    int choice2 = rand() % _blkNum;
    while(choice1 == choice2){
        choice2 = rand() % _blkNum;
    }
    Block* blk2 = _blkList[choice2];
    bool side2 = blk2->getSide();
    size_t blk_choice2 = blk2->getIdx();

    bool isparent = 0;

    switch(move_choice){
        case 0:
            // cout << "rotate " << blk1->getIdx() << " " << blk1->getName() << endl;
            rotate(blk1);
            break;
        case 1:
            //  cannot be root
            while(blk1->getParent()==NULL){
                choice1 = rand() % _blkNum;
                blk1 = _blkList[choice1];
            }
            side1 = blk1->getSide();
            blk_choice1 = blk1->getIdx();

            // blk2 cannot be the parent of blk1
            while(choice1==choice2 || blk1->getParent() == blk2){
                choice2 = rand() % _blkNum;
                blk2 = _blkList[choice2];
            }
            side2 = blk2->getSide();
            blk_choice2 = blk2->getIdx();

            // cout << "delete insert [" << blk_choice1 << "] " << blk1->getName() << "   ";
            // cout << " [" << blk_choice2 << "] " << blk2->getName() << endl;
            delIns(blk1, blk2);
            break;
        case 2:
            if(blk1->getParent() != NULL){
                if(blk1->getParent() == blk2){
                    isparent = 1;
                }
            }
            
            if(blk2->getParent() != NULL){
                if(blk2->getParent() == blk1){
                    isparent = 1;
                }
            }

            // cout << "swap [" << blk_choice1 << "] " << blk1->getName();
            // cout << " and [" << blk_choice2 << "] " << blk2->getName() << endl;
            swap(blk_choice1, side1, blk_choice2, side2, isparent);
            break;
        default:
            break;
    }

    pair<int, int> choice(choice1, choice2);
    return choice;
}


void Floorplanner::delIns(Block* const block1,Block* const block2){
    bool side1 = block1->getSide();
    size_t idx1 = block1->getIdx();
    Block* const rchild1 = block1->getRight();
    Block* const lchild1 = block1->getLeft();
    Block* const prev1 = block1->getParent();

    // step1: delete blk1
    // rchild1 == NULL
    if(rchild1 == NULL){
        // lchild1 == NULL
        if(lchild1==NULL){
            _blkVec.erase(idx1);
            if(side1){
                prev1->setRight(NULL);
            }else{
                prev1->setLeft(NULL);
            }
        }
        // lchild1 != NULL
        else{
            eraseVec(block1, idx1);
            lchild1->setSide(side1);
            if(side1){
                prev1->setRight(lchild1);
            }else{
                prev1->setLeft(lchild1);               
            }
            lchild1->setParent(prev1);
            block1->setLeft(NULL);
            reconstuctVec(prev1, prev1->getIdx());   


        }
    }
    // rchild1 != NULL
    else{
        // lchild1 == NULL
        if(lchild1==NULL){
            eraseVec(block1, idx1);           
            rchild1->setSide(side1);
            if(side1){
                prev1->setRight(rchild1);
            }else{
                prev1->setLeft(rchild1);
            }
            rchild1->setParent(prev1);
            block1->setRight(NULL);
            reconstuctVec(prev1, prev1->getIdx());
        }
        // lchild1 != NULL
        else{
            // change until rchild1==NULL or lchild1==NULL or both==NULL
            // notice that if rchild or lchild == block2 should be also valid
            Block* rchild = rchild1;
            Block* lchild = lchild1;
            while(rchild!=NULL && lchild!=NULL){
                bool change = rand()%2;

                // change with right child
                if(change){
                    swap(idx1, side1, (idx1+1)*2, 1, 1);
                    rchild = block1->getRight();
                    lchild = block1->getLeft();
                }
                // change with left child
                else{
                    swap(idx1, side1, idx1*2+1, 0, 1);
                    rchild = block1->getRight();
                    lchild = block1->getLeft();
                } 
                side1 = change;
                idx1=block1->getIdx();            
            }
        
            Block* prev = block1->getParent();
            // both == NULL
            if(rchild==NULL && lchild==NULL){
                _blkVec.erase(idx1);
                if(side1){
                    prev->setRight(NULL);
                }else{
                    prev->setLeft(NULL);
                }              
            }
            else{
                // rchild1!=NULL and lchild1==NULL
                if(rchild!=NULL){
                    eraseVec(block1, idx1);
                    rchild->setSide(side1);
                    if(side1){
                        prev->setRight(rchild);
                    }else{
                        prev->setLeft(rchild);
                    }
                    rchild->setParent(prev);
                    block1->setRight(NULL);
                    reconstuctVec(prev, prev->getIdx());
                }
                // rchild1==NULL and lchild1!=NULL
                else{
                    eraseVec(block1, idx1);
                    lchild->setSide(side1);
                    if(side1){
                        prev->setRight(lchild);
                    }else{
                        prev->setLeft(lchild);               
                    }
                    lchild->setParent(prev);
                    block1->setLeft(NULL);
                    reconstuctVec(prev, prev->getIdx());  

                }

            }
       
        }
    }

    // step2: insert blk1
    Block* const rchild2 = block2->getRight();
    Block* const lchild2 = block2->getLeft();
    const size_t idx2 = block2->getIdx();
    bool insert_side = rand()%2;

    eraseVec(block2, idx2);
    block1->setParent(block2);
    if(insert_side){
        block1->setRight(rchild2);
        if(rchild2!=NULL){
            rchild2->setParent(block1);
        }
        block2->setRight(block1);
        block1->setSide(1);
    }
    else{
        block1->setLeft(lchild2);
        if(lchild2!=NULL){
            lchild2->setParent(block1);
        }
        block2->setLeft(block1);
        block1->setSide(0);
    }
    reconstuctVec(block2, idx2);

    return;
}


void Floorplanner::swap(const size_t block1, const bool side1, const size_t block2, const bool side2, const bool isparent){
    Block* const blk1 = _blkVec[block1];
    Block* const blk2 = _blkVec[block2];

    Block* const rchild1 = blk1->getRight();
    Block* const lchild1 = blk1->getLeft();
    Block* const prev1 = blk1->getParent();

    Block* const rchild2 = blk2->getRight();
    Block* const lchild2 = blk2->getLeft();
    Block* const prev2 = blk2->getParent();

    if(isparent){
        // blk2 is parent
        if(prev1 == blk2){
            // resest blk1 parent
            blk1->setParent(prev2);
            if(prev2!=NULL){
                if(side2){
                    prev2->setRight(blk1);
                }else{
                    prev2->setLeft(blk1);
                }
            }
            // reset blk1 child
            blk2->setParent(blk1);
            // blk1 is right child
            if(side1){
                blk1->setRight(blk2);
                blk1->setLeft(lchild2);
                if(lchild2!=NULL){
                    lchild2->setParent(blk1);
                }
            }
            // blk1 is left child
            else{
                blk1->setLeft(blk2);
                blk1->setRight(rchild2);
                if(rchild2!=NULL){
                    rchild2->setParent(blk1);
                }
            }

            // reset blk2 child
            blk2->setRight(rchild1);
            if(rchild1!=NULL){
                rchild1->setParent(blk2);
            }
            blk2->setLeft(lchild1);
            if(lchild1!=NULL){
                lchild1->setParent(blk2);
            }
        }
        // blk1 is parent
        else{
            // resest blk2 parent
            blk2->setParent(prev1);
                if(prev1!=NULL){
                if(side1){
                    prev1->setRight(blk2);
                }else{
                    prev1->setLeft(blk2);
                }
            }
            // reset blk2 child
            blk1->setParent(blk2);   
            // blk2 is right child
            if(side2){
                blk2->setRight(blk1);
                blk2->setLeft(lchild1);
                if(lchild1!=NULL){
                    lchild1->setParent(blk2);
                }
            }
            // blk2 is left child
            else{
                blk2->setLeft(blk1);
                blk2->setRight(rchild1);
                if(rchild1!=NULL){
                    rchild1->setParent(blk2);
                }
            }

            // reset blk1 child
            blk1->setRight(rchild2);
            if(rchild2!=NULL){
                rchild2->setParent(blk1);
            }
            blk1->setLeft(lchild2);
            if(lchild2!=NULL){
                lchild2->setParent(blk1);
            }
        }
    }
    // not child parent relationship
    else{
        blk1->setRight(rchild2);
        blk1->setLeft(lchild2);
        blk2->setRight(rchild1);
        blk2->setLeft(lchild1);

        blk1->setParent(prev2);
        if(rchild1!=NULL){
            rchild1->setParent(blk2);
        }
        if(lchild1!=NULL){
            lchild1->setParent(blk2);
        }

        blk2->setParent(prev1);
        if(rchild2!=NULL){
            rchild2->setParent(blk1);
        }
        if(lchild2!=NULL){
            lchild2->setParent(blk1); 
        } 

        // blk1 is root
        if(prev1 == NULL){
            // blk2 is rightchild
            if(side2){
                prev2->setRight(blk1);
            }
            // blk2 is leftchild
            else{
                prev2->setLeft(blk1);
            }
        }else{
            // blk2 is root
            if(prev2 == NULL){
                // blk1 is rightchild
                if(side1){
                    prev1->setRight(blk2);
                }
                // blk2 is leftchild
                else{
                    prev1->setLeft(blk2);
                }
            }
            // both not root
            else{
                // side1 == side2
                if(side1 == side2){
                    // exchange parent
                    if(side1 == 1){
                        prev1->setRight(blk2);
                        prev2->setRight(blk1);
                    }else{
                        prev1->setLeft(blk2);
                        prev2->setLeft(blk1);
                    }
                }
                // side1 != side2
                else{
                    // blk1 right child, blk2 left child 
                    if(side1 == 1){
                        prev1->setRight(blk2);
                        prev2->setLeft(blk1);
                    }
                    // blk1 left child, blk2 right child 
                    else{
                        prev1->setLeft(blk2);
                        prev2->setRight(blk1);           
                    }
                }
            }
        }
    }
    blk1->setSide(side2);
    blk1->setIdx(block2);
    blk2->setSide(side1);
    blk2->setIdx(block1);

    // exchange _blkVec index
    _blkVec[block1] = blk2;
    _blkVec[block2] = blk1;

    return;
}