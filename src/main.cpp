#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <chrono>
#include "floorplanner.h"
using namespace std;


int main(int argc, char** argv)
{
    clock_t tStart = clock();
    fstream input_blk, input_net;
    ofstream output;
    double alpha;

    if (argc == 5) {
        alpha = stod(argv[1]);
        input_blk.open(argv[2], ios::in);
        input_net.open(argv[3], ios::in);
        output.open(argv[4], ios::out);
        if (!input_blk) {
            cerr << "Cannot open the input file \"" << argv[2]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!input_net) {
            cerr << "Cannot open the input file \"" << argv[3]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!output) {
            cerr << "Cannot open the output file \"" << argv[4]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
    }
    else {
        cerr << "Usage: ./Floorplanner <alpha> <input block file> " <<
                "<input net file> <output file>" << endl;
        exit(1);
    }

    Floorplanner* fp = new Floorplanner(input_blk, input_net, alpha);
    // // fp->buildTree();
    fp->floorplan();

    fp->writeResult(output, tStart);
    fp->printResult(tStart);

    string SVG = "out.svg" ;
    fp->plotSVG(&SVG);

    return 0;
}
