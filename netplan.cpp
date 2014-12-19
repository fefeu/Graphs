#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "UndirectedGraph.hpp"

using namespace std;

// Output values.
unsigned int totalCost, totalDist, totalMCost, totalMDist;
vector<string> str;   // Strings read from input file.

// Calculates total cost and distance on the original graph.
void originalTotal() {
  UndirectedGraph g;

  // Adds edges according to the input file.
  for(unsigned long i=0; i<str.size(); i+=4) {
    string from = str[i];
    string to = str[i+1];
    unsigned int cost = stoi(str[i+2]);
    unsigned int length = stoi(str[i+3]);

    g.addEdge(from, to, cost, length);
  }

  totalCost = g.totalEdgeCost();
  totalDist = g.totalDistance();
}

// Calculates total cost and distance on the MST of the original graph.
void minTotal() {
  UndirectedGraph m;

  // Adds edges according to the input file.
  for(unsigned long i=0; i<str.size(); i+=4) {
    string from = str[i];
    string to = str[i+1];
    unsigned int cost = stoi(str[i+2]);
    unsigned int length = stoi(str[i+3]);

    m.addEdge(from, to, cost, length);
  }
  
  m.minSpanningTree();
  totalMCost = m.totalEdgeCost();
  totalMDist = m.totalDistance();
}

/**
 * Entry point into the netplan program.
 *
 * -Reads a file from the filesystem according to the specification for
 *  PA3, creating an UndirectedGraph.
 * -Finds the total cost & ping time of the graph as presented in the input
 *  file.
 * -Determines the minimum cost graph from the original graph.
 * -Finds the total cost & ping time of the minimum cost graph.
 * -Finds the change of cost & ping time from the original graph to the
 *  minimum cost graph.
 * -Prints the results to stdout.
 *
 * Usage:
 *   ./netplan infile
 *
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " infile" << endl;
        return EXIT_FAILURE;
    }
    
    ifstream in(argv[1]);
    if (!in) {
        cerr << "Unable to open file for reading." << endl;
        return EXIT_FAILURE;
    }

    // If the file is empty, print six 0s.
    if(in.peek() == EOF) {
      for(int i=0; i<6; i++) 
      cout << "0" << endl;

      return EXIT_SUCCESS;
    }

    // Read the input file and store the strings in the str vector.
    while(in.peek() != EOF) {
      string from, to, cost, length;
        
      getline(in, from, ' ');
      str.push_back(from);

      getline(in, to, ' ');
      str.push_back(to);

      getline(in, cost, ' ');
      str.push_back(cost);

      getline(in, length);
      str.push_back(length);
    }

    // Use two threads to simultaneously calculate total costs and distances 
    // on the original graph and its MST.
    thread first(originalTotal);
    thread second(minTotal);

    // Wait for the threads to finish running.
    first.join();
    second.join();

    cout << totalCost << endl;
    cout << totalMCost << endl;
    cout << totalCost - totalMCost << endl;
    cout << totalDist << endl; 
    cout << totalMDist << endl;
    cout << totalMDist - totalDist << endl;

    return EXIT_SUCCESS;
}
