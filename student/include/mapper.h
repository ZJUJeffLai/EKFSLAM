
#pragma once

#include "./common.h"

// Define a LandMark on the map
// Consists of ID, x position, and y position
struct MapPoint{
    uint64_t id;
    float x;
    float y;
};

// Mapper Class
// data: Vector containing MapPoints which represent all landmarks
// initialize: Function to read in landmarks from specified filename
class Mapper {
 public:
    Mapper() {}
    void initialize(const string& filename);
    vector<MapPoint> data;
};

// TODO: Implement initialize
void Mapper::initialize(const string& filename) {
    ifstream in_file(filename, ifstream::in);
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << filename << endl;
        exit(EXIT_FAILURE);
    }

    string line;
    while (getline(in_file, line)) {
        istringstream ss(line);
        MapPoint mp;
        ss >> mp.id;
        ss >> mp.x;
        ss >> mp.y;
        data.push_back(mp);
        if (debug)
        cout << data.back().id << ": " << data.back().x
            << ": " << data.back().y << endl;
    }

    if (in_file.is_open()) {
        in_file.close();
    }
}
