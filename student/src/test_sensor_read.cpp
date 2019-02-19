#include <iostream>
#include "../include/sensor_info.h"

void print_odo_reading(OdoReading* odo) 
{
    std::cout << "ODO: " << odo->r1 << "\t" << odo->t << "\t" << odo->r2 << std::endl;
}

void print_laser_reading(LaserReading* lr) 
{
    std::cout << "LSR: " << lr->id << "\t" << lr->range << "\t" << lr->bearing << std::endl;
}

void print_record(Record* record)
{
    std::cout << "RECORD:\n\t";
    print_odo_reading(&record->odo);
    for (LaserReading lr : record->scans)
        print_laser_reading(&lr);
}

int main()
{

    std::cout << "Hello world" << std::endl;

    string file_name = "../data/sensor.dat";
    MeasurementPackage* mp = new MeasurementPackage;
    mp->initialize(file_name);
    
    // Print records
    for (Record rec : mp->data) {
        print_record(&rec);
        std::cout << std::endl;
    }


    return 0;
}