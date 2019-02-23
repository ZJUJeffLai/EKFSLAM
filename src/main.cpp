
#include <iomanip>

#include "../include/common.h"
#include "../include/mapper.h"
#include "../include/plotter.h"
#include "../include/sensor_info.h"
#include "ekfslam.h"
#include <ctime>

double calc_time(clock_t start, clock_t end)
{
    return double(end - start) / CLOCKS_PER_SEC;
}

int main(int arc, char* argv[])
{
    if (arc < 3 || arc > 4) {
        std::cerr << "Usage: ./main.x map_file sensor_file [true/false]\n";
        std::cerr << "The last argument will time the program if set to true\n";
        exit(1);
    }
    string map_name = argv[1];
    string sensor_name = argv[2];
    string time;
    if (arc == 4)
        time = argv[3];
    else
        time = "false";
    bool time_prog = false;
    if (time == "true")
        time_prog = true;

    // Read all landmarks for the map
    Mapper mapper;
    mapper.initialize(map_name);

    // Read the measurements(odometry and radar)
    MeasurementPackage measurements;
    measurements.initialize(sensor_name);
    std::cout << measurements.data.size() << std::endl;

    // Input: landmark_size, the other two inputs have been set as certain numbers
    EKFSLAM ekfslam(mapper.data.size());

    // Matplotlib window class
    Draw draw;
    draw.Show();

    clock_t start;

    for (unsigned int i = 0; i< measurements.data.size(); i++) {
        const auto& record = measurements.data[i];
        start = clock();
        // Reset the frame
        draw.Clear();
        double time_clear = calc_time(start, clock());

        start = clock();
        // Step the kalman filter
        ekfslam.ProcessMeasurement(record);
        double time_pro = calc_time(start, clock());

        start = clock();
        // Draw current state
        draw.Plot_State(ekfslam.getMu(), ekfslam.getSigma(),
                        mapper, ekfslam.getObservedLandmaks(), record.scans);
        double time_plt = calc_time(start, clock());

        // Wait a little bit
        start = clock();
        draw.Pause();
        double time_pause = calc_time(start, clock());

        if (time_prog) {
            std::cout << "Time clear:\t" << time_clear << std::endl;
            std::cout << "Time process\t: " << time_pro << std::endl;
            std::cout << "Time plot:\t" << time_plt << std::endl;
            std::cout << "Time pause:\t" << time_pause << "\n\n";
        }
    }
    // Save the frame to picture
    draw.Save("last_frame.png");

    return 0;
}
