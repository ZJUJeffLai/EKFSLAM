
#include <iomanip>

#include "../include/common.h"
#include "../include/mapper.h"
#include "../include/plotter.h"
#include "../include/sensor_info.h"
#include "ekfslam.h"
#include <ctime>

int main(int arc, char* argv[])
{
    if (arc != 3) {
        std::cout << "Usage: ./main.x map_file sensor_file\n";
        exit(1);
    }
    string map_name = argv[1];
    string sensor_name = argv[2];

    // read all landmarks for the map
    // class Mapper from mapper.h
    Mapper mapper;
    mapper.initialize(map_name);

    // read the measurements(odometry and radar)
    // class MeasurementPackage from sensor_info.h
    MeasurementPackage measurements;
    measurements.initialize(sensor_name);
    cout << measurements.data.size() << endl;

    // class Draw from plotter.h
    Draw draw;

    // class EKFSLAM form ekfslam.h
    // input: landmark_size, the other two inputs have been set as certain numbers
    EKFSLAM ekfslam(mapper.data.size());

    for (unsigned int i = 0; i< measurements.data.size(); i++)
    {
        const auto& record = measurements.data[i];

        //try { draw.Clear(); }
        //catch (...) {std::cout << "X11 forwarding not setup, window doesn't exist\n";}
        draw.Clear();

        clock_t start = clock();
        ekfslam.ProcessMeasurement(record);
        clock_t end = clock();
        double time_pro = double(end - start) / CLOCKS_PER_SEC;


        //Use function Plot_State
        //Plot_State(const VectorXd& mu, const MatrixXd& sigma,
        //const Mapper& mapper, const vector<bool>&observedLandmarks,
        //const vector<LaserReading>& Z)

        start = clock();
        draw.Plot_State(ekfslam.getMu(), ekfslam.getSigma(),
                        mapper, ekfslam.getObservedLandmaks(), record.scans);
        end = clock();
        double time_plt = double(end - start) / CLOCKS_PER_SEC;

        std::cout << "Time from process: " << time_pro << std::endl;
        std::cout << "Time from plot: " << time_plt << std::endl;

        //draw.Save("test.png");

        //try { draw.Pause(); }
        //catch (...) {std::cout << "X11 forwarding not setup, window doesn't exist\n";}
        draw.Pause();
    }
    draw.Show();
    return 0;
}
