
#include <iomanip>

#include "../include/common.h"
#include "../include/mapper.h"
#include "../include/plotter.h"
#include "../include/sensor_info.h"
#include "ekfslam.h"

int main(int arc, char* argv[])
{
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

    for (int i = 0; i< measurements.data.size(); i++)
    {
        const auto& record = measurements.data[i];
        draw.Clear();
        ekfslam.ProcessMeasurement(record);
        //Use function Plot_State
        //Plot_State(const VectorXd& mu, const MatrixXd& sigma,
        //const Mapper& mapper, const vector<bool>&observedLandmarks,
        //const vector<LaserReading>& Z)
        draw.Plot_State(ekfslam.mu, ekfslam.Sigma, mapper, ekfslam.observedLandmarks, record.scans);
        draw.Pause();
        // May nee to use draw clear between frames
    }
    draw.Show();
    return -1;
}
