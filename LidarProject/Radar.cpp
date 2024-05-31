/*
#include "Lidar.hpp"

int main() {
    cv::viz::Viz3d window("Radar viz");
    cv::namedWindow("Control", cv::WINDOW_NORMAL);

    Sensor::toolBox tb;
    //Sensor::OfflineSmartMicroRadar radar(tb,"/Users/alis/Documents/Codes/Lidar-PlayGround/Raw/RadarLog/newLog.csv");
    Sensor::OfflineSmartMicroRadar radar(tb,"/Users/alis/Documents/Codes/RawFriday19April/19AprilFriday.csv");
    
//  Lidar::CANSmartMicroRadar radar(tb);
    
    int i;
    
    while (true)
    {
        auto myfile = std::ofstream("/Users/alis/Documents/Codes/LidarProject/LidarProject/RadarTimeLog.txt",std::ios_base::app);
        auto data = radar.getFrame();
        i=1;
        std::cout << "Cycle: " << radar.getCycleCount() << " | #Targets: " << radar.getnumberOfTargets()<< " | Cycle Duration: " << radar.getCycleDuration() << "s | Time Stamp: " << radar.getTimeStamp() << "s\n" ;
        myfile << radar.getCycleDuration() << ", " << radar.getnumberOfTargets() << "\n";
        for (auto& point : data)
        {
           
            std::cout << "ID : " << point.ObjID <<"\n"
                      << "|\tR  : " <<  point.R <<" m\n"
                      << "|\tRR : " << point.RR <<" m/s\n"
                      << "|\tAZT: " <<  point.Azimuth <<" deg\n"
                      << "|\tELV: " << point.Elevation <<" deg\n"
                      << std::endl;
            
            int X3d = point.R * cos(toRad(point.Azimuth)) * cos(toRad(point.Elevation));
            int Y3d = point.R * sin(toRad(point.Azimuth)) * cos(toRad(point.Elevation));
            int Z3d = point.R * sin(toRad(point.Elevation));
            
            window.showWidget(std::to_string(i), cv::viz::WSphere(cv::Point3d(X3d,Y3d,Z3d), 0.2));
            window.showWidget("strng"+std::to_string(i++),cv::viz::WText3D(std::to_string((int)round(point.R)), cv::Point3d(X3d,Y3d,Z3d+0.3),0.2));
        }
        
        window.showWidget("gimble", cv::viz::WCoordinateSystem(1));
        window.showWidget("grid", cv::viz::WGrid());
        window.spinOnce(1, true);
        cv::waitKey(1);
        window.removeAllWidgets();
        myfile.close();
    }
    
      
    return 0;
}
*/
