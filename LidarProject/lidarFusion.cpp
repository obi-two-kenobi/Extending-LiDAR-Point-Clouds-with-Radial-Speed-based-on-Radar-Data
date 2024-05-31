
#include "Lidar.hpp"



int main() {
    Sensor::toolBox tb;
    Sensor::pcapXT32Lidar lidar("/Users/alis/Documents/Codes/RawFriday19April/xt32pcap.pcap", tb);
    Sensor::OfflineSmartMicroRadar radar(tb, "/Users/alis/Documents/Codes/RawFriday19April/19AprilFriday.csv");
    auto degree =159;
    auto seconddegree=-3;
    std::vector<cv::Point3f> lidarPC;
    std::vector<Sensor::radarPoint> radarPC;
    std::vector<Sensor::combinedObject> combinedObjects;
    
    
    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::viz::Viz3d window("DBSCAN Lidar Objects with IoU and Radar Intersection");
        cv::Affine3d viewerPose = window.getViewerPose();
        viewerPose.translation() = cv::Vec3d(0, 0, 0);
        window.setViewerPose(viewerPose);
    

    
    while (true)
    {
        lidarPC.clear();
        radarPC.clear();
        combinedObjects.clear();

        std::thread t1([&](){lidarPC = lidar.getFrame();});
        std::thread t2([&](){radarPC = radar.getFrame();});
        t1.join();
        
        //auto start = std::chrono::high_resolution_clock::now();
        Sensor::DBSCAN3D dbscan(lidarPC,tb);
        dbscan.run();
        std::vector<Sensor::DBSCANOBJ> objects = dbscan.getObjects();
        
        int i = 0;
        for (auto& object : objects)
        {
            if (object.points.empty()) continue;
            cv::viz::WCloud cloudWidget(object.points, std::vector<cv::Vec3b>(object.points.size(),dbscan.getColor(cv::norm(object.centroid))));
                cloudWidget.setRenderingProperty(cv::viz::POINT_SIZE, 5.0);
                cloudWidget.setRenderingProperty(cv::viz::OPACITY, 1);

            
            cv::viz::WCube cube(object.minPoint, object.maxPoint, true, cv::viz::Color::white());
            cv::viz::WText3D OBJdistance(std::to_string(int(cv::norm(object.centroid))), object.maxPoint ,20);
            window.showWidget("cloud" + std::to_string(i), cloudWidget);
            window.showWidget("dis"+std::to_string(i), OBJdistance);
            window.showWidget("cube"+std::to_string(i), cube);
            i++;
                
        }
        window.showWidget("cube", cv::viz::WCube(cv::Point3f(tb.box.NXlim, tb.box.NYlim, tb.box.NZlim), cv::Point3f(tb.box.PXlim, tb.box.PYlim, tb.box.PZlim), true, cv::viz::Color::white()));
        window.showWidget("coordinate", cv::viz::WCoordinateSystem(100));
        //cv::viz::WText textWidget("eps: " + std::to_string(tb.cluster.eps) + " minPts: " + std::to_string(tb.cluster.minPts), cv::Point(10, 50), 20, cv::viz::Color::white());
        //window.showWidget("text", textWidget);
        
        
        t2.join();
        //other Code
        Sensor::rotateVector(radarPC, degree , 'z');
        Sensor::rotateVector(radarPC, seconddegree , 'y');
        
        int Rc = 0;
        for (auto& point : radarPC)
        {
            
            auto X3d = point.R*100 * cos(toRad(point.Azimuth)) * cos(toRad(point.Elevation));
            auto Y3d = point.R*100 * sin(toRad(point.Azimuth)) * cos(toRad(point.Elevation));
            auto Z3d = point.R*100 * sin(toRad(point.Elevation));
            Y3d-=250;
            X3d+= 380-100;
            
            auto Radius = 2*(point.R*100)*sin(toRad(0.5)/2)*5;

            for (auto& object : objects) {
                if (X3d + Radius < object.minPoint.x || X3d - Radius > object.maxPoint.x ||
                    Y3d + Radius < object.minPoint.y || Y3d - Radius > object.maxPoint.y ||
                    Z3d + Radius < object.minPoint.z || Z3d - Radius > object.maxPoint.z) {
                    continue; // No intersection in this dimension, check next object
                }
                combinedObjects.push_back(Sensor::combinedObject{point, object});
            }

            window.showWidget(std::to_string(Rc++)+"radar", cv::viz::WSphere(cv::Point3d(X3d,Y3d,Z3d),Radius));
        }
        //auto end = std::chrono::high_resolution_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //dbscan.myfile <<duration<<"\n";

        std::cout << "Es gibt " << combinedObjects.size() << " Objekte" << std::endl;
        
        window.spinOnce(1, true);
        auto val = cv::waitKey(1);
        if (val == '<') seconddegree -= 1;
        else if (val == '>')seconddegree  += 1;
        else if (val == '+')degree += 1;
        else if (val == '-')degree -= 1;
        else if (val == 'q') {/*dbscan.myfile.close();*/break;}
        window.removeAllWidgets();
        
    }
    
    return 0;
}

