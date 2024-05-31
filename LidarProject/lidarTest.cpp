/*
#include "Lidar.hpp"
int main() {
    Sensor::toolBox tb;
    Sensor::ipXT32Lidar lidar(2370, tb);
    Sensor::CANSmartMicroRadar radar(tb);
    std::vector<cv::Point3f> dataAll, ignoredAll;
    std::vector<Sensor::radarPoint> radarData;
    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::viz::Viz3d window;
        cv::Affine3d viewerPose = window.getViewerPose();
        viewerPose.translation() = cv::Vec3d(0, 0, 0);
        window.setViewerPose(viewerPose);
    
    while (true)
    {
        dataAll.clear();
        ignoredAll.clear();

        std::thread t1([&](){dataAll = lidar.getFrame(); ignoredAll = lidar.getIgnoredFrame();});
        std::thread t2([&](){ radarData = radar.getFrame();});
        t1.join();
        
        
        Sensor::DBSCAN3D dbscan(dataAll,tb);
        dbscan.run();
        auto objects = dbscan.getObjects();

        int i = 0;
        for (auto& object : objects)
        {
            if (object.points.empty()) continue;
            cv::viz::WCloud cloudWidget(object.points, std::vector<cv::Vec3b>(object.points.size(),dbscan.getColor(cv::norm(object.centroid))));
                cloudWidget.setRenderingProperty(cv::viz::POINT_SIZE, 5.0);
                cloudWidget.setRenderingProperty(cv::viz::OPACITY, 1);
            
            
            //get the minimum and maximum points of the object
             cv::Point3f minPoint= cv::Point3f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
             cv::Point3f maxPoint= cv::Point3f(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
            for (auto& point : object.points) {
                minPoint.x = std::min(minPoint.x, point.x);
                minPoint.y = std::min(minPoint.y, point.y);
                minPoint.z = std::min(minPoint.z, point.z);

                maxPoint.x = std::max(maxPoint.x, point.x);
                maxPoint.y = std::max(maxPoint.y, point.y);
                maxPoint.z = std::max(maxPoint.z, point.z);
            }
            cv::viz::WCube cube(minPoint, maxPoint, true, cv::viz::Color::white());
             
            cv::viz::WText3D OBJdistance(std::to_string(int(cv::norm(object.centroid))), maxPoint ,10);
            window.showWidget("cloud" + std::to_string(i), cloudWidget);
            window.showWidget("dis"+std::to_string(i), OBJdistance);
            //window.showWidget("cube"+std::to_string(i), cube);
            i++;
                
        }
        
        t2.join();
        for (auto& point : radarData)
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
            auto newPoint = Sensor::rotatePoint({static_cast<float>(X3d), static_cast<float>(Y3d), static_cast<float>(Z3d)}, 96.0, 'z'); X3d = newPoint.x; Y3d = newPoint.y; Z3d = newPoint.z;

            
            window.showWidget(std::to_string(i), cv::viz::WSphere(cv::Point3d(X3d,Y3d,Z3d), 0.2));
            window.showWidget("strng"+std::to_string(i++),cv::viz::WText3D(std::to_string((int)round(point.R)), cv::Point3d(X3d,Y3d,Z3d+0.3),0.2));
        }
            
            
        
        window.showWidget("bcube", cv::viz::WCube(cv::Point3f(tb.box.NXlim, tb.box.NYlim, tb.box.NZlim), cv::Point3f(tb.box.PXlim, tb.box.PYlim, tb.box.PZlim), true, cv::viz::Color::white()));

        
        window.showWidget("coordinate", cv::viz::WCoordinateSystem(100));

        if (tb.view.showIgnored && ignoredAll.size() > 0)
        {
            cv::viz::WCloud ignoredCloudWidget(ignoredAll, cv::viz::Color::white());
            ignoredCloudWidget.setRenderingProperty(cv::viz::POINT_SIZE, 2.0);
            ignoredCloudWidget.setRenderingProperty(cv::viz::OPACITY, 1);
            window.showWidget("ignoredCloud", ignoredCloudWidget);
        }

        cv::viz::WText textWidget("eps: " + std::to_string(tb.cluster.eps) + " minPts: " + std::to_string(tb.cluster.minPts), cv::Point(10, 50), 20, cv::viz::Color::white());
        window.showWidget("text", textWidget);
        
        

        window.spinOnce(1, true);
        auto val = cv::waitKey(100);
        if (val == '<') tb.cluster.minPts -= 1;
        else if (val == '>')tb.cluster.minPts  += 1;
        else if (val == '+')tb.cluster.eps += 1;
        else if (val == '-')tb.cluster.eps -= 1;
        else if (val == 'q') break;
        window.removeAllWidgets();


    }
    
    return 0;
}

*/
