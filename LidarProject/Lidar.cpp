#include "Lidar.hpp"


using namespace std;
cv::Point3f Sensor::rotatePoint(const cv::Point3f& point, float degAngle, char axis) {
    cv::Point3f rotatedPoint;
    float angle = toRad(degAngle);
    float s = sin(angle);
    float c = cos(angle);
    if (axis == 'x') {
        rotatedPoint.x = point.x;
        rotatedPoint.y = point.y * c - point.z * s;
        rotatedPoint.z = point.y * s + point.z * c;
    } else if (axis == 'y') {
        rotatedPoint.x = point.x * c + point.z * s;
        rotatedPoint.y = point.y;
        rotatedPoint.z = -point.x * s + point.z * c;
    } else if (axis == 'z') {
        rotatedPoint.x = point.x * c - point.y * s;
        rotatedPoint.y = point.x * s + point.y * c;
        rotatedPoint.z = point.z;
    }
    return rotatedPoint;
}

void Sensor::rotateVector(std::vector<cv::Point3f>& vec, float degAngle, char axis ){
    for (auto& point : vec){
        point = Sensor::rotatePoint(point, degAngle,axis);
    }
    return;
}
void Sensor::rotateVector(std::vector<Sensor::radarPoint>& vec, float degAngle, char axis ){
    for (auto& point : vec){
        if (axis == 'x') {
            continue; //unimplemented
        } else if (axis == 'y') {
            point.Elevation+=degAngle;
        } else if (axis == 'z') {
            point.Azimuth+=degAngle;
        }
    }
    return;
}


//BDSCAN3D CLASS------------------------------------------------------------
Sensor::DBSCAN3D::DBSCAN3D (std::vector<cv::Point3f>& data, Sensor::toolBox& tb)
    : eps(tb.cluster.eps), minPts(tb.cluster.minPts), data(data), labels(data.size(), 0) {
        //myfile = std::ofstream("/Users/alis/Documents/Codes/LidarProject/LidarProject/TimeLog.txt",std::ios_base::app);
        //if (!myfile.is_open()) { std::cout << "Failed to open file!" << std::endl; exit(1);}
        
    }

void Sensor::DBSCAN3D:: run() {
   
    //auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < data.size(); ++i) {
        if (labels[i] != 0) continue;  // Already visited

        vector<int> neighbors = rangeQuery(i);

        if (neighbors.size() < minPts) {
            labels[i] = -1;  // Mark as noise
        } else {
            ++clusterId;
            expandCluster(i, clusterId,neighbors);
        }
    }
    //auto end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    //myfile <<duration<<",";
}

const vector<int>& Sensor::DBSCAN3D::getLabels() const {
    return labels;
}

double Sensor::DBSCAN3D::distance(const cv::Point3f& p1, const cv::Point3f& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

vector<int> Sensor::DBSCAN3D::rangeQuery(int dataIndex) {
    vector<int> neighbors;

    for (int i = 0; i < data.size(); ++i) {
        if (labels[i] != 0) continue;  // Already visited
        if (distance(data[dataIndex], data[i]) < eps) {
            neighbors.push_back(i);
        }
    }

    return neighbors;
}

void Sensor::DBSCAN3D::expandCluster(int dataIndex, int clusterId,vector<int> neighbors) {
    labels[dataIndex] = clusterId;

    // vector<int> neighbors = rangeQuery(dataIndex);
    // if (neighbors.empty()) return;  // No neighbors
    for (int i : neighbors) {
        labels[i] = clusterId;
        // if (labels[i] == 0) {  // Unvisited
        //     labels[i] = clusterId;

            // // vector<int> neighborsPrime = rangeQuery(i);

            // // if (neighborsPrime.size() >= minPts) {
            // //     neighbors.insert(neighbors.end(), neighborsPrime.begin(), neighborsPrime.end());
            // // }
        // }
    }
}

int Sensor::DBSCAN3D::getClusterCount() const {
    return clusterId;
}

std::vector<Sensor::DBSCANOBJ>& Sensor::DBSCAN3D::getObjects(){
    
    // returns a map with center of object, and all the points in that object
    objects.clear();
    Sensor::DBSCANOBJ object;
    //auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < clusterId; i++){
        object.points.clear();
        for (int j = 0; j < data.size(); j++){
            if (labels[j] == i){
                object.points.push_back(data[j]);
            }
        }
        auto center = cv::Point3f(0,0,0);
        for (auto& point : object.points){
            center.x += point.x;
            center.y += point.y;
            center.z += point.z;
        }
        center.x /= object.points.size();
        center.y /= object.points.size();
        center.z /= object.points.size();
        object.centroid = center;
        objects.push_back(object);
    }
    //auto end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    //myfile <<duration<<",";
    
    //start = std::chrono::high_resolution_clock::now();
    //if the distance between two objects is less than eps, then merge them
    for (int idk=0; idk < objects.size(); idk++){
        for (int i = 0; i < objects.size(); i++){
            for (int j = i+1; j < objects.size(); j++){
                if (distance(objects[i].centroid, objects[j].centroid) < eps){
                    objects[i].points.insert(objects[i].points.end(), objects[j].points.begin(), objects[j].points.end());
                    objects.erase(objects.begin() + j);
                }
            }
        }
    }
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    //myfile <<duration<<",";

    //start = std::chrono::high_resolution_clock::now();
    this->createBoxes(objects);
    this->IoU(objects);
    this->createBoxes(objects);
    //end = std::chrono::high_resolution_clock::now();
    //duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    //myfile <<duration<<",";
 
    return objects;
}

cv::Vec3b Sensor::DBSCAN3D::getColor(float distance){
   
    int np = 1500;
    float r=0.0, g=0.0, b=0.0;
    float inc = 6.0 / np;
    float x = distance * inc;
    
    if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) r = 1.0f;
        else if (4 <= x && x <= 5) r = x - 4;
        else if (1 <= x && x <= 2) r = 1.0f - (x - 1);
    
    if (1 <= x && x <= 3) g = 1.0f;
        else if (0 <= x && x <= 1) g = x - 0;
        else if (3 <= x && x <= 4) g = 1.0f - (x - 3);
    
    if (3 <= x && x <= 5) b = 1.0f;
        else if (2 <= x && x <= 3) b = x - 2;
        else if (5 <= x && x <= 6) b = 1.0f - (x - 5);
    
    return cv::Vec3b(b*255,g*255,r*255);
    
}

void Sensor::DBSCAN3D::createBoxes(std::vector<DBSCANOBJ>& objects){
    for (auto& obj : objects) {
        for(auto& point : obj.points){
            obj.minPoint.x = std::min(obj.minPoint.x, point.x);
            obj.minPoint.y = std::min(obj.minPoint.y, point.y);
            obj.minPoint.z = std::min(obj.minPoint.z, point.z);

            obj.maxPoint.x = std::max(obj.maxPoint.x, point.x);
            obj.maxPoint.y = std::max(obj.maxPoint.y, point.y);
            obj.maxPoint.z = std::max(obj.maxPoint.z, point.z);
        }
    }
}
void Sensor::DBSCAN3D::IoU(std::vector<DBSCANOBJ>& objects){
    for (int i = 0; i < objects.size(); i++){
        for (int j = i+1; j < objects.size(); j++){
            cv::Point3f minPoint = cv::Point3f(std::max(objects[i].minPoint.x, objects[j].minPoint.x), std::max(objects[i].minPoint.y, objects[j].minPoint.y), std::max(objects[i].minPoint.z, objects[j].minPoint.z));
            cv::Point3f maxPoint = cv::Point3f(std::min(objects[i].maxPoint.x, objects[j].maxPoint.x), std::min(objects[i].maxPoint.y, objects[j].maxPoint.y), std::min(objects[i].maxPoint.z, objects[j].maxPoint.z));
            float intersection = std::max(0.0f, maxPoint.x - minPoint.x) * std::max(0.0f, maxPoint.y - minPoint.y) * std::max(0.0f, maxPoint.z - minPoint.z);
            float unionn = (objects[i].maxPoint.x - objects[i].minPoint.x) * (objects[i].maxPoint.y - objects[i].minPoint.y) * (objects[i].maxPoint.z - objects[i].minPoint.z) + (objects[j].maxPoint.x - objects[j].minPoint.x) * (objects[j].maxPoint.y - objects[j].minPoint.y) * (objects[j].maxPoint.z - objects[j].minPoint.z) - intersection;
            float iou = intersection / unionn;
            if (iou > 0.1){
                objects[i].points.insert(objects[i].points.end(), objects[j].points.begin(), objects[j].points.end());
                objects.erase(objects.begin() + j);
            }
        }
    }
}
 
//---------------------------------------------------------------------------
//pcapXT32Lidar CLASS--------------------------------------------------------
Sensor::pcapXT32Lidar::pcapXT32Lidar(std::string filename, Sensor::toolBox& tb): tb(tb){
    reader = pcpp::IFileReaderDevice::getReader(filename);
    this->reader = pcpp::IFileReaderDevice::getReader(filename);
    if (this->reader == NULL)
    {
        std::cerr<< "Cannot determine reader for file type" << std::endl;
        exit(1);
    }
    if (!this->reader->open())
    {
        std::cerr<< "Cannot open input.pcapng for reading" << std::endl;
        exit(1);
    }    

}

std::vector<cv::Point3f>& Sensor::pcapXT32Lidar::getFrame(){
    frame.clear();
    ignoredFrame.clear();
    for (size_t k = 0; k < 500; k++)
    {
        if(reader->getNextPacket(rawPacket) == false) break;
        memcpy(&LidarPayload, rawPacket.getRawData(), sizeof(LidarPayload));
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 32 ; i+=1) {

                float azimuth = toRad((LidarPayload.block[j].azimuth * 0.01f));
                float elevation = toRad((((i) * 1.0f) - 16.0f));
                float distance = (LidarPayload.block[j].channel[i].distance * LidarPayload.header.DisUnit)/10.0f; //mm //cm

                float X3d = distance * cos(azimuth) * cos(elevation);
                float Y3d = distance * sin(azimuth) * cos(elevation);
                float Z3d = distance * sin(elevation);

                // Perform rotation around the Z-axis
                //auto newPoint = Sensor::rotatePoint({X3d, Y3d, Z3d}, 10.5, 'z'); X3d = newPoint.x; Y3d = newPoint.y; Z3d = newPoint.z;
                
                //rotate 10 degrees around y axis
                 //newPoint = Sensor::rotatePoint({X3d, Y3d, Z3d}, -1, 'y'); X3d = newPoint.x; Y3d = newPoint.y; Z3d = newPoint.z;
                auto newPoint = Sensor::rotatePoint({X3d, Y3d, Z3d}, -29.5, 'z'); X3d = newPoint.x; Y3d = newPoint.y; Z3d = newPoint.z;
                Y3d-=250;
                X3d+= 380;
                
                if (X3d >= tb.box.NXlim && X3d <= tb.box.PXlim && Y3d >= tb.box.NYlim && Y3d <= tb.box.PYlim && Z3d >= tb.box.NZlim && Z3d <= tb.box.PZlim) frame.push_back({X3d, Y3d, Z3d});
                else ignoredFrame.push_back({X3d, Y3d, Z3d});
            }
        }  
    }
    this->frameCounter += 1;
    return frame;
}   

std::vector<cv::Point3f>& Sensor::pcapXT32Lidar::getIgnoredFrame(){
    return ignoredFrame;
}

void Sensor::pcapXT32Lidar::skipNFrames(int n){
    for (size_t k = 0; k < n*500; k++)
    {
        if(reader->getNextPacket(rawPacket) == false) break;
    }
    this->frameCounter += n;
}

int Sensor::pcapXT32Lidar::howManyFrames() const{
    return this->frameCounter;
}

unsigned int Sensor::pcapXT32Lidar::getTimeStamp(Sensor::timeUnit tu) const{
    switch (tu) {
        case Sensor::timeUnit::year :
            return  LidarPayload.tail.DateTime.year + 1900;
            break;
        case Sensor::timeUnit::month :
            return  LidarPayload.tail.DateTime.month;
            break;
        case Sensor::timeUnit::day :
            return  LidarPayload.tail.DateTime.day;
            break;
        case Sensor::timeUnit::hour :
            return  LidarPayload.tail.DateTime.hour;
            break;
        case Sensor::timeUnit::minute :
            return  LidarPayload.tail.DateTime.min;
            break;
        case Sensor::timeUnit::second :
            return LidarPayload.tail.DateTime.sec;
            break;
        case Sensor::timeUnit::us :
            return LidarPayload.tail.TimeStamp;
            break;
        default:
            return -1;
    }
}
//---------------------------------------------------------------------------
//ipXT32Lidar CLASS---------------------------------------------------------
Sensor::ipXT32Lidar::ipXT32Lidar(int port, Sensor::toolBox& tb) : tb(tb){
    // Create a UDP socket
    if ((udpSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr<<"Failed to create socket"<<std::endl;
        exit(1);
    }
    // Set up the server address and port to listen on
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to the server address
    if (::bind(udpSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr<<"Bind failed"<<std::endl;
        ::exit(1);
    }
    clientAddrLen = sizeof(clientAddr);
}

std::vector<cv::Point3f>& Sensor::ipXT32Lidar::getFrame(){
    frame.clear();
    ignoredFrame.clear();
    for (size_t k = 0; k < 500; k++)
    {
        ssize_t bytes_received = recvfrom(udpSocket, receive_buffer, sizeof(receive_buffer), 0,(struct sockaddr*)&clientAddr, &clientAddrLen);
        if (bytes_received < 0) {
            perror("recvfrom");
            close(udpSocket);
            exit(4);
        }
        memcpy(&LidarPayload, receive_buffer, sizeof(LidarPayload));
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 32 ; i+=1) {

                float azimuth = toRad((LidarPayload.block[j].azimuth * 0.01f));
                float elevation = toRad((((i) * 1.0f) - 16.0f));
                float distance = (LidarPayload.block[j].channel[i].distance * LidarPayload.header.DisUnit)/10.0f; //mm //cm

                float X3d = distance * cos(azimuth) * cos(elevation);
                float Y3d = distance * sin(azimuth) * cos(elevation);
                float Z3d = distance * sin(elevation);

                // Perform rotation around the Z-axis if needed
                auto newPoint = Sensor::rotatePoint({X3d, Y3d, Z3d}, 96.0, 'z'); X3d = newPoint.x; Y3d = newPoint.y; Z3d = newPoint.z;
                //Y3d+=230;
                //X3d-= 850;

                if (X3d >= tb.box.NXlim && X3d <= tb.box.PXlim && Y3d >= tb.box.NYlim && Y3d <= tb.box.PYlim && Z3d >= tb.box.NZlim && Z3d <= tb.box.PZlim) frame.push_back({X3d, Y3d, Z3d});
                else ignoredFrame.push_back({X3d, Y3d, Z3d});
            }
        }  
    }
    this->frameCounter += 1;
    return frame;
}

std::vector<cv::Point3f>& Sensor::ipXT32Lidar::getIgnoredFrame(){
    return ignoredFrame;
}

int Sensor::ipXT32Lidar::howManyFrames () const{
    return this->frameCounter;
}

unsigned int Sensor::ipXT32Lidar::getTimeStamp(Sensor::timeUnit tu) const{
    switch (tu) {
        case Sensor::timeUnit::year :
            return LidarPayload.tail.DateTime.year + 1900;
            break;
        case Sensor::timeUnit::month :
            return LidarPayload.tail.DateTime.month;
            break;
        case Sensor::timeUnit::day :
            return LidarPayload.tail.DateTime.day;
            break;
        case Sensor::timeUnit::hour :
            return LidarPayload.tail.DateTime.hour;
            break;
        case Sensor::timeUnit::minute :
            return LidarPayload.tail.DateTime.min;
            break;
        case Sensor::timeUnit::second :
            return LidarPayload.tail.DateTime.sec;
            break;
        case Sensor::timeUnit::us :
            return LidarPayload.tail.TimeStamp;
            break;
        default:
            return -1;
    }
}
//----------------------------------------------------------------------------
//ipPandarQTLidar CLASS-------------------------------------------------------
Sensor::ipPandarQTLidar::ipPandarQTLidar(int port, Sensor::toolBox& tb): tb(tb){
    // Create a UDP socket
    if ((udpSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr<<"Failed to create socket"<<std::endl;
        exit(1);
    }
    // Set up the server address and port to listen on
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to the server address
    if (::bind(udpSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr<<"Bind failed"<<std::endl;
        ::exit(1);
    }
    clientAddrLen = sizeof(clientAddr);
}

std::vector<cv::Point3f>& Sensor::ipPandarQTLidar::getFrame(){
    frame.clear();
    ignoredFrame.clear();
    for (size_t k = 0; k < 300; k++)
    {
        ssize_t bytes_received = recvfrom(udpSocket, receive_buffer, sizeof(receive_buffer), 0,(struct sockaddr*)&clientAddr, &clientAddrLen);
        if (bytes_received < 0) {
            perror("recvfrom");
            close(udpSocket);
            exit(4);
        }
        memcpy(&LidarPayload, receive_buffer, sizeof(LidarPayload));
        for (int j = 0; j < 4; j++) {
            if (j == 0 || j == 2) continue; //skipping last return since it's in dual mode
            for (int i = 0; i < 64 ; i+=1) {   
                float azimuth = (LidarPayload.block[j].azimuth / 100.0f); 
                float elevation = elevationRef[i];
                float distance = (LidarPayload.block[j].channel[i].distance * LidarPayload.header.DisUnit) /10.0f; //mm //cm
                elevation = toRad(elevation);
                azimuth = toRad(azimuth); 

                float X3d = distance * cos(azimuth) * cos(elevation); 
                float Y3d = distance * sin(azimuth) * cos(elevation); 
                float Z3d = distance * sin(elevation); 

                // rotate -35 degrees around the y axis, specific case for the side-mounted lidat
                //rotate 180 degrees around the x axis, specific case for the side-mounted lidar
                //rotate 90 degrees around the z axis, specific case for the side-mounted lidar
                // Z3d+=15;
                // Y3d+=15;

                if (X3d >= tb.box.NXlim && X3d <= tb.box.PXlim && Y3d >= tb.box.NYlim && Y3d <= tb.box.PYlim && Z3d >= tb.box.NZlim && Z3d <= tb.box.PZlim) frame.push_back({X3d, Y3d, Z3d});
                else ignoredFrame.push_back({X3d, Y3d, Z3d});

            }
        }
    }

    return frame;
}

std::vector<cv::Point3f>& Sensor::ipPandarQTLidar::getIgnoredFrame(){
    return ignoredFrame;
}

int Sensor::ipPandarQTLidar::howManyFrames () const{
    return this->frameCounter;
}

unsigned int Sensor::ipPandarQTLidar::getTimeStamp(Sensor::timeUnit tu) const{
    switch (tu) {
        case Sensor::timeUnit::year :
            return LidarPayload.tail.DateTime.year;
            break;
        case Sensor::timeUnit::month :
            return LidarPayload.tail.DateTime.month;
            break;
        case Sensor::timeUnit::day :
            return LidarPayload.tail.DateTime.day;
            break;
        case Sensor::timeUnit::hour :
            return LidarPayload.tail.DateTime.hour;
            break;
        case Sensor::timeUnit::minute :
            return LidarPayload.tail.DateTime.min;
            break;
        case Sensor::timeUnit::second :
            return LidarPayload.tail.DateTime.sec;
            break;
        case Sensor::timeUnit::us :
            return LidarPayload.tail.TimeStamp;
            break;
        default:
            return -1;
    }
}

//----------------------------------------------------------------------------
//pcapPandarQTLidar CLASS-------------------------------------------------------
Sensor::pcapPandarQTLidar::pcapPandarQTLidar(std::string filename, Sensor::toolBox& tb):tb(tb){
    reader = pcpp::IFileReaderDevice::getReader(filename);
    this->reader = pcpp::IFileReaderDevice::getReader(filename);
    if (this->reader == NULL)
    {
        std::cerr<< "Cannot determine: "<< filename  <<" for file type" << std::endl;
        exit(1);
    }
    if (!this->reader->open())
    {
        std::cerr<< "Cannot open: "<< filename  <<" for reading" << std::endl;
        exit(1);
    }    
}

std::vector<cv::Point3f>& Sensor::pcapPandarQTLidar::getFrame(){
    frame.clear();
    ignoredFrame.clear();
    for (size_t k = 0; k < 300; k++)
    {
        if(reader->getNextPacket(rawPacket) == false) break;
        memcpy(&LidarPayload, rawPacket.getRawData(), sizeof(LidarPayload));
        for (int j = 0; j < 4; j++) {
            if (j == 0 || j == 2) continue; //skipping last return since it's in dual mode
            for (int i = 0; i < 64 ; i+=1) {   
                float azimuth = (LidarPayload.block[j].azimuth / 100.0f); 
                float elevation = elevationRef[i];
                float distance = (LidarPayload.block[j].channel[i].distance * LidarPayload.header.DisUnit) /10.0f; //mm //cm
                elevation = toRad(elevation);
                azimuth = toRad(azimuth); 

                float X3d = distance * cos(azimuth) * cos(elevation); 
                float Y3d = distance * sin(azimuth) * cos(elevation); 
                float Z3d = distance * sin(elevation); 

                // rotate -35 degrees around the y axis, specific case for the side-mounted lidat
                auto newPoint = Sensor::rotatePoint({X3d, Y3d, Z3d},-35,'y');

                //rotate 180 degrees around the x axis, specific case for the side-mounted lidar
                newPoint = Sensor::rotatePoint(newPoint, 180, 'x');

                //rotate 90 degrees around the z axis, specific case for the side-mounted lidar
                newPoint = Sensor::rotatePoint(newPoint, 90, 'z'); X3d = newPoint.x; Y3d = newPoint.y; Z3d = newPoint.z;


                // Z3d+=15;
                // Y3d+=15;

                if (X3d >= tb.box.NXlim && X3d <= tb.box.PXlim && Y3d >= tb.box.NYlim && Y3d <= tb.box.PYlim && Z3d >= tb.box.NZlim && Z3d <= tb.box.PZlim) frame.push_back({X3d, Y3d, Z3d});
                else ignoredFrame.push_back({X3d, Y3d, Z3d});

            }
        }
    }
    this->frameCounter += 1;
    return frame;
}

std::vector<cv::Point3f>& Sensor::pcapPandarQTLidar::getIgnoredFrame(){
    return ignoredFrame;
}

void Sensor::pcapPandarQTLidar::skipNFrames(int n){
    for (size_t k = 0; k < n*150; k++)
    {
        if(reader->getNextPacket(rawPacket) == false) break;
    }
    this->frameCounter += n;
}

int Sensor::pcapPandarQTLidar::howManyFrames() const{
    return this->frameCounter;
}

unsigned char Sensor::pcapPandarQTLidar::returnType() const{
    return LidarPayload.tail.returnMode;
}

unsigned int Sensor::pcapPandarQTLidar::getTimeStamp(Sensor::timeUnit tu) const{
    switch (tu) {
        case Sensor::timeUnit::year :
            return LidarPayload.tail.DateTime.year;
            break;
        case Sensor::timeUnit::month :
            return LidarPayload.tail.DateTime.month;
            break;
        case Sensor::timeUnit::day :
            return LidarPayload.tail.DateTime.day;
            break;
        case Sensor::timeUnit::hour :
            return LidarPayload.tail.DateTime.hour;
            break;
        case Sensor::timeUnit::minute :
            return LidarPayload.tail.DateTime.min;
            break;
        case Sensor::timeUnit::second :
            return LidarPayload.tail.DateTime.sec;
            break;
        case Sensor::timeUnit::us :
            return LidarPayload.tail.TimeStamp;
            break;
        default:
            return -1;
    }
}
//----------------------------------------------------------------------------
//pcapVLD16Lidar Radar-------------------------------------------------------
Sensor::pcapVLD16Lidar::pcapVLD16Lidar(std::string filename, Sensor::toolBox& tb):tb(tb){
    reader = pcpp::IFileReaderDevice::getReader(filename);

    this->reader = pcpp::IFileReaderDevice::getReader(filename);
    if (this->reader == NULL)
    {
        std::cerr<< "Cannot determine reader for file type" << std::endl;
        exit(1);
    }
    if (!this->reader->open())
    {
        std::cerr<< "Cannot open input.pcapng for reading" << std::endl;
        exit(1);
    }    
}

std::vector<cv::Point3f>& Sensor::pcapVLD16Lidar::getFrame(){
    frame.clear();
    ignoredFrame.clear();
    unsigned short temp = 0;
    float azimuthN0,azimuthN1 = 0;
    float distanceN0, distanceN1 = 0;
    float elevationN0, elevationN1 = 0;
    float X3dN0, Y3dN0, Z3dN0 = 0;
    float X3dN1, Y3dN1, Z3dN1 = 0;
    
    for (size_t k = 0; k < 75; k++)
    {
        if(reader->getNextPacket(rawPacket) == false) break;
        memcpy(&LidarPayload, rawPacket.getRawData(), sizeof(LidarPayload));
        for (int j=0; j<12;j++){
            temp = (LidarPayload.block[j].azimuth[1] << 8) | LidarPayload.block[j].azimuth[0];
            azimuthN0 = ((float)temp * 0.01f);
            azimuthN1 = azimuthN0 + 0.2f;
            azimuthN0 = toRad(azimuthN0);
            azimuthN1 = toRad(azimuthN1);

            for (int i = 0; i < 16; i++){
                temp = (LidarPayload.block[j].channelN0[i].distance[1] << 8) | LidarPayload.block[j].channelN0[i].distance[0];
                distanceN0 = (float)temp * 2.0f / 10.0f ; //mm //cm
                elevationN0 = toRad(elevation[i]);
                X3dN0 = distanceN0 * cos(elevationN0) * sin(azimuthN0);
                Y3dN0 = distanceN0 * cos(azimuthN0) * cos(elevationN0);
                Z3dN0 = distanceN0 * sin(elevationN0);
                //rotate 180 about the y axis
                auto NPN0 = rotatePoint({X3dN0, Y3dN0, Z3dN0}, 180, 'y'); X3dN0 = NPN0.x; Y3dN0 = NPN0.y; Z3dN0 = NPN0.z;
                //rotate 90 about the z axis
                NPN0 = rotatePoint({X3dN0, Y3dN0, Z3dN0}, 88, 'z'); X3dN0 = NPN0.x; Y3dN0 = NPN0.y; Z3dN0 = NPN0.z;
                
                X3dN0+=-27; 

                if(X3dN0 >= tb.box.NXlim && X3dN0 <= tb.box.PXlim && Y3dN0 >= tb.box.NYlim && Y3dN0 <= tb.box.PYlim && Z3dN0 >= tb.box.NZlim && Z3dN0 <= tb.box.PZlim) frame.push_back({X3dN0, Y3dN0, Z3dN0});
                else ignoredFrame.push_back({X3dN0, Y3dN0, Z3dN0});

                temp = (LidarPayload.block[j].channelN1[i].distance[1] << 8) | LidarPayload.block[j].channelN1[i].distance[0];
                distanceN1 = (float)temp * 2.0f/10.0f; //mm //cm
                elevationN1 = toRad(elevation[i]);
                X3dN1 = distanceN1 * cos(elevationN1) * sin(azimuthN1);
                Y3dN1 = distanceN1 * cos(azimuthN1) * cos(elevationN1);
                Z3dN1 = distanceN1 * sin(elevationN1);
                //rotate 180 about the y axis
                auto NPN1 = rotatePoint({X3dN1, Y3dN1, Z3dN1}, 180, 'y'); X3dN1 = NPN1.x; Y3dN1 = NPN1.y; Z3dN1 = NPN1.z;
                //rotate 90 about the z axis
                NPN1 = rotatePoint({X3dN1, Y3dN1, Z3dN1}, 88, 'z'); X3dN1 = NPN1.x; Y3dN1 = NPN1.y; Z3dN1 = NPN1.z;
   
                X3dN1+=-27;
                if(X3dN1 >= tb.box.NXlim && X3dN1 <= tb.box.PXlim && Y3dN1 >= tb.box.NYlim && Y3dN1 <= tb.box.PYlim && Z3dN1 >= tb.box.NZlim && Z3dN1 <= tb.box.PZlim) frame.push_back({X3dN1, Y3dN1, Z3dN1});
                else ignoredFrame.push_back({X3dN1, Y3dN1, Z3dN1});

            }

        }
    }

    this->frameCounter += 1;
    return frame;
}

std::vector<cv::Point3f>& Sensor::pcapVLD16Lidar::getIgnoredFrame(){
    return ignoredFrame;
}

int Sensor::pcapVLD16Lidar::howManyFrames() const {
    return frameCounter;
}

void Sensor::pcapVLD16Lidar::skipNFrames(int n){
    for (size_t k = 0; k < n*75; k++)
    {
        if(reader->getNextPacket(rawPacket) == false) break;
    }
    this->frameCounter += n;
    return;
}

unsigned char Sensor::pcapVLD16Lidar::returnType() const{
    return this->LidarPayload.returnType;
}

unsigned int Sensor::pcapVLD16Lidar::getTimeStamp() const{
    return LidarPayload.timeStamp; //might need to be flipped

}

//----------------------------------------------------------------------------
//CANSmartMicro Radar-------------------------------------------------------

Sensor::CANSmartMicroRadar::CANSmartMicroRadar(Sensor::toolBox& tb):tb(tb){
    opMode.byte = CANMODE_DEFAULT;
    bitrate.index = BAUDRATE;
    std::cout << CKvaserCAN::GetVersion() << std::endl;
    if ((retVal = myDriver.InitializeChannel(CHANNEL, opMode)) != CCanApi::NoError) {
        std::cerr << "+++ error: interface could not be initialized" << std::endl;
        exit(retVal);
    }
    if ((retVal = myDriver.StartController(bitrate)) != CCanApi::NoError) {
        std::cerr << "+++ error: interface could not be started" << std::endl;
        stop();
    }
}

Sensor::CANSmartMicroRadar::parser Sensor::CANSmartMicroRadar::Parse(CANAPI_Message_t& message){
    parser p;
    p.id = message.id;
    for (uint8_t i = 0; i < CCanApi::Dlc2Len(message.dlc); i++){
        auto val = message.data[i];
        switch (i){
            case 0:
                p.byte0 = val;
                break;
            case 1:
                p.byte1 = val;
                break;
            case 2:
                p.byte2 = val;
                break;
            case 3:
                p.byte3 = val;
                break;
            case 4:
                p.byte4 = val;
                break;
            case 5:
                p.byte5 = val;
                break;
            case 6:
                p.byte6 = val;
                break;
            case 7:
                p.byte7 = val;
                break;
            default:
                break;
        } 
    }
    p.data = (static_cast<uint64_t>(p.byte7) << 56) | (static_cast<uint64_t>(p.byte6) << 48) | (static_cast<uint64_t>(p.byte5) << 40) | (static_cast<uint64_t>(p.byte4) << 32) | (static_cast<uint64_t>(p.byte3) << 24) | (static_cast<uint64_t>(p.byte2) << 16) | (static_cast<uint64_t>(p.byte1) << 8) | static_cast<uint64_t>(p.byte0);
    return p;

}

std::vector<Sensor::radarPoint>& Sensor::CANSmartMicroRadar::getFrame(){

    frame.clear();
    while (true)
    {
       if ((retVal = myDriver.ReadMessage(message)) == CCanApi::NoError) parsed = Parse(message);
        if (parsed.id == 0x400){
            uint64_t modeBit = 0;
            BM::copyBits(modeBit, parsed.data, 62, 63);
            if (modeBit == 0){
                BM::copyBits(numberOfTargets, parsed.data, 47, 54);
                BM::copyBits(CycleCount, parsed.data, 7, 46);
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 11);
                CycleDuration = (tmp * 0.000064);
                
            }
            else if (modeBit == 1){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp = tmp * 1.0;

            }
            else if (modeBit == 2){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp += (tmp * 0.00000000023283064365386962890625);
                break;
            }
            else continue;
        }
    }
    auto targetCounter = numberOfTargets;
    while(targetCounter > 0){
        if ((retVal = myDriver.ReadMessage(message)) == CCanApi::NoError)
        {
            parsed = Parse(message);
            if (parsed.id > 0x400 && parsed.id < 0x4FF)
            {
                if(BM::getBit(parsed.data, 0) == 0) //DataFrame 0
                {
                    uint64_t tmp;
                    BM::copyBits(tmp, parsed.data, 1, 13);
                    point.R = (tmp * 0.04); //meter
                    BM::copyBits(tmp, parsed.data, 39, 50);
                    point.RR = (tmp * 0.04); //m/s
                    BM::copyBits(tmp, parsed.data, 22, 31);
                    point.Azimuth = tmp * 0.16; //degree
                    point.ObjID = parsed.id - 0x400;
                  
                }
                if(BM::getBit(parsed.data, 0) == 1) //DataFrame 1
                {
                    uint64_t tmp;
                    BM::copyBits(tmp, parsed.data, 37, 46);
                    point.Elevation = tmp * 0.04; //degree
                    frame.push_back(point);
                    targetCounter--;
                }
            }
        }
    }
    frameCounter++;
    return frame;
}

void Sensor::CANSmartMicroRadar::stop() {
    myDriver.SignalChannel();
    if ((retVal = myDriver.TeardownChannel()) != CCanApi::NoError)
        std::cerr << "+++ error: interface could not be shutdown" << std::endl;
    std::cout << "Radar Stopped" << std::endl;

}

int Sensor::CANSmartMicroRadar::getFrameCount() const{
    return frameCounter;
}

double Sensor::CANSmartMicroRadar::getCycleDuration() const{
    return CycleDuration;
}

uint64_t Sensor::CANSmartMicroRadar::getCycleCount() const{
    return CycleCount;
}

double Sensor::CANSmartMicroRadar::getTimeStamp() const{
    return TimeStamp;
}

uint64_t Sensor::CANSmartMicroRadar::getnumberOfTargets() const{
    return numberOfTargets;
}


//----------------------------------------------------------------------------
//OfflineSmartMicroRadar-------------------------------------------------------
    Sensor::OfflineSmartMicroRadar::OfflineSmartMicroRadar(Sensor::toolBox& tb, std::string filename):tb(tb){
    file = std::ifstream(filename);
        if (!file.is_open()) {
        std::cerr << "Failed to open log.csv" << std::endl;
        exit(7);
    }
    std::getline(file, line);  // Skip the first line for compatibility issues.
}

Sensor::OfflineSmartMicroRadar::parser Sensor::OfflineSmartMicroRadar::Parse(std::string line){
    std::stringstream ss(line);
    std::string value;
    parser p;

    // Read each value separated by semicolon
    std::getline(ss, value, ';');
    p.id = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte7 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte6 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte5 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte4 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte3 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte2 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte1 = std::stoi(value, nullptr, 10);
    std::getline(ss, value, ';');
    p.byte0 = std::stoi(value, nullptr, 10);

    p.data = (static_cast<uint64_t>(p.byte7) << 56) | (static_cast<uint64_t>(p.byte6) << 48) | (static_cast<uint64_t>(p.byte5) << 40) | (static_cast<uint64_t>(p.byte4) << 32) | (static_cast<uint64_t>(p.byte3) << 24) | (static_cast<uint64_t>(p.byte2) << 16) | (static_cast<uint64_t>(p.byte1) << 8) | static_cast<uint64_t>(p.byte0);

    return p;
}

std::vector<Sensor::radarPoint>& Sensor::OfflineSmartMicroRadar::getFrame(){
    frame.clear();

    while (std::getline(file, line))
    {
        parsed = Parse(line);
        if (parsed.id == 0x400)
        {
            uint64_t modeBit = 0;
            BM::copyBits(modeBit, parsed.data, 62, 63);
            if (modeBit == 0){
                BM::copyBits(numberOfTargets, parsed.data, 47, 54);
                BM::copyBits(CycleCount, parsed.data, 7, 46);
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 11);
                CycleDuration = (tmp * 0.000064);
                
            }
            else if (modeBit == 1){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp = tmp * 1.0;

            }
            else if (modeBit == 2){
                uint64_t tmp = 0;
                BM::copyBits(tmp, parsed.data, 0, 31);
                TimeStamp += (tmp * 0.00000000023283064365386962890625);
                break;
            }
            else continue;
        }
    }
    
    auto targetCounter = numberOfTargets;
    while(targetCounter > 0){
        if (std::getline(file, line))
        {
            parsed = Parse(line);
            if (parsed.id > 0x400 && parsed.id < 0x4FF)
            {
                if(BM::getBit(parsed.data, 0) == 0) //DataFrame 0
                {
                    uint64_t tmp=0;
                    BM::copyBits(tmp, parsed.data, 1, 13);
                    point.R = (tmp * 0.04); //meter
                    tmp=0;
                    BM::copyBits(tmp, parsed.data, 39, 50);
                    point.RR = (tmp * 0.04); //m/s
                    tmp=0;
                    BM::copyBits(tmp, parsed.data, 22, 31);
                    int64_t tmps = 0;
                    tmps=static_cast<int64_t>(tmp);
                    tmps-=511;
                    point.Azimuth = tmps * 0.16; //degree
                    point.ObjID = parsed.id - 0x400;
                  
                }
                if(BM::getBit(parsed.data, 0) == 1) //DataFrame 1
                {
                    uint64_t tmp=0;
                    BM::copyBits(tmp, parsed.data, 37, 46);
                    int64_t tmps = 0;
                    tmps = static_cast<int64>(tmp);
                    tmps-=511;
                    point.Elevation = tmps * 0.04; //degree
                    frame.push_back(point);
                    targetCounter--;
                }
            }

        }
    }
    frameCounter++;
    return frame;
}

void Sensor::OfflineSmartMicroRadar::stop() {
    file.close();
}

int Sensor::OfflineSmartMicroRadar::getFrameCount() const{
    return frameCounter;
}

double Sensor::OfflineSmartMicroRadar::getCycleDuration() const{
    return CycleDuration;
}

uint64_t Sensor::OfflineSmartMicroRadar::getCycleCount() const{
    return CycleCount;
}

double Sensor::OfflineSmartMicroRadar::getTimeStamp() const{
    return TimeStamp;
}

uint64_t Sensor::OfflineSmartMicroRadar::getnumberOfTargets() const{
    return numberOfTargets;
}
