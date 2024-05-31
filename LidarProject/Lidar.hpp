#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgcodecs.hpp>



#include <iostream>
#include <vector>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include "PcapFileDevice.h"

#include <thread>
#include <algorithm>
#include <arpa/inet.h>
#include <pcap.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <unistd.h>
#include "Includes/KvaserCAN.h"
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <sys/types.h>
#include <signal.h>

#include <ostream>
#include <chrono>


#define uchar unsigned char
#define toRad(x) (x * 3.14159265358979323846 / 180.00000)
#define toDeg(x) (x * 180.00000 / 3.14159265358979323846)
using namespace std;

namespace Sensor{
    
    enum timeUnit{
        year,month,day,hour,minute,second,us
    };

    struct toolBox{
        struct box{
            int NXlim=-1150/*+850*/,
                NYlim=-475/*-230*/,
                NZlim=-110,
                PXlim=780/*+850*/,
                PYlim=430/*-230*/,
                PZlim=105;
        }box;
        
        struct cluster{
            int minPts = 30;
            double eps = 125;
        }cluster;
        
        struct view{
            bool showIgnored = false;
            bool showIDs = true;
        }view;
    };
    
    struct radarPoint{
        int ObjID;
        double R,RR,Azimuth,Elevation;
    };

    
    struct DBSCANOBJ{
        cv::Point3f centroid;
        std::vector<cv::Point3f> points;
        cv::Point3f minPoint= cv::Point3f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
        cv::Point3f maxPoint= cv::Point3f(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
     
    };

    struct combinedObject{
        radarPoint& RP; //Radar point
        DBSCANOBJ& LO; // Lidar Object
    };

    class DBSCAN3D {
        private:
            double eps;
            int minPts;
            int clusterId = 0;
            vector<cv::Point3f> data;
            vector<int> labels;
            std::vector<DBSCANOBJ> objects;
            

            double distance(const cv::Point3f& p1, const cv::Point3f& p2);
            vector<int> rangeQuery(int dataIndex);
            void expandCluster(int dataIndex, int clusterId, vector<int> neighbors);
            void createBoxes(std::vector<DBSCANOBJ>& objects);
            void IoU(std::vector<DBSCANOBJ>& objects);

        public:
            //std::ofstream myfile;
            DBSCAN3D(std::vector<cv::Point3f>& data, Sensor::toolBox& tb);
            void run();
            const vector<int>& getLabels() const;
            int getClusterCount() const;
            std::vector<Sensor::DBSCANOBJ>& getObjects();
            cv::Vec3b getColor(float distance);

    };

    class sensor{

        
    };
   
    class pcapXT32Lidar: public sensor{
        private:
            struct recvBufferType
            {
                char ethernetHeader[42];
                struct PreHeaderType
                {
                    uchar SOP0;
                    uchar SOP1;
                    uchar ProtocolversionMajor;
                    uchar ProtocolversionMinor;
                    unsigned short Reserverd;
                }PreHeader;

                struct headerType
                {
                    uchar LaserNum;
                    uchar BlockNum;
                    uchar Reserverd;
                    uchar DisUnit;
                    uchar ReturnNumber;
                    uchar UDPSeq;
                }header;

                struct blockType
                {
                    unsigned short azimuth;
                    struct channelType {
                        unsigned short distance;
                        uchar reflectance;
                        uchar reserved;
                    }channel[32];

                }block[8];

                struct tailType {
                    uchar reserved[10];
                    uchar returnMode;
                    unsigned short motorSpeed;
                    struct DateTimeStruct{
                        uchar year; // year = current year - 1900
                        uchar month;
                        uchar day;
                        uchar hour;
                        uchar min;
                        uchar sec;
                    }DateTime;
                    unsigned int TimeStamp;
                    uchar FactoryInfo;
                }tail;

                struct additionalInfoType {
                    unsigned int udpSeq;
                }additionalInfo;

            }LidarPayload;
            pcpp::RawPacket rawPacket;
            pcpp::IFileReaderDevice* reader;
        Sensor::toolBox& tb;
            int frameCounter = 0;
            std::vector<cv::Point3f> frame;
            std::vector<cv::Point3f> ignoredFrame;
            
        public:
            int XT = 0,YT = 0,ZT = 0;
            float xTheta = 0, yTheta = 0, zTheta = 0;
            pcapXT32Lidar(std::string filename, toolBox& tb);  
            std::vector<cv::Point3f>& getFrame(); 
            std::vector<cv::Point3f>& getIgnoredFrame();    
            void skipNFrames(int n);
            int howManyFrames() const;
            unsigned int getTimeStamp(Sensor::timeUnit tu) const;
            
    };

    class ipXT32Lidar: public sensor{
        private:
            int udpSocket;
            struct sockaddr_in serverAddr;
            ssize_t bytesReceived;
            struct sockaddr_in clientAddr;
            socklen_t clientAddrLen;
            char receive_buffer[1080];
            #pragma pack(push, 1)
            struct recvBufferTypeXT32{
                struct PreHeaderType{
                    uchar SOP0;
                    uchar SOP1;
                    uchar ProtocolversionMajor;
                    uchar ProtocolversionMinor;
                    unsigned short Reserverd;
                }PreHeader;

                struct headerType{
                    uchar LaserNum;
                    uchar BlockNum;
                    uchar Reserverd;
                    uchar DisUnit;
                    uchar ReturnNumber;
                    uchar UDPSeq;
                }header;

                struct blockType{
                    unsigned short azimuth;
                    struct channelType {
                        unsigned short distance;
                        uchar reflectance;
                        uchar reserved;
                    }channel[32];
                }block[8];

                struct tailType {
                    uchar reserved[10];
                    uchar returnMode;
                    unsigned short motorSpeed;
                    struct DateTimeStruct{
                        uchar year; // year = currentYear - 1900
                        uchar month;
                        uchar day;
                        uchar hour;
                        uchar min;
                        uchar sec;
                    }DateTime;
                    unsigned int TimeStamp;
                    uchar FactoryInfo;
                }tail;

                struct additionalInfoType {
                    unsigned int udpSeq;
                }additionalInfo;

            }LidarPayload; //32 channel lidar 4000by32
        
            #pragma pack(pop)
        Sensor::toolBox& tb;
            std::vector<cv::Point3f> frame;
            std::vector<cv::Point3f> ignoredFrame;
            int frameCounter = 0;
        public:
            int XT = 0,YT = 0,ZT = 0;
            float xTheta = 0, yTheta = 0, zTheta = 0;
            ipXT32Lidar(int port, toolBox& tb); // port = 2370 
            std::vector<cv::Point3f>& getFrame();   
            std::vector<cv::Point3f>& getIgnoredFrame();  
            int howManyFrames() const;
            unsigned int getTimeStamp(Sensor::timeUnit tu) const;
    };
    
    class ipPandarQTLidar : public sensor{
        private:
            float elevationRef[64] = {-52.121, -49.785, -47.577, -45.477, -43.465, -41.528, -39.653, -37.831, -36.055, -34.32, -32.619, -30.95, -29.308, -27.69, -26.094, -24.51, -22.964, -21.42, -19.889, -18.372, -16.865, -15.368, -13.88, -12.399, -10.925, -9.457, -7.994, -6.535, -5.079, -3.626, -2.175, -0.725, 0.725, 2.175, 3.626, 5.079, 6.534, 7.993, 9.456, 10.923, 12.397, 13.877, 15.365, 16.861, 18.368, 19.885, 21.415, 22.959, 24.524, 26.101, 27.697, 29.315, 30.957, 32.627, 34.328, 36.064, 37.84, 39.662, 41.537, 43.475, 45.487, 47.587, 49.795, 52.133};
            int udpSocket;
            struct sockaddr_in serverAddr;
            ssize_t bytesReceived;
            struct sockaddr_in clientAddr;
            socklen_t clientAddrLen;
            char receive_buffer[1080];
            #pragma pack(push, 1)
            struct recvBufferTypePandarQT{
                //char ethernetHeader[42];
                struct PreHeaderType
                {
                    uchar SOP0;
                    uchar SOP1;
                    uchar ProtocolversionMajor;
                    uchar ProtocolversionMinor;
                    unsigned short Reserverd;
                }PreHeader;

                struct headerType
                {
                    uchar LaserNum;
                    uchar BlockNum;
                    uchar FirstBlockReturn;
                    uchar DisUnit;
                    uchar ReturnNumber;
                    uchar UDPSeq;
                }header;

                struct blockType
                {
                    unsigned short azimuth;
                    struct channelType {
                        unsigned short distance;
                        uchar reflectance;
                        uchar reserved;
                    }channel[64]; //each block has 64 channels

                }block[4];

                struct tailType {
                    uchar reserved[10];
                    unsigned short motorSpeed;
                    uchar returnMode;
                    struct DateTimeStruct{
                        uchar year;
                        uchar month;
                        uchar day;
                        uchar hour;
                        uchar min;
                        uchar sec;
                    }DateTime;
                    uchar FactoryInfo;
                    unsigned int TimeStamp;
                    
                }tail;

                struct additionalInfoType {
                    unsigned int udpSeq;
                }additionalInfo;

            }LidarPayload; //64 channel lidar 600by64LidarPayload; 
            #pragma pack(pop)
        Sensor::toolBox& tb;
            std::vector<cv::Point3f> frame;
            std::vector<cv::Point3f> ignoredFrame;
            int frameCounter = 0;
        public:
            int XT = 0,YT = 0,ZT = 0;
            float xTheta = 0, yTheta = 0, zTheta = 0;
            ipPandarQTLidar(int port , toolBox& tb);  //port = 2369
            std::vector<cv::Point3f>& getFrame();   
            std::vector<cv::Point3f>& getIgnoredFrame();  
            int howManyFrames() const;
            unsigned int getTimeStamp(Sensor::timeUnit tu) const;
    };
    
    class pcapPandarQTLidar : public sensor{
        private:
            float elevationRef[64] = {-52.121, -49.785, -47.577, -45.477, -43.465, -41.528, -39.653, -37.831, -36.055, -34.32, -32.619, -30.95, -29.308, -27.69, -26.094, -24.51, -22.964, -21.42, -19.889, -18.372, -16.865, -15.368, -13.88, -12.399, -10.925, -9.457, -7.994, -6.535, -5.079, -3.626, -2.175, -0.725, 0.725, 2.175, 3.626, 5.079, 6.534, 7.993, 9.456, 10.923, 12.397, 13.877, 15.365, 16.861, 18.368, 19.885, 21.415, 22.959, 24.524, 26.101, 27.697, 29.315, 30.957, 32.627, 34.328, 36.064, 37.84, 39.662, 41.537, 43.475, 45.487, 47.587, 49.795, 52.133};
            pcpp::RawPacket rawPacket;
            pcpp::IFileReaderDevice* reader;
            #pragma pack(push, 1)
            struct recvBufferTypePandarQT{
                char ethernetHeader[42];
                struct PreHeaderType
                {
                    uchar SOP0;
                    uchar SOP1;
                    uchar ProtocolversionMajor;
                    uchar ProtocolversionMinor;
                    unsigned short Reserverd;
                }PreHeader;

                struct headerType
                {
                    uchar LaserNum;
                    uchar BlockNum;
                    uchar FirstBlockReturn;
                    uchar DisUnit;
                    uchar ReturnNumber;
                    uchar UDPSeq;
                }header;

                struct blockType
                {
                    unsigned short azimuth;
                    struct channelType {
                        unsigned short distance;
                        uchar reflectance;
                        uchar reserved;
                    }channel[64]; //each block has 64 channels

                }block[4];

                struct tailType {
                    uchar reserved[10];
                    unsigned short motorSpeed;
                    uchar returnMode;
                    struct DateTimeStruct{
                        uchar year;
                        uchar month;
                        uchar day;
                        uchar hour;
                        uchar min;
                        uchar sec;
                    }DateTime;
                    uchar FactoryInfo;
                    unsigned int TimeStamp;
                    
                }tail;

                struct additionalInfoType {
                    unsigned int udpSeq;
                }additionalInfo;

            }LidarPayload; //64 channel lidar 600by64LidarPayload; 
            #pragma pack(pop)
        Sensor::toolBox& tb;
            std::vector<cv::Point3f> frame;
            std::vector<cv::Point3f> ignoredFrame;
            const int tmpDeg = -34; // lidar is mounted tilted. This is approx. the angle of the tilt
            int frameCounter = 0;
        public:
            
            pcapPandarQTLidar(std::string filename, toolBox& tb);  
            std::vector<cv::Point3f>& getFrame();   
            std::vector<cv::Point3f>& getIgnoredFrame();  
            void skipNFrames(int n);
            int howManyFrames() const;
            unsigned char returnType() const;
            unsigned int getTimeStamp(Sensor::timeUnit tu) const;
    };
    
    class pcapVLD16Lidar : public sensor{
        private:
            float elevation[16] = {-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15};
            pcpp::RawPacket rawPacket;
            pcpp::IFileReaderDevice* reader;
            #pragma pack(push, 1)
            struct recvBufferTypeVLD16
            {
                char ethernetHeader[42];
                struct blockType{
                    unsigned short flag; //always 0xffee
                    uchar azimuth[2]; //azimuth angle
                    struct channelType{
                        uchar distance[2]; 
                        uchar reflectance; 
                    }channelN0[16],channelN1[16]; 
                }block[12]; 
                int timeStamp; 
                uchar returnType; //factory info
                uchar lidarType;
            }LidarPayload; //16 channel lidar 400by16LidarPayload; 
            #pragma pack(pop)
        Sensor::toolBox& tb;
            std::vector<cv::Point3f> frame;
            std::vector<cv::Point3f> ignoredFrame;
            int frameCounter = 0;
        public:
            pcapVLD16Lidar(std::string filename, toolBox& tb);  
            std::vector<cv::Point3f>& getFrame();   
            std::vector<cv::Point3f>& getIgnoredFrame();  
            void skipNFrames(int n);
            int howManyFrames() const;
            unsigned char returnType() const;
        unsigned int getTimeStamp() const;
    };

    class CANSmartMicroRadar : public sensor{
    private:
        #define CHANNEL  0
        #define BAUDRATE  CANBTR_INDEX_500K

        CKvaserCAN myDriver = CKvaserCAN();
        CANAPI_OpMode_t opMode = {};
        CANAPI_Bitrate_t bitrate = {};
        CANAPI_Message_t message = {};
        CANAPI_Return_t retVal = 0;
        double CycleDuration; // in seconds
        uint64_t CycleCount;
        double TimeStamp; //in seconds
        uint64_t numberOfTargets = 0;
        int frameCounter = 0;
        Sensor::toolBox& tb;
        volatile bool running = true;
        Sensor::radarPoint point;
        std::vector<Sensor::radarPoint> frame;
        class BM {
        public:
            // Get the value of a specific bit at a given position
            static bool getBit(uint64_t value, int position) {
                return (value & (1ULL << position)) != 0;
            }

            // Set a specific bit at a given position to 1
            static void setBit(uint64_t& value, int position) {
                value |= (1ULL << position);
            }

            // Set a specific bit at a given position to 0
            static void clearBit(uint64_t& value, int position) {
                value &= ~(1ULL << position);
            }

            // Toggle a specific bit at a given position
            static void toggleBit(uint64_t& value, int position) {
                value ^= (1ULL << position);
            }

            // Copy a range of bits from source to destination
            static void copyBits(uint64_t& destination, const uint64_t& source, int start, int end) {
                uint64_t mask = ((1ULL << (end - start + 1)) - 1) << start;
                destination &= ~mask;  // Clear the bits in the destination range
                destination |= ((source >> start) & ((1ULL << (end - start + 1)) - 1)) << start;
                destination = destination >> start;
            }
        };
        struct parser{
        uint16_t id;
        uint8_t     byte7; // 1-byte variable
        uint8_t     byte6; // 1-byte variable
        uint8_t     byte5; // 1-byte variable
        uint8_t     byte4; // 1-byte variable
        uint8_t     byte3; // 1-byte variable
        uint8_t     byte2; // 1-byte variable
        uint8_t     byte1; // 1-byte variable
        uint8_t     byte0; // 1-byte variable
        uint64_t    data;

    }parsed;
        parser Parse(CANAPI_Message_t& message);
        
    public:
        CANSmartMicroRadar(Sensor::toolBox& tb);
        std::vector<Sensor::radarPoint>& getFrame();
        int getFrameCount() const;
        double getCycleDuration() const;
        uint64_t getCycleCount() const;
        double getTimeStamp() const;
        uint64_t getnumberOfTargets() const;
        void stop();
      };
    
    class OfflineSmartMicroRadar : public sensor{
        private:
            double CycleDuration; // in seconds
            uint64_t CycleCount;
            double TimeStamp; //in seconds
            uint64_t numberOfTargets = 0;
            int frameCounter = 0; //frame counter
        Sensor::radarPoint point; //{ int ObjID; double R,RR,Azimuth,Elevation;}
            std::vector<Sensor::radarPoint> frame;
        Sensor::toolBox& tb;
        
            class BM {
            public:
                // Get the value of a specific bit at a given position
                static bool getBit(uint64_t value, int position) {
                    return (value & (1ULL << position)) != 0;
                }

                // Set a specific bit at a given position to 1
                static void setBit(uint64_t& value, int position) {
                    value |= (1ULL << position);
                }

                // Set a specific bit at a given position to 0
                static void clearBit(uint64_t& value, int position) {
                    value &= ~(1ULL << position);
                }

                // Toggle a specific bit at a given position
                static void toggleBit(uint64_t& value, int position) {
                    value ^= (1ULL << position);
                }

                // Copy a range of bits from source to destination
                static void copyBits(uint64_t& destination, const uint64_t& source, int start, int end) {
                    uint64_t mask = ((1ULL << (end - start + 1)) - 1) << start;
                    destination &= ~mask;  // Clear the bits in the destination range
                    destination |= ((source >> start) & ((1ULL << (end - start + 1)) - 1)) << start;
                    destination = destination >> start;
                }
            }; //Bit Manipulator static class
            struct parser{
                uint16_t id;
                uint8_t     byte7; // 1-byte variable
                uint8_t     byte6; // 1-byte variable
                uint8_t     byte5; // 1-byte variable
                uint8_t     byte4; // 1-byte variable
                uint8_t     byte3; // 1-byte variable
                uint8_t     byte2; // 1-byte variable
                uint8_t     byte1; // 1-byte variable
                uint8_t     byte0; // 1-byte variable
                uint64_t    data;

            }parsed; //parsed can message
        
            
            std::ifstream file;
            std::string line;
            parser Parse(std::string line);

        public:
            OfflineSmartMicroRadar(toolBox& tb, std::string filename);
            std::vector<Sensor::radarPoint>& getFrame();
            int getFrameCount() const;
            double getCycleDuration() const;
            uint64_t getCycleCount() const;
            double getTimeStamp() const;
            uint64_t getnumberOfTargets() const;
            void stop();
        };


inline cv::Point3f rotatePoint(const cv::Point3f& point, float degAngle, char axis);
void rotateVector(std::vector<cv::Point3f>& vec, float degAngle, char axis );
void rotateVector(std::vector<Sensor::radarPoint>& vec, float degAngle, char axis );

}
