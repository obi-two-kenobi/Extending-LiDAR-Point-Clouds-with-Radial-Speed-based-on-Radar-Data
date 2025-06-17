## Project thesis: Extending LiDAR Point Clouds with Radial Speed based on Radar Data.
This project thesis describes the creation of a Sensor class in C++ for unifying different sensor measurements into one point cloud.

### Supported hardware:
-	Hesai’s Pandar XT32, a 32-channel medium-range 360-degree LiDAR,
- Hesai’s PandarQT, a 64-channel short-range 360-degree LiDAR,
- Velodyne’s VLP-16, a 16-channel 360-degree LiDAR,
- and Smart Microwave’s UMR11-Type 132 Automotive-grade Radar.

### Supported protocols:
- CAN,
- Ethernet (UDP),
- Network PCAP files (from Wireshark, etc.).

### Class structure: 
![UML Class Diagram of the system]([./Pictures/UML Class Diagram of the system.png](https://github.com/obi-two-kenobi/Extending-LiDAR-Point-Clouds-with-Radial-Speed-based-on-Radar-Data/blob/main/Pictures/UML%20Class%20diagram%20of%20drivers%20implementing%20the%20sensor%20parent%20class.png))
