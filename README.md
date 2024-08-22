Introduction
ACE (AI Caption Project) is focused on developing a control system for the AutoCar kit. The project includes the following components and directories:
    ACEKit: Directory containing the firmware for the car kit.
    AI: Directory containing the code for controlling the car.
    Lap: Directory containing practice exercises and examples.

Team Members
    Nohope (Bùi Phi Hùng)
    Tripper (Phạm Thị Thục Trinh)
    4H1H1H0H0 (Nguyễn Quang Hiệp)
Mentor
    AnhKD3 (Khuất Đức Anh)
Supporter
    Louisvu69 (Vũ Tuấn Linh)

Deployment Instructions
To deploy the ACE system, you will need three computers:
    Main Computer: Capture images from the car kit and sends them to the two auxiliary computers. The main computer will also identify the lane in the images.
    Auxiliary Computer 1: Receives images from the main computer and detect signs then send information back to the main computer.
    Auxiliary Computer 2: Receives images from the main computer and detect vehicles then send information back to the main computer.
Note: 
    *The main computer will aggregate the information from both auxiliary computers and send control commands to the AutoCar kit.
    *All computers must connect to the same Wifi

Start the System:
    Start the main computer with the LaneMain file.
    Start Auxiliary Computer 1 with the SignMain file.
    Start Auxiliary Computer 2 with the VehicleMain file.

Notes
    Ensure all computers are connected to the network and have the necessary permissions.
    Check configurations and parameters before starting the system to avoid unexpected issues.
