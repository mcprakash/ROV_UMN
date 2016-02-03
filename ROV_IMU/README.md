This repository contains the source code (Aruino and R-Pi) for my Masters Project
- Vision Aided Inertial Localization for ROV 

The Arduino Due must be connected with the Raspberry Pi using an
ethernet cable to enable cross-talk. Specific libraries must be installed
and components (IMU,CAM) must be connected to ensure smooth operation.

Raspberry Pi Libraries:

	# Install Wifi Hotspot

	1. Follow the guide here - http://elinux.org/RPI-Wireless-Hotspot

	(Do this step at the end of all other install steps if you are using wifi for internet)
	* With rpi2_hotspot, you may need to set a static ip of 192.168.2.100 to your pc if the cmd/ipconfig does not give an ip with 192.168.2.xxx

	Make sure that the hotspot is turned on and you are able to connect to the hotspot

	# Install OpenCV

	2. Follow the guide here (recommended) - http://robertcastle.com/2014/02/installing-opencv-on-a-raspberry-pi/

	* Use sudo apt-get install libopencv-dev for easier installation

	Follow these steps to link Opencv directories

	sudo nano /etc/ld.so.conf.d/opencv.conf

	--->(Add to end of file) /usr/local/lib to the file

	sudo nano /etc/bash.bashrc

	---> (Add to end of file) PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig export PKG_CONFIG_PATH

	sudo ldconfig

	Run sample opencv code to make sure it is running fine

	# Library ncurses is required to run the GPU of the program
	3. sudo apt-get install ncurses-dev

	# Install the WiringPi Library - Required to run the GPIO pins of the RPi
	4. Download from - http://wiringpi.com/

	# Get latest working folder of picam_gpu
	5. Use Chandra's dropbox or github website for the code

	# Install a good text-editor for easy coding and viewing
	6. sudo apt-get install geany

	# Copy the userland-master file into the picam_gpu folder if not provided
	7. Download from here - https://github.com/raspberrypi/userland

Arduino Libraries:

	# Install Eigen3 library for Arduino Due 
	1. Follow the tutorial - https://github.com/mcprakash/ardunav/issues/1

	# Install the L3G and LSM303 libraries for pololu IMU
	2. Available at Github - https://github.com/pololu

	# Install Visual Studio 2013 Ultimate (or latest version) available for free through Microsoft Dreamspark for UMN students
	# Install Visual Micro (Arduino plugin for Visual Studio)
	# This gives a nice view of all the functions and modules
	3. Follow the link - http://www.visualmicro.com/

Windows:

	# Install Filezilla client and configure RPI-Lan and RPI-Wifi for transmission of data
	# Useful for rapidly transferring .h264 videos used in calibration from R-Pi to Windows/Laptop
	1. Follow the tutorial - https://www.raspberrypi.org/documentation/remote-access/ssh/sftp.md

	# Install Processing 2.2.1 or latest version for GUI
	2. Download here - https://processing.org/

	# Install necessary libraries for processing 
	3. Library names - Shapes3D and UDP 

Deployment/Field Testing:

	1. Conncet to the R-Pi Hotspot
	2. Run the vision code on R-Pi (remotely using wifi or through keyboard/mouse on R-Pi) and let it detect the landmark.
	3. Start the processing code (it resets the data.txt file, so store any previous file before starting)
	4. In stable position(1 min. for bias estimation),  start the arduino by sending 'a' through serial monitor
	5. After initialization, move the Sensor/ROV for testing




