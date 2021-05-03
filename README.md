# StalkerCar
Contains code and project files for the Stalker-Car

Implementing an RC circuit design, this self-driving car is set to follow (i.e. 'stalk') a specific given object, such as a shoe, backpack, etc.

See also the original documentation in SelfDrive.docx

Currently no code runs on startup. The Pi autoconnects to eduroam and you can ssh into the pi at pi@stalkercar.app.byu.edu , X forwarding is enabled and useful when testing the vision pipeline.

**This lists which python scripts do what:**
* comTest.py sends some simple steering and driving commands to the arduino through the serial connection.
* controller.py is the main controller script for the project. It uses pre programmed computer vision 
	settings to track an object and drive towards it. Tracking appears to be working but the driving is not 100%.
* controller2.py appears to be a rewrite of controller.py made by the origional project creators, Im not sure of the 
	motivation behind it	
* newController.py looks the same as above
* opencv_practice.py was probably a learning script made by the origional project creator
* range_detector.py is a tool to calibrate the computer vision settings. It uses the same vision pipeline as 
	controller.py. use this to find the settings for your object and then put those settings into controller.py	
* test.py looks like it was used to test camera functions by the origional project creator
