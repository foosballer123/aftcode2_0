Official repository for the AFT 2.0 project at UNCW

Platform Funded By: The Office of Innovation and Commercialization \
Platform Created By: Charlie Caputo, Joe Scott, and Carter Thumm
Platform Developed By: Benjamin Simpson, Charlie Caputo 
With Contributions From: Anna Knorr and Braiden Fallbright
Under Mentorship From: Dr. Hamed Saeidi

Instructions to run:
- Launch roscore (roscore)
- Navigate to config folder and load desired config (rosparam load config.yaml)
- ssh into raspberry pi and run low level controller (rosrun pi_codes spawn_rod.py --rod=1)
- Launch pylon camera node
- Navigate to ball_vision folder and launch blob_tracking_mono8.py (python3 blob_tracking_mono8.py)
- Launch desired filtering node (python3 filtering_node.py)
- Navigate to solvers folder and launch desired solvers (python3 solver.py)
- Navigate to sandbox folder and launch the combiner node (python3 combiner_node.py)
