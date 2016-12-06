all: RobotMain

RobotMain: RobotMain.o irobot-create.o RobotSafety.o RobotSensors.o
	g++ -std=c++11 -o RobotMain RobotMain.o irobot-create.o RobotSafety.o RobotSensors.o -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lrt

RobotMain.o: RobotMain.cc
	g++ -std=c++11 -c RobotMain.cc

irobot-create.o: irobot-create.cc
	g++ -std=c++11 -c irobot-create.cc

RobotSafety.o: RobotSafety.cc
	g++ -std=c++11 -c RobotSafety.cc

RobotSensors.o: RobotSensors.cc
	g++ -std=c++11 -c RobotSensors.cc

clean:
	rm -f RobotMain *.o
