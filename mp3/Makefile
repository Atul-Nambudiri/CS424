all: RobotMain

RobotMain: RobotMain.o irobot-create.o RobotVision.o RobotSafety.o RobotMovement.o
	g++ -std=c++11 -o robotest robotest.o irobot-create.o RobotVision.o RobotSafety.o -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

RobotMain.o: RobotMain.cc
	g++ -std=c++11 -c RobotMain.cc

RobotMovement.o: RobotMovement.cc
	g++ -std=c++11 -c RobotMovement.cc

irobot-create.o: irobot-create.cc
	g++ -std=c++11 -c irobot-create.cc

RobotVision.o: RobotVision.cc
	g++ -std=c++11 -c RobotVision.cc

RobotSafety.o: RobotSafety.cc
	g++ -std=c++11 -c RobotSafety.cc

debug:
	g++ -g -std=c++11 -o robotest robotest.cc irobot-create.cc RobotVision.cc RobotSafety.cc -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

clean:
	rm -f RobotMain *.o
