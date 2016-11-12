all: command_test

command_test: RobotCommandTest.o irobot-create.o RobotCommand.o
	g++ -std=c++11 -o command_test RobotCommandTest.o RobotCommand.o irobot-create.o -pthread -L/opt/vc/lib -lserial -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d -lopencv_xfeatures2d -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

RobotCommandTest.o: RobotCommandTest.cc
	g++ -std=c++11 -c RobotCommandTest.cc

RobotCommand.o: RobotCommand.cc
	g++ -std=c++11 -c RobotCommand.cc

irobot-create.o: irobot-create.cc
	g++ -std=c++11 -c irobot-create.cc

