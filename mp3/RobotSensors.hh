#ifndef ROBOTVISION_HH
#define ROBOTVISION_HH

class RobotSensors {

private:

  bool leftWheelOvercurrent;
  bool rightWheelOvercurrent;

  short cliffLeftSignal;
  short cliffRightSignal;
  short cliffFrontLeftSignal;
  short cliffFrontRightSignal;

  bool wheeldropLeft;
  bool wheeldropRight;
  bool wheeldropCaster;

  short angle;
  short wallSignal;
  bool bumpLeft;
  bool bumpRight;

  void setLeftWheelOvercurrent(bool val);
  void setrightWheelOvercurrent(bool val);

  void setCliffLeftSignal(short val);
  void setCliffRightSignal(short val);
  void setCliffFrontLeftSignal(short val);
  void setCliffFrontRightSignal(short val);

  void setWheeldropLeft(bool val);
  void setWheeldropRight(bool val);
  void setWheeldropCaster(bool val);

  void setAngle(short val);
  void setWallSignal(short val);
  void setBumpLeft(bool val);
  void setBumpRight(bool val);

public:

  RobotSensors(Create robot);

  void getLeftWheelOvercurrent(bool val);
  void getrightWheelOvercurrent(bool val);

  void getCliffLeftSignal(short val);
  void getCliffRightSignal(short val);
  void getCliffFrontLeftSignal(short val);
  void getCliffFrontRightSignal(short val);

  void getWheeldropLeft(bool val);
  void getWheeldropRight(bool val);
  void getWheeldropCaster(bool val);

  void getAngle(short val);
  void getWallSignal(short val);
  void getBumpLeft(bool val);
  void getBumpRight(bool val);

};

#endif
