
class Controller
{
public:
  Controller();
  void keyLoop();
  void watchdog();

  double linear_, angular_;
  double l_scale_, a_scale_;
  //double static intensities[30];



  ros::NodeHandle nh_,ph_, nSub;
  ros::Time first_publish_;
  ros::Time last_publish_;
  ros::Publisher vel_pub_;
  boost::mutex publish_mutex_;

  void publish(double, double);
  void static getHokuyoVal(const sensor_msgs::LaserScan laser);

  void neunanteDegRot(void);

  void forwardOne(void);

  void printHokuyoRanges(void);

 private:
};
