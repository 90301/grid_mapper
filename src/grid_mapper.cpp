#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>


/*
 * Inhaler UTILS
 * Created by Josh Benton
 * Version .1
 */
namespace IUtils {
  std::string toString(double&x, double&y);
  std::string toString(double& input);
  std::string
  toString(int input);

  class doubleVarTracker {
  public:

    /*Do not use the default constrcutor
     * this is only here to allow the use of a map
     */
    doubleVarTracker()
    {
      this->id = "DEFAULT CONSTRCUTOR";
      this->min = SPECIAL_VALUE;
      this->max = SPECIAL_VALUE;
      this->bufferSize = DEFAULT_BUFFER_SIZE;
      this->useBuffer = DEFAULT_USE_BUFFER;
    }

    doubleVarTracker(std::string id)
    {
      //constructor
      this->id = id;
      this->min = SPECIAL_VALUE;
      this->max = SPECIAL_VALUE;
      this->bufferSize = DEFAULT_BUFFER_SIZE;
      this->useBuffer = DEFAULT_USE_BUFFER;
      this->valuesToPrint = DEFAULT_VALUES_TO_PRINT;
    }
    std::string id;
    double min; //min and max do not depend on the buffer
    double max;
    int bufferSize;
    int valuesToPrint;
    bool useBuffer;
    const static double SPECIAL_VALUE = -99999;
    const static int DEFAULT_BUFFER_SIZE = 100;
    const static int DEFAULT_VALUES_TO_PRINT = 3;
    const static bool DEFAULT_USE_BUFFER = true;
    std::vector<double> bufferValues;

    void
    addValue(double val)
    {
      if (this->min == SPECIAL_VALUE || this->max == SPECIAL_VALUE) {
        this->min = val;
        this->max = val;
      } else {
        if (this->min > val) {
          this->min = val;
        }
        if (this->max < val) {
          this->max = val;
        }
      }
      if (this->useBuffer) {
        if (bufferValues.size() < bufferSize) {
          bufferValues.push_back(val);
        } else {
          //remove the first value
          bufferValues.push_back(val);
          bufferValues.erase(bufferValues.begin());
        }
      }
    }

    std::string
    toStr()
    {
      std::string output = id + " :" + " min,max: " + IUtils::toString(this->min, this->max);

      if (useBuffer) {
        //put printed values on a new line
        output += "\n";
        int stopAt = bufferValues.size() - 1 - valuesToPrint;
        if (stopAt < 0) {
          stopAt = 0;
        }
        for (int i = bufferValues.size() - 1; i >= stopAt; i--) {
          output += " " + IUtils::toString(bufferValues[i]);
        }
      }
      return output;

      //return "";
    }
  };

  //class doubleVarTracker;
  std::map<std::string, int> ids;
  std::vector<int> counter;
  std::vector<int> resetValue;
  std::map<std::string, doubleVarTracker> doubleVarTrackers;

  bool
  toleranceCheck(double realValue, double expectedValue, double tolerance)
  {
    double min = expectedValue - tolerance;
    double max = expectedValue + tolerance;
    bool rtrn = false;


    if (realValue <= max && realValue >= min) {
      rtrn = true;
    }
    /*
    debugOutput ("Min: " +  IUtils::toString(min) +
            " | " + IUtils::toString(realValue) + 
            " | Max: " + IUtils::toString(max) +
            " | " + IUtils::toString(rtrn)
            , TCHECK_DEBUG);
     * */
    return rtrn;
  };

  double
  calcHypot(const double& x, const double& y)
  {
    return hypot(x, y);
  }




  //todo add other methods to determine information based on bufferValues

  //todo make a buffer class

  /*
   * Output the output text if condition is true
   * may be expanded in the future
   */
  void
  debugOutput(std::string outputText, bool condition)
  {
    //current code uses cout
    if (condition) {
      std::cout << outputText << std::endl;
    }
  };

  void
  debugOutput(std::string outputText, bool condition, std::string id, int outputEveryX)
  {
    //current code uses cout
    if (condition) {
      std::map<std::string, int>::iterator it = ids.find(id);
      if (it != ids.end()) {
        int index = it->second;

        if (counter[index] <= 0) {
          debugOutput(outputText, true);
          counter[index] = resetValue[index];
        } else {
          counter[index]--;
        }
      } else {
        ids[id] = counter.size();
        counter.push_back(outputEveryX);
        resetValue.push_back(outputEveryX);
      }
      //TODO: add id if not found in the vector, add relevant info
      //std::cout << outputText << std::endl;
    }



  };

  std::string
  toString(double& input)
  {
    std::ostringstream s;
    s << input;
    return s.str();
  };

  std::string
  toString(int input)
  {
    std::ostringstream s;
    s << input;
    return s.str();
  };

  std::string
  toString(uint32_t input)
  {
    std::ostringstream s;
    s << input;
    return s.str();
  };

  /*
   * SPECIAL TOSTRING FUNCTIONS
   */
  std::string
  toString(bool input)
  {
    std::string rtrn = "";
    if (input) {
      rtrn = "TRUE";
    } else {
      rtrn = "FALSE";
    }
    return rtrn;
  };

  std::string
  toString(double& inputX, double& inputY)
  {
    std::ostringstream s;
    s << inputX << " , " << inputY;
    return s.str();
  };

  /*
   * Variable tracking
   */

  void
  trackDouble(std::string id, double val)
  {

    std::map<std::string, doubleVarTracker>::iterator it = doubleVarTrackers.find(id);

    if (it != doubleVarTrackers.end()) {
      //it->second.addValue (val);
      doubleVarTrackers[id].addValue(val);
    } else {

      //id not found, add it to the list
      doubleVarTracker tracker(id);
      tracker.addValue(val);
      doubleVarTrackers[id] = tracker;

    }
  }

  std::string
  trackDoubleToString(std::string id, bool debugOut)
  {

    std::map<std::string, doubleVarTracker>::iterator it;
    it = doubleVarTrackers.find(id);
    std::string output = "";
    if (it != doubleVarTrackers.end()) {
      output = it->second.toStr();
    } else {
      output = "ERROR: tracked variable with id: " + id + " not found."; //id not found, report an error
    }
    IUtils::debugOutput(output, debugOut);


    return output;
  }

  /*
   * output all tracked variable to the debug console
   */
  void
  debugAllTrackedVariables(bool debugOut)
  {
    std::map<std::string, doubleVarTracker> ::iterator it;
    for (it = doubleVarTrackers.begin(); it != doubleVarTrackers.end(); it++) {
      trackDoubleToString(it->first, debugOut);
    }
  }




  const static bool TCHECK_DEBUG = false;


  /*
   * Inhaler Laser Utils
   * >>LASER_UTILS<<
   * 
   */
  const static double DEFAULT_LASER_STRIKE = 5000;

  /*
   * output variables are x and y
   */
  void convertAngleToXY(double& x, double& y,
          const double x_start, const double y_start,
          const double angle, const double mag)
  {
    x = x_start + cos(angle) * mag;
    y = y_start + sin(angle) * mag;
  }
  const static double MAX_ANGLE = 1 * M_PI;
  const static double MIN_ANGLE = -1 * M_PI;

  double addAngles(const double a1, const double a2)
  {
    double angleRtrn = a1 + a2;
    if (angleRtrn > MAX_ANGLE) {
      angleRtrn -= 2 * M_PI;
    }
    if (angleRtrn < MIN_ANGLE) {
      angleRtrn += 2 * M_PI;
    }
    return angleRtrn;
  }

  const static bool REMOVE_LONG_RANGE_HITS = true;
  const static double LONG_RANGE_HIT = 5; //the furthest hit we should consider.

  /*
   * Get the laser hits in x and y coordinates
   *
   */
  void getLaserHits(const sensor_msgs::LaserScan::ConstPtr& msg, //incoming msg
          std::vector<double>& x_strikes, std::vector<double>& y_strikes, // return value
          const double robot_x, const double robot_y, const double robot_rotation)
  {
    //gets the x/y coordinates of laser


    double minRads = msg->angle_min;
    double maxRads = msg->angle_max;
    double maxDist = msg->range_max;
    double minDist = msg->range_min;
    unsigned int minIndex = 0;
    //unsigned int maxIndex = ceil ((maxRads - minRads) / msg->angle_increment);
    unsigned int maxIndexSize = msg->ranges.size();
    //ROS_INFO_STREAM("max: " << maxIndex);
    x_strikes.clear();
    y_strikes.clear();
    for (unsigned int currIndex = minIndex + 1; currIndex < maxIndexSize; currIndex++) {


      double angleFromStart = ((double) currIndex) * msg->angle_increment;
      double angle = minRads + angleFromStart;
      //add angles
      double trueAngle = addAngles(angle, robot_rotation);


      double x_hit = DEFAULT_LASER_STRIKE;
      double y_hit = DEFAULT_LASER_STRIKE;
      double mag = msg->ranges[currIndex];

      if (REMOVE_LONG_RANGE_HITS && mag > LONG_RANGE_HIT || !REMOVE_LONG_RANGE_HITS) {
        convertAngleToXY(x_hit, y_hit, robot_x, robot_y, trueAngle, mag);
        x_strikes.push_back(x_hit);
        y_strikes.push_back(y_hit);
      }

    }//end for loop
  } //end get Laser Hits


}; //END UTILS


using namespace boost::posix_time;

class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics

  GridMapper(ros::NodeHandle& nh, int width, int height) :
  canvas(height, width, CV_8UC1)
  {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &GridMapper::laserCallback, this);

    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("base_pose_ground_truth", 1, \
      &GridMapper::poseCallback, this);

    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };


  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed

  void saveSnapshot()
  {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };


  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)

  void plot(int x, int y, char value)
  {
    canvasMutex.lock();
    x += canvas.rows / 2;
    y += canvas.cols / 2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)

  void plotImg(int x, int y, char value)
  {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command

  void move(double linearVelMPS, double angularVelRadPS)
  {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)



  };


  // Process incoming ground truth robot pose message

  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double roll, pitch;
    x = -msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading = tf::getYaw(msg->pose.pose.orientation);
  };


  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state

  void spin()
  {
    int key = 0;

    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove following demo code and make robot move around the environment
      plot(x, y, CELL_ROBOT); // Demo code: plot robot's current position on canvas
      plotImg(0, 0, CELL_OCCUPIED); // Demo code: plot different colors at 4 canvas corners
      plotImg(0, canvas.rows - 1, CELL_UNKNOWN);
      plotImg(canvas.cols - 1, 0, CELL_FREE);
      plotImg(canvas.cols - 1, canvas.rows - 1, CELL_ROBOT);

      if (USE_RANDOM_WALK) {
        
      }


      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000 / SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }

    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  const static bool USE_RANDOM_WALK = true;
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI / 2;

  const static int SPIN_RATE_HZ = 30;

  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};

int main(int argc, char **argv)
{
  int width, height;
  bool printUsage = false;

  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) {
        printUsage = true;
      } else if (height <= 0) {
        printUsage = true;
      }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robbie(n, width, height); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
