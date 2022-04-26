#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"


namespace ph = std::placeholders;

namespace gb_dialog
{
class ExampleDF: public DialogInterface
{
  public:
    ExampleDF(): nh_()
    {
      sound_pub = nh_.advertise<std_msgs::String>("/sound/word", 1);
      this->registerCallback(std::bind(&ExampleDF::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&ExampleDF::welcomeIntentCB, this, ph::_1),
        "FMM");
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ExampleDF] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String msg;
      msg.data = "alone";
      ROS_INFO("[ExampleDF] welcomeIntentCB: intent [%s]", result.intent.c_str());
      sound_pub.publish(msg);
      speak(result.fulfillment_text);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher sound_pub;
    
};
}  // namespace gb_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_df_node");
  gb_dialog::ExampleDF forwarder;
  forwarder.listen();
  ros::spin();
  return 0;
}
