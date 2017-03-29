/*****************************************************************

 MIT License

Copyright (c) 2017 CWRU Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************/ 

// intent: receive two camera streams, grab pairs, reset their time stamps
// and republish both.  This is to "spoof" the stereo process, which requires that
// images have nearly identical time stamps

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


// The ImageSyncher class is a lightweight class
// and main function used for synchronizing two independant ros camera streams.
class ImageSyncher
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_left_;
  image_transport::CameraSubscriber camera_sub_right_;
  image_transport::CameraPublisher camera_pub_left_;
  image_transport::CameraPublisher camera_pub_right_;

  sensor_msgs::Image img_pair_[2];
  sensor_msgs::CameraInfo info_pair_[2];

  int sequence_;

  bool got_new_image_pair_[2];

  // The image call back is used twice
  // once for the left image (lr = 0)
  // once for the right image (lr = 1)
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, int lr)
  {
    // get the incoming image size and resize the local image if necessary.
    int img_datasize = image_msg->data.size();
    int cpy_img_datasize = img_pair_[lr].data.size();

    if (cpy_img_datasize!= img_datasize)
    {
      img_pair_[lr].data.resize(img_datasize);
      ROS_INFO("resizing output image index %d", lr);
    }

    // deep copy the image data.
    img_pair_[lr] = *image_msg;

    // get the incoming camera info size and resize if necessary
    int info_datasize = info_msg->D.size();
    int cpy_info_datasize = info_pair_[lr].D.size();

    if (cpy_info_datasize != info_datasize)
    {
      info_pair_[lr].D.resize(info_datasize);
      ROS_INFO("resizing data info %d", lr);
    }

    // deep copy the camera info.
    info_pair_[lr] = *info_msg;

    // flag that the new image data is available.
    got_new_image_pair_[lr] = true;
  }

public:
  // get the status of the left and right image data.
  bool newImageLeft()
  {
    return got_new_image_pair_[0];
  }

  bool newImageRight()
  {
    return got_new_image_pair_[1];
  }

  // explicit constructor call.
  explicit ImageSyncher(ros::NodeHandle &nh): nh_(nh), it_(nh_),
    camera_sub_left_(it_.subscribeCamera("unsynced/left/image_raw", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&)>
      (boost::bind(&ImageSyncher::imageCb, this, _1, _2, 0)))),
    camera_sub_right_(it_.subscribeCamera("unsynced/right/image_raw", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&)>
      (boost::bind(&ImageSyncher::imageCb, this, _1, _2, 1)))),
    camera_pub_left_(it_.advertiseCamera("synced/left/image_raw", 1)),
    camera_pub_right_(it_.advertiseCamera("synced/right/image_raw", 1)),
    sequence_(0)
  {
    got_new_image_pair_[0] = false;
    got_new_image_pair_[1] = false;
  }

  // time synch and publish.
  // once the publishing is done, increment the sequence number.
  void pub_both_images()
  {
    // resets the time stamps of both images to be identicallly set to now.
    // be cautious when using this for time-sensitive situations.
    ros::Time tnow = ros::Time::now();

    for (int lr(0); lr < 2; lr++)
    {
      img_pair_[lr].header.stamp = tnow;
      info_pair_[lr].header.stamp = tnow;
      img_pair_[lr].header.seq = sequence_;
      info_pair_[lr].header.seq = sequence_;
    }

    camera_pub_left_.publish(img_pair_[0], info_pair_[0]);
    camera_pub_right_.publish(img_pair_[1], info_pair_[1]);

    got_new_image_pair_[0] = false;
    got_new_image_pair_[1] = false;
    sequence_++;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_synchronizer");
  ros::NodeHandle nh;

  // For safety, limit the speed to 120 Hz.
  ros::Rate ratetimer(120);


  ImageSyncher is(nh);
  while (ros::ok())
  {
    ros::spinOnce();
    if (is.newImageLeft() && is.newImageRight())
        is.pub_both_images();
    ratetimer.sleep();
  }
  return 0;
}
