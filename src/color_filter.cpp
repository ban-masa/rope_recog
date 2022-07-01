#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <rope_recog/ColorFilterConfig.h>


void printmatinfo(cv::Mat& m)
{
  std::cerr << "col: " << m.cols << std::endl;
  std::cerr << "row: " << m.rows << std::endl;
  std::cerr << "type: " << m.type() << std::endl;
}

class ColorFilter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher hist_pub_;

  dynamic_reconfigure::Server<rope_recog::ColorFilterConfig> server;
  dynamic_reconfigure::Server<rope_recog::ColorFilterConfig>::CallbackType f;

  int h_ulimit, h_llimit, s_ulimit, s_llimit, v_ulimit, v_llimit;
  cv::Mat input_img;
  cv::Mat store_img;
  cv::Mat store_img2;

  cv::MatND hist;

  std::array<cv::Mat, 4> result_img_array;
  bool initial_flag;
public:
  ColorFilter(ros::NodeHandle nh) : nh_(nh), it_(nh_)
  {
    image_sub_ = it_.subscribe("image_raw", 1, &ColorFilter::imageCb, this);
    image_pub_ = it_.advertise("output_video", 1);
    hist_pub_ = it_.advertise("output_histogram", 1);

    f = boost::bind(&ColorFilter::configureCb, this, _1, _2);
    server.setCallback(f);

    int rope_type = 1;
    if (rope_type == 0) {
      h_ulimit = 130;
      h_llimit = 100;
      s_ulimit = 256;
      s_llimit = 0;
      v_ulimit = 256;
      v_llimit = 0;
    } else if (rope_type == 1) {
      h_ulimit = 180;
      h_llimit = 0;
      s_ulimit = 30;
      s_llimit = 0;
      v_ulimit = 256;
      v_llimit = 150;
    }
    std::cout << "initialize" << std::endl;

    initial_flag = true;

  }

  ~ColorFilter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    if (initial_flag) {
      initial_flag = false;
      printmatinfo(cv_ptr->image);
    }

    cv::Mat hsv_image, color_mask, result;
    cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
    cv::cvtColor(cv_ptr->image, input_img, CV_BGR2HSV);

    cv::Mat black_img(hsv_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < 4; i++) black_img.copyTo(result_img_array.at(i));

    cv::cvtColor(hsv_image, result_img_array.at(0), CV_HSV2BGR);

    cv::inRange(hsv_image, cv::Scalar(h_llimit, s_llimit, v_llimit), cv::Scalar(h_ulimit, s_ulimit, v_ulimit), color_mask);

    cv_bridge::CvImagePtr cv_ptr2;
    cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //cv::cvtColor(hsv_image, result, CV_HSV2BGR);
    //cv::cvtColor(color_mask, cv_ptr2->image, CV_GRAY2BGR);

    //color_mask
    //movingTrimming
    hsv_image.copyTo(result, color_mask);
    cv::cvtColor(result, result_img_array.at(1), CV_HSV2BGR);
    //cannyEdge(result, cv_ptr2->image);
    //movingTrimming(result, cv_ptr2->image);
    cv::Mat trimmed_img;
    //movingTrimming(result, trimmed_img);
    movingTrimming2(hsv_image, trimmed_img);
    //cv::cvtColor(result, cv_ptr2->image, CV_HSV2BGR);

    autoFilter2();

    combinedResultImg(cv_ptr2->image);
    image_pub_.publish(cv_ptr2->toImageMsg());
    

  }

  void combinedResultImg(cv::Mat& output_img)
  {
    cv::putText(result_img_array.at(0), "orig_img", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, CV_AA);
    cv::putText(result_img_array.at(1), "color_mask", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, CV_AA);
    cv::putText(result_img_array.at(2), "motion_trim", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, CV_AA);
    cv::putText(result_img_array.at(3), "mosaic", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, CV_AA);

    cv::Mat img1;
    cv::hconcat(result_img_array.at(0), result_img_array.at(1), img1);
    cv::Mat img2;
    cv::hconcat(result_img_array.at(2), result_img_array.at(3), img2);
    cv::Mat combined_img;
    cv::vconcat(img1, img2, combined_img);
    cv::resize(combined_img, output_img, cv::Size(0, 0), 0.5, 0.5, cv::INTER_AREA);
  }

  void cannyEdge(cv::Mat& src_img, cv::Mat& output_img)
  {
    cv::Mat bgr_img, gray_img;
    cv::cvtColor(src_img, bgr_img, CV_HSV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);

    /*
    cv::Mat tmp_img, sobel_img;
    cv::Sobel(gray_img, tmp_img, CV_32F, 1, 1);
    cv::convertScaleAbs(tmp_img, sobel_img, 1, 0);

    cv::Mat laplacian_img;
    cv::Laplacian(gray_img, tmp_img, CV_32F, 3);
    cv::convertScaleAbs(tmp_img, laplacian_img, 1, 0);
    */

    cv::Mat canny_img;
    cv::Canny(gray_img, canny_img, 50, 200);

    cv::cvtColor(canny_img, output_img, CV_GRAY2BGR);
  }

  void movingTrimming(cv::Mat& src_img, cv::Mat& output_img)
  {
    cv::Mat bgr_img, gray_img;
    cv::cvtColor(src_img, bgr_img, CV_HSV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::Mat gray_float_img;
    gray_img.convertTo(gray_float_img, CV_32FC1);

    if (store_img.empty()) {
      gray_float_img.copyTo(store_img);
    }
    cv::Mat diff_img(gray_img.size(), CV_32FC1, cv::Scalar(0));
    cv::accumulateWeighted(gray_img, store_img, 0.5);
    cv::absdiff(gray_float_img, store_img, diff_img);
    diff_img.convertTo(gray_img, CV_8UC1);
    cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);
    cv::morphologyEx(gray_img, gray_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 10)));
    //cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);

    cv::morphologyEx(gray_img, gray_img, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 10)));
    //cv::cvtColor(gray_img, result_img_array.at(2), CV_GRAY2BGR);
    cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);


    cv::Mat result_img(bgr_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    result_img.copyTo(output_img);
    //cv::cvtColor(gray_img, result_img, CV_GRAY2BGR);
    bgr_img.copyTo(output_img, gray_img);
    //cv::cvtColor(gray_img, output_img, CV_GRAY2BGR);

    //output_img.copyTo(result_img_array.at(2));
    cv::Mat output_gray_img;
    cv::cvtColor(output_img, output_gray_img, CV_BGR2GRAY);

    //cv::morphologyEx(output_gray_img, output_gray_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::Mat resized_output_img;
    double resize_scale = 0.2;
    cv::resize(output_gray_img, resized_output_img, cv::Size(0, 0), resize_scale, resize_scale, cv::INTER_AREA);
    cv::threshold(resized_output_img, resized_output_img, 30, 255, cv::THRESH_BINARY);
    cv::resize(resized_output_img, output_gray_img, cv::Size(0, 0), 1.0 / resize_scale, 1.0 / resize_scale, cv::INTER_LINEAR);
    cv::threshold(output_gray_img, output_gray_img, 50, 255, cv::THRESH_BINARY);

    /*
    cv::resize(output_gray_img, resized_output_img, cv::Size(0, 0), 1, resize_scale, cv::INTER_AREA);
    cv::threshold(resized_output_img, resized_output_img, 30, 255, cv::THRESH_BINARY);
    cv::resize(resized_output_img, output_gray_img, cv::Size(0, 0), 1, 1.0 / resize_scale, cv::INTER_LINEAR);
    cv::threshold(output_gray_img, output_gray_img, 50, 255, cv::THRESH_BINARY);
    */


    cv::Mat canny_img;
    cv::Canny(output_gray_img, canny_img, 50, 200);

    //cv::cvtColor(output_gray_img, result_img_array.at(3), CV_GRAY2BGR);
    threshArea(output_gray_img, result_img_array.at(3));

    result_img.copyTo(output_img);
    bgr_img.copyTo(output_img, output_gray_img);
    cv::Mat blue_img(bgr_img.size(), CV_8UC3, cv::Scalar(255, 0, 0));
    blue_img.copyTo(output_img, canny_img);
  }

  void movingTrimming2(cv::Mat& src_img, cv::Mat& output_img)
  {
    cv::Mat bgr_img, gray_img;
    cv::cvtColor(src_img, bgr_img, CV_HSV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::Mat gray_float_img;
    gray_img.convertTo(gray_float_img, CV_32FC1);

    if (store_img.empty()) {
      gray_float_img.copyTo(store_img);
    }
    cv::Mat diff_img(gray_img.size(), CV_32FC1, cv::Scalar(0));
    cv::accumulateWeighted(gray_img, store_img, 0.5);
    cv::absdiff(gray_float_img, store_img, diff_img);
    diff_img.convertTo(gray_img, CV_8UC1);
    cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);

    cv::Mat trimmed_img;
    src_img.copyTo(trimmed_img, gray_img);
    cv::inRange(trimmed_img, cv::Scalar(h_llimit, s_llimit, v_llimit), cv::Scalar(h_ulimit, s_ulimit, v_ulimit), gray_img);

    cv::morphologyEx(gray_img, gray_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 10)));
    //cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);

    cv::morphologyEx(gray_img, gray_img, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 10)));
    //cv::cvtColor(gray_img, result_img_array.at(2), CV_GRAY2BGR);
    cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);


    cv::Mat result_img(bgr_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    result_img.copyTo(output_img);
    //cv::cvtColor(gray_img, result_img, CV_GRAY2BGR);
    bgr_img.copyTo(output_img, gray_img);
    //cv::cvtColor(gray_img, output_img, CV_GRAY2BGR);

    //output_img.copyTo(result_img_array.at(2));
    cv::Mat output_gray_img;
    cv::cvtColor(output_img, output_gray_img, CV_BGR2GRAY);

    //cv::morphologyEx(output_gray_img, output_gray_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::Mat resized_output_img;
    double resize_scale = 0.2;
    cv::resize(output_gray_img, resized_output_img, cv::Size(0, 0), resize_scale, resize_scale, cv::INTER_AREA);
    cv::threshold(resized_output_img, resized_output_img, 30, 255, cv::THRESH_BINARY);
    cv::resize(resized_output_img, output_gray_img, cv::Size(0, 0), 1.0 / resize_scale, 1.0 / resize_scale, cv::INTER_LINEAR);
    cv::threshold(output_gray_img, output_gray_img, 50, 255, cv::THRESH_BINARY);

    /*
    cv::resize(output_gray_img, resized_output_img, cv::Size(0, 0), 1, resize_scale, cv::INTER_AREA);
    cv::threshold(resized_output_img, resized_output_img, 30, 255, cv::THRESH_BINARY);
    cv::resize(resized_output_img, output_gray_img, cv::Size(0, 0), 1, 1.0 / resize_scale, cv::INTER_LINEAR);
    cv::threshold(output_gray_img, output_gray_img, 50, 255, cv::THRESH_BINARY);
    */


    cv::Mat canny_img;
    cv::Canny(output_gray_img, canny_img, 50, 200);

    //cv::cvtColor(output_gray_img, result_img_array.at(3), CV_GRAY2BGR);
    threshArea(output_gray_img, result_img_array.at(3));

    result_img.copyTo(output_img);
    bgr_img.copyTo(output_img, output_gray_img);
    cv::Mat blue_img(bgr_img.size(), CV_8UC3, cv::Scalar(255, 0, 0));
    blue_img.copyTo(output_img, canny_img);
  }

  void lineDetection(cv::Mat& src_img, cv::Mat& output_img)
  {
    cv::Mat canny_img;
    cv::Canny(src_img, canny_img, 50, 200);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(canny_img, lines, 1, CV_PI / 180, 80, 30, 10);
    cv::cvtColor(canny_img, output_img, CV_GRAY2BGR);
    for (int i = 0; i < lines.size(); i++) {
      cv::line(output_img, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
    }
  }

  void threshArea(cv::Mat& src_img, cv::Mat& output_img)
  {
    cv::Mat rope_img(src_img.size(), CV_8UC1, cv::Scalar(0));

    cv::cvtColor(src_img, output_img, CV_GRAY2BGR);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(src_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point> > selected_contours;

    for (int i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i]);
      cv::RotatedRect rrect = cv::minAreaRect(contours[i]);
      float w = rrect.size.width;
      float h = rrect.size.height;
      float angle = rrect.angle;

      float rect_ratio = 2.0;
      bool shape_check_flag = false;

      /*
      if (h > (w * rect_ratio)) {
        if (((angle <= 45) && (angle >= 0)) || ((angle <= 360) && (angle >= 315))) shape_check_flag = true;
      }
      */
      if ((h > w * rect_ratio) || (w > h * rect_ratio)) {
        shape_check_flag = true;
      }
      if ((area > 100.0)) {// && shape_check_flag) {
        selected_contours.push_back(contours[i]);
        cv::Point2f vertices2f[4];
        rrect.points(vertices2f);
        std::vector<cv::Point> vertices;
        for (int i = 0; i < 4; i++) vertices.push_back(vertices2f[i]);
        const cv::Point* pts = (const cv::Point*)cv::Mat(vertices).data;
        int npts = cv::Mat(vertices).rows;
        cv::polylines(output_img, &pts, &npts, 1, true, cv::Scalar(0, 0, 255), 3);
      }
    }
    cv::drawContours(output_img, selected_contours, -1, cv::Scalar(0, 255, 0), CV_FILLED);
    cv::drawContours(rope_img, selected_contours, -1, cv::Scalar(255), CV_FILLED);

    std::vector<std::array<int, 2> > rope_point_list;
    ropeShapeEstimation(rope_img, rope_point_list);

    for (int i = 0; i < rope_point_list.size(); i++) {
      std::array<int, 2> p = rope_point_list[i];
      cv::circle(output_img, cv::Point(p.at(0), p.at(1)), 5, cv::Scalar(0, 0, 255), -1);
    }
    for (int i = 0; i < ((int)rope_point_list.size() - 1); i++) {
      std::array<int, 2> p0 = rope_point_list[i];
      std::array<int, 2> p1 = rope_point_list[i + 1];
      cv::line(output_img, cv::Point(p0.at(0), p0.at(1)), cv::Point(p1.at(0), p1.at(1)), cv::Scalar(0, 0, 255), 3);
    }
  }

  void ropeShapeEstimation(cv::Mat& filtered_img, std::vector<std::array<int, 2> >& rope_point_list)
  {
    cv::Mat resized_img;
    cv::resize(filtered_img, resized_img, cv::Size(filtered_img.cols, 100));

    for (int y = 0; y < resized_img.rows; y++) {
      unsigned char* line_img = resized_img.ptr<unsigned char>(y);
      bool seq_flag = false;
      int startx;
      int max_len = -1;
      int centerx = -1;
      for (int x = 0; x < resized_img.cols; x++) {
        if(line_img[x] != 0) {
          if (!seq_flag) {
            seq_flag = true;
            startx = x;
          }
        } else {
          if (seq_flag) {
            seq_flag = false;
            if ((x - startx) > max_len) {
              max_len = x -startx;
              centerx = (int)(0.5 * (x + startx));
            }
          }
        }
      }
      if (max_len > 0) {
        std::array<int, 2> p;
        p.at(0) = centerx;
        p.at(1) = (int)(filtered_img.rows * y / 100.0);
        rope_point_list.push_back(p);
      }
    }
  }

  void autoFilter2(void)
  {
    cv::Mat bgr_img, gray_img;
    cv::cvtColor(input_img, bgr_img, CV_HSV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::Mat gray_float_img;
    gray_img.convertTo(gray_float_img, CV_32FC1);

    if (store_img2.empty()) {
      gray_float_img.copyTo(store_img2);
    }

    cv::Mat diff_img(gray_img.size(), CV_32FC1, cv::Scalar(0));
    cv::accumulateWeighted(gray_img, store_img2, 0.5);
    cv::absdiff(gray_float_img, store_img2, diff_img);
    diff_img.convertTo(gray_img, CV_8UC1);
    cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);
    //opening
    cv::morphologyEx(gray_img, gray_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    cv::Mat result_img(bgr_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::cvtColor(gray_img, result_img, CV_GRAY2BGR);
    bgr_img.copyTo(result_img, gray_img);
    result_img.copyTo(result_img_array.at(2));

  }

  void autoFilter(void)
  {
    cv::Mat bgr_img, gray_img;
    cv::cvtColor(input_img, bgr_img, CV_HSV2BGR);
    cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
    cv::Mat gray_float_img;
    gray_img.convertTo(gray_float_img, CV_32FC1);

    if (store_img.empty()) {
      gray_float_img.copyTo(store_img);
    }

    cv::Mat diff_img(gray_img.size(), CV_32FC1, cv::Scalar(0));
    cv::accumulateWeighted(gray_img, store_img, 0.5);
    cv::absdiff(gray_float_img, store_img, diff_img);


    //cv::cvtColor(gray_img, cv_ptr->image, CV_GRAY2BGR);
    //cv::cvtColor(input_img, cv_ptr->image, CV_HSV2BGR);

    diff_img.convertTo(gray_img, CV_8UC1);
    //opening
    cv::morphologyEx(gray_img, gray_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    cv::threshold(gray_img, gray_img, 20, 255, cv::THRESH_BINARY);


    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gray_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    /*
    cv::Mat ctr_img;
    bgr_img.copyTo(ctr_img);

    cv::drawContours(ctr_img, contours, -1, cv::Scalar(0, 0, 255), -1);
    */

    {//trimming
      double k = (double)gray_img.rows / (0.5 * 0.5 * gray_img.cols * gray_img.cols);

      for (int y = 0; y < gray_img.rows; y++) {
        for (int x = 0; x < gray_img.cols; x++) {
          unsigned char* src = gray_img.ptr<unsigned char>(y);
          double lx = x - 0.5 * gray_img.cols;
          double ly = gray_img.rows - y;
          if (k * lx * lx > ly) {
            src[x] = 0;
          }
          /*
          if (x < 0.5 * gray_img.cols && (y > x - 0.5 * gray_img.cols + gray_img.rows)) {
            src[x] = 0;
          }
          if (x > 0.5 * gray_img.cols && (y > -x + 0.5 * gray_img.cols + gray_img.rows)) {
            src[x] = 0;
          }
          */
        }
      }
    }
    
    cv::Mat result_img(bgr_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::cvtColor(gray_img, result_img, CV_GRAY2BGR);
    bgr_img.copyTo(result_img, gray_img);

    int channels[] = {0, 1, 2};
    int histSize[] = {20, 20, 20};
    float hranges[] = {0, 180};
    float sranges[] = {0, 256};
    float vranges[] = {0, 256};
    const float* ranges[] = {hranges, sranges, vranges};

    cv::Mat hsv_result_img;
    cv::cvtColor(result_img, hsv_result_img, CV_BGR2HSV);

    cv::calcHist(&hsv_result_img, 1, channels, gray_img, hist, 3, histSize, ranges, true, true);

    std::stringstream hist_ss;

    float max_val = -100;
    int max_pos[3] = {0, 0, 0};
    for (int i = 0; i < histSize[0]; i++) {
      for (int j = 0; j < histSize[1]; j++) {
        for (int k = 0; k < histSize[2]; k++) {
          float val = hist.at<float>(i, j, k);
          if (val > max_val) {
            max_val = val;
            max_pos[0] = i;
            max_pos[1] = j;
            max_pos[2] = k;
          }
        }
      }
    }
    int max_hsv[3];
    for (int i = 0; i < 3; i++) {
      max_hsv[i] = (int)((float)max_pos[i] / (float)histSize[i] * ranges[i][1]);
    }
    hist_ss << max_hsv[0] << ", " << max_hsv[1] << ", " << max_hsv[2];

    int lower_limit[3];
    int upper_limit[3];
    for (int i = 0; i < 3; i++) {
      lower_limit[i] = (int)(max_hsv[i] - 0.5 * ranges[i][1] / (float)histSize[i]);
      upper_limit[i] = (int)(max_hsv[i] + 0.5 * ranges[i][1] / (float)histSize[i]);
    }
    /*
    if (max_hsv[1] < 50) {
      lower_limit[0] = 0;
      upper_limit[0] = 180;
    }
    */
    cv::Mat filtered_img, color_mask;
    cv::inRange(hsv_result_img, cv::Scalar(lower_limit[0], lower_limit[1], lower_limit[2]), cv::Scalar(upper_limit[0], upper_limit[1], upper_limit[2]), color_mask);
    hsv_result_img.copyTo(filtered_img, color_mask);

    cv::putText(filtered_img, hist_ss.str(), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2, CV_AA);

    cv::cvtColor(filtered_img, filtered_img, CV_HSV2BGR);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, filtered_img).toImageMsg();
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, ctr_img).toImageMsg();
    image_pub_.publish(msg);
  }

  void configureCb(rope_recog::ColorFilterConfig& config, uint32_t level)
  {
    h_ulimit = config.h_ulimit;
    h_llimit = config.h_llimit;
    s_ulimit = config.s_ulimit;
    s_llimit = config.s_llimit;
    v_ulimit = config.v_ulimit;
    v_llimit = config.v_llimit;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_filter");
  ros::NodeHandle nh("~");
  ColorFilter cf(nh);
  ros::spin();
  return 0;
}
