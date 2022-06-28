#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

#define POINT_NUM_THR 500
#define COLOR_DIST 20
#define HSV_MODE 1

class ColorNode
{
  bool is_child;
  ColorNode* left;
  ColorNode* right;
  double left_right_dist;
  std::array<double, 3> centroid;
  int point_num;
public:
  ColorNode(double x, double y, double z, int p_num): left(NULL), right(NULL), point_num(p_num), is_child(false), left_right_dist(0.0)
  {
    centroid.at(0) = x;
    centroid.at(1) = y;
    centroid.at(2) = z;
  }

  ColorNode(ColorNode& n1, ColorNode& n2): is_child(false)
  {
    left = &n1;
    right = &n2;
    point_num = n1.getPointnum() + n2.getPointnum();
    std::array<double, 3> c1 = n1.getCentroid();
    std::array<double, 3> c2 = n2.getCentroid();
    for (int i = 0; i < 3; i++) {
      centroid.at(i) = (c1.at(i) * n1.getPointnum() + c2.at(i) * n2.getPointnum()) / point_num;
    }
    left_right_dist = std::sqrt(std::pow((c1.at(0) - c2.at(0)), 2)
                              + std::pow((c1.at(1) - c2.at(1)), 2)
                              + std::pow((c1.at(2) - c2.at(2)), 2));
  }

  std::array<double, 3> getCentroid(void)
  {
    return centroid;
  }

  int getPointnum(void)
  {
    return point_num;
  }

  double getLeftRightDistance(void)
  {
    return left_right_dist;
  }
  
  ColorNode* getLeft(void)
  {
    return left;
  }

  /*
  void setLeft(ColorNode* l)
  {
    left = l;
  }
  */

  ColorNode* getRight(void)
  {
    return right;
  }

  /*
  void setRight(ColorNode* r)
  {
    right = r;
  }
  */

  bool isChild(void)
  {
    return is_child;
  }

  void setChildFlag(bool flag)
  {
    is_child = flag;
  }

};

double calcCentroidDistance(ColorNode& n1, ColorNode& n2)
{
  std::array<double, 3> c1 = n1.getCentroid();
  std::array<double, 3> c2 = n2.getCentroid();
  double dist = std::sqrt(std::pow((c1.at(0) - c2.at(0)), 2)
                        + std::pow((c1.at(1) - c2.at(1)), 2)
                        + std::pow((c1.at(2) - c2.at(2)), 2));
  return dist;
}

void updateNodeTree(std::vector<ColorNode>& node_tree)
{
  int tree_size = node_tree.size();
  double min_dist = -1;
  int idx1, idx2;
  for (int i = 0; i < tree_size; i++) {
    if (node_tree[i].isChild()) continue;
    for (int j = i + 1; j < tree_size; j++) {
      if (node_tree[j].isChild()) continue;
      double dist = calcCentroidDistance(node_tree[i], node_tree[j]);
      if ((min_dist < 0) || (dist < min_dist)) {
        if (node_tree[i].getPointnum() > node_tree[j].getPointnum()) {
          idx1 = i;
          idx2 = j;
        } else {
          idx2 = i;
          idx1 = j;
        }
        min_dist = dist;
      }
    }
  }
  node_tree[idx1].setChildFlag(true);
  node_tree[idx2].setChildFlag(true);
  ColorNode new_node(node_tree[idx1], node_tree[idx2]);
  node_tree.push_back(new_node);
}

void printNodeInfo(ColorNode& n)
{
  std::cout << "point_num: " << n.getPointnum() << std::endl;
  std::array<double, 3> c = n.getCentroid();
  std::cout << "centroid: " << c.at(0) << ", " << c.at(1) << ", " << c.at(2) << std::endl;
  std::cout << "lr_dist: " << n.getLeftRightDistance() << std::endl;
  std::cout << std::endl;
}

void printNodeTreeInfo(ColorNode& top_node, std::vector<ColorNode>& result_tree)
{
  if (top_node.getLeftRightDistance() < COLOR_DIST) {
    printNodeInfo(top_node);
    result_tree.push_back(top_node);
    return;
  }
  bool last_flag = true;
  if (top_node.getLeft()->getPointnum() > POINT_NUM_THR) {
    printNodeTreeInfo(*top_node.getLeft(), result_tree);
    last_flag = false;
  }
  if (top_node.getRight()->getPointnum() > POINT_NUM_THR) {
    printNodeTreeInfo(*top_node.getRight(), result_tree);
    last_flag = false;
  }
  if (last_flag) {
    printNodeInfo(top_node);
    result_tree.push_back(top_node);
  }
  /*
  if (top_node.getLeftRightDistance() > COLOR_DIST) {
    printNodeTreeInfo(*top_node.getLeft(), result_tree);
    printNodeTreeInfo(*top_node.getRight(), result_tree);
  } else {
    printNodeInfo(top_node);
    result_tree.push_back(top_node);
  }
  */
}

void viewResult(cv::Mat& src_img, std::vector<ColorNode>& result_tree)
{
  for (int i = 0; i < result_tree.size(); i++) {
    cv::Mat color_mask;
    ColorNode& n = result_tree[i];
    std::array<double, 3> c = n.getCentroid();
#if HSV_MODE == 1
    double h = 2 * 180.0 / 30.0;
    double s = 2 * 256.0 / 30.0;
    cv::inRange(src_img, cv::Scalar(c.at(0) - h, c.at(1) - s, 0), cv::Scalar(c.at(0) + h, c.at(1) + s, 256), color_mask);
    cv::Mat ret_img(src_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat tmp_img;
    cv::cvtColor(src_img, tmp_img, CV_HSV2RGB);
    tmp_img.copyTo(ret_img, color_mask);
#else
    double r = 2 * 256.0 / 20.0;
    cv::inRange(src_img, cv::Scalar(c.at(0) - r, c.at(1) - r, c.at(2) - r), cv::Scalar(c.at(0) + r, c.at(1) + r, c.at(2) + r), color_mask);
    cv::Mat ret_img(src_img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    src_img.copyTo(ret_img, color_mask);
#endif

    std::stringstream window_ss;
    window_ss << "window:" << i;
    cv::namedWindow(window_ss.str(), 1);
    cv::imshow(window_ss.str(), ret_img);
  }

  cv::waitKey();
}

void colorClusteringHSV(cv::Mat& src_img, cv::Mat& mask_img)
{
  if ((src_img.rows != mask_img.rows) || (src_img.cols != mask_img.cols)) return;

  std::vector<ColorNode> node_tree;

  int channels[] = {0, 1};
  int histSize[] = {30, 30};
  float hrange[] = {0, 180};
  float srange[] = {0, 256};
  float vrange[] = {0, 256};
  const float* ranges[] = {hrange, srange};
  cv::MatND hist;
  cv::calcHist(&src_img, 1, channels, mask_img, hist, 2, histSize, ranges, true, false);

  for (int y = 0; y < histSize[1]; y++) {
    for (int x = 0; x < histSize[0]; x++) {
      float binVal = hist.at<float>(y, x);
      if (binVal > 5) {
        ColorNode c((x + 0.5) * ranges[0][1] / histSize[0], (y + 0.5) * ranges[1][1] / histSize[1], 0, (int)binVal);
        node_tree.push_back(c);
      }
    }
  }

  int all_elm_num = node_tree.size();

  for (int i = 1; i < all_elm_num; i++) {
    updateNodeTree(node_tree);
  }

  std::vector<ColorNode> result_tree;
  ColorNode& top_node = node_tree[node_tree.size() - 1];
  printNodeTreeInfo(top_node, result_tree);
  viewResult(src_img, result_tree);
}

void colorClusteringHSV(cv::Mat& src_img)
{
  cv::Mat mask_img(src_img.size(), CV_8UC1, cv::Scalar(255));
  colorClusteringHSV(src_img, mask_img);
}

void colorClustering(cv::Mat& src_img, cv::Mat& mask_img)
{
  if ((src_img.rows != mask_img.rows) || (src_img.cols != mask_img.cols)) return;

  std::vector<ColorNode> node_tree;

  int channels[] = {0, 1, 2};
  int histSize[] = {20, 20, 20};
  float hrange[] = {0, 256};
  float srange[] = {0, 256};
  float vrange[] = {0, 256};
  const float* ranges[] = {hrange, srange, vrange};
  cv::MatND hist;
  cv::calcHist(&src_img, 1, channels, mask_img, hist, 3, histSize, ranges, true, false);

  for (int z = 0; z < histSize[2]; z++) {
    for (int y = 0; y < histSize[1]; y++) {
      for (int x = 0; x < histSize[0]; x++) {
        float binVal = hist.at<float>(z, y, x);
        if (binVal > 5) {
          ColorNode c((x + 0.5) * ranges[0][1] / histSize[0], (y + 0.5) * ranges[1][1] / histSize[1], (z + 0.5) * ranges[2][1] / histSize[2], (int)binVal);
          node_tree.push_back(c);
        }
      }
    }
  }

  /*
  for (int y = 0; y < src_img.rows; y++) {
    for (int x = 0; x < src_img.cols; x++) {
      unsigned char* mask = mask_img.ptr<unsigned char>(y);
      cv::Vec3b *src = src_img.ptr<cv::Vec3b>(y);
      if (mask[x] != 0) {
        ColorNode c(src[x][0], src[x][1], src[x][2], 1);
        node_tree.push_back(c);
      }
    }
  }
  */

  int all_elm_num = node_tree.size();

  for (int i = 1; i < all_elm_num; i++) {
    updateNodeTree(node_tree);
  }

  std::vector<ColorNode> result_tree;
  ColorNode& top_node = node_tree[node_tree.size() - 1];
  printNodeTreeInfo(top_node, result_tree);
  viewResult(src_img, result_tree);
}

void colorClustering(cv::Mat& src_img)
{
  cv::Mat mask_img(src_img.size(), CV_8UC1, cv::Scalar(255));
  colorClustering(src_img, mask_img);
}

int main(void)
{
  cv::Mat image = cv::imread("/home/leus/Pictures/lake.jpg");
  cv::Mat src_img;
  cv::resize(image, src_img, cv::Size(0, 0), 0.25, 0.25, cv::INTER_AREA);
#if HSV_MODE == 1
  cv::cvtColor(src_img, src_img, CV_RGB2HSV);
  colorClusteringHSV(src_img);
#else
  colorClustering(src_img);
#endif
  return 0;
}
