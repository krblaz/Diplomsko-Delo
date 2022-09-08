#ifndef PLYTEST_CLUSTERING_H
#define PLYTEST_CLUSTERING_H
#include <rapidcsv.h>
#include <spdlog/spdlog.h>
#include <fmt/ostream.h>

#include <json.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

typedef cv::Vec<float, 1> Vec1f;
typedef cv::Vec<int, 1> Vec1i;

inline auto final_results = nlohmann::json();

struct PointRGB {
  int cluster_num = 0;
  float cluster_distance = 0;
  bool cluster_head = false;
  cv::Vec3f pos;

  PointRGB() {}
  PointRGB(const cv::Vec3f& vec) : pos(vec) {}
};

struct PointXYZRGB : PointRGB {};

float distance(const PointRGB& p1, const PointRGB& p2);

template <typename VecT>
struct VecCompare {
  auto operator()(const VecT& v1, const VecT& v2) const {
    for (int i = 0; i < v1.rows; i++) {
      if (v1[i] < v2[i]) return true;
      if (v1[i] > v2[i]) return false;
    }
    return false;
  }
};

void clusterImage(const std::string& base_path, const std::string& output_path, const std::string& image_name_ext,
                  const std::vector<int>& cluster_f, const std::vector<int>& perf_f, int repeat_f, bool combs = false,
                  bool offset_start = true);

void clusterPC(const std::string& base_path, const std::string& output_path, const std::string& pc_name_ext,
               const std::vector<int>& cluster_f, const std::vector<int>& perf_f, int repeat_f, bool combs = false,
               bool offset_start = true);

#endif  // PLYTEST_CLUSTERING_H
