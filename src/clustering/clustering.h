#ifndef PLYTEST_CLUSTERING_H
#define PLYTEST_CLUSTERING_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <json.hpp>
#include <rapidcsv.h>

typedef cv::Vec<float, 1> Vec1f;
typedef cv::Vec<int, 1> Vec1i;

inline auto final_results = nlohmann::json();

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

template <typename VecT>
float distance(const VecT& p1, const VecT& p2);

void clusterImage(const std::string& base_path, const std::string& output_path, const std::string& image_name_ext,
                  const std::vector<int>& cluster_f, const std::vector<int>& perf_f, int repeat_f, bool combs = false,
                  bool offset_start = true);

void clusterPC(const std::string& base_path, const std::string& output_path, const std::string& pc_name_ext,
               const std::vector<int>& cluster_f, const std::vector<int>& perf_f, int repeat_f, bool combs = false,
               bool offset_start = true);

#endif  // PLYTEST_CLUSTERING_H
