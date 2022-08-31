#include <fmt/core.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <unordered_set>

#include "argparse.hpp"
#include "json.hpp"
#include "rapidcsv.h"

#define LOG(...) printf(__VA_ARGS__);

typedef cv::Vec<float, 1> Vec1f;
typedef cv::Vec<int, 1> Vec1i;

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
float distance(const VecT& p1, const VecT& p2) {
  typedef cv::Vec<float, VecT::channels> VecTf;
  auto diff = p2 - p1;
  return diff.dot(diff);
}

template <typename VecT>
auto cluster(std::vector<VecT> points, int num_of_clusters, int perf_factor) {
  std::vector<VecT> filtered_points;
  std::vector<float> cluster_data_distances(points.size());

  filtered_points.emplace_back(points.at(0));
  for (int i = 1; i < points.size(); ++i) {
    cluster_data_distances[i] = distance(points.at(0), points.at(i));
  }

  for (int i = 1; i < num_of_clusters; ++i) {
    auto new_head_index = -1;
    auto new_head_distance = 0.f;
    for (int j = i % perf_factor; j < points.size(); j += perf_factor) {
      if (new_head_distance < cluster_data_distances[j]) {
        new_head_index = j;
        new_head_distance = cluster_data_distances[j];
      }
    }

    filtered_points.emplace_back(points.at(new_head_index));

    for (int j = i % perf_factor; j < points.size(); j += perf_factor) {
      auto new_distance = distance(points.at(new_head_index), points.at(j));
      if (new_distance < cluster_data_distances[j]) {
        cluster_data_distances[j] = new_distance;
      }
    }
  }
  return filtered_points;
}

auto final_results = nlohmann::json();

void clusterImage(const std::string& base_path, const std::string& output_path,
                  const std::string& image_name_ext,
                  const std::vector<int>& cluster_f,
                  const std::vector<int>& perf_f, int repeat_f) {
  auto image_name = image_name_ext.substr(0, image_name_ext.find('.'));

  LOG("Loading image %s\n", image_name_ext.c_str());
  cv::Mat image = cv::imread(base_path + "/" + image_name_ext);
  std::set<cv::Vec3b, VecCompare<cv::Vec3b>> image_unique_colors_set(
      image.begin<cv::Vec3b>(), image.end<cv::Vec3b>());
  std::vector<cv::Vec3f> image_unique_colors(image_unique_colors_set.begin(),
                                             image_unique_colors_set.end());
  LOG("Loaded image with size %dx%d and %d unique colors\n", image.rows,
      image.cols, image_unique_colors.size());

  nlohmann::json res;
  for (const auto& c_f : cluster_f) {
    auto num_of_clusters = image_unique_colors.size() / c_f;
    for (const auto& p_f : perf_f) {
      std::vector<long> durations;
      for (int i = 0; i < repeat_f; ++i) {
        auto start_time = std::chrono::steady_clock::now();
        auto clustered_points =
            cluster<cv::Vec3f>(image_unique_colors, num_of_clusters, p_f);
        auto end_time = std::chrono::steady_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                              end_time - start_time)
                              .count();
        LOG("Finished clustering %s with c:%d p:%d num_of_points: %d %d/%d "
            "duration: %lld\n",
            image_name_ext.c_str(), c_f, p_f, num_of_clusters, i + 1, repeat_f,
            total_time);

        std::map<cv::Vec3b, cv::Vec3b, VecCompare<cv::Vec3b>> og_to_cluster_map;
        for (const auto& original_p : image_unique_colors) {
          cv::Vec3b closest_p;
          float min_distance = std::numeric_limits<float>::max();
          for (const auto& clustered_p : clustered_points) {
            auto dis = distance(clustered_p, original_p);
            if (dis < min_distance) {
              min_distance = dis;
              closest_p = clustered_p;
            }
          }
          og_to_cluster_map.emplace(original_p, closest_p);
        }

        cv::Mat output_image(image.size(), image.type());
        for (int i = 0; i < output_image.rows * output_image.cols; ++i) {
          output_image.at<cv::Vec3b>(i) =
              og_to_cluster_map.at(image.at<cv::Vec3b>(i));
        }

        if (!cv::imwrite(fmt::format("{}/{}#_c{}_p{}_{}.png", output_path,
                                     image_name, c_f, p_f, num_of_clusters),
                         output_image)) {
          LOG("Error writing image\n");
        }
        durations.emplace_back(total_time);
      }
      res[fmt::format("c{}_p{}", c_f, p_f)] = {
          {"num_of_points", num_of_clusters}, {"clustering_time", durations}};
    }
  }
  final_results[image_name] = res;
}

void clusterPC(const std::string& base_path, const std::string& output_path,
               const std::string& pc_name_ext,
               const std::vector<int>& cluster_f,
               const std::vector<int>& perf_f, int repeat_f) {
  auto pc_name = pc_name_ext.substr(0, pc_name_ext.find('.'));

  LOG("Loading image %s\n", pc_name_ext.c_str());

  rapidcsv::Document pc_csv(base_path + "/" + pc_name_ext,
                            rapidcsv::LabelParams(0, -1));

  std::vector<cv::Vec3f> pc;
  for (int i = 0; i < pc_csv.GetRowCount(); ++i) {
    auto row = pc_csv.GetRow<float>(i);
    pc.emplace_back(row[0], row[1], row[2]);
  }
  LOG("Loaded image with size %zu\n", pc.size());

  nlohmann::json res;
  for (const auto& c_f : cluster_f) {
    auto num_of_clusters = pc.size() / c_f;
    for (const auto& p_f : perf_f) {
      std::vector<long> durations;
      for (int i = 0; i < repeat_f; ++i) {
        auto start_time = std::chrono::steady_clock::now();
        auto clustered_points = cluster<cv::Vec3f>(pc, num_of_clusters, p_f);
        auto end_time = std::chrono::steady_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                              end_time - start_time)
                              .count();
        LOG("Finished clustering %s with c:%d p:%d num_of_points: %d %d/%d "
            "duration: %lld\n",
            pc_name_ext.c_str(), c_f, p_f, num_of_clusters, i + 1, repeat_f,
            total_time);

        durations.emplace_back(total_time);
      }
      res[fmt::format("c{}_p{}", c_f, p_f)] = {
          {"num_of_points", num_of_clusters}, {"clustering_time", durations}};
    }
  }
  final_results[pc_name] = res;
}

auto stringToIntVector(std::string string) {
  std::vector<int> res;
  size_t pos = 0;
  while ((pos = string.find(",")) != std::string::npos) {
    auto val = std::stoi(string.substr(0, pos));
    res.emplace_back(val);
    string.erase(0, pos + 1);
  }
  auto val = std::stoi(string.substr(0, pos));
  res.emplace_back(val);
  return res;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("ClusterPC");

  program.add_argument("--base_path");
  program.add_argument("--output_path");
  program.add_argument("-c_f");
  program.add_argument("-p_f");

  try {
    program.parse_args(argc, argv);
    auto base_path = program.get("--base_path");
    auto output_path = program.get("--output_path");
    auto cluster_f = stringToIntVector(program.get("-c_f"));
    auto perf_f = stringToIntVector(program.get("-p_f"));

    for (const auto& image_path :
         std::filesystem::directory_iterator(base_path)) {
      clusterImage(base_path, output_path, image_path.path().filename().string(),
                   cluster_f, perf_f, 1);
    }
  } catch (const std::exception& e) {
    fmt::print("{}\n", e.what());
  }

  return 0;
}
