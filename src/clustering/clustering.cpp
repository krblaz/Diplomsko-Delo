#include "clustering.h"

int LOOP_NUM;
int LOOP0_PERF = 1;
int LOOP1_PERF = 1;

extern "C" {
int CLANG_LOOP_PERFORATION_FUNCTION(int loopId) {
  switch (LOOP_NUM) {
    case 0:
      return LOOP0_PERF;
    case 1:
      return LOOP1_PERF;
    default:
      return 1;
  }
}
}

auto cluster(const std::vector<PointRGB>& points, int num_of_clusters) {
  std::vector<PointRGB> filtered_points(points);

  filtered_points[0].cluster_num = 0;
  filtered_points[0].cluster_head = true;
  for (int i = 1; i < filtered_points.size(); ++i) {
    filtered_points[i].cluster_distance = distance(filtered_points[0], filtered_points[i]);
    filtered_points[i].cluster_num = 0;
  }

  LOOP_NUM = 0;
  //#pragma clang loop perforate(enable)
  for (int i = 1; i < num_of_clusters; ++i) {
    auto new_head_index = -1;
    auto new_head_distance = 0.f;

    for (int j = 0; j < filtered_points.size(); ++j) {
      if (new_head_distance < filtered_points[j].cluster_distance) {
        new_head_index = j;
        new_head_distance = filtered_points[j].cluster_distance;
      }
    }

    filtered_points[new_head_index].cluster_num = i;
    filtered_points[new_head_index].cluster_head = true;

    LOOP_NUM = 1;
    //#pragma clang loop perforate(enable)
    for (int j = 0; j < filtered_points.size(); ++j) {
      auto new_distance = distance(filtered_points[new_head_index], filtered_points[j]);
      if (new_distance < filtered_points[j].cluster_distance) {
        filtered_points[j].cluster_distance = new_distance;
        filtered_points[j].cluster_num = i;
      }
    }
  }
  return filtered_points;
}

auto clusterOffset(const std::vector<PointRGB>& points, int num_of_clusters) {
  std::vector<PointRGB> filtered_points(points);

  filtered_points[0].cluster_num = 0;
  filtered_points[0].cluster_head = true;
  for (int i = 1; i < filtered_points.size(); ++i) {
    filtered_points[i].cluster_distance = distance(filtered_points[0], filtered_points[i]);
    filtered_points[i].cluster_num = 0;
  }

  for (int i = 1; i < num_of_clusters; ++i) {
    auto new_head_index = -1;
    auto new_head_distance = 0.f;

    for (int j = i % LOOP0_PERF; j < filtered_points.size(); j += LOOP0_PERF) {
      if (new_head_distance < filtered_points[j].cluster_distance) {
        new_head_index = j;
        new_head_distance = filtered_points[j].cluster_distance;
      }
    }

    filtered_points[new_head_index].cluster_num = i;
    filtered_points[new_head_index].cluster_head = true;

    for (int j = i % LOOP1_PERF; j < filtered_points.size(); j += LOOP1_PERF) {
      auto new_distance = distance(filtered_points[new_head_index], filtered_points[j]);
      if (new_distance < filtered_points[j].cluster_distance) {
        filtered_points[j].cluster_distance = new_distance;
        filtered_points[j].cluster_num = i;
      }
    }
  }
  return filtered_points;
}

void clusterImage(const std::string& base_path, const std::string& output_path, const std::string& image_name_ext,
                  const std::vector<int>& cluster_f, const std::vector<int>& perf_f, int repeat_f, bool combs,
                  bool offset_start) {
  auto image_name = image_name_ext.substr(0, image_name_ext.find('.'));

  spdlog::info("Loading image {}", image_name_ext);
  cv::Mat image = cv::imread(base_path + "/" + image_name_ext);
  std::set<cv::Vec3b, VecCompare<cv::Vec3b>> image_unique_colors_set(image.begin<cv::Vec3b>(), image.end<cv::Vec3b>());

  std::vector<PointRGB> image_unique_colors(image_unique_colors_set.size());
  std::transform(image_unique_colors_set.begin(), image_unique_colors_set.end(), image_unique_colors.begin(),
                 [](const cv::Vec3b& p) { return PointRGB(p); });

  spdlog::info("Loaded image with size {}x{} and {} unique colors", image.rows, image.cols, image_unique_colors.size());

  std::vector<std::array<int, 2>> perf_ff;
  if (combs) {
    for (const auto& p_f1 : perf_f) {
      for (const auto& p_f2 : perf_f) {
        perf_ff.emplace_back(std::array{p_f1, p_f2});
      }
    }
  } else {
    for (const auto& p_f : perf_f) {
      perf_ff.emplace_back(std::array{p_f, p_f});
    }
  }

  nlohmann::json res;
  for (const auto& c_f : cluster_f) {
    auto num_of_clusters = image_unique_colors.size() / c_f;
    for (const auto& p_f : perf_ff) {
      std::vector<long> durations;
      for (int repeat = 0; repeat < repeat_f; ++repeat) {
        LOOP0_PERF = p_f[0];
        LOOP1_PERF = p_f[0];

        auto start_time = std::chrono::steady_clock::now();
        auto clustered_points = offset_start ? clusterOffset(image_unique_colors, num_of_clusters)
                                             : cluster(image_unique_colors, num_of_clusters);
        auto end_time = std::chrono::steady_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        spdlog::info("Finished clustering {} {}/{} with c:{}, p1:{}, p2:{}, num_of_points:{}  duration: {}",
                     image_name_ext, repeat + 1, repeat_f, c_f, p_f[0], p_f[1], num_of_clusters, total_time);

        std::vector<cv::Vec3f> output_colors(num_of_clusters);
        std::vector<int> cluster_size(num_of_clusters);
        std::map<cv::Vec3b, int, VecCompare<cv::Vec3b>> og_to_cluster_map;
        for (const auto& point : clustered_points) {
          output_colors[point.cluster_num] += point.pos;
          cluster_size[point.cluster_num]++;
          og_to_cluster_map.emplace(point.pos, point.cluster_num);
        }

        for (int i = 0; i < output_colors.size(); ++i) {
          output_colors[i] /= cluster_size[i];
        }

        cv::Mat output_image(image.size(), image.type());
        for (int i = 0; i < output_image.rows * output_image.cols; ++i) {
          output_image.at<cv::Vec3b>(i) = output_colors.at(og_to_cluster_map.at(image.at<cv::Vec3b>(i)));
        }

        auto image_output_name =
            combs ? fmt::format("{}/{}#c{}_p{}_p{}.png", output_path, image_name, c_f, p_f[0], p_f[1])
                  : fmt::format("{}/{}#c{}_p{}.png", output_path, image_name, c_f, p_f[0]);

        if (!cv::imwrite(image_output_name, output_image)) {
          spdlog::error("Error writing image");
        }
        durations.emplace_back(total_time);
      }

      if (combs)
        res[fmt::format("c{}_p{},{}", c_f, p_f[0], p_f[1])] = {{"num_of_points", num_of_clusters},
                                                               {"clustering_time", durations}};
      else
        res[fmt::format("c{}_p{}", c_f, p_f[0])] = {{"num_of_points", num_of_clusters}, {"clustering_time", durations}};
    }
  }
  final_results[image_name] = res;
}

void clusterPC(const std::string& base_path, const std::string& output_path, const std::string& pc_name_ext,
               const std::vector<int>& cluster_f, const std::vector<int>& perf_f, int repeat_f, bool combs,
               bool offset_start) {
  auto pc_name = pc_name_ext.substr(0, pc_name_ext.find('.'));

  spdlog::info("Loading point cloud {}", pc_name_ext);

  rapidcsv::Document pc_csv(base_path + "/" + pc_name_ext, rapidcsv::LabelParams(0, -1));

  std::vector<PointRGB> pc;
  for (int i = 0; i < pc_csv.GetRowCount(); ++i) {
    auto row = pc_csv.GetRow<float>(i);
    pc.emplace_back(cv::Vec3f(row[0], row[1], row[2]));
  }
  spdlog::info("Loaded pc with size {}", pc.size());

  std::vector<std::array<int, 2>> perf_ff;
  if (combs) {
    for (const auto& p_f1 : perf_f) {
      for (const auto& p_f2 : perf_f) {
        perf_ff.emplace_back(std::array{p_f1, p_f2});
      }
    }
  } else {
    for (const auto& p_f : perf_f) {
      perf_ff.emplace_back(std::array{p_f, p_f});
    }
  }

  nlohmann::json res;
  for (const auto& c_f : cluster_f) {
    auto num_of_clusters = pc.size() / c_f;
    for (const auto& p_f : perf_ff) {
      std::vector<long> durations;
      for (int repeat = 0; repeat < repeat_f; ++repeat) {
        LOOP0_PERF = p_f[0];
        LOOP1_PERF = p_f[0];

        auto start_time = std::chrono::steady_clock::now();

        auto clustered_points = offset_start ? clusterOffset(pc, num_of_clusters) : cluster(pc, num_of_clusters);

        auto end_time = std::chrono::steady_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        spdlog::info("Finished clustering {} {}/{} with c:{}, p1:{}, p2:{}, num_of_points:{}  duration: {}",
                     pc_name_ext, repeat + 1, repeat_f, c_f, p_f[0], p_f[1], num_of_clusters, total_time);

        rapidcsv::Document outdoc;
        outdoc.InsertRow(-1, pc_csv.GetColumnNames());
        auto row = 0;
        for (int i = 0; i < clustered_points.size(); ++i) {
          if (clustered_points[i].cluster_head) {
            outdoc.InsertRow(row++, pc_csv.GetRow<float>(i));
          }
        }

        auto pc_output_name = combs ? fmt::format("{}/{}#c{}_p{}_p{}.csv", output_path, pc_name, c_f, p_f[0], p_f[1])
                                       : fmt::format("{}/{}#c{}_p{}.csv", output_path, pc_name, c_f, p_f[0]);
        outdoc.Save(pc_output_name);

        durations.emplace_back(total_time);
      }

      if (combs)
        res[fmt::format("c{}_p{}", c_f, p_f[0])] = {{"num_of_points", num_of_clusters}, {"clustering_time", durations}};
      else
        res[fmt::format("c{}_p{},{}", c_f, p_f[0], p_f[1])] = {{"num_of_points", num_of_clusters},
                                                               {"clustering_time", durations}};
    }
  }
  final_results[pc_name] = res;
}

float distance(const PointRGB& p1, const PointRGB& p2) {
  auto diff = p1.pos - p2.pos;
  return diff.dot(diff);
}