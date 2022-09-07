#include <fmt/core.h>

#include <filesystem>
#include <opencv2/core/matx.hpp>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/printf.h>

#include "argparse.hpp"
#include "rapidcsv.h"
#include "FastMarching.h"

#define LOG(...)            \
  fmt::printf(__VA_ARGS__); \
  fmt::printf("\n");

void dumpJson(const std::vector<cv::Vec3f>& points, const std::string& path){
  nlohmann::json res = nlohmann::json::array();
  for (const auto& point : points) {
    res.emplace_back(point.val);
  }
  std::ofstream ofstream(path);
  ofstream << res.dump(2);
}

void fastMarching(const std::string& base_path, const std::string& output_path, const std::string& pc_name_ext) {
  rapidcsv::Document pc_csv(base_path + "/" + pc_name_ext, rapidcsv::LabelParams(0, -1));

  std::vector<cv::Vec3f> pc;
  for (int i = 0; i < pc_csv.GetRowCount(); ++i) {
    auto row = pc_csv.GetRow<float>(i);
    pc.emplace_back(row[0], row[1], row[2]);
  }
  LOG("Loaded pc with size %zu", pc.size());

  auto num_of_clusters = pc.size() / 2;
  FastMarching fm(pc,num_of_clusters,0.04,0.04);
  //fm.filter();

  dumpJson(fm.getGridPoints(), fmt::format("{}/grid.json", output_path));
}

int main(int argc, char* argv[]) {
  fastMarching("/mnt/n/Diploma/Data/Stanford3dDatasetCSV", "/mnt/c/Users/krist/Desktop/CPlyTest/fmm_results", "Area_2_storage_1.csv");
  return 0;

  /*argparse::ArgumentParser program("ClusterPC");

  program.add_argument("--base_path");
  program.add_argument("--output_path");

  try {
    auto base_path = program.get("--base_path");
    auto output_path = program.get("--output_path");

    std::filesystem::create_directory(output_path);
    if (!std::filesystem::exists(output_path)) fmt::print(stderr, "Can't create directory with path {}\n", output_path);

    for (const auto& file_path : std::filesystem::directory_iterator(base_path)) {

    }

  } catch (const std::exception& e) {
    fmt::print("{}\n", e.what());
  }
  return 0;*/
}