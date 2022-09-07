#include <argparse.hpp>
#include <fstream>

#include "clustering/clustering.h"

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

  program.add_argument("--mode");
  program.add_argument("--comb").implicit_value(true).default_value(false);
  program.add_argument("--disable_offset_start").implicit_value(false).default_value(true);

  try {
    program.parse_args(argc, argv);
    auto mode = program.get("--mode");
    auto comb = program.get<bool>("--comb");
    auto offset_start = program.get<bool>("--disable_offset_start");
    auto base_path = program.get("--base_path");
    auto output_path = program.get("--output_path");
    auto cluster_f = stringToIntVector(program.get("-c_f"));
    auto perf_f = stringToIntVector(program.get("-p_f"));

    std::filesystem::create_directory(output_path);
    if (!std::filesystem::exists(output_path)) fmt::print(stderr, "Can't create directory with path {}\n", output_path);

    for (const auto& file_path : std::filesystem::directory_iterator(base_path)) {
      if (mode == "image")
        clusterImage(base_path, output_path, file_path.path().filename().string(), cluster_f, perf_f, 1, comb,
                     offset_start);
      else if (mode == "pc")
        clusterPC(base_path, output_path, file_path.path().filename().string(), cluster_f, perf_f, 1, comb, offset_start);
    }

    std::ofstream ofstream(fmt::format("{}/results.json", output_path));
    ofstream << final_results.dump(2);
  } catch (const std::exception& e) {
    fmt::print("{}\n", e.what());
  }

  return 0;
}
