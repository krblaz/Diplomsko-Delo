#ifndef PC_FMM_FASTMARCHING_H
#define PC_FMM_FASTMARCHING_H

#include <fmt/core.h>
#include <fmt/ostream.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <json.hpp>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <random>

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

struct Point {
  enum class Type { ORIGINAL, BEND };
  enum class Status { ALIVE, CLOSE, FFAR };

  const cv::Vec3f vec;
  const Type type;
  int bend_step;

  double distance{std::numeric_limits<double>::infinity()};
  Status status{Status::FFAR};
  std::array<Point*, 6> neighbors{};

  Point(const cv::Vec3f& vec, Type type, int bend_step) : vec{vec}, type{type}, bend_step{bend_step} {};
};

class FastMarching {
 private:
  std::vector<cv::Vec3f> m_original_points;
  std::map<cv::Vec3f, Point, VecCompare<cv::Vec3f>> m_grid;
  int m_output_points_num;
  double m_grid_step;
  double m_bend_size;

  int m_initial_points{3};
  double m_speed{1};
  std::vector<std::reference_wrapper<Point>> m_points_max_heap;
  std::vector<std::reference_wrapper<Point>> m_close;
  std::vector<std::reference_wrapper<Point>> m_voronoi;

  std::vector<cv::Vec3f> m_filtered_points;

  void addPoints(const std::vector<cv::Vec3f>& points);
  void makeGrid();
  void initRandomStartingPoints();
  void initFarthestStartingPoint();
  void initStartingPoint(Point& point);
  double calculateDistance(Point& point);
  void marchStep();
  void march();

 public:
  FastMarching(const std::vector<cv::Vec3f>& points, int output_points_num, double grid_step, double bend_size)
      : m_output_points_num{output_points_num}, m_grid_step{grid_step}, m_bend_size{bend_size} {
    addPoints(points);
    fmt::print("Loaded {} points\n", m_grid.size());
    makeGrid();
    fmt::print("Finished making grid of {} points\n", m_grid.size());
  }

  void filter();

  auto getGrid() const { return m_grid; }
  auto getGridPoints() const {
    std::vector<cv::Vec3f> res;
    for (const auto& point : m_grid) {
      res.emplace_back(point.second.vec);
    }
    return res;
  }

  auto getVoronoiPoints() const {
    std::vector<cv::Vec3f> res;
    for (const Point& point : m_voronoi) {
      res.emplace_back(point.vec);
    }
    return res;
  }

  /*auto getOutputPoints() {
    if(m_filtered_points.empty()) {
      m_filtered_points.reserve(m_voronoi.size());
      for (const Point& voronoi_point : m_voronoi) {
        double min_distance{std::numeric_limits<double>::max()};
        cv::Vec3f min_coord{m_original_points.at(0)};
        for (const cv::Vec3f& original_point : m_original_points) {
          double distance{utils::distance(voronoi_point.coord, original_point)};
          if (distance < min_distance) {
            min_distance = distance;
            min_coord = original_point;
          }
        }
        m_filtered_points.emplace_back(min_coord);
      }
    }
    return m_filtered_points;
  }*/
};

#endif  // PC_FMM_FASTMARCHING_H
