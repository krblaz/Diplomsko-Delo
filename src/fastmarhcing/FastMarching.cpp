#include "FastMarching.h"

void FastMarching::addPoints(const std::vector<cv::Vec3f>& points) {
  m_original_points = points;
  for (auto& coord : points) {
    double x = std::round(coord[0] / m_grid_step) * m_grid_step;
    double y = std::round(coord[1] / m_grid_step) * m_grid_step;
    double z = std::round(coord[2] / m_grid_step) * m_grid_step;
    cv::Vec3f new_coord(x,y,z);
    m_grid.emplace(new_coord, Point(new_coord, Point::Type::ORIGINAL, 0));
  }
}
void FastMarching::makeGrid() {
  std::queue<std::reference_wrapper<Point>> point_queue;
  for (auto& [coord, point] : m_grid) {
    point_queue.emplace(point);
  }

  std::array<std::array<double, 3>, 6> offsets{};
  for (int direction{0}; direction < 3; ++direction) {
    auto offset = std::bitset<3>(1) << (direction % 3);
    for (int i = 0; i < 3; ++i) {
      offsets[direction][i] = offset[i];
      offsets[direction + 3][i] = -offset[i];
    }
  }

  int num_of_steps{static_cast<int>(m_bend_size / m_grid_step)};

  while (!point_queue.empty()) {
    Point& point = point_queue.front();
    point_queue.pop();
    for (int direction{0}; direction < 6; ++direction) {
      cv::Vec3f neighbor_coord(point.vec[0] + offsets[direction][0] * m_grid_step,
                           point.vec[1] + offsets[direction][1] * m_grid_step,
                           point.vec[2] + offsets[direction][2] * m_grid_step);

      auto neighbor_point_iter{m_grid.find(neighbor_coord)};
      if (neighbor_point_iter != m_grid.end()) {
        Point& neighbor_point = neighbor_point_iter->second;
        point.neighbors[direction] = &neighbor_point;
        neighbor_point.neighbors[(direction + 3) % 6] = &point;
      } else if (point.bend_step + 1 <= num_of_steps) {
        Point&& temp_point{Point(neighbor_coord, Point::Type::BEND, point.bend_step + 1)};
        Point& neighbor_point = m_grid.emplace(neighbor_coord, temp_point).first->second;
        point_queue.emplace(neighbor_point);
        point.neighbors[direction] = &neighbor_point;
        neighbor_point.neighbors[(direction + 3) % 6] = &point;
      }
    }
  }

  // TODO remove max heap creation
  for (auto& [coord, point] : m_grid) {
    m_points_max_heap.emplace_back(point);
  }
}
void FastMarching::initRandomStartingPoints() {
  std::default_random_engine engine(42);
  std::uniform_int_distribution<int> distribution(0, m_grid.size());

  int num_of_added_points{0};
  while (num_of_added_points < m_initial_points) {
    Point& point{std::next(m_grid.begin(), distribution(engine))->second};

    if (point.status == Point::Status::FFAR) {
      initStartingPoint(point);
      m_voronoi.emplace_back(point);
      ++num_of_added_points;
    }
  }
}
void FastMarching::initFarthestStartingPoint() {
  for (auto& [coord, point] : m_grid) {
    point.status = Point::Status::FFAR;
  }

  // TODO pop max heap
  std::sort(m_points_max_heap.begin(), m_points_max_heap.end(),
            [](const auto p1, const auto p2) { return p1.get().distance > p2.get().distance; });

  Point& max_point = m_points_max_heap.front();
  max_point.distance = 0;
  max_point.status = Point::Status::ALIVE;
  m_voronoi.emplace_back(max_point);

  m_close.clear();
  initStartingPoint(max_point);
}
double FastMarching::calculateDistance(Point& point) {
  std::array<double, 6> delta{};
  for (int i = 0; i < 6; ++i) {
    if (point.neighbors[i] != nullptr && point.neighbors[i]->status == Point::Status::ALIVE) {
      delta[i] = point.neighbors[i]->distance;
    } else {
      delta[i] = std::numeric_limits<double>::max();
    }
  }

  double a{std::min(delta[0], delta[3])};
  double b{std::min(delta[1], delta[4])};

  if (std::abs(a - b) < 1)
    return (a + b + std::sqrt(2 * (1 / m_speed) * (1 / m_speed) - (a - b) * (a - b))) / 2;
  else
    return (1 / m_speed) * (1 / m_speed) + std::min(a, b);
}
void FastMarching::initStartingPoint(Point& point) {
  point.status = Point::Status::ALIVE;
  point.distance = 0;

  for (auto neighbor : point.neighbors) {
    if (neighbor != nullptr && neighbor->status == Point::Status::FFAR) {
      neighbor->status = Point::Status::CLOSE;
      neighbor->distance = calculateDistance(*neighbor);
      m_close.emplace_back(*neighbor);
    }
  }
}
void FastMarching::marchStep() {
  std::sort(m_close.begin(), m_close.end(),
            [](const auto p1, const auto p2) { return p1.get().distance > p2.get().distance; });
  Point& min_point = m_close.back();
  min_point.status = Point::Status::ALIVE;
  m_close.pop_back();

  for (auto neighbor : min_point.neighbors) {
    if (neighbor != nullptr) {
      double distance{calculateDistance(*neighbor)};
      if (distance < neighbor->distance) {
        neighbor->distance = distance;  // TODO Update max heap distance
        if (neighbor->status == Point::Status::FFAR) {
          neighbor->status = Point::Status::CLOSE;
          m_close.emplace_back(*neighbor);
        }
      }
    }
  }
}
void FastMarching::march() {
  int i{0};
  while (!m_close.empty()) {
    marchStep();
    ++i;
  }
}
void FastMarching::filter() {
  initRandomStartingPoints();
  march();

  for (int i = 0; i < m_output_points_num; ++i) {
    fmt::print("\rMarching step {}/{}", i, m_output_points_num);
    initFarthestStartingPoint();
    march();
  }
  fmt::print("\n");
}
