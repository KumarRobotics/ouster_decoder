#include <benchmark/benchmark.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/parallel_for.h>

#include <Eigen/Geometry>

namespace {

template <typename PointT>
using CloudT = pcl::PointCloud<PointT>;

using PointXYZ = pcl::PointXYZ;
using PointXYZRGB = pcl::PointXYZRGB;
using CloudXYZ = pcl::PointCloud<PointXYZ>;
using CloudXYZRGB = pcl::PointCloud<PointXYZRGB>;

struct RowMajor;
struct ColMajor;
struct Serial;
struct Parallel;

constexpr int kHeight = 64;
constexpr int kMinWidth = 512;
constexpr int kMaxWidth = 2048;

template <typename PointT>
void ProcessCloudRowMajor(CloudT<PointT>& cloud) {
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      auto& p = cloud.at(j, i);
      p.x = p.y = p.z = 1;
    }
  }
}

template <typename PointT>
void ProcessCloudRowMajorPar(CloudT<PointT>& cloud) {
  tbb::parallel_for(tbb::blocked_range<int>(0, cloud.height),
                    [&cloud](tbb::blocked_range<int> range) {
                      for (int i = range.begin(); i < range.end(); ++i) {
                        for (int j = 0; j < cloud.width; ++j) {
                          auto& p = cloud.at(j, i);
                          p.x = p.y = p.z = 1;
                        }
                      }
                    }

  );
}

template <typename PointT>
void ProcessCloudColMajor(CloudT<PointT>& cloud) {
  for (int j = 0; j < cloud.width; ++j) {
    for (int i = 0; i < cloud.height; ++i) {
      auto& p = cloud.at(j, i);
      p.x = p.y = p.z = 1;
    }
  }
}

template <typename PointT>
void ProcessCloudColMajorPar(CloudT<PointT>& cloud) {
  tbb::parallel_for(tbb::blocked_range<int>(0, cloud.width),
                    [&cloud](const tbb::blocked_range<int>& range) {
                      for (int j = range.begin(); j < range.end(); ++j) {
                        for (int i = 0; i < cloud.height; ++i) {
                          auto& p = cloud.at(j, i);
                          p.x = p.y = p.z = 1;
                        }
                      }
                    }

  );
}

template <typename PointT, typename Order = RowMajor, typename Execute = Serial>
void ProcessCloud(CloudT<PointT>& cloud) {
  if constexpr (std::is_same_v<Order, RowMajor>) {
    if constexpr (std::is_same_v<Execute, Serial>) {
      ProcessCloudRowMajor(cloud);
    } else {
      ProcessCloudRowMajorPar(cloud);
    }
  } else {
    if constexpr (std::is_same_v<Execute, Serial>) {
      ProcessCloudColMajor(cloud);
    } else {
      ProcessCloudColMajorPar(cloud);
    }
  }
}

template <typename PointT, typename Order, typename Execute>
void BM_ProcessCloud(benchmark::State& state) {
  CloudT<PointT> cloud(kHeight, state.range(0));

  for (auto _ : state) {
    ProcessCloud<PointT, Order, Execute>(cloud);
  }
}

BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZ, RowMajor, Serial)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZ, RowMajor, Parallel)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZ, ColMajor, Serial)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZ, ColMajor, Parallel)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZRGB, RowMajor, Serial)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZRGB, RowMajor, Parallel)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZRGB, ColMajor, Serial)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_ProcessCloud, PointXYZRGB, ColMajor, Parallel)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);

template <typename PointT>
void TransformCloud(CloudT<PointT>& cloud, const Eigen::Affine3f& transform) {
  CloudT<PointT> out;
  pcl::transformPointCloud(cloud, out, transform);
}

void BM_TransformCloudBase(benchmark::State& state) {
  CloudXYZ cloud(kHeight, state.range(0));
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();

  for (auto _ : state) {
    TransformCloud(cloud, tf);
  }
}
BENCHMARK(BM_TransformCloudBase)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);

template <typename PointT>
void TransformCloudRowMajor(CloudT<PointT>& cloud,
                            const std::vector<Eigen::Affine3f>& transforms) {
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& tf = transforms[j];
      auto& pt = cloud.at(j, i);
      auto p = pt.getArray3fMap();
      p = tf * p;
    }
  }
}

template <typename PointT>
void TransformCloudRowMajorPar(CloudT<PointT>& cloud,
                               const std::vector<Eigen::Affine3f>& transforms) {
  tbb::parallel_for(tbb::blocked_range<int>(0, cloud.height),
                    [&](const tbb::blocked_range<int>& range) {
                      for (int i = range.begin(); i < range.end(); ++i) {
                        for (int j = 0; j < cloud.width; ++j) {
                          const auto& tf = transforms[j];
                          auto& pt = cloud.at(j, i);
                          auto p = pt.getArray3fMap();
                          p = tf * p;
                        }
                      }
                    });
}

template <typename Execute>
void BM_TransformCloudRowMajor(benchmark::State& state) {
  CloudXYZ cloud(kHeight, state.range(0));
  std::vector<Eigen::Affine3f> transforms(state.range(0),
                                          Eigen::Affine3f::Identity());

  for (auto _ : state) {
    if constexpr (std::is_same_v<Execute, Serial>) {
      TransformCloudRowMajor(cloud, transforms);
    } else {
      TransformCloudRowMajorPar(cloud, transforms);
    }
  }
}

template <typename PointT>
void TransformCloudColMajor(CloudT<PointT>& cloud,
                            const std::vector<Eigen::Affine3f>& transforms) {
  for (int i = 0; i < cloud.height; ++i) {
    const auto& tf = transforms[i];
    for (int j = 0; j < cloud.width; ++j) {
      auto& pt = cloud.at(j, i);
      auto p = pt.getArray3fMap();
      p = tf * p;
    }
  }
}

template <typename PointT>
void TransformCloudColMajorPar(CloudT<PointT>& cloud,
                               const std::vector<Eigen::Affine3f>& transforms) {
  tbb::parallel_for(tbb::blocked_range<int>(0, cloud.height),
                    [&](const tbb::blocked_range<int>& range) {
                      for (int i = range.begin(); i < range.end(); ++i) {
                        const auto& tf = transforms[i];
                        for (int j = 0; j < cloud.width; ++j) {
                          auto& pt = cloud.at(j, i);
                          auto p = pt.getArray3fMap();
                          p = tf * p;
                        }
                      }
                    });
}

template <typename Execute>
void BM_TransformCloudColMajor(benchmark::State& state) {
  CloudXYZ cloud(state.range(0), kHeight);
  std::vector<Eigen::Affine3f> transforms(state.range(0),
                                          Eigen::Affine3f::Identity());

  for (auto _ : state) {
    if constexpr (std::is_same_v<Execute, Serial>) {
      TransformCloudColMajor(cloud, transforms);
    } else {
      TransformCloudColMajorPar(cloud, transforms);
    }
  }
}

BENCHMARK_TEMPLATE(BM_TransformCloudRowMajor, Serial)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_TransformCloudColMajor, Serial)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_TransformCloudRowMajor, Parallel)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);
BENCHMARK_TEMPLATE(BM_TransformCloudColMajor, Parallel)
    ->RangeMultiplier(2)
    ->Range(kMinWidth, kMaxWidth);

}  // namespace