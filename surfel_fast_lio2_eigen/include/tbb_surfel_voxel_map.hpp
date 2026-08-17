#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/parallel_for.h>

#include "unordered_dense.h"


struct TbbSurfel
{
    Eigen::Vector3f centroid_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal_ = Eigen::Vector3f::Zero();
};

class TbbSurfelVoxelMap
{
public:
    struct Parameters
    {
        float leaf_voxel_size_ = 0.5F;
        float map_half_extent_ = 100.0F;
        float recenter_distance_ = 50.0F;
        float maximum_flatness_ = 0.03F;
        float minimum_linearity_ = 0.3F;
        std::size_t minimum_occupied_leaf_count_ = 5U;
    };

    TbbSurfelVoxelMap();
    explicit TbbSurfelVoxelMap(const Parameters &_parameters);

    void configure(const Parameters &_parameters);
    void clear();
    void update(const pcl::PointCloud<pcl::PointXYZINormal> &_points,
                const Eigen::Vector3d &_sensor_position);

    [[nodiscard]] bool findSurfel(const Eigen::Vector3d &_point, TbbSurfel &_surfel) const;
    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] std::size_t voxelCount() const noexcept;
    [[nodiscard]] std::size_t surfelCount() const noexcept;

private:
    struct VoxelKey
    {
        std::int32_t x_ = 0;
        std::int32_t y_ = 0;
        std::int32_t z_ = 0;

        [[nodiscard]] bool operator==(const VoxelKey &_other) const noexcept
        {
            return x_ == _other.x_ && y_ == _other.y_ && z_ == _other.z_;
        }
    };

    struct VoxelKeyHash
    {
        [[nodiscard]] std::size_t operator()(const VoxelKey &_key) const noexcept;

    private:
        [[nodiscard]] static std::uint64_t expandBits(std::int32_t _value) noexcept;
    };

    struct VoxelKeyHashCompare
    {
        [[nodiscard]] static std::size_t hash(const VoxelKey &_key) noexcept
        {
            return VoxelKeyHash{}(_key);
        }

        [[nodiscard]] static bool equal(const VoxelKey &_left,
                                        const VoxelKey &_right) noexcept
        {
            return _left == _right;
        }
    };

    struct LeafVoxel
    {
        Eigen::Vector3f centroid_ = Eigen::Vector3f::Zero();
        std::uint32_t point_count_ = 0U;
    };

    struct ParentVoxel
    {
        TbbSurfel surfel_;
        bool valid_ = false;
    };

    using LeafMap = tbb::concurrent_hash_map<VoxelKey, LeafVoxel, VoxelKeyHashCompare>;
    using ParentMap = ankerl::unordered_dense::map<VoxelKey, ParentVoxel, VoxelKeyHash>;
    using DirtySet = ankerl::unordered_dense::set<VoxelKey, VoxelKeyHash>;
    using ConcurrentDirtySet = tbb::concurrent_unordered_set<VoxelKey, VoxelKeyHash>;

    [[nodiscard]] VoxelKey pointToLeafKey(const Eigen::Vector3f &_point) const noexcept;
    [[nodiscard]] static VoxelKey parentKey(const VoxelKey &_leaf) noexcept;
    [[nodiscard]] static std::int32_t floorDivideByThree(std::int32_t _value) noexcept;

    void insertPointConcurrent(const Eigen::Vector3f &_point);
    void recomputeDirtySurfels();
    void recomputeSurfel(const VoxelKey &_parent_key, ParentVoxel &_parent);
    void rebuildParents();
    void pruneIfNeeded(const Eigen::Vector3f &_sensor_position);

    Parameters parameters_;
    LeafMap leaves_;
    ParentMap parents_;
    DirtySet dirty_parents_;
    ConcurrentDirtySet concurrent_dirty_parents_;
    std::vector<std::pair<VoxelKey, ParentVoxel *>> dirty_nodes_;
    Eigen::Vector3f map_center_ = Eigen::Vector3f::Zero();
    std::size_t surfel_count_ = 0U;
    bool map_initialized_ = false;
};

inline TbbSurfelVoxelMap::TbbSurfelVoxelMap():
    TbbSurfelVoxelMap(Parameters{})
{
}

inline TbbSurfelVoxelMap::TbbSurfelVoxelMap(const Parameters &_parameters)
{
    configure(_parameters);
}

inline void TbbSurfelVoxelMap::configure(const Parameters &_parameters)
{
    if (!(_parameters.leaf_voxel_size_ > 0.0F))
    {
        throw std::invalid_argument("TbbSurfel leaf size must be positive");
    }
    if (!(_parameters.map_half_extent_ > 0.0F))
    {
        throw std::invalid_argument("TbbSurfel map half extent must be positive");
    }
    if (_parameters.minimum_occupied_leaf_count_ < 3U || _parameters.minimum_occupied_leaf_count_ > 27U)
    {
        throw std::invalid_argument("TbbSurfel minimum occupied leaf count must be in [3, 27]");
    }

    const bool geometry_changed = parameters_.leaf_voxel_size_ != _parameters.leaf_voxel_size_ ||
                                  parameters_.map_half_extent_ != _parameters.map_half_extent_;
    parameters_ = _parameters;
    if (geometry_changed && !leaves_.empty())
    {
        clear();
    }
}

inline void TbbSurfelVoxelMap::clear()
{
    leaves_.clear();
    parents_.clear();
    dirty_parents_.clear();
    concurrent_dirty_parents_.clear();
    dirty_nodes_.clear();
    map_center_.setZero();
    surfel_count_ = 0U;
    map_initialized_ = false;
}

inline void TbbSurfelVoxelMap::update(const pcl::PointCloud<pcl::PointXYZINormal> &_points,
                                      const Eigen::Vector3d &_sensor_position)
{
    if (_points.empty())
    {
        return;
    }

    const Eigen::Vector3f sensor_position_float = _sensor_position.cast<float>();
    if (!map_initialized_)
    {
        map_center_ = sensor_position_float;
        map_initialized_ = true;
    }
    pruneIfNeeded(sensor_position_float);

    parents_.reserve(parents_.size() + _points.size() / 8U + 1U);
    dirty_parents_.reserve(_points.size() / 4U + 1U);
    concurrent_dirty_parents_.clear();
#if TBB_VERSION_MAJOR >= 2021
    concurrent_dirty_parents_.reserve(_points.size() / 4U + 1U);
#endif
    //clang-format off
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0U, _points.size()),
        [this, &_points](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                const pcl::PointXYZINormal &point = _points[index];
                insertPointConcurrent(Eigen::Vector3f(point.x, point.y, point.z));
            }
        });
    //clang-format on
    for (const VoxelKey &parent_key : concurrent_dirty_parents_)
    {
        parents_.try_emplace(parent_key);
        dirty_parents_.insert(parent_key);
    }
    recomputeDirtySurfels();
}

inline bool TbbSurfelVoxelMap::findSurfel(const Eigen::Vector3d &_point, TbbSurfel &_surfel) const
{
    const VoxelKey leaf_key = pointToLeafKey(_point.cast<float>());
    const auto parent_iterator = parents_.find(parentKey(leaf_key));
    if (parent_iterator == parents_.end() || !parent_iterator->second.valid_)
    {
        return false;
    }

    _surfel = parent_iterator->second.surfel_;
    return true;
}

inline bool TbbSurfelVoxelMap::empty() const noexcept
{
    return leaves_.empty();
}

inline std::size_t TbbSurfelVoxelMap::voxelCount() const noexcept
{
    return leaves_.size();
}

inline std::size_t TbbSurfelVoxelMap::surfelCount() const noexcept
{
    return surfel_count_;
}

inline std::uint64_t TbbSurfelVoxelMap::VoxelKeyHash::expandBits(const std::int32_t _value) noexcept
{
    std::uint64_t expanded = static_cast<std::uint64_t>(
                                 static_cast<std::int64_t>(_value) + (1LL << 20)) &
                             0x1fffffULL;
    expanded = (expanded | (expanded << 32U)) & 0x1f00000000ffffULL;
    expanded = (expanded | (expanded << 16U)) & 0x1f0000ff0000ffULL;
    expanded = (expanded | (expanded << 8U)) & 0x100f00f00f00f00fULL;
    expanded = (expanded | (expanded << 4U)) & 0x10c30c30c30c30c3ULL;
    expanded = (expanded | (expanded << 2U)) & 0x1249249249249249ULL;
    return expanded;
}

inline std::size_t TbbSurfelVoxelMap::VoxelKeyHash::operator()(const VoxelKey &_key) const noexcept
{
    return static_cast<std::size_t>(
        expandBits(_key.x_) | (expandBits(_key.y_) << 1U) | (expandBits(_key.z_) << 2U));
}

inline TbbSurfelVoxelMap::VoxelKey TbbSurfelVoxelMap::pointToLeafKey(const Eigen::Vector3f &_point) const noexcept
{
    const float inverse_leaf_voxel_size = 1.0F / parameters_.leaf_voxel_size_;
    return {
        static_cast<std::int32_t>(std::floor(_point.x() * inverse_leaf_voxel_size)),
        static_cast<std::int32_t>(std::floor(_point.y() * inverse_leaf_voxel_size)),
        static_cast<std::int32_t>(std::floor(_point.z() * inverse_leaf_voxel_size))};
}

inline TbbSurfelVoxelMap::VoxelKey TbbSurfelVoxelMap::parentKey(const VoxelKey &_leaf) noexcept
{
    return {floorDivideByThree(_leaf.x_), floorDivideByThree(_leaf.y_), floorDivideByThree(_leaf.z_)};
}

inline std::int32_t TbbSurfelVoxelMap::floorDivideByThree(const std::int32_t _value) noexcept
{
    return _value >= 0 ? _value / 3 : (_value - 2) / 3;
}

inline void TbbSurfelVoxelMap::insertPointConcurrent(const Eigen::Vector3f &_point)
{
    const VoxelKey leaf_key = pointToLeafKey(_point);
    LeafMap::accessor leaf_accessor;
    const bool inserted = leaves_.insert(leaf_accessor, leaf_key);
    LeafVoxel &leaf = leaf_accessor->second;
    if (inserted)
    {
        leaf.centroid_ = _point;
        leaf.point_count_ = 1U;
    }
    else
    {
        const float old_count = static_cast<float>(leaf.point_count_);
        leaf.centroid_ += (_point - leaf.centroid_) / (old_count + 1.0F);
        ++leaf.point_count_;
    }
    leaf_accessor.release();

    const VoxelKey parent_key = parentKey(leaf_key);
    concurrent_dirty_parents_.insert(parent_key);
}

inline void TbbSurfelVoxelMap::recomputeDirtySurfels()
{
    dirty_nodes_.clear();
    dirty_nodes_.reserve(dirty_parents_.size());
    std::size_t previously_valid = 0U;
    for (const VoxelKey &key : dirty_parents_)
    {
        auto iterator = parents_.find(key);
        if (iterator != parents_.end())
        {
            previously_valid += iterator->second.valid_ ? 1U : 0U;
            dirty_nodes_.emplace_back(key, &iterator->second);
        }
    }

    //clang-format off
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0U, dirty_nodes_.size()),
        [this](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                recomputeSurfel(dirty_nodes_[index].first, *dirty_nodes_[index].second);
            }
        });
    //clang-format on

    std::size_t currently_valid = 0U;
    for (const auto &entry : dirty_nodes_)
    {
        currently_valid += entry.second->valid_ ? 1U : 0U;
    }
    surfel_count_ += currently_valid;
    surfel_count_ -= previously_valid;
    dirty_parents_.clear();
}

inline void TbbSurfelVoxelMap::recomputeSurfel(const VoxelKey &_parent_key, ParentVoxel &_parent)
{
    std::array<Eigen::Vector3f, 27U> centroids;
    std::size_t centroid_count = 0U;
    for (std::int32_t x_offset = 0; x_offset < 3; ++x_offset)
    {
        for (std::int32_t y_offset = 0; y_offset < 3; ++y_offset)
        {
            for (std::int32_t z_offset = 0; z_offset < 3; ++z_offset)
            {
                const VoxelKey leaf_key{
                    _parent_key.x_ * 3 + x_offset,
                    _parent_key.y_ * 3 + y_offset,
                    _parent_key.z_ * 3 + z_offset};
                LeafMap::const_accessor leaf_accessor;
                if (leaves_.find(leaf_accessor, leaf_key))
                {
                    centroids[centroid_count++] = leaf_accessor->second.centroid_;
                }
            }
        }
    }

    if (centroid_count < parameters_.minimum_occupied_leaf_count_)
    {
        _parent.valid_ = false;
        return;
    }

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (std::size_t index = 0U; index < centroid_count; ++index)
    {
        centroid += centroids[index];
    }
    centroid /= static_cast<float>(centroid_count);

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (std::size_t index = 0U; index < centroid_count; ++index)
    {
        const Eigen::Vector3f difference = centroids[index] - centroid;
        covariance.noalias() += difference * difference.transpose();
    }
    covariance /= static_cast<float>(centroid_count);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
        _parent.valid_ = false;
        return;
    }

    const Eigen::Vector3f eigenvalues = solver.eigenvalues();
    const float largest = std::max(eigenvalues.z(), std::numeric_limits<float>::epsilon());
    const float flatness = eigenvalues.x() / largest;
    const float linearity = eigenvalues.y() / largest;
    if (flatness > parameters_.maximum_flatness_ || linearity < parameters_.minimum_linearity_)
    {
        _parent.valid_ = false;
        return;
    }

    _parent.surfel_.centroid_ = centroid;
    _parent.surfel_.normal_ = solver.eigenvectors().col(0);
    _parent.valid_ = true;
}

inline void TbbSurfelVoxelMap::rebuildParents()
{
    parents_.clear();
    dirty_parents_.clear();
    surfel_count_ = 0U;
    parents_.reserve(leaves_.size() / 4U + 1U);
    dirty_parents_.reserve(leaves_.size() / 4U + 1U);
    for (const auto &entry : leaves_)
    {
        const VoxelKey parent_key = parentKey(entry.first);
        parents_.try_emplace(parent_key);
        dirty_parents_.insert(parent_key);
    }
    recomputeDirtySurfels();
}

inline void TbbSurfelVoxelMap::pruneIfNeeded(const Eigen::Vector3f &_sensor_position)
{
    if ((_sensor_position - map_center_).norm() <= parameters_.recenter_distance_)
    {
        return;
    }

    map_center_ = _sensor_position;
    const float half_extent = parameters_.map_half_extent_;
    std::vector<VoxelKey> keys_to_remove;
    keys_to_remove.reserve(leaves_.size() / 8U + 1U);
    for (const auto &entry : leaves_)
    {
        const Eigen::Vector3f leaf_center = parameters_.leaf_voxel_size_ *
                                            Eigen::Vector3f(
                                                static_cast<float>(entry.first.x_) + 0.5F,
                                                static_cast<float>(entry.first.y_) + 0.5F,
                                                static_cast<float>(entry.first.z_) + 0.5F);
        if ((leaf_center.array() - map_center_.array()).abs().maxCoeff() > half_extent)
        {
            keys_to_remove.push_back(entry.first);
        }
    }
    for (const VoxelKey &key : keys_to_remove)
    {
        leaves_.erase(key);
    }
    rebuildParents();
}
