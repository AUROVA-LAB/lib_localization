#pragma once

#include <Eigen/Dense>

namespace static_data_association {

/**
 * @brief Denotes that a measurement or landmark is assigned to no measurement.
 */
static const size_t CLUTTER = std::numeric_limits<std::size_t>::max();

/**
 * @brief Class to encapsulate an association 
 * 
 */
struct Association {
    Association() : associations_{} { }
    Association(const std::vector<std::pair<size_t, size_t>>& associations) :
        associations_{associations} { }

    /**
     * @return the number of valid associations, i.e. landmarks that have been assigned a detection and vice versa (non-clutter, non-misses)
     */
    size_t numValidAssociations() const {return associations_.size();}

    std::vector<std::pair<size_t, size_t>> associations_; // Pair of (detectionIdx, landmarkIdx)
    //std::vector<size_t> clutter_; // Vector of detectionIdx which are assigned to clutter
    //std::vector<size_t> misses_; // Vector of landmarkIdx which are assigned to clutter
};

}
