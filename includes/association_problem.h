#pragma once

#include "association.hpp"
#include "detection.hpp"
#include "forward.hpp"
#include "hypothesis.hpp"
#include "types_ap.h"

#include "sacbased_association.hpp"

namespace static_data_association {

template<int Dim>
class AssociationProblem {
public:
    AssociationProblem() : precalculated_{false} {
        // Check dimensionality
        static_assert((Dim == 2 || Dim == 3), "AssociationProblem is only implemented for dimensions 2 and 3.");
    }
    ~AssociationProblem() { }

    //// SET METHODS
    void addDetection(std::unique_ptr<AbstractDetection> detection) {
        detections_.emplace_back(std::move(detection));
        precalculated_ = false;
    }
    void addLandmark(std::unique_ptr<AbstractDetection> landmark) {
        landmarks_.emplace_back(std::move(landmark));
        precalculated_ = false;
    }

    //// GET METHODS
    const std::vector<std::unique_ptr<AbstractDetection>>& getDetections() {
        return detections_;
    }
    const std::vector<std::unique_ptr<AbstractDetection>>& getLandmarks() {
        return landmarks_;
    }

    //// CLASS METHODS
    bool areIndividuallyCompatible(const size_t i, const size_t j) const {
        return detections_[i]->individuallyCompatible(*(landmarks_[j])) && landmarks_[j]->individuallyCompatible(*(detections_[i]));
    }
    bool areDistanceCompatible(const Hypothesis<Dim>& h, const size_t i, const size_t j, const double distance) const;
    bool areDistanceCompatible(const size_t i1, const size_t j1, const size_t i2, const size_t j2, const double distance) const;

    void clearDetections (void) { detections_.clear(); }
    void clearLandmarks (void) { landmarks_.clear(); }
    Hypothesis<Dim> dcsac(const SacBasedCfg sac_cfg, Eigen::Isometry3d& tf);
    

private:
    //// Caching for e.g. JCBB which queries distances many many times
    mutable bool precalculated_; // True if caches are filled/valid

    //mutable Eigen::MatrixXd detectionDistanceCache_;
    //mutable Eigen::MatrixXd landmarkDistanceCache_;

    // Pointers allow downcasting
    std::vector<std::unique_ptr<AbstractDetection>> detections_;
    std::vector<std::unique_ptr<AbstractDetection>> landmarks_;
};

}