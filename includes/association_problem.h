#pragma once

#include "association.hpp"
#include "detection.hpp"
#include "forward.hpp"
#include "hypothesis.hpp"

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
    void clearDetections (void) { detections_.clear(); }
    void clearLandmarks (void) { landmarks_.clear(); }
    void dcsac (void);

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