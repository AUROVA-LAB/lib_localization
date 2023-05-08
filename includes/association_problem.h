#include "detection.hpp"

namespace static_data_association {

template<int Dim>
class AssociationProblem {
public:
    AssociationProblem() : precalculated_{false} {
        // Check dimensionality
        static_assert((Dim == 2 || Dim == 3), "AssociationProblem is only implemented for dimensions 2 and 3.");
    }
    ~AssociationProblem() { }

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