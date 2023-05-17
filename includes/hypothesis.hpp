#pragma once

#include <vector>
#include <functional>

#include <Eigen/Dense>

namespace static_data_association {

class Hypothesis {
public:
    Hypothesis(AssociationProblem& associationProblem);
    Hypothesis(const Hypothesis& hypothesisIn,
               const size_t detectionIdx,
               const size_t landmarkIdx);
    const Association& av() const { return association_; }

    /**
     * @return number of valid (non-miss, non-clutter) associations
     */
    size_t numValidAssociations() const {return association_.numValidAssociations(); }

    /**
     * @return true if the hypothesis is valid, i.e. has sufficiently many associations
     */
    bool valid() const;

private:
    std::reference_wrapper<AssociationProblem> associationProblem_;
    Association association_;
};

inline bool Hypothesis::valid() const {
    return  numValidAssociations() >= 3;
}

Hypothesis::Hypothesis(AssociationProblem& associationProblem) : associationProblem_{associationProblem},
    association_{} {}


Hypothesis::Hypothesis(const Hypothesis& hypothesisIn,
           const size_t detectionIdx,
           const size_t landmarkIdx) : associationProblem_{hypothesisIn.associationProblem_} {
    association_.associations_ = hypothesisIn.association_.associations_;

    if (detectionIdx == CLUTTER || landmarkIdx == CLUTTER) {
        // do nothing
    } else {
        association_.associations_.push_back(std::make_pair(detectionIdx, landmarkIdx));
    }
}

}
