#pragma once

#include <vector>
#include <functional>

#include <Eigen/Dense>

#include "association_problem.h"
#include "forward.hpp"

namespace static_data_association {

template <int Dim>
class Hypothesis {
public:
    Hypothesis<Dim>(AssociationProblem<Dim>& associationProblem);
    Hypothesis<Dim>(const Hypothesis<Dim>& hypothesisIn,
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
    std::reference_wrapper<AssociationProblem<Dim> > associationProblem_;
    Association association_;
};

template<>
inline bool Hypothesis<2>::valid() const {
    return  numValidAssociations() >= 2;
}

template<>
inline bool Hypothesis<3>::valid() const {
    return  numValidAssociations() >= 3;
}


template<int Dim>
Hypothesis<Dim>::Hypothesis(AssociationProblem<Dim>& associationProblem) : associationProblem_{associationProblem},
    association_{} {}

template<int Dim>
Hypothesis<Dim>::Hypothesis(const Hypothesis<Dim>& hypothesisIn,
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
