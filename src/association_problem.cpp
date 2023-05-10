#include "../includes/sacbased_association.hpp"
#include "../includes/association_problem.h"

namespace static_data_association{

template<int Dim>
Hypothesis<Dim> AssociationProblem<Dim>::dcsac(const SacBasedCfg sac_cfg, Eigen::Isometry3d& tf) {

	SacBasedAssociation<Dim> sac_asso(sac_cfg, *this);

    Hypothesis<Dim> best{*this};

	sac_asso.nnImplementation();

	for (size_t i = 0; i < sac_asso.getAssociations().size(); i++)
	{
		size_t d_id = sac_asso.getAssociations()[i].first;
		size_t l_id = sac_asso.getAssociations()[i].second;
		best = Hypothesis<Dim>(best, d_id, l_id);
	}

    return best;
}

}