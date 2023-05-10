#pragma once

namespace static_data_association {

struct SacBasedCfg {
	float error_threshold;
	float x_var;
	float y_var;
	float w_var;
	double compatibility_distance;
	float threshold_asso;
};

using PairIds = std::pair<size_t, size_t>;
using PairIdsVec = std::vector<PairIds>;

struct AssoCandidates {

	// Pair of detections
	PairIds detect_pair_ids;

	// Pairs of landmarks, distance compatible with pair of detections
	PairIdsVec landmarks_pairs_ids;
};

}