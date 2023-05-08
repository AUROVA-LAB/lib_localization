#pragma once

#include <memory>
#include <iostream>

#include <Eigen/Dense>
#include <vector>

namespace static_data_association {

template<typename T1, typename T2>
bool isType(const T2& value) {
    return dynamic_cast<const T1*>(&value) != nullptr;
}

class AbstractDetection {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual bool individuallyCompatible(const AbstractDetection&) const {
        return false;
    }
    virtual ~AbstractDetection() {}
    virtual Eigen::Vector3d position() const = 0;
    virtual std::vector<Eigen::Vector3d> positionVector() const = 0;
    virtual int identifier() const = 0;
    virtual float distanceBetweenDetections (const AbstractDetection& detection) const = 0;
    virtual std::vector<Eigen::Vector3d> transformDetection (Eigen::Isometry3d tf) const = 0;

};


class LineSegmentDetection : public AbstractDetection {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LineSegmentDetection(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, int identifier){
	  start_point_[0] = start_point[0];
	  start_point_[1] = start_point[1];
	  start_point_[2] = 0.0;

	  end_point_[0] = end_point[0];
	  end_point_[1] = end_point[1];
	  end_point_[2] = 0.0;

	  identifier_ = identifier;
  }

  virtual bool individuallyCompatible(const AbstractDetection& ad) const override {
	 return isType<LineSegmentDetection>(ad);
	 // Here, we could additionally check e.g. length or color
  }

  std::vector<Eigen::Vector3d> positionVector() const override {
	  std::vector<Eigen::Vector3d> output;
	  output.push_back(start_point_);
	  output.push_back(end_point_);
	  return output;
  }

  Eigen::Vector3d position() const override {
	 return 0.5*(start_point_ + end_point_);
	 //return end_point_;
  }

  int identifier() const override {
	  return identifier_;
  }

  float distanceBetweenDetections (const AbstractDetection& detection) const override;
  std::vector<Eigen::Vector3d> transformDetection (Eigen::Isometry3d tf) const override;

private:
  Eigen::Vector3d start_point_;
  Eigen::Vector3d end_point_;
  int identifier_;
};

class DalmrPolyDetection : public AbstractDetection {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DalmrPolyDetection(const Eigen::Vector3d& point, int identifier) : point_{point} {
	  identifier_ = identifier;
  }

  virtual bool individuallyCompatible(const AbstractDetection& ad) const override {
	 return isType<DalmrPolyDetection>(ad);
  }

  std::vector<Eigen::Vector3d> positionVector() const override {
	  std::vector<Eigen::Vector3d> output;
	  output.push_back(point_);
	  return output;
  }

  Eigen::Vector3d position() const override {
	 return point_;
  }

  int identifier() const override {
	  return identifier_;
  }

  float distanceBetweenDetections (const AbstractDetection& detection) const override;
  std::vector<Eigen::Vector3d> transformDetection (Eigen::Isometry3d tf) const override;

private:
  Eigen::Vector3d point_;
  int identifier_;
};

class DalmrDashDetection : public AbstractDetection {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DalmrDashDetection(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point, float z_weight, int identifier){
	  start_point_[0] = start_point[0];
	  start_point_[1] = start_point[1];
	  start_point_[2] = 0.0;

	  end_point_[0] = end_point[0];
	  end_point_[1] = end_point[1];
	  end_point_[2] = 0.0;

	  identifier_ = identifier;
	  z_weight_ = z_weight;
  }

  virtual bool individuallyCompatible(const AbstractDetection& ad) const override {
	 return isType<DalmrDashDetection>(ad);
	 // Here, we could additionally check e.g. length or color
  }

  std::vector<Eigen::Vector3d> positionVector() const override {
	  std::vector<Eigen::Vector3d> output;
	  output.push_back(start_point_);
	  output.push_back(end_point_);
	  return output;
  }

  Eigen::Vector3d position() const override {
	 return 0.5*(start_point_ + end_point_);
	 //return end_point_;
  }

  int identifier() const override {
	  return identifier_;
  }

  float distanceBetweenDetections (const AbstractDetection& detection) const override;
  std::vector<Eigen::Vector3d> transformDetection (Eigen::Isometry3d tf) const override;

private:
  Eigen::Vector3d start_point_;
  Eigen::Vector3d end_point_;
  int identifier_;
  float z_weight_;
};

Eigen::Vector3d pointProjection(const Eigen::Vector3d& u,
                                const Eigen::Vector3d& v,
                                const Eigen::Vector3d& p){
    double l = (u - v).norm();
    double t = 0.0;
    for (size_t i = 0; i < 2; i++)
        t += ((v[i] - u[i]) * (p[i] - u[i]));
    t /= l;
    Eigen::Vector3d proj{0.0, 0.0, 0.0};
    for (size_t i = 0; i < 2; i++)
        proj[i] = (u[i] + t * (v[i] - u[i]));
    return proj;
}

float LineSegmentDetection::distanceBetweenDetections (const AbstractDetection& detection) const {

	std::vector<Eigen::Vector3d> segment1 = this->positionVector();
	std::vector<Eigen::Vector3d> segment2 = detection.positionVector();

	//// HAUSDORFF DISTANCE
	float distance = 0.0;
	if (this->individuallyCompatible(detection)){
		Eigen::Vector3d s1_start = segment1.at(0);
		Eigen::Vector3d s1_end = segment1.at(1);
		Eigen::Vector3d s2_start = segment2.at(0);
		Eigen::Vector3d s2_end = segment2.at(1);

		// Angle distance
		Eigen::Vector3d ang_dist = (s1_end - s1_start).cross(s2_end - s2_start);

		// Parallel distance
		Eigen::Vector3d mpi = s1_start + s1_end;
	    for (int i = 0; i < 3; i++) {
	        mpi[i] = 0.5 * mpi[i];
	    }
	    Eigen::Vector3d mpj = s2_start + s2_end;
	    for (int i = 0; i < 3; i++) {
	        mpj[i] = 0.5 * mpj[i];
	    }
	    Eigen::Vector3d mpi_p = pointProjection(s2_start, s2_end, mpi);
	    Eigen::Vector3d par_dist = mpj - mpi_p;

	    // Perpendicular distance
	    mpi = s1_start + s1_end;
	    for (int i = 0; i < 3; i++) {
	        mpi[i] = 0.5 * mpi[i];
	    }
	    mpi_p = pointProjection(s2_start, s2_end, mpi);
	    Eigen::Vector3d per_dist = mpi_p - mpi;

	    // Hausdorff distance
	    distance = (ang_dist + par_dist + per_dist).norm();

	    //std::cout << distance << std::endl;
	}

	return distance;
}

float DalmrDashDetection::distanceBetweenDetections (const AbstractDetection& detection) const {

	std::vector<Eigen::Vector3d> segment1 = this->positionVector();
	std::vector<Eigen::Vector3d> segment2 = detection.positionVector();

	float distance = 0.0;
	double u, v, u1, v1, u2, v2, scalar_dot;
	if (this->individuallyCompatible(detection)){
		Eigen::Vector3d s1_start = segment1.at(0);
		Eigen::Vector3d s1_end = segment1.at(1);
		Eigen::Vector3d s2_start = segment2.at(0);
		Eigen::Vector3d s2_end = segment2.at(1);

		// Angle distance
		Eigen::Vector3d ang_dist = (s1_end - s1_start).cross(s2_end - s2_start);

		// Distance calculation
		Eigen::Vector3d point1 = this->position();
		point1.z() = 0.0;//z_weight_ * ang_dist.norm();
		Eigen::Vector3d point2 = detection.position();
		point2.z() = 0.0;//z_weight_ * (M_PI - ang_dist.norm());
		distance = (point2 - point1).norm();

		//std::cout << distance << std::endl;
	}

	return distance;
}

float DalmrPolyDetection::distanceBetweenDetections (const AbstractDetection& detection) const {

	float distance = 0.0;
	if (this->individuallyCompatible(detection)) distance = (detection.position() - this->position()).norm();
	return distance;
}

std::vector<Eigen::Vector3d> LineSegmentDetection::transformDetection (Eigen::Isometry3d tf) const {
	Eigen::Vector3d start_point_tf = tf * start_point_;
	Eigen::Vector3d end_point_tf = tf * end_point_;
	std::vector<Eigen::Vector3d> output;
	output.push_back(start_point_tf);
	output.push_back(end_point_tf);
	return output;
}

std::vector<Eigen::Vector3d> DalmrDashDetection::transformDetection (Eigen::Isometry3d tf) const {
	Eigen::Vector3d start_point_tf = tf * start_point_;
	Eigen::Vector3d end_point_tf = tf * end_point_;
	std::vector<Eigen::Vector3d> output;
	output.push_back(start_point_tf);
	output.push_back(end_point_tf);
	return output;
}

std::vector<Eigen::Vector3d> DalmrPolyDetection::transformDetection (Eigen::Isometry3d tf) const {
	Eigen::Vector3d point_tf = tf * point_;
	std::vector<Eigen::Vector3d> output;
	output.push_back(point_tf);
	return output;
}

}
