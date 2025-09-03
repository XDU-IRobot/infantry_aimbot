#include "estimation/car_tracker.hpp"


namespace ia {
namespace estimation {
CarTracker::CarTracker() {}

CarTracker::~CarTracker() = default;

result_void CarTracker::Initialize() { return outcome_v2::success(); }

result_void CarTracker::Reset() { return outcome_v2::success(); }

result_void CarTracker::Update(std::shared_ptr<Eigen::VectorXd>) { return outcome_v2::success(); }

}  // namespace estimation
}  // namespace ia