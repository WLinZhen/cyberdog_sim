/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/VisualizationData.h"

#include "message/LowlevelState.h"
// CONTACT DETECT
#include "../include/Dynamics/Quadruped.h"
/*!
 * Result of state estimation
 */
template <typename T>
struct StateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;
  Vec3<T> position;
  Vec3<T> vBody;
  Quat<T> orientation;
  Vec3<T> omegaBody;
  RotMat<T> rBody;
  Vec3<T> rpy;

  Vec3<T> omegaWorld;
  Vec3<T> vWorld;
  Vec3<T> aBody, aWorld;
  // CONTACT DETECT
  Vec4<T> contactEstimate_KF;
  Vec4<T> swingEstimate;
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
// Always initialize the leg controller and state entimator
template <typename T>
struct StateEstimatorData {
    StateEstimate<T>* result;  // where to write the output to
    VectorNavData* vectorNavData;
    CheaterState<double>* cheaterState;
    LegControllerData<T>* legControllerData;
    Vec4<T>* contactPhase;
    RobotControlParameters* parameters;
    LowlevelState *lowState;
    // CONTACT DETECT
    Quadruped<T>*  quadruped;
    Vec4<T>* swingPhase;
};

/*!
 * All Estimators should inherit from this class
 */
template <typename T>
class GenericEstimator {
 public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
template <typename T>
class StateEstimatorContainer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container
   */
  StateEstimatorContainer(CheaterState<double>* cheaterState,
                          VectorNavData* vectorNavData,
                          LegControllerData<T>* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          RobotControlParameters* parameters,
                          LowlevelState *lowState,
                          // CONTACT DETECT
                          Quadruped<T>& quad) {
    _data.cheaterState = cheaterState;
    _data.vectorNavData = vectorNavData;
    _data.legControllerData = legControllerData;
    _data.result = stateEstimate;
    _phase = Vec4<T>::Zero();
    _data.contactPhase = &_phase;
    _data.parameters = parameters;
    _data.lowState = lowState;
    // CONTACT DETECT
    _data.quadruped = &quad;
    _data.swingPhase = &_swingphase;
  }
  /*!
   * Run all estimators
   */
  void run(CheetahVisualization* visualization = nullptr) {
    for (auto estimator : _estimators) {
      estimator->run();
    }
    if (visualization) {
      visualization->quat = _data.result->orientation.template cast<float>();
      visualization->p = _data.result->position.template cast<float>();
      // todo contact!
    }
  }

  /*!
   * Get the result
   */
  const StateEstimate<T>& getResult() { return *_data.result; }
  StateEstimate<T> * getResultHandle() { return _data.result; }

  /*!
   * Set the contact phase
   */
  void setContactPhase(Vec4<T>& phase) { 
    *_data.contactPhase = phase; 
  }
  // CONTACT DETECCT
  void setSwingPhase(Vec4<T>& phase) {
    *_data.swingPhase = phase;
  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() {
    auto* estimator = new EstimatorToAdd();
    estimator->setData(_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
   * Remove all estimators of a given type
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() {
    int nRemoved = 0;
    _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator<T>* e) {
                         if (dynamic_cast<EstimatorToRemove*>(e)) {
                           delete e;
                           nRemoved++;
                           return true;
                         } else {
                           return false;
                         }
                       }),
        _estimators.end());
  }

  /*!
   * Remove all estimators
   */
  void removeAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  ~StateEstimatorContainer() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

 private:
  StateEstimatorData<T> _data;
  std::vector<GenericEstimator<T>*> _estimators;
  Vec4<T> _phase;
  // CONTACT DETECT
  Vec4<T> _swingphase;
};

#endif  // PROJECT_STATEESTIMATOR_H
