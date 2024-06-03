#include "WBC_Ctrl.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/Timer.h>

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
  _full_config(cheetah::num_act_joint + 7),
  _tau_ff(cheetah::num_act_joint),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint)
{
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();
  // WBC解析：对应于论文公式25中的Q1 Q2权重系数
  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[5] = 0.1;
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  // WBC解析：论文式16-17的实现，计算出期望关节位置与速度
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  // WBC解析：将动力学参数更新至本类中，后文要用到
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  // WBC解析：论文式18-25的实现，目的是计算出期望的关节扭矩
  _wbic->MakeTorque(_tau_ff, _wbic_data);
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T> & data){
  ++_iter;

  // Update Model
  // WBC解析：首先根据机器人当前状态更新动力学模型，如雅克比矩阵、动力学公式等
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);
  // WBC解析：WBC中融合了多项任务，这里更新每个任务的参数，具体见论文IV-A 中式22，更新的即K_p与K_d
  // Task & Contact Update
  _ContactTaskUpdate(input, data);
  // WBC解析：WBC算法拿到上文中的多个任务，本函数依据公式16-21、25计算出最终的关节指令
  _ComputeWBC();
  // WBC解析：将WBC中的关节指令传输给腿控制器，最终作用到机器人
  _UpdateLegCMD(data);

  // // LCM publish
  // _LCM_PublishData();
}



template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> & data){
  LegControllerCommand<T> * cmd = data._legController->commands;
  //Vec4<T> contact = data._stateEstimator->getResult().contactEstimate;

  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];

        cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
        cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
       
       //if(contact[leg] > 0.){ // Contact
        //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
        //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
      //}else{
        //cmd[leg].kpJoint(jidx, jidx) = _Kp_joint_swing[jidx];
        //cmd[leg].kdJoint(jidx, jidx) = _Kd_joint_swing[jidx];
      //}

    }
  }


  // Knee joint non flip barrier
  for(size_t leg(0); leg<4; ++leg){
    if(cmd[leg].qDes[2] < 0.3){
      cmd[leg].qDes[2] = 0.3;
    }
    if(data._legController->datas[leg].q[2] < 0.3){
      T knee_pos = data._legController->datas[leg].q[2]; 
      cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
    }
  }
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    const LegControllerData<T> * leg_data){

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];

    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = leg_data[leg].q[i];
      _state.qd[3*leg + i] = leg_data[leg].qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
