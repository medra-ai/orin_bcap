/**
 * Class for sending commands to the Denso robot controller via the bCAP service.
 * Based on 
 * https://github.com/DENSORobot/denso_robot_ros2/blob/humble/denso_robot_core/src/denso_robot_core.cpp
 * https://github.com/DENSORobot/denso_robot_ros2/blob/humble/denso_robot_core/src/denso_controller.cpp
 * https://github.com/DENSORobot/denso_robot_ros2/blob/humble/denso_robot_core/src/denso_controller_rc9.cpp
 * https://github.com/DENSORobot/denso_robot_ros2/blob/humble/denso_robot_core/src/denso_robot.cpp
 * https://github.com/DENSORobot/denso_robot_ros2/blob/humble/denso_robot_core/src/denso_robot_rc9.cpp
 * 
 */

#include "bcap_funcid.h"

#include "medra_denso_robot.hpp"
#include "medra_bcap_service.hpp"

#include <sstream>
#include <iostream>
#include <cstring>

namespace medra_denso_robot {

std::string ConvertBSTRToString(const BSTR bstr)
{
  std::string strRet;
  char * chTmp;

  chTmp = ConvertWideChar2MultiByte(bstr);
  if (chTmp != NULL) {
    strRet = chTmp;
    free(chTmp);
  }

  return strRet;
}

BSTR ConvertStringToBSTR(const std::string& str)
{
  BSTR strRet = NULL;
  wchar_t * chTmp;

  chTmp = ConvertMultiByte2WideChar(str.c_str());
  if (chTmp != NULL) {
    strRet = SysAllocString(chTmp);
    free(chTmp);
  }

  return strRet;
}

MedraDensoRobot::MedraDensoRobot(
  const std::string& name, const int* mode, const std::string& ip_address, const int port, const int connect_timeout)
  : m_name(name), m_mode(mode), m_addr(ip_address)
{
  BCAPService_Ptr service = std::make_shared<medra_bcap_service::MedraBCAPService>(m_addr, port, connect_timeout);
  service->put_Type("tcp");
  service->Connect();
}

// Based on DensoControllerRC9::AddController()
HRESULT MedraDensoRobot::ControllerConnect()
{
    static const std::string CTRL_CONNECT_OPTION[BCAP_CONTROLLER_CONNECT_ARGS]
                                                = { "", "CaoProv.DENSO.VRC9", "localhost", "" };
    HRESULT hr = E_FAIL;
    int argc;

    std::stringstream ss;
    std::string strTmp;
    VARIANT_Ptr vntRet(new VARIANT());
    VARIANT_Vec vntArgs;

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_CONTROLLER_CONNECT_ARGS; argc++)
    {
        VARIANT_Ptr vntTmp(new VARIANT());
        VariantInit(vntTmp.get());

        vntTmp->vt = VT_BSTR;
        strTmp = CTRL_CONNECT_OPTION[argc];

        vntTmp->bstrVal = ConvertStringToBSTR(strTmp);

        vntArgs.push_back(*vntTmp.get());
    }

    hr = _bcap_service->ExecFunction(ID_CONTROLLER_CONNECT, vntArgs, vntRet);
    if (FAILED(hr))
        return hr;

    _controller_handle = vntRet->ulVal;

    return hr;
}

// Based on DensoRobot::ExecTakeArm
HRESULT MedraDensoRobot::ExecTakeArm()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  int32_t * pval;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = _controller_handle;
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"TakeArm");
        break;
      case 2:
        vntTmp->vt = (VT_ARRAY | VT_I4);
        vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
        SafeArrayAccessData(vntTmp->parray, (void**)&pval);
        pval[0] = ARM_GROUP;  // Arm group
        pval[1] = 1L;          // Keep
        SafeArrayUnaccessData(vntTmp->parray);
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return _bcap_service->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

// DensoRobot::ExecGiveArm
HRESULT MedraDensoRobot::ExecGiveArm()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = _controller_handle;
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GiveArm");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return _bcap_service->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

// Based on DensoRobot::ExecCurJnt
HRESULT MedraDensoRobot::ExecCurJnt(std::vector<double>& pose)
{
  HRESULT hr;

  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = _controller_handle;
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"HighCurJntEx");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = _bcap_service->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);

  if (SUCCEEDED(hr) && (vntRet->vt == (VT_ARRAY | VT_R8))) {
    uint32_t num;
    double* pdblval;

    num = vntRet->parray->rgsabound->cElements;
    SafeArrayAccessData(vntRet->parray, (void**)&pdblval);
    pose.resize(num - 1);
    std::copy(&pdblval[1], &pdblval[num], pose.begin());
    SafeArrayUnaccessData(vntRet->parray);
  }

  return hr;
}

// Based on DensoRobot::ExecSlaveMove
HRESULT MedraDensoRobot::ExecSlaveMove(const std::vector<double>& pose, std::vector<double>& joint)
{
  HRESULT hr = S_OK;
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = _controller_handle;
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"slvMove");
        break;
      case 2:
        hr = CreateSendParameter(pose, vntTmp, m_send_miniio, m_send_handio, m_recv_userio_offset, m_recv_userio_size,
                                 m_send_userio_offset, m_send_userio_size, m_send_userio);
        if (FAILED(hr)) {
          return hr;
        }
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = _bcap_service->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
  if (SUCCEEDED(hr)) {
    HRESULT hrTmp = ParseRecvParameter(
      vntRet, m_position, m_joint, m_trans, m_recv_miniio, m_recv_handio,
      m_timestamp, m_recv_userio, m_current);

    joint = m_joint;

    if (FAILED(hrTmp)) {
      hr = hrTmp;
    }
  }

  return hr;
}

// Based on DensoRobot::ChangeMode
HRESULT MedraDensoRobot::ChangeMode(int mode)
{
  HRESULT hr = S_OK;

  if (*m_mode == 0) {
    // Change to slave mode
    if (mode != 0) {
      hr = ExecSlaveMode("slvSendFormat", m_sendfmt);
      if (FAILED(hr)) {
        std::cerr << "Invalid argument value send_format = 0x%x" << m_sendfmt << std::endl;
        return hr;
      }
      hr = ExecSlaveMode("slvRecvFormat", m_recvfmt, m_tsfmt);
      if (FAILED(hr)) {
        std::cerr << "Invalid argument value recv_format = 0x%x" << m_recvfmt << std::endl;
        return hr;
      }
      hr = ExecTakeArm();
      if (FAILED(hr)) {
        return hr;
      }

      hr = ExecSlaveMode("slvChangeMode", mode);
      if (FAILED(hr)) {
        return hr;
      }

      m_memTimeout = _bcap_service->get_Timeout();
      m_memRetry = _bcap_service->get_Retry();
      if (mode & SLVMODE_SYNC_WAIT) {
        _bcap_service->put_Timeout(this->SLVMODE_TIMEOUT_SYNC);
      } else {
        _bcap_service->put_Timeout(this->SLVMODE_TIMEOUT_ASYNC);
      }
      std::cout <<
        "bcap-slave timeout changed to " << _bcap_service->get_Timeout() << " msec mode: 0x" << mode << std::endl;
      _bcap_service->put_Retry(3);
    }
  } else {
    _bcap_service->put_Timeout(m_memTimeout);
    _bcap_service->put_Retry(m_memRetry);

    hr = ExecSlaveMode("slvChangeMode", mode);
    ExecGiveArm();
  }

  return hr;
}

// Based on DensoRobot::ExecSlaveMode
HRESULT MedraDensoRobot::ExecSlaveMode(const std::string& name, int32_t format, int32_t option)
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  int32_t* pval;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = _controller_handle;
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = ConvertStringToBSTR(name);
        break;
      case 2:
        if (option == 0) {
          vntTmp->vt = VT_I4;
          vntTmp->lVal = format;
        } else {
          vntTmp->vt = (VT_ARRAY | VT_I4);
          vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
          SafeArrayAccessData(vntTmp->parray, (void**)&pval);
          pval[0] = format;
          pval[1] = option;
          SafeArrayUnaccessData(vntTmp->parray);
        }
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return _bcap_service->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

HRESULT MedraDensoRobot::CreateSendParameter(
  const std::vector<double>& pose, VARIANT_Ptr& send, const int miniio,
  const int handio, const int recv_userio_offset, const int recv_userio_size,
  const int send_userio_offset, const int send_userio_size,
  const std::vector<uint8_t>& send_userio)
{
  int type = *m_mode & SLVMODE_POSE;

  // Check pose type
  int joints = 0;
  switch (type) {
    case SLVMODE_POSE_P:
      joints = NUM_POSITION;
      break;
    case SLVMODE_POSE_J:
      joints = NUM_JOINT;
      break;
    case SLVMODE_POSE_T:
      joints = NUM_TRANS;
      break;
    default:
      return E_FAIL;
  }

  // TODO: do we need this control ? Why?
  // if(joints < pose.size())
  // {
    // return E_FAIL;
  // }

  // Check send format
  bool send_hio, send_mio, send_uio, recv_uio;
  send_hio = m_sendfmt & SENDFMT_HANDIO;
  send_mio = m_sendfmt & SENDFMT_MINIIO;
  send_uio = m_sendfmt & SENDFMT_USERIO;

  if (send_uio) {
    if (static_cast<std::size_t>(send_userio_size) < send_userio.size()) {
      return E_FAIL;
    }
  }

  // Check receive format
  recv_uio = m_recvfmt & RECVFMT_USERIO;

  // Number of arguments
  int num = 1 + send_hio + send_mio + 3 * send_uio + 2 * recv_uio;

  VARIANT* pvnt;
  double* pdbl;
  uint8_t* pbool;

  // Number of joints + option
  joints += 1;
  if (num == 1) {
    // Pose only
    send->vt = (VT_ARRAY | VT_R8);
    send->parray = SafeArrayCreateVector(VT_R8, 0, joints);
    SafeArrayAccessData(send->parray, (void**)&pdbl);
    memset(pdbl, 0, joints * sizeof(double));
    std::copy(pose.begin(), pose.end(), pdbl);
    SafeArrayUnaccessData(send->parray);
  } else {
    send->vt = (VT_ARRAY | VT_VARIANT);
    send->parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

    SafeArrayAccessData(send->parray, (void**)&pvnt);

    int offset = 0;

    // Pose
    {
      pvnt[offset].vt = (VT_ARRAY | VT_R8);
      pvnt[offset].parray = SafeArrayCreateVector(VT_R8, 0, joints);
      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      memset(pdbl, 0, joints * sizeof(double));
      std::copy(pose.begin(), pose.end(), pdbl);
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Mini I/O
    if (send_mio) {
      pvnt[offset].vt = VT_I4;
      pvnt[offset].lVal = miniio;

      offset++;
    }

    // Send User I/O
    if (send_uio) {
      pvnt[offset + 0].vt = VT_I4;
      pvnt[offset + 0].lVal = send_userio_offset;

      pvnt[offset + 1].vt = VT_I4;
      pvnt[offset + 1].lVal = send_userio_size * USERIO_ALIGNMENT;

      pvnt[offset + 2].vt = (VT_ARRAY | VT_UI1);
      pvnt[offset + 2].parray = SafeArrayCreateVector(VT_UI1, 0, send_userio_size);
      SafeArrayAccessData(pvnt[offset + 2].parray, (void**)&pbool);
      memset(pbool, 0, send_userio_size * sizeof(uint8_t));
      std::copy(send_userio.begin(), send_userio.end(), pbool);
      SafeArrayUnaccessData(pvnt[offset + 2].parray);

      offset += 3;
    }

    // Receive User I/O
    if (recv_uio) {
      pvnt[offset + 0].vt = VT_I4;
      pvnt[offset + 0].lVal = recv_userio_offset;

      pvnt[offset + 1].vt = VT_I4;
      pvnt[offset + 1].lVal = recv_userio_size * USERIO_ALIGNMENT;

      offset += 2;
    }

    // Hand I/O
    if (send_hio) {
      pvnt[offset].vt = VT_I4;
      pvnt[offset].lVal = handio;

      offset++;
    }

    SafeArrayUnaccessData(send->parray);
  }

  return S_OK;
}

HRESULT MedraDensoRobot::ParseRecvParameter(
  const VARIANT_Ptr& recv, std::vector<double>& position,
  std::vector<double>& joint, std::vector<double>& trans, int& miniio, int& handio,
  int& timestamp, std::vector<uint8_t>& recv_userio, std::vector<double>& current)
{
  int type = m_recvfmt & SLVMODE_POSE;

  // Check pose type
  int j1 = 0, j2 = 0;
  uint32_t joints = 0;
  std::vector<double>*pose1 = NULL, *pose2 = NULL;

  switch (type) {
    case RECVFMT_POSE_P:
      j1 = NUM_POSITION;
      pose1 = &position;
      break;
    case RECVFMT_POSE_J:
      j1 = NUM_JOINT;
      pose1 = &joint;
      break;
    case RECVFMT_POSE_T:
      j1 = NUM_TRANS;
      pose1 = &trans;
      break;
    case RECVFMT_POSE_PJ:
      j1 = NUM_POSITION;
      j2 = NUM_JOINT;
      pose1 = &position;
      pose2 = &joint;
      break;
    case RECVFMT_POSE_TJ:
      j1 = NUM_TRANS;
      j2 = NUM_JOINT;
      pose1 = &trans;
      pose2 = &joint;
      break;
    default:
      return E_FAIL;
  }

  joints = j1 + j2;

  // Check receive format
  bool recv_ts, recv_hio, recv_mio, recv_uio, recv_crt;
  recv_ts = m_recvfmt & RECVFMT_TIME;
  recv_hio = m_recvfmt & RECVFMT_HANDIO;
  recv_mio = m_recvfmt & RECVFMT_MINIIO;
  recv_uio = m_recvfmt & RECVFMT_USERIO;
  recv_crt = m_recvfmt & RECVFMT_CURRENT;

  // Number of arguments
  uint32_t num = 1 + recv_ts + recv_hio + recv_mio + recv_uio + recv_crt;

  HRESULT hr = S_OK;
  VARIANT* pvnt;
  double* pdbl;
  uint8_t* pbool;

  if (recv->vt == (VT_ARRAY | VT_R8)) {
    if (joints != recv->parray->rgsabound->cElements) {
      return E_FAIL;
    }

    // Pose only
    SafeArrayAccessData(recv->parray, (void**)&pdbl);
    pose1->resize(j1);
    std::copy(pdbl, &pdbl[j1], pose1->begin());
    if (pose2 != NULL) {
      pose2->resize(j2);
      std::copy(&pdbl[j1], &pdbl[joints], pose2->begin());
    }
    SafeArrayUnaccessData(recv->parray);
  } else if (recv->vt == (VT_ARRAY | VT_VARIANT)) {
    if (num != recv->parray->rgsabound->cElements) {
      return E_FAIL;
    }

    SafeArrayAccessData(recv->parray, (void**)&pvnt);

    int offset = 0;

    // Timestamp
    if (recv_ts) {
      if (pvnt[offset].vt != VT_I4) {
        hr = E_FAIL;
        goto exit_proc;
      }

      timestamp = pvnt[offset].lVal;

      offset++;
    }

    // Pose
    {
      if (
        (pvnt[offset].vt != (VT_ARRAY | VT_R8)) || (joints != pvnt[offset].parray->rgsabound->cElements))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      pose1->resize(j1);
      std::copy(pdbl, &pdbl[j1], pose1->begin());
      if (pose2 != NULL) {
        pose2->resize(j2);
        std::copy(&pdbl[j1], &pdbl[joints], pose2->begin());
      }
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Mini I/O
    if (recv_mio) {
      if (pvnt[offset].vt != VT_I4) {
        hr = E_FAIL;
        goto exit_proc;
      }

      miniio = pvnt[offset].lVal;

      offset++;
    }

    // User I/O
    if (recv_uio) {
      if (pvnt[offset].vt != (VT_ARRAY | VT_UI1)) {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pbool);
      recv_userio.resize(pvnt[offset].parray->rgsabound->cElements);
      std::copy(pbool, &pbool[pvnt[offset].parray->rgsabound->cElements], recv_userio.begin());
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Hand I/O
    if (recv_hio) {
      if (pvnt[offset].vt != VT_I4) {
        hr = E_FAIL;
        goto exit_proc;
      }

      handio = pvnt[offset].lVal;

      offset++;
    }

    // Current
    if (recv_crt) {
      if (
        (pvnt[offset].vt != (VT_ARRAY | VT_R8)) || (8 != pvnt[offset].parray->rgsabound->cElements))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      current.resize(8);
      std::copy(pdbl, &pdbl[8], current.begin());
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    exit_proc:
    SafeArrayUnaccessData(recv->parray);
  } else {
    return E_FAIL;
  }

  return hr;
}



}  // namespace medra_denso_robot


int main(int argc, char *argv[]) {
  std::string error_msg = "";
  const std::vector<double> dummy_pose = {1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0};
  const std::vector<double> dummy_joint_position = {1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0};
  std::vector<double> current_joint_position = dummy_joint_position;
  std::vector<double> new_joint_position = dummy_joint_position;

  // Setup
  const std::string ip_address = "192.168.0.1";
  const int port = 5007;
  const int connect_timeout = 2000;
  const int mode = 0x202;

  // service_start
  // controller_connect
  // TODO: clear errors
  // controller_getrobot
  medra_denso_robot::MedraDensoRobot robot("", &mode, ip_address, port, connect_timeout);

  // TODO: manual reset
  // TODO: turn on motors
  // TODO: set speed

  HRESULT result;
  result = robot.ExecTakeArm();
  if (FAILED(result)) {
    error_msg = "Failed to take arm.";
    goto err_proc;
  }

  // TODO: turn on slave mode
  result = robot.ChangeMode(mode);

  // move
  current_joint_position = dummy_joint_position;
  result = robot.ExecCurJnt(current_joint_position);
  if (FAILED(result) || current_joint_position == dummy_joint_position) {
    error_msg = "Failed to get current joint position.";
    goto err_proc;
  }

  for (int i = 0; i < 10; i++) {
    // add a small increment to each joint
    for (int j = 0; j < 6; j++) {
      new_joint_position[j] = new_joint_position[j] + 0.001;
    }
    // Use dummy pose because we are controlling in joint mode
    result = robot.ExecSlaveMove(dummy_pose, new_joint_position);
    if (FAILED(result)) {
      error_msg = "Failed to move robot.";
      goto err_proc;
    }
  }

  // Teardown (destructor)
  // robot_release
  // controller_disconnect
  // service_stop

  post_proc:
    return 0;
  err_proc:
    throw std::runtime_error(error_msg);
    return 1;
}
