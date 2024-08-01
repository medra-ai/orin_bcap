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
  const std::string& name, const int* mode, const std::string& ip_address)
  : m_name(name), m_mode(mode), m_addr(ip_address)
{
  BCAPService_Ptr service = std::make_shared<medra_bcap_service::MedraBCAPService>(m_addr);
  service->put_Type("tcp");

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

    _handle = vntRet->ulVal;

    return hr;
}

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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"TakeArm");
        break;
      case 2:
        vntTmp->vt = (VT_ARRAY | VT_I4);
        vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
        SafeArrayAccessData(vntTmp->parray, (void**)&pval);
        pval[0] = m_ArmGroup;  // Arm group
        pval[1] = 1L;          // Keep
        SafeArrayUnaccessData(vntTmp->parray);
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GiveArm");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

}  // namespace medra_denso_robot