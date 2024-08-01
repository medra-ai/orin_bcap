#ifndef MEDRA_DENSO_ROBOT_H
#define MEDRA_DENSO_ROBOT_H

#include <string>
#include "dn_common.h"

#include "medra_bcap_service.hpp"


namespace medra_denso_robot {

typedef std::string Name;
typedef uint32_t Handle;
typedef BCAPService_Ptr Service;


// String conversion utils
std::string ConvertBSTRToString(const BSTR bstr);
BSTR ConvertStringToBSTR(const std::string& str);


class MedraDensoRobot
{
public:
    // Controller command constants
    static constexpr int BCAP_CONTROLLER_CONNECT_ARGS = 4;
    static constexpr int BCAP_CONTROLLER_EXECUTE_ARGS = 3;

    MedraDensoRobot(const std::string& name, const int* mode, const std::string& ip_address);
    ~MedraDensoRobot();

    HRESULT ControllerConnect();


private:

    BCAPService_Ptr _bcap_service;
    Handle _handle;

    std::string m_name;
    const int * m_mode;
    std::string m_addr;
};


}  // namespace medra_denso_robot

#endif  // MEDRA_DENSO_ROBOT_H
