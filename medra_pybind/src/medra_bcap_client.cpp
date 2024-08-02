#include <iostream>
#include "stdint.h"
#include "bCAPClient/bcap_client.h"

#include <string.h>

#define E_BUF_FULL (0x83201483)

namespace SlvMode
{
    enum Modes
    {
        P0 = 0x001,
        J0 = 0x002,
        T0 = 0x003,
        P1 = 0x101,
        J1 = 0x102,
        T1 = 0x103,
        P2 = 0x201,
        J2 = 0x202,
        T2 = 0x203
    };

    enum Formats
    {
        None = 0x0000,
        HandIO = 0x0020,
        Electric = 0x0040,
        MiniIO = 0x0100,
        MiniAndHandIO = 0x0120,
        UserIO = 0x0200,
        UserAndHandIO = 0x0220,
        Ptype = 0x0001,
        Jtype = 0x0002,
        Ttype = 0x0003,
        PJtypes = 0x0004,
        TJtypes = 0x0005,
        Timestamp = 0x0010
    };
}

// class MedraBCAPClient
class MedraBCAPClient
{
public:
    MedraBCAPClient();
    ~MedraBCAPClient();

    int connect();
    int disconnect();

    int clear_error();

    int get_robot_handle();
    int manual_reset();

    int take_arm();
    int give_arm();

    int motor_on();
    int motor_off();

    int get_current_angles();
    void print_current_angles();

    void move_robot();

    int enable_slv_mode();
    int disable_slv_mode();

    int send_servo_command(double *target);

private:
    int m_fd, i;
    long *plData;
    uint32_t hCtrl, hRob;
    double *pdData, dAng[8];
    BSTR bstr1, bstr2, bstr3, bstr4;
    VARIANT vntParam, vntRet, *pvntData;
    HRESULT hr;
    std::string m_ip_address = "192.168.0.1";
    std::string m_connect;
    int m_timeout = 1000;
    int m_retry = 3;
};

MedraBCAPClient::MedraBCAPClient()
{
    std::cout << "MedraBCAPClient constructor" << std::endl;
}

MedraBCAPClient::~MedraBCAPClient()
{
    std::cout << "MedraBCAPClient destructor" << std::endl;
}

int MedraBCAPClient::connect()
{
    std::cout << "MedraBCAPClient connect" << std::endl;
    m_connect = "udp:" + m_ip_address + ":5007";
    hr = bCap_Open_Client(m_connect.c_str(), m_timeout, m_retry, &m_fd);
    if (SUCCEEDED(hr))
    {
        std::cout << "MedraBCAPClient connect success" << std::endl;
    }
    else
    {
        std::cout << "MedraBCAPClient connect failed" << std::endl;
        throw std::runtime_error("MedraBCAPClient connect failed");
    }

    hr = bCap_ServiceStart(m_fd, NULL);

    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient service start failed");
    }

    // connect to controller
    bstr1 = SysAllocString(L"");
    bstr2 = SysAllocString(L"CaoProv.DENSO.VRC9");
    bstr3 = SysAllocString(L"localhost");
    bstr4 = SysAllocString(L"");
    hr = bCap_ControllerConnect(m_fd, bstr1, bstr2, bstr3, bstr4, &hCtrl);
    SysFreeString(bstr1);
    SysFreeString(bstr2);
    SysFreeString(bstr3);
    SysFreeString(bstr4);

    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient controller connect failed");
    }

    return 0;
}

int MedraBCAPClient::disconnect()
{
    std::cout << "MedraBCAPClient disconnect" << std::endl;

    bCap_RobotRelease(m_fd, &hRob);
    bCap_ControllerDisconnect(m_fd, &hCtrl);
    bCap_ServiceStop(m_fd);
    bCap_Close_Client(&m_fd);
    return 0;
}

int MedraBCAPClient::clear_error()
{
    std::cout << "MedraBCAPClient clear_error" << std::endl;

    bstr1 = SysAllocString(L"ClearError");
    vntParam.bstrVal = SysAllocString(L"");
    vntParam.vt = VT_BSTR;
    hr = bCap_ControllerExecute(m_fd, hCtrl, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient clear error failed");
    }
    return 0;
}

int MedraBCAPClient::get_robot_handle()
{
    std::cout << "MedraBCAPClient get_robot_handle" << std::endl;

    bstr1 = SysAllocString(L"Robot");
    bstr2 = SysAllocString(L"");
    hr = bCap_ControllerGetRobot(m_fd, hCtrl, bstr1, bstr2, &hRob);
    SysFreeString(bstr1);
    SysFreeString(bstr2);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient get robot handle failed");
    }
    return 0;
}

int MedraBCAPClient::manual_reset()
{
    std::cout << "MedraBCAPClient manual_reset" << std::endl;

    bstr1 = SysAllocString(L"ManualReset");
    vntParam.bstrVal = SysAllocString(L"");
    vntParam.vt = VT_BSTR;
    hr = bCap_ControllerExecute(m_fd, hCtrl, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient manual reset failed");
    }
    return 0;
}

int MedraBCAPClient::take_arm()
{
    std::cout << "MedraBCAPClient take_arm" << std::endl;


    SafeArrayAccessData(vntParam.parray, (void**)&pvntData);

    std::cout << 0 << std::endl;

    std::cout << 1 << std::endl;
    bstr1 = SysAllocString(L"TakeArm");

    std::cout << 2 << std::endl;
    vntParam.vt = (VT_ARRAY | VT_I4);
    vntParam.parray = SafeArrayCreateVector(VT_I4, 0, 2);
    SafeArrayAccessData(vntParam.parray, (void **)&plData);
    plData[0] = 0;
    plData[1] = 1L;
    SafeArrayUnaccessData(vntParam.parray);

    std::cout << 3 << std::endl;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    // SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient take arm failed" + std::to_string(hr));
    }
    return 0;
}

int MedraBCAPClient::give_arm()
{
    std::cout << "MedraBCAPClient give_arm" << std::endl;

    bstr1 = SysAllocString(L"Givearm");
    vntParam.bstrVal = SysAllocString(L"");
    vntParam.vt = VT_BSTR;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient give arm failed");
    }
    return 0;
}

int MedraBCAPClient::motor_on()
{
    std::cout << "MedraBCAPClient motor_on" << std::endl;

    bstr1 = SysAllocString(L"Motor");
    vntParam.bstrVal = SysAllocString(L"1");
    vntParam.vt = VT_BSTR;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient motor on failed");
    }
    return 0;
}

int MedraBCAPClient::motor_off()
{
    std::cout << "MedraBCAPClient motor_off" << std::endl;

    bstr1 = SysAllocString(L"Motor");
    vntParam.bstrVal = SysAllocString(L"0");
    vntParam.vt = VT_BSTR;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient motor off failed");
    }
    return 0;
}

int MedraBCAPClient::get_current_angles()
{
    std::cout << "MedraBCAPClient get_current_angles" << std::endl;

    bstr1 = SysAllocString(L"CurJnt");
    vntParam.bstrVal = SysAllocString(L"");
    vntParam.vt = VT_BSTR;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SafeArrayAccessData(vntRet.parray, (void **)&pdData);
    memcpy(dAng, pdData, 8 * sizeof(double));
    SafeArrayUnaccessData(vntRet.parray);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
    {
        throw std::runtime_error("MedraBCAPClient get current angles failed");
    }
    return 0;
}

void MedraBCAPClient::print_current_angles()
{
    // print dAng
    std::cout << "Current angles: "
              << dAng[0] << " "
              << dAng[1] << " "
              << dAng[2] << " "
              << dAng[3] << " "
              << dAng[4] << " "
              << dAng[5] << " "
              << dAng[6] << " "
              << dAng[7] << std::endl;
}

int MedraBCAPClient::enable_slv_mode()
{
    std::cout << "MedraBCAPClient enable_slv_mode" << std::endl;
    // Change slvSendFormat
    bstr1 = SysAllocString(L"slvSendFormat");
    vntParam.vt = VT_I4;
    vntParam.lVal = SlvMode::None;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
        throw std::runtime_error("Failed to change slvSendFormat");

    // Change slvRecvFormat
    bstr1 = SysAllocString(L"slvRecvFormat");
    vntParam.vt = VT_I4;
    vntParam.lVal = SlvMode::J0;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
        throw std::runtime_error("Failed to change slvRecvFormat");

    // Start slave mode
    bstr1 = SysAllocString(L"slvChangeMode");
    vntParam.vt = VT_I4;
    vntParam.lVal = SlvMode::J2;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
        throw std::runtime_error("Failed to start slave mode");
    return 0;
}

int MedraBCAPClient::disable_slv_mode()
{
    std::cout << "MedraBCAPClient disable_slv_mode" << std::endl;
    bstr1 = SysAllocString(L"slvChangeMode");
    vntParam.vt = VT_I4;
    vntParam.lVal = SlvMode::None;
    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);
    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
        throw std::runtime_error("Failed to stop slave mode");
    return 0;
}

void MedraBCAPClient::move_robot()
{
    std::cout << "MedraBCAPClient move_robot" << std::endl;

    get_current_angles();
    enable_slv_mode();
    
    double target[8];
    for (int i = 0; i < 8; i++)
    {
        target[i] = dAng[i];
    }
    for (int i = 0; i < 100; i++)
    {
        target[5] += 0.2;
        send_servo_command(target);
    }
    // for (int i = 0; i < 100; i++)
    // {
    //     target[5] -= 0.2;
    //     send_servo_command(target);
    // }

    disable_slv_mode();

    return;
}


int MedraBCAPClient::send_servo_command(double *target)
{
    std::cout << "MedraBCAPClient send_servo_command" << std::endl;
    bstr1 = SysAllocString(L"slvMove");
    vntParam.vt = (VT_VARIANT | VT_ARRAY);
    vntParam.parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
    SafeArrayAccessData(vntParam.parray, (void **)&pvntData);
    pvntData[0].vt = (VT_R8 | VT_ARRAY);
    pvntData[0].parray = SafeArrayCreateVector(VT_R8, 0, 8);
    SafeArrayAccessData(pvntData[0].parray, (void **)&pdData);
    for (i = 0; i < 8; i++)
    {
        pdData[i] = target[i];
    }
    SafeArrayUnaccessData(pvntData[0].parray);
    pvntData[1].vt = VT_I4;
    pvntData[1].lVal = 0x1000000;
    SafeArrayUnaccessData(vntParam.parray);

    hr = bCap_RobotExecute(m_fd, hRob, bstr1, vntParam, &vntRet);

    SysFreeString(bstr1);
    VariantClear(&vntParam);
    VariantClear(&vntRet);
    if (FAILED(hr))
        throw std::runtime_error("Failed to send servo command");
    return 0;
}

// main
int main()
{
    {
        MedraBCAPClient client;
        client.connect();
        client.get_robot_handle();
        client.clear_error();
        client.manual_reset();
        client.take_arm();
        client.motor_on();
        client.get_current_angles();

        client.print_current_angles();
        client.move_robot();

        client.give_arm();
        client.motor_off();
        client.disconnect();
    }
    return 0;
}
