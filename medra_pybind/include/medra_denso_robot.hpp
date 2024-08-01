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

    // Robot command constants
    static constexpr int BCAP_ROBOT_EXECUTE_ARGS = 3;
    static constexpr int ARM_GROUP = 0;

    // From denso_robot_core_interfaces::msg::UserIO
    static constexpr int USERIO_ALIGNMENT=8;

    enum
    {
    NUM_POSITION = 7,
    NUM_JOINT = 8,
    NUM_TRANS = 10
    };

    enum
    {
        SLVMODE_NONE = 0,
        SLVMODE_POSE_P = 0x0001,
        SLVMODE_POSE_J = 0x0002,
        SLVMODE_POSE_T = 0x0003,
        SLVMODE_POSE = 0x000F,
        SLVMODE_ASYNC = 0x0100,
        SLVMODE_SYNC_WAIT = 0x0200,
    };

    enum
    {
        SENDFMT_NONE = 0,
        SENDFMT_HANDIO = 0x0020,
        SENDFMT_MINIIO = 0x0100,
        SENDFMT_USERIO = 0x0200,
    };

    enum
    {
        RECVFMT_NONE = 0,
        RECVFMT_POSE_P = 0x0001,
        RECVFMT_POSE_J = 0x0002,
        RECVFMT_POSE_T = 0x0003,
        RECVFMT_POSE_PJ = 0x0004,
        RECVFMT_POSE_TJ = 0x0005,
        RECVFMT_POSE = 0x000F,
        RECVFMT_TIME = 0x0010,
        RECVFMT_HANDIO = 0x0020,
        RECVFMT_CURRENT = 0x0040,
        RECVFMT_MINIIO = 0x0100,
        RECVFMT_USERIO = 0x0200,
    };

    enum
    {
        TSFMT_MILLISEC = 0,
        TSFMT_MICROSEC = 1,
    };

    enum
    {
        SLVMODE_TIMEOUT_SYNC = 16,
        SLVMODE_TIMEOUT_ASYNC = 8,
    };

    // Constructor and destructor
    MedraDensoRobot(const std::string & name, const int * mode, const std::string & ip_address, const int port, const int connect_timeout);
    ~MedraDensoRobot();

    // Controller commands
    HRESULT ControllerConnect();

    // Robot commands
    HRESULT ExecTakeArm();
    HRESULT ExecGiveArm();



    HRESULT ExecCurJnt(std::vector<double>& pose);
    HRESULT ExecSlaveMove(const std::vector<double>& pose, std::vector<double>& joint);

private:
    HRESULT ChangeMode(int mode);
    HRESULT ExecSlaveMode(const std::string& name, int32_t format, int32_t option=0);
    
    HRESULT CreateSendParameter(
        const std::vector<double>& pose, VARIANT_Ptr& send, const int miniio = 0,
        const int handio = 0, const int recv_userio_offset = 0, const int recv_userio_size = 0,
        const int send_userio_offset = 0, const int send_userio_size = 0,
        const std::vector<uint8_t>& send_userio = std::vector<uint8_t>());
    HRESULT ParseRecvParameter(
        const VARIANT_Ptr& recv, std::vector<double>& position, std::vector<double>& joint,
        std::vector<double>& trans, int& miniio, int& handio, int& timestamp,
        std::vector<uint8_t>& recv_userio, std::vector<double>& current);


    BCAPService_Ptr _bcap_service;
    Handle _controller_handle;

    std::string m_name;
    const int * m_mode;
    std::string m_addr;

    uint32_t m_memTimeout;
    unsigned int m_memRetry;
    int m_tsfmt, m_timestamp;
    int m_sendfmt, m_send_miniio, m_send_handio;
    int m_recvfmt, m_recv_miniio, m_recv_handio;
    int m_send_userio_offset, m_send_userio_size;
    int m_recv_userio_offset, m_recv_userio_size;
    std::vector<uint8_t> m_send_userio, m_recv_userio;
    std::vector<double> m_position, m_joint, m_trans, m_current;

};


}  // namespace medra_denso_robot

#endif  // MEDRA_DENSO_ROBOT_H
