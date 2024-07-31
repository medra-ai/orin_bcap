#ifndef MEDRA_BCAP_SERVICE_HPP_
#define MEDRA_BCAP_SERVICE_HPP_
/**
 * Service for communication with the Denso robot controller.
 * Creates and maintains connections with the robot controller using the bCAP driver.
 * Based on 
 * https://github.com/DENSORobot/denso_robot_ros2/blob/humble/bcap_service/include/bcap_service/bcap_service.h
 */

#include <string>

#include "variant_allocator.hpp"

#include "dn_common.h"
#include "dn_device.h"


typedef std::pair<int32_t,uint32_t> KeyHandle;
typedef std::vector<KeyHandle> KeyHandle_Vec;

struct variant_deleter
{
  void operator()(VARIANT *p) const
  {
    VariantClear(p);
    delete p;
  }
};

typedef std::unique_ptr<VARIANT, variant_deleter> VARIANT_Ptr;
typedef std::vector<VARIANT, VariantAllocator<VARIANT> > VARIANT_Vec;


namespace medra_bcap_service {

class MedraBCAPService
{
public:
  MedraBCAPService(const std::string& ip_address);
  virtual ~MedraBCAPService();

  HRESULT Connect();
  HRESULT Disconnect();

  HRESULT StartService();
  HRESULT StopService();

  const std::string& get_Type() const;
  void put_Type(const std::string& type);

  uint32_t get_Timeout() const;
  void put_Timeout(uint32_t value);

  unsigned int get_Retry() const;
  void put_Retry(unsigned int value);

  HRESULT ExecFunction(
    int32_t func_id, VARIANT_Vec& vntArgs,
    VARIANT_Ptr& vntRet);

private:
//   bool CallFunction(
//   const std::shared_ptr<bcap_service_interfaces::srv::Bcap::Request> request,
//     std::shared_ptr<bcap_service_interfaces::srv::Bcap::Response> response);

  // Connect parameters
  std::string m_type, m_addr;
  int m_port, m_timeout, m_retry, m_wait;

  // Socket parameters
  int m_fd;

  // ServiceStart parameters
  int m_wdt, m_invoke;

  // Handle vector
  KeyHandle_Vec m_vecKH;

  // Service
//   rclcpp::Service<bcap_service_interfaces::srv::Bcap>::SharedPtr m_svr;
};

}  // namespace medra_bcap_service

#endif  // MEDRA_BCAP_SERVICE_HPP_