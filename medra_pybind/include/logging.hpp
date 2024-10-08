
#ifndef logging_hpp
#define logging_hpp

#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <spdlog/details/null_mutex.h>
#include <spdlog/details/synchronous_factory.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

#include <mutex>

namespace py = pybind11;

enum LevelFilter : int {
  Off = 0,
  Trace = 5,
  Debug = 10,
  Info = 20,
  Warn = 30,
  Error = 40,
  Critical = 50,
};

LevelFilter map_level(spdlog::level::level_enum level) {
  switch (level) {
    case spdlog::level::trace:
      return LevelFilter::Trace;
    case spdlog::level::debug:
      return LevelFilter::Debug;
    case spdlog::level::info:
      return LevelFilter::Info;
    case spdlog::level::warn:
      return LevelFilter::Warn;
    case spdlog::level::err:
      return LevelFilter::Error;
    case spdlog::level::critical:
      return LevelFilter::Critical;
    case spdlog::level::n_levels:
    case spdlog::level::off:
    default:
      return LevelFilter::Off;
  }
}

template <typename Mutex>
class pybind11_sink : public spdlog::sinks::base_sink<Mutex>
{
public:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        py::gil_scoped_acquire acquire;
        const std::string target = "medra_bcap";
        if (py_logger_.is_none())
        {
            auto py_logging = py::module_::import("logging");
            py_logger_ = py_logging.attr("getLogger")(target);
        }
        std::string filename = msg.source.filename ? msg.source.filename : "";
        std::string msg_payload =
            std::string(msg.payload.begin(), msg.payload.end());
        auto record = py_logger_.attr("makeRecord")(
            target, static_cast<int>(map_level(msg.level)), filename,
            msg.source.line, msg_payload, py::none(), py::none());
        py_logger_.attr("handle")(record);
        py::gil_scoped_release release;
    }

    void flush_() override {}

private:
    py::object py_logger_ = py::none();
};

using pybind11_sink_mt = pybind11_sink<std::mutex>;
using pybind11_sink_st = pybind11_sink<spdlog::details::null_mutex>;

template <typename Factory = spdlog::synchronous_factory>
SPDLOG_INLINE std::shared_ptr<spdlog::logger> pybind11_mt(
    const std::string &logger_name)
{
    return Factory::template create<pybind11_sink_mt>(logger_name);
}

template <typename Factory = spdlog::synchronous_factory>
SPDLOG_INLINE std::shared_ptr<spdlog::logger> pybind11_st(
    const std::string &logger_name)
{
    return Factory::template create<pybind11_sink_st>(logger_name);
}

void setup_pybind11_logging()
{
    auto logger = pybind11_st("medra_bcap");
    spdlog::set_default_logger(logger);
    spdlog::set_pattern("%s:%# %v");
}

#endif // logging_hpp