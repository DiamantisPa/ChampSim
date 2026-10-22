#ifndef PROMETHEUS_RUN_H
#define PROMETHEUS_RUN_H

#include <string>

namespace prometheus
{
// the first trace on the command line (set by main before the cores initialize): per-workload
// side files (u-op-cost profiles) are named after it
inline std::string trace_name;
} // namespace prometheus

#endif
