#ifndef ROSCON_2023_REALTIME_WORKSHOP_TRACING_H_
#define ROSCON_2023_REALTIME_WORKSHOP_TRACING_H_

#include <cactus_rt/tracing.h>

void StartTracing(const char* app_name, const char* filename);
void StopTracing();
void RegisterThreadTracer(std::shared_ptr<cactus_rt::tracing::ThreadTracer> thread_tracer);

#endif
