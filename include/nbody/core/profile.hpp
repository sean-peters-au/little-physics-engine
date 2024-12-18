#pragma once

#include <string>
#include <chrono>
#include <unordered_map>

namespace Profiling {

class Profiler {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::nanoseconds;

    struct ProfileData {
        Duration total_time{0};
        uint64_t call_count{0};
        Duration min_time{Duration::max()};
        Duration max_time{Duration::min()};
    };

    static void startSection(const std::string& name);
    static void endSection(const std::string& name);
    static void printStats();
    static void reset();

private:
    struct SectionData {
        TimePoint start_time;
        ProfileData profile_data;
    };
    
    std::unordered_map<std::string, SectionData> sections;

    static Profiler& getInstance();
};

class ScopedProfiler {
public:
    ScopedProfiler(const std::string& name);
    ~ScopedProfiler();

private:
    std::string section_name;
};

} // namespace Profiling

#define PROFILE_SCOPE(name) Profiling::ScopedProfiler profiler##__LINE__(name) 