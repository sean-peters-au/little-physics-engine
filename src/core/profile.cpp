#include "nbody/core/profile.hpp"
#include <iostream>
#include <iomanip>

namespace Profiling {

void Profiler::startSection(const std::string& name) {
    auto& data = getInstance().sections[name];
    data.start_time = Clock::now();
}

void Profiler::endSection(const std::string& name) {
    auto& data = getInstance().sections[name];
    Duration duration = Clock::now() - data.start_time;
    data.profile_data.total_time += duration;
    data.profile_data.call_count++;
    data.profile_data.min_time = std::min(data.profile_data.min_time, duration);
    data.profile_data.max_time = std::max(data.profile_data.max_time, duration);
}

void Profiler::printStats() {
    auto& instance = getInstance();
    std::cout << "\nProfiling Statistics:\n";
    std::cout << std::fixed << std::setprecision(3);
    
    // Calculate total time across all sections for percentage calculation
    Duration total_all_sections{0};
    for (const auto& [name, data] : instance.sections) {
        total_all_sections += data.profile_data.total_time;
    }

    for (const auto& [name, data] : instance.sections) {
        auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(data.profile_data.total_time).count();
        auto avg_ms = total_ms / static_cast<double>(data.profile_data.call_count);
        auto min_ms = std::chrono::duration_cast<std::chrono::milliseconds>(data.profile_data.min_time).count();
        auto max_ms = std::chrono::duration_cast<std::chrono::milliseconds>(data.profile_data.max_time).count();
        
        double percentage = (data.profile_data.total_time.count() * 100.0) / total_all_sections.count();
        
        std::cout << name << ":\n"
                  << "  Calls: " << data.profile_data.call_count << "\n"
                  << "  Total: " << total_ms << "ms (" << percentage << "%)\n"
                  << "  Avg: " << avg_ms << "ms\n"
                  << "  Min: " << min_ms << "ms\n"
                  << "  Max: " << max_ms << "ms\n";
    }
}

void Profiler::reset() {
    getInstance().sections.clear();
}

Profiler& Profiler::getInstance() {
    static Profiler instance;
    return instance;
}

ScopedProfiler::ScopedProfiler(const std::string& name) : section_name(name) {
    Profiler::startSection(section_name);
}

ScopedProfiler::~ScopedProfiler() {
    Profiler::endSection(section_name);
}

} // namespace Profiling 