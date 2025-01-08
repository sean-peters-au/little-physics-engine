#include "nbody/core/profile.hpp"
#include <iostream>
#include <iomanip>
#include <stack>
#include <vector>

namespace Profiling {

void Profiler::startSection(const std::string& name) {
    auto& instance = getInstance();
    auto& data = instance.sections[name];
    data.start_time = Clock::now();

    // Set up parent-child relationship
    if (!instance.scope_stack.empty()) {
        std::string parent = instance.scope_stack.top();
        data.profile_data.parent_name = parent;
        instance.sections[parent].profile_data.children.push_back(name);
    }

    instance.scope_stack.push(name);
}

void Profiler::endSection(const std::string& name) {
    auto& instance = getInstance();
    auto& data = instance.sections[name];
    Duration duration = Clock::now() - data.start_time;
    
    // Update timing statistics
    data.profile_data.total_time += duration;
    data.profile_data.self_time += duration;
    data.profile_data.call_count++;
    data.profile_data.min_time = std::min(data.profile_data.min_time, duration);
    data.profile_data.max_time = std::max(data.profile_data.max_time, duration);

    // Subtract this time from parent's self_time
    if (!data.profile_data.parent_name.empty()) {
        auto& parent_data = instance.sections[data.profile_data.parent_name];
        parent_data.profile_data.self_time -= duration;
    }

    instance.scope_stack.pop();
}

void Profiler::printStats() {
    auto& instance = getInstance();
    std::cout << "\nProfiling Statistics:\n";
    std::cout << std::fixed << std::setprecision(3);
    
    // Find root nodes (those without parents)
    std::vector<std::string> root_nodes;
    for (const auto& [name, data] : instance.sections) {
        if (data.profile_data.parent_name.empty()) {
            root_nodes.push_back(name);
        }
    }

    // Calculate total time for percentage calculations
    Duration total_time{0};
    for (const auto& root : root_nodes) {
        total_time += instance.sections[root].profile_data.total_time;
    }

    // Print tree starting from root nodes
    for (const auto& root : root_nodes) {
        printNode(root, "", true, total_time);
    }
}

void Profiler::printNode(const std::string& name, const std::string& prefix, bool is_last, Duration total_program_time) {
    auto& instance = getInstance();
    const auto& data = instance.sections[name];
    const auto& profile = data.profile_data;

    // Calculate percentages
    double total_pct = (profile.total_time.count() * 100.0) / total_program_time.count();
    double self_pct = (profile.self_time.count() * 100.0) / total_program_time.count();
    
    // Convert times to milliseconds
    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(profile.total_time).count();
    auto self_ms = std::chrono::duration_cast<std::chrono::milliseconds>(profile.self_time).count();

    // Print current node
    std::cout << prefix << (is_last ? "└── " : "├── ")
              << name << " [" << profile.call_count << " calls] "
              << total_ms << "ms (total: " << total_pct << "%, "
              << "self: " << self_pct << "%)\n";

    // Print children
    for (size_t i = 0; i < profile.children.size(); ++i) {
        bool last_child = (i == profile.children.size() - 1);
        std::string new_prefix = prefix + (is_last ? "    " : "│   ");
        printNode(profile.children[i], new_prefix, last_child, total_program_time);
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