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

    // CHANGE: Always update parent relationship based on current scope
    if (!instance.scope_stack.empty()) {
        std::string parent = instance.scope_stack.top();
        
        // Only update if parent changed
        if (data.profile_data.parent_name != parent) {
            // Remove from old parent's children list if it exists
            if (!data.profile_data.parent_name.empty()) {
                auto& old_parent_children = instance.sections[data.profile_data.parent_name].profile_data.children;
                auto it = std::find(old_parent_children.begin(), old_parent_children.end(), name);
                if (it != old_parent_children.end()) {
                    old_parent_children.erase(it);
                }
            }
            
            // Add to new parent
            data.profile_data.parent_name = parent;
            auto& parent_children = instance.sections[parent].profile_data.children;
            if (std::find(parent_children.begin(), parent_children.end(), name) == parent_children.end()) {
                parent_children.push_back(name);
            }
        }
    } else {
        // No parent - this is a root node
        data.profile_data.parent_name.clear();
    }

    instance.scope_stack.push(name);
}

void Profiler::endSection(const std::string& name) {
    auto& instance = getInstance();
    
    // Safety check - don't pop if stack is empty
    if (instance.scope_stack.empty()) {
        std::cerr << "Warning: Trying to end section '" << name << "' but scope stack is empty\n";
        return;
    }

    // Safety check - ensure we're ending the correct section
    if (instance.scope_stack.top() != name) {
        std::cerr << "Warning: Ending section '" << name << "' but current scope is '" 
                  << instance.scope_stack.top() << "'\n";
        return;
    }

    auto& data = instance.sections[name];
    Duration duration = Clock::now() - data.start_time;
    
    // Update timing statistics
    data.profile_data.total_time += duration;
    data.profile_data.self_time += duration;
    data.profile_data.call_count++;
    data.profile_data.min_time = std::min(data.profile_data.min_time, duration);
    data.profile_data.max_time = std::max(data.profile_data.max_time, duration);

    // Subtract this time from parent's self_time if we have a parent
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

    // Calculate percentages, avoiding division by zero
    double total_pct = 0.0;
    double self_pct = 0.0;
    if (total_program_time.count() > 0) {
        total_pct = (profile.total_time.count() * 100.0) / total_program_time.count();
        self_pct = (profile.self_time.count() * 100.0) / total_program_time.count();
    }
    
    // Convert times to milliseconds
    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(profile.total_time).count();
    auto self_ms = std::chrono::duration_cast<std::chrono::milliseconds>(profile.self_time).count();

    // Print current node
    std::cout << prefix << (is_last ? "└── " : "├── ")
              << name << " [" << profile.call_count << " calls] "
              << total_ms << "ms (total: " << std::fixed << std::setprecision(2) << total_pct << "%, "
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

ScopedProfiler::ScopedProfiler(const std::string& name) : section_name(name), in_scope(false) {}
    
void ScopedProfiler::beginScope() {
    if (!in_scope) {  // Only start if we're not already in the scope
        Profiler::startSection(section_name);
        in_scope = true;
    }
}
    
ScopedProfiler::~ScopedProfiler() {
    if (in_scope) {  // Only end if we're actually in the scope
        Profiler::endSection(section_name);
        in_scope = false;
    }
}

} // namespace Profiling 