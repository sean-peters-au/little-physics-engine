/**
 * @file profile.cpp
 * @brief Implementation of the profiling system described in profile.hpp
 */

#include "core/profile.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <utility>

namespace Profiling {

// ------------------ Profiler Singleton Methods ------------------

Profiler& Profiler::getInstance() {
    static Profiler instance;
    return instance;
}

void Profiler::startSection(const std::string& name) {
    auto& instance = getInstance();
    auto& section  = instance.sections[name];

    // Record the start time
    section.start_time = Clock::now();

    // If we have a currently active parent scope, record that relationship
    if (!instance.scope_stack.empty()) {
        const std::string& parentName = instance.scope_stack.top();

        // If the parent changed since last time, remove from old parent's children
        if (!section.profile_data.parent_name.empty() &&
            section.profile_data.parent_name != parentName)
        {
            auto& oldParentChildren =
                instance.sections[section.profile_data.parent_name].profile_data.children;
            oldParentChildren.erase(
                std::remove(oldParentChildren.begin(), oldParentChildren.end(), name),
                oldParentChildren.end()
            );
        }

        // Set the new parent
        section.profile_data.parent_name = parentName;

        // Add to parent's list of children if not already present
        auto& parentKids = instance.sections[parentName].profile_data.children;
        if (std::find(parentKids.begin(), parentKids.end(), name) == parentKids.end()) {
            parentKids.push_back(name);
        }
    } else {
        // No parent => this is a root node
        section.profile_data.parent_name.clear();
    }

    // Push this scope onto the stack
    instance.scope_stack.push(name);
}

void Profiler::endSection(const std::string& name) {
    auto& instance = getInstance();

    // Safety checks
    if (instance.scope_stack.empty()) {
        std::cerr << "[Profiler] Warning: endSection(\"" << name << "\") but scope stack empty.\n";
        return;
    }
    if (instance.scope_stack.top() != name) {
        std::cerr << "[Profiler] Warning: endSection(\"" << name
                  << "\") but top of stack is \"" << instance.scope_stack.top() << "\".\n";
        return;
    }

    auto endTime = Clock::now();
    auto& section = instance.sections[name];

    // Calculate how long this scope took
    Duration const duration = endTime - section.start_time;

    // Update total time, self time, call count, min/max
    section.profile_data.total_time += duration;
    section.profile_data.self_time  += duration;
    section.profile_data.call_count += 1;

    if (duration < section.profile_data.min_time) {
        section.profile_data.min_time = duration;
    }
    if (duration > section.profile_data.max_time) {
        section.profile_data.max_time = duration;
    }

    // Subtract this scope’s duration from its parent’s self_time
    if (!section.profile_data.parent_name.empty()) {
        auto& parentData = instance.sections[section.profile_data.parent_name].profile_data;
        parentData.self_time -= duration;
    }

    // Pop the scope
    instance.scope_stack.pop();
}

void Profiler::printStats() {
    auto& instance = getInstance();
    std::cout << "\nProfiling Statistics:\n";

    // Gather root nodes (those with no parent)
    std::vector<std::string> roots;
    roots.reserve(instance.sections.size());
    for (auto& [name, sdata] : instance.sections) {
        if (sdata.profile_data.parent_name.empty()) {
            roots.push_back(name);
        }
    }

    // Compute total program time by summing top-level nodes
    Duration totalTime{0};
    for (auto& r : roots) {
        totalTime += instance.sections[r].profile_data.total_time;
    }

    // Print each root node (with the rest of the hierarchy inside)
    for (size_t i = 0; i < roots.size(); ++i) {
        bool const isLast = (i == roots.size() - 1);
        printNode(roots[i], "", isLast, totalTime);
    }
}

void Profiler::printNode(const std::string& name,
                         const std::string& prefix,
                         bool isLast,
                         Duration totalProgramTime)
{
    const auto& instance = getInstance();
    const auto& pd       = instance.sections.at(name).profile_data;

    // Compute percentages (avoid div by zero)
    double totalPercent = 0.0;
    double selfPercent = 0.0;
    if (totalProgramTime.count() > 0) {
        totalPercent = (pd.total_time.count() * 100.0) / totalProgramTime.count();
        selfPercent  = (pd.self_time.count()  * 100.0) / totalProgramTime.count();
    }

    // Convert total_time to ms for printing
    auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(pd.total_time).count();
    // auto selfMs  = std::chrono::duration_cast<std::chrono::milliseconds>(pd.self_time).count();
    // If you want to print self time in ms too, uncomment above and adapt printing.

    std::cout << prefix << (isLast ? "└── " : "├── ")
              << name << " [" << pd.call_count << " calls] "
              << totalMs << "ms (total: "
              << std::fixed << std::setprecision(2) << totalPercent << "%, "
              << "self: " << selfPercent << "%)\n";

    // Print children
    for (size_t i = 0; i < pd.children.size(); ++i) {
        bool const childLast = (i == pd.children.size() - 1);
        // Next prefix includes vertical line or blank space
        std::string const childPrefix = prefix + (isLast ? "    " : "│   ");
        printNode(pd.children[i], childPrefix, childLast, totalProgramTime);
    }
}

void Profiler::reset() {
    getInstance().sections.clear();
    // stack will be empty if nothing is running at reset time
}

// ------------------ ScopedProfiler RAII Wrapper ------------------

ScopedProfiler::ScopedProfiler(std::string  name)
    : section_name(std::move(name))
{
    Profiler::startSection(section_name);
}

ScopedProfiler::~ScopedProfiler() {
    Profiler::endSection(section_name);
}

} // namespace Profiling