/**
 * @file profile.hpp
 * @brief High-precision performance profiling system for timing code sections
 *
 * This profiling system provides tools for measuring execution time of code sections
 * in a hierarchical manner (parent-child scopes). It:
 * - Uses RAII to automatically track scope durations
 * - Aggregates statistics like total time, call count, min/max times
 * - Computes and prints percentages of total execution time per section
 * - Displays a tree-structured summary
 *
 * Example usage:
 * @code
 * void myFunc() {
 *     PROFILE_SCOPE("MyFunction");  // Automatically times this scope
 *     // ... code ...
 * }  // endSection called automatically here
 *
 * Profiling::Profiler::printStats();  // Print timing results somewhere in your code
 * @endcode
 */

#pragma once

#include <string>
#include <chrono>
#include <unordered_map>
#include <vector>
#include <stack>

namespace Profiling {

/**
 * @brief Manages timing data across the application.
 *
 * This follows a singleton pattern. Use the static methods to start/end
 * sections and to print/reset the statistics.
 */
class Profiler {
public:
    using Clock     = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration  = std::chrono::nanoseconds;

    /**
     * @brief Stores aggregated timing statistics for a named scope
     */
    struct ProfileData {
        Duration total_time{0};        ///< Accumulated total time for this scope
        Duration self_time{0};         ///< Time excluding children’s time
        uint64_t call_count{0};        ///< Number of times this scope was entered
        Duration min_time{Duration::max()};
        Duration max_time{0};

        std::string parent_name;       ///< Parent scope in the tree
        std::vector<std::string> children; ///< Child scope names
    };

    /**
     * @brief Start timing a named scope, pushing it on the internal scope stack.
     * @param name The scope’s name (must be ended with endSection).
     */
    static void startSection(const std::string& name);

    /**
     * @brief End timing a named scope, popping it from the scope stack.
     * @param name The scope’s name (must match the most recent startSection).
     */
    static void endSection(const std::string& name);

    /**
     * @brief Print a hierarchical summary of all timing data to stdout.
     *
     * Shows the total time, self time, call counts, and percentage of total
     * program time for each scope. Child scopes are listed under parents
     * with ASCII tree lines (like ├── and └──).
     */
    static void printStats();

    /**
     * @brief Reset all recorded profiling data (clears the Profiler’s map).
     */
    static void reset();

private:
    /**
     * @brief Internal structure to hold timing state for each named section.
     */
    struct SectionData {
        TimePoint start_time;   ///< The last time we started this scope
        ProfileData profile_data;
    };

    // Map from scope name -> the data we collect about that scope.
    std::unordered_map<std::string, SectionData> sections;

    // Stack of active scope names (the “call stack” for profiling).
    std::stack<std::string> scope_stack;

    // Private constructor for singleton
    Profiler() = default;

    /**
     * @brief Return the singleton instance of this profiler.
     */
    static Profiler& getInstance();

    /**
     * @brief Recursive helper for printStats() that prints one scope node.
     */
    static void printNode(const std::string& name,
                          const std::string& prefix,
                          bool is_last,
                          Duration total_program_time);
};

/**
 * @brief RAII guard that automatically starts timing in the constructor
 *        and ends timing in the destructor.
 */
class ScopedProfiler {
public:
    /**
     * @brief Constructor: immediately starts timing the given scope.
     */
    explicit ScopedProfiler(std::string  name);

    /**
     * @brief Destructor: ends the scope’s timing.
     */
    ~ScopedProfiler();

    // No copy allowed
    ScopedProfiler(const ScopedProfiler&) = delete;
    ScopedProfiler& operator=(const ScopedProfiler&) = delete;

private:
    std::string section_name;
};

} // namespace Profiling

/**
 * @brief Convenience macro for scoping. Creates a local ScopedProfiler
 *        whose destructor will end the scope automatically.
 */
#define PROFILE_SCOPE(name) \
    ::Profiling::ScopedProfiler _scopedProfiler##__LINE__ { name }