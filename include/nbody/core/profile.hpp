/**
 * @file profile.hpp
 * @brief High-precision performance profiling system for timing code sections
 *
 * This profiling system provides tools for measuring execution time of code sections
 * with nanosecond precision. It supports:
 * - Named section timing with automatic scope-based measurement
 * - Aggregated statistics (total time, call count, min/max times)
 * - Percentage of total execution time per section
 * - Thread-safe singleton design
 *
 * Example usage:
 * Do not use PROFILE_SCOPE in a global scope, use it in a function or add { } (e.g. for a switch statement)
 * @code
 * {
 *     PROFILE_SCOPE("MyFunction");  // Automatically times this scope
 *     // ... code to profile ...
 * }
 * Profiling::Profiler::printStats();  // Print timing results
 * @endcode
 */

#pragma once

#include <string>
#include <chrono>
#include <unordered_map>

namespace Profiling {

/**
 * @brief Core profiler class that manages timing measurements
 * 
 * Implements a singleton pattern to track timing data across the application.
 * All timing data is stored until explicitly reset.
 */
class Profiler {
public:
    /// High-precision clock type for timing measurements
    using Clock = std::chrono::high_resolution_clock;
    /// Time point type for storing start/end times
    using TimePoint = std::chrono::time_point<Clock>;
    /// Duration type for storing elapsed time
    using Duration = std::chrono::nanoseconds;

    /**
     * @brief Contains aggregated timing statistics for a profiled section
     */
    struct ProfileData {
        Duration total_time{0};           ///< Total time spent in this section
        uint64_t call_count{0};          ///< Number of times this section was executed
        Duration min_time{Duration::max()}; ///< Minimum single-call duration
        Duration max_time{Duration::min()}; ///< Maximum single-call duration
    };

    /**
     * @brief Starts timing a named section
     * @param name Unique identifier for the code section
     */
    static void startSection(const std::string& name);

    /**
     * @brief Ends timing a named section and updates statistics
     * @param name Unique identifier for the code section (must match startSection)
     */
    static void endSection(const std::string& name);

    /**
     * @brief Prints formatted timing statistics to stdout
     * 
     * Output includes:
     * - Total execution time per section
     * - Percentage of total program time
     * - Number of calls
     * - Average/min/max execution times
     */
    static void printStats();

    /**
     * @brief Clears all accumulated timing data
     */
    static void reset();

private:
    /**
     * @brief Internal data structure for tracking active timing sessions
     */
    struct SectionData {
        TimePoint start_time;      ///< Start time of current execution
        ProfileData profile_data;  ///< Aggregated statistics
    };
    
    std::unordered_map<std::string, SectionData> sections;

    /**
     * @brief Gets the singleton instance of the profiler
     * @return Reference to the global profiler instance
     */
    static Profiler& getInstance();
};

/**
 * @brief RAII wrapper for automatic section timing
 * 
 * Automatically starts timing on construction and ends on destruction,
 * ensuring timing data is collected even if exceptions occur.
 */
class ScopedProfiler {
public:
    /**
     * @brief Creates a new scoped timing session
     * @param name Unique identifier for the code section
     */
    ScopedProfiler(const std::string& name);
    
    /**
     * @brief Ends the timing session and updates statistics
     */
    ~ScopedProfiler();

private:
    std::string section_name;  ///< Name of the section being timed
};

} // namespace Profiling

/**
 * @brief Convenience macro for creating a scoped profiler
 * @param name String literal or variable containing section name
 */
#define PROFILE_SCOPE(name) Profiling::ScopedProfiler profiler##__LINE__(name)