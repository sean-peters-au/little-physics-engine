/**
 * @file gpu.hpp
 * @brief Provides WebGPU initialization and cleanup functionality for the simulator.
 *
 * This file declares functions and global variables used by the simulator and GPU-accelerated
 * fluid systems. It encapsulates the low-level details of initializing and cleaning up the
 * WebGPU device and queue.
 */

#pragma once

#include <webgpu/webgpu.h>

/**
 * @brief Global WebGPU device used across systems.
 */
extern WGPUDevice g_wgpuDevice;

/**
 * @brief Global WebGPU queue obtained from the device.
 */
extern WGPUQueue g_wgpuQueue;

/**
 * @brief Initializes the WebGPU instance, adapter, device, and queue.
 *
 * This function creates a default WGPUInstance, requests a suitable adapter, and then
 * synchronously obtains a WGPUDevice. On success, the global variables @c g_wgpuDevice and
 * @c g_wgpuQueue are set accordingly.
 *
 * @return true if the GPU was successfully initialized; false otherwise.
 */
bool initializeGpu();

/**
 * @brief Cleans up GPU resources by resetting global device and queue pointers.
 *
 * This function resets the global GPU references. In Dawn-based applications, explicit resource
 * cleanup is minimal due to internal reference counting.
 */
void cleanupGpu();
