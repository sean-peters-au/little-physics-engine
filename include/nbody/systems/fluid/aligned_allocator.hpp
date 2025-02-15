#pragma once

#include <cstdlib>
#include <cstddef>
#include <memory>
#include <stdexcept>

// A simple aligned allocator template.
template <typename T, std::size_t Alignment>
struct AlignedAllocator {
    using value_type = T;

    AlignedAllocator() noexcept {}

    template <class U>
    AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    T* allocate(std::size_t n) {
        void* ptr = nullptr;
        std::size_t size = n * sizeof(T);
    #if defined(_MSC_VER) || defined(__MINGW32__)
        ptr = _aligned_malloc(size, Alignment);
        if (!ptr) {
            throw std::bad_alloc();
        }
    #else
        if (posix_memalign(&ptr, Alignment, size) != 0) {
            throw std::bad_alloc();
        }
    #endif
        return static_cast<T*>(ptr);
    }

    void deallocate(T* p, std::size_t) noexcept {
    #if defined(_MSC_VER) || defined(__MINGW32__)
        _aligned_free(p);
    #else
        free(p);
    #endif
    }
};

template <class T, class U, std::size_t Alignment>
bool operator==(const AlignedAllocator<T, Alignment>&, const AlignedAllocator<U, Alignment>&) noexcept {
    return true;
}

template <class T, class U, std::size_t Alignment>
bool operator!=(const AlignedAllocator<T, Alignment>& a, const AlignedAllocator<U, Alignment>& b) noexcept {
    return !(a == b);
}