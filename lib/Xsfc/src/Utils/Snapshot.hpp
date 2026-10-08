#pragma once

#include <atomic>
#include <type_traits>

namespace Xsfc::Utils {

template<typename T>
class Snapshot
{
  static_assert(std::is_trivially_copyable<T>::value, "Sensor snapshots must be plain data");
public:
  Snapshot() = default;
  Snapshot(const Snapshot& other): _value(other.load()) {}
  Snapshot& operator=(const Snapshot& other)
  {
    if (this != &other) store(other.load());
    return *this;
  }

  T load() const
  {
    while (_lock.test_and_set(std::memory_order_acquire)) {}
    const T copy = _value;
    _lock.clear(std::memory_order_release);
    return copy;
  }

  void store(const T& value)
  {
    while (_lock.test_and_set(std::memory_order_acquire)) {}
    _value = value;
    _lock.clear(std::memory_order_release);
  }

private:
  // Serial/sensor and control tasks can run on different ESP32 cores.
  mutable std::atomic_flag _lock = ATOMIC_FLAG_INIT;
  T _value{};
};

} // namespace Xsfc::Utils
