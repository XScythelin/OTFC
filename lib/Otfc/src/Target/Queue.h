#pragma once

// https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#queue
// https://techtutorialsx.com/2017/08/20/esp32-arduino-freertos-queues/
// https://www.freertos.org/a00116.html

namespace Otfc {

enum EventType
{
  EVENT_IDLE,
  EVENT_GYRO_READ,
  EVENT_ACCEL_READ,
  EVENT_DISARM,
};

class Event
{
  public:
    Event(): type(EVENT_IDLE) {}
    Event(EventType t): type(t) {}
    Event(const Event& e) = default;
  public:
    EventType type;
};

}

#if defined(OTFC_ATOMIC_QUEUE)
  #include "QueueAtomic.h"
  using TargetQueueHandle = Otfc::QueueAtomic<Otfc::Event, 63>;
#elif !defined(OTFC_MULTI_CORE)
  using TargetQueueHandle = int;
#else
  #error "Not yet implelented multicore queue"
#endif

namespace Otfc {

namespace Target {

class Queue
{
  public:
    void begin();
    void send(const Event& e);
    Event receive();
    bool isEmpty() const;
    bool isFull() const;

  private:
    TargetQueueHandle _q;
};

}

}
