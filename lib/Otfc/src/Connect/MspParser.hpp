#pragma once

#include "Connect/Msp.hpp"

namespace Otfc {

namespace Connect {

class MspParser
{
  public:
    MspParser();
    void parse(char c, MspMessage& msg);
};

}

}
