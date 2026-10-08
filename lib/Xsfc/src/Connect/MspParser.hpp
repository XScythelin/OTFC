#pragma once

#include "Connect/Msp.hpp"

namespace Xsfc {

namespace Connect {

class MspParser
{
  public:
    MspParser();
    void parse(char c, MspMessage& msg);
};

}

}
