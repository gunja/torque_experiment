#pragma once
#include <hardware_interface/hardware_interface.h>

class ArminHWInterface : public hardware_interface::HardwareInterface
{
  public:
    ArminHWInterface();
    virtual ~ArminHWInterface();

    void claim(std::string resource) override;
};
