#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/*
    Hello, student reader!

    This is a quick primer on the C++ standard library classes used in this header.
    If you are not familiar with the concept of a "class", think of it like a struct with some extras.
    (If you are familiar with C++, you can probably skip this comment.)

    - std::array<TYPE, COUNT>
        is a fixed-size array; think of it like "TYPE VARNAME[COUNT];" in C
        a big advantage is that C++ arrays support things like assignment and comparisons - so you can compare two std::arrays for equality (a == b), and this will compare the array contents!
    - std::vector<TYPE>
        is a dynamic-size array; it can hold any number of TYPEs, and manages its own memory internally as you add/remove elements.
    - std::string
        is a dynamic-size character string, which manages its own memory
        if you need a c-style string (null-terminated char*), use the .c_str() method!
*/

// (These are C++-style typedefs; "using A = b;" is a more readable way to write "typedef b A;", especially for complex types)
// You are NOT permitted to modify these definitions
using MACAddress = std::array<uint8_t, 6>;
using EtherType = uint16_t;
using CRC32Checksum = uint32_t;

constexpr MACAddress BROADCAST = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

struct EthernetFrame
{
    MACAddress destinationAddress;
    MACAddress sourceAddress;
    EtherType etherType;
    std::vector<uint8_t> data;
    CRC32Checksum checksum;
};

// This function is useful for debugging purposes -- it's implemented by us, you can simply use it!
std::string MACAddressToString(MACAddress address);

// This function is already implemented by us -- you can simply use it!
CRC32Checksum CalculateFrameChecksum(MACAddress destAddress, MACAddress srcAddress, EtherType type, std::vector<uint8_t> const& data);

// The functionality of the Ethernet port is implemented by us -- you can simply use it!
// (You can disregard the "virtual" business, it has no impact on your use of the class.)
// (Don't let that stop you from figuring out what it does if you're interested, of course.)
struct EthernetPort
{
    virtual bool hasFrame() const = 0;
    virtual EthernetFrame getFrame() = 0;
    virtual void queueSend(EthernetFrame frame) = 0;
    
    virtual ~EthernetPort() = default;
};

// You will need to implement the Ethernet switch functionality for this task.
// - EthernetSwitch::processFrames will be automatically called once per transmission window by the framework.
//   You need to add the required processing functionality to this method (in switch.cpp).
// - You ARE permitted to add fields or methods to EthernetSwitch.
// - You ARE permitted to add a constructor or destructor to EthernetSwitch. If you do, there MUST be a no-args constructor.
// - You are NOT permitted to use global variables to store information.
// - You are NOT permitted to rename the existing methods and members of EthernetSwitch.
class EthernetSwitch
{
    public:
        static constexpr size_t NUM_PORTS = 8;

        void processFrames();
        
        using VLANId = uint64_t;
        void setPortVLAN(size_t port, VLANId id);
        VLANId getPortVLAN(size_t port) const;

    protected:
        EthernetPort* _ports[NUM_PORTS];
};
