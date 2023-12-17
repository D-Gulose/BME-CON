#include "switch.h"
#include <cstdio>
#include <cinttypes>

void EthernetSwitch::processFrames()
{
    for (size_t incomingPortNumber = 0; incomingPortNumber < NUM_PORTS; ++incomingPortNumber)
    {
        if (!_ports[incomingPortNumber]->hasFrame())
            continue;

        // This port has received a frame during the last transmission interval.
        EthernetFrame receivedFrame = _ports[incomingPortNumber]->getFrame();
        printf(
            "Frame received on port %zu (VLAN %" PRIu64 "): %s -> %s\n",
            incomingPortNumber,
            getPortVLAN(incomingPortNumber),
            MACAddressToString(receivedFrame.sourceAddress).c_str(),
            MACAddressToString(receivedFrame.destinationAddress).c_str()
        );

        // Validate the frame, and forward it out of the appropriate port(s) to clear this task!
// TODO: Implement this method!

    }
}

void EthernetSwitch::setPortVLAN(size_t port, VLANId vlan)
{
// TODO: Implement this method!

    printf("Port %zu should now belong to VLAN ID %" PRIu64 "\n", port, vlan);
}

EthernetSwitch::VLANId EthernetSwitch::getPortVLAN(size_t port) const
{
// TODO: Implement this method!
    return 0;
}
