#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>


void handle_connection(int connection, char* remoteAddress, uint16_t remotePort)
{
    printf("Incoming connection from %s:%u...\n", remoteAddress, remotePort);

// TODO: Implement this method!

    close(connection);
}
