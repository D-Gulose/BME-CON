#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
    Hello there, student reader!
    This file implements the basic functionality of listening for, and accepting, TCP connections.

    If you're interested in how you do that, feel free to keep reading. The entire file is annotated thoroughly to hopefully explain what's going on!
    Note that, to solve the task, you do NOT need to keep reading. Don't let that stop you if you're interested, of course.

    If all you want to do for now is work on the task, you'll want to head over to http.cpp instead.
    Good hunting!
*/

/*
    this is a function declaration - a function signature without a body (with a body, it would be called the "definition")
    essentially, it's telling your compiler that a function with this signature is defined _somewhere_ in the program, and to just assume it exists
    (if the function isn't defined anywhere, you will get a linker error if you've tried using it)

    function declarations are very commonly found in header files, but you can have them in source files just fine
    (though you typically want to avoid duplicating code like that...)

    here, we're telling the server framework that your handle_connection function (which you have to implement over in http.cpp) exists
*/
void handle_connection(int connection, char* remoteAddress, uint16_t remotePort);

int main(int, char const**)
{
    // feel free to change this if you want, it's chosen arbitrarily
    constexpr unsigned short PORT = 8000;

    /*    [ see also: man socket(2) ]
        we create a socket (connection endpoint), and get a file descriptor for that socket back:
        * AF_INET     - we create an IPv4 endpoint
        * SOCK_STREAM - we will be using it for a sequenced, two-way, reliable byte stream (even just specifying this will usually get us TCP!)
        * IPPROTO_TCP - we specifically request a TCP socket just to be safe
    */
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    {
        int opt = 1;
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    }

    // we're going to populate this struct with information about our socket
    sockaddr_in listen_info;

    // we're going to be using IPv4
    listen_info.sin_family = AF_INET;

    /*    [ see also: man htons(3) ]
        "htons" is a relic from an era when descriptive function names were considered unnecessary
        it stands for:
        - (H)ost
        - (TO)
        - (N)etwork
        - (S)hort
        in other words, it:
        - takes a "short" (16-byte) integer in host byte order (whatever your device "naturally" uses)
        - returns the same integer in network byte order ("big-endian", or most significant byte first)
        always using big-endian over the network lets different deviecs interoperate!
    */
    listen_info.sin_port = htons(PORT);

    // we could specifically only listen on a single interface (i.e., a single IP address) if we wanted to
    // this can be a useful feature - for example, we might never want to accept remote connections, at which
    // point we could simply listen only to "127.0.0.1" (the local loopback address)
    // here, we specify INADDR_ANY, which is a special value that means "accept connections on any address"
    listen_info.sin_addr.s_addr = INADDR_ANY;

    // errors being indicated by the return code is a relic of early C days; negative values are typically bad
    // (consult the documentation of the function in question to be sure)
    int result;

    /*    [ see also: man bind(2) ]
        here, we actually tie our socket from earlier to the address we specify (all assigned addresses, on port PORT)
        this might fail if, for example, the port is already in use (did you remember to kill your previous server?)

        the ugly mess of the second and third parameters are a relic of C, which has no concept of struct inheritance
        (we are just passing in listen_info, everything else is just hoops to jump through)
    */
    result = bind(listener, (sockaddr*)(&listen_info), sizeof(sockaddr_in));
    if (result < 0)
    {
        perror("Failed to bind to address");
        exit(1);
    }

    /*    [ see also: man listen(2) ]
        we set our socket to wait for incoming connections
        the alternative would be to use the socket to establish a connection ourselves
        go have a look at the TCP slides if you aren't sure what this means!

        later, we will need to accept connections on this socket one-by-one to process them
        the "10" is the size of the "backlog" of connections the socket is allowed to maintain
        if we have 10 outstanding connections already, any new connection will be refused outright

        (you really should not need to change this, but feel free to if you want to see what happens)
    */
    result = listen(listener, 10);
    if (result < 0)
    {
        perror("Failed to switch socket to listen mode");
        exit(2);
    }
    
    /*    [ see also: man sigaction(2), man write(2) ]
        if you try to write to a closed socket, the kernel will send you a SIGPIPE signal, which typically kills the entire process
        we don't want this -- just because someone's web browser dies mid-connection, we don't want to lose the entire server
        
        if we ignore SIGPIPE, the other side going away will simply result in an error return value from write() on the socket
        make sure you handle this return value!
    */
    {
        sigset_t sigpipe_set;
        sigemptyset(&sigpipe_set);
        sigaddset(&sigpipe_set, SIGPIPE);
        sigprocmask(SIG_BLOCK, &sigpipe_set, NULL);
    }

    printf("Server startup ok - listening on port %u.\n", PORT);

    do
    {
        // this will be an output parameter - see below
        sockaddr_in peer_info;
        socklen_t len = sizeof(sockaddr_in);

        /*    [ see also: man accept(2) ]
            we grab an outstanding connection from our backlog to process it
            if there are no outstanding connections, this call will block (i.e., it will not return until we receive a connection)

            the "real" return value is a file descriptor that we can use to read/write bytes from our newly-established TCP connection
            the peer_info struct is a secondary return value - we pass a pointer to it, and the function will fill it out for us

            the dual arguments are a C relic, which you can safely disregard
            (we are, essentially, passing a pointer to peer_info with some extra hoops)
        */
        int connection = accept(listener, reinterpret_cast<sockaddr*>(&peer_info), &len);

        // as usual, negative values are bad
        if (connection < 0)
        {
            perror("Failed to accept incoming connection on socket");
            exit(3);
        }

        /*    [ see also: man inet_ntoa(3) ]
            the "inet_" prefix indicates IPv4; "ntoa" is (N)etwork (TO) (A)ddress, probably?
            it takes the network-byte-order (4-byte) IPv4 address and returns a pointer to a string in decimal-dotted notation
            (e.g., it converts 0xc0a80001 to "192.168.0.1")

            the returned pointer is an internal buffer, which will be overwritten on the next call to inet_ntoa...
        */
        char* addr = inet_ntoa(peer_info.sin_addr);

        /*    [ see also: man strncpy(3) ]
            ... so we copy the IP address into our own buffer
            (just to avoid any accidents if your code, for whatever reason, also decides to call inet_ntoa)

            strncpy(dest, source, n) copies up to n bytes from source to dest, stopping at a null terminator if it finds one earlier
            dotted-notation IP addresses should be at most 15 bytes (plus one for the terminator),
            but we still explicitly null out buf[15] to ensure null termination is always there
            (it never hurts to be paranoid...)

            if you were using c++, you would just use a std::string here and never think about null terminators again
        */
        char buf[16];
        strncpy(buf, addr, 16);
        buf[15] = '\0';

        /*    [ see also: man ntohs(3) ]
            see earlier comment on htons; this is the reverse operation
            (N)etwork (TO) (H)ost, (S)hort 
            
            it takes a 16-byte integer in network byte order (big-endian), and converts it to be in whatever endianness your device prefers
        */
        uint16_t port = ntohs(peer_info.sin_port);

        // and now, finally, we hand off to your function over in handle_connection.cpp, which will (hopefully) deal with the request successfully!
        handle_connection(connection, &buf[0], port);

        // after this connection is dealt with, we go back up, call accept() again to get the next incoming connection, and repeat...
    } while (true);

    // we're never going to get here, but let's keep the language standard happy
    return 0;
}
