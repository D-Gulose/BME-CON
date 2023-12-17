#include "../switch.h"

/*
    Hello there, student reader!
    This file contains the control framework for task 5.

    You do not need to, and should not, modify this file to solve task 5. Any modifications you make to this file will be overwritten by our test system.
    Of course, you're more than welcome to keep exploring if you're interested. Let us know if you have any questions!
*/

#include <cassert>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <queue>

#include <csignal>

#include "dep/json.hpp"

#define ASIO_STANDALONE 1
#include "dep/Simple-Web-Server/server_http.hpp"

struct ActualEthernetPort : EthernetPort
{
    size_t const which;
    ActualEthernetPort(size_t i) : which(i) {}

    bool hasFrame() const override
    {
        return !_recvQueue.empty();
    }

    EthernetFrame getFrame() override
    {
        assert(!_recvQueue.empty());
        EthernetFrame frame = _recvQueue.front();
        _recvQueue.pop();
        return frame;
    }

    void stuffRecvQueue(EthernetFrame const& frame)
    {
        _recvQueue.emplace(frame);
    }

    void queueSend(EthernetFrame frame) override
    {
        _sendQueue.emplace(frame);
    }

    std::optional<EthernetFrame> popSendQueue()
    {
        if (_sendQueue.empty())
            return std::nullopt;
        else
        {
            EthernetFrame frame = _sendQueue.front();
            _sendQueue.pop();
            return frame;
        }
    }

    private:
        std::queue<EthernetFrame> _recvQueue;
        std::queue<EthernetFrame> _sendQueue;
};

class ActualEthernetSwitch : public EthernetSwitch
{
    public:
        ActualEthernetSwitch(std::string const& id) : id(id)
        {
            for (size_t i=0; i < NUM_PORTS; ++i)
                _ports[i] = new ActualEthernetPort(i);
        }
        ~ActualEthernetSwitch()
        {
            for (size_t i=0; i < NUM_PORTS; ++i)
                delete static_cast<ActualEthernetPort*>(_ports[i]);
        }
        ActualEthernetPort& port(size_t i) { return static_cast<ActualEthernetPort&>(*_ports[i]); }

        std::string const id;
};

constexpr std::array<char,16> hexchars = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
std::string MACAddressToString(MACAddress address)
{
    std::string str(17, ':');
    for (size_t i=0; i<6; ++i)
    {
        str[i*3+0] = hexchars[(address[i]>>4)&0xf];
        str[i*3+1] = hexchars[(address[i]>>0)&0xf];
    }
    return str;
}

constexpr std::array<CRC32Checksum, 0x100> __crc32lookup = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};
static void crc32step(CRC32Checksum& sum, uint8_t v)
{
    sum = ((sum >> 8) ^ __crc32lookup[(sum ^ v) & 0xff]);
}

template <typename C>
static void crc32steps(CRC32Checksum& sum, C const& data)
{
    for (uint8_t v : data)
        crc32step(sum, v);
}

CRC32Checksum CalculateFrameChecksum(MACAddress destAddress, MACAddress srcAddress, EtherType type, std::vector<uint8_t> const& data)
{
    CRC32Checksum sum = 0xffffffffu;
    crc32steps(sum, destAddress);
    crc32steps(sum, srcAddress);

    static_assert(sizeof(decltype(type)) == 2);
    crc32step(sum, static_cast<uint8_t>(type >> 8));
    crc32step(sum, static_cast<uint8_t>(type >> 0));
    
    crc32steps(sum, data);
    
    return sum ^ 0xffffffffu;
}

SimpleWeb::Server<SimpleWeb::HTTP> httpServer; /* needs to be global because signal handlers can't take arbitrary types, only plain function pointers */
int main(int,char const**)
{
    std::mutex THE_MUTEX;
    std::unique_ptr<ActualEthernetSwitch> THE_SWITCH;

    std::signal(SIGINT, [](int) { printf("SIGINT received, exiting...\n"); httpServer.stop(); });
    
    httpServer.config.port = 13354;
    httpServer.config.thread_pool_size = 4;

    httpServer.resource["^/helo$"]["POST"] = [&THE_MUTEX, &THE_SWITCH](std::shared_ptr<decltype(httpServer)::Response> resp, std::shared_ptr<decltype(httpServer)::Request> req)
    {
        std::lock_guard<std::mutex> g { THE_MUTEX };
        static size_t id = 0;
        std::string thisId = std::to_string(++id);
        if (THE_SWITCH)
            printf("Shutting down simulation #%s.\n", THE_SWITCH->id.c_str());
        printf("Starting simulation #%s.\n", thisId.c_str());
        THE_SWITCH = std::make_unique<ActualEthernetSwitch>(thisId);

        resp->write(SimpleWeb::StatusCode::success_ok, "", { { "X-Switch-ID", thisId } });
    };

    httpServer.resource["^/tick$"]["POST"] = [&THE_MUTEX, &THE_SWITCH](std::shared_ptr<decltype(httpServer)::Response> resp, std::shared_ptr<decltype(httpServer)::Request> req)
    {
        try
        {
            auto idIt = req->header.find("X-Switch-ID");
            if (idIt == req->header.end())
                throw nullptr;
            
            std::lock_guard<std::mutex> g { THE_MUTEX };
            if (!THE_SWITCH || (THE_SWITCH->id != idIt->second))
            {
                resp->write(SimpleWeb::StatusCode::client_error_precondition_failed);
                return;
            }
    
            auto jsonArray = nlohmann::json::parse(req->content.string()).get<std::array<nlohmann::json, EthernetSwitch::NUM_PORTS>>();
            for (size_t switchport = 0; switchport < EthernetSwitch::NUM_PORTS; ++switchport)
            {
                nlohmann::json const& content = jsonArray[switchport];
                if (content.is_null())
                    continue;
                
                if (nlohmann::json const& jsonVLAN = content["vlan"]; !jsonVLAN.is_null())
                    THE_SWITCH->setPortVLAN(switchport, jsonVLAN.get<EthernetSwitch::VLANId>());
                
                if (nlohmann::json const& jsonFrame = content["frame"]; !jsonFrame.is_null())
                {
                    ActualEthernetPort& port = THE_SWITCH->port(switchport);
                    EthernetFrame frame;
                    frame.destinationAddress = jsonFrame["target"].get<decltype(frame.destinationAddress)>();
                    frame.sourceAddress = jsonFrame["source"].get<decltype(frame.sourceAddress)>();
                    frame.etherType = jsonFrame["etherType"].get<decltype(frame.etherType)>();
                    frame.data = jsonFrame["data"].get<decltype(frame.data)>();
                    frame.checksum = CalculateFrameChecksum(frame.destinationAddress, frame.sourceAddress, frame.etherType, frame.data);
                    if (!jsonFrame["checksumOK"].get<bool>())
                        frame.checksum ^= 0x1;
                    port.stuffRecvQueue(frame);
                }
            }

            THE_SWITCH->processFrames();

            nlohmann::json response = nlohmann::json::array();
            for (size_t switchport = 0; switchport < EthernetSwitch::NUM_PORTS; ++switchport)
            {
                nlohmann::json& elm = response.emplace_back();
                elm["vlan"] = THE_SWITCH->getPortVLAN(switchport);
                
                if (std::optional<EthernetFrame> frame = THE_SWITCH->port(switchport).popSendQueue())
                {
                    nlohmann::json& jsonFrame = elm["frame"];
                    jsonFrame["target"] = frame->destinationAddress;
                    jsonFrame["source"] = frame->sourceAddress;
                    jsonFrame["etherType"] = frame->etherType;
                    jsonFrame["data"] = frame->data;
                    jsonFrame["checksumOK"] = (frame->checksum == CalculateFrameChecksum(frame->destinationAddress, frame->sourceAddress, frame->etherType, frame->data));
                }
                else
                    elm["frame"] = nullptr;
            }

            resp->write(SimpleWeb::StatusCode::success_ok, response.dump(), { { "Content-Type", "application/json" } });
        } catch (...) {
            resp->write(SimpleWeb::StatusCode::client_error_bad_request);
        }
    };

    httpServer.default_resource["GET"] = [](std::shared_ptr<decltype(httpServer)::Response> resp, std::shared_ptr<decltype(httpServer)::Request> req)
    {
        static std::unordered_map<std::string, std::pair<std::string, std::string>> filePaths = {
            { "/", { "framework/web/index.html", "text/html" } },
            { "/framework.js", { "framework/web/framework.js", "text/javascript" } },
            { "/framework.css", { "framework/web/framework.css", "text/css" } },
            { "/computer.png", { "framework/web/computer.png", "image/png" } },
        };

        if (auto it = filePaths.find(req->path); it != filePaths.end())
        {
            std::ifstream ifs(it->second.first.c_str(), std::ios::binary | std::ios::ate);
            if (!ifs)
            {
                resp->write(SimpleWeb::StatusCode::client_error_not_found);
                return;
            }

            std::vector<char> file;
            file.resize(ifs.tellg());
            
            SimpleWeb::CaseInsensitiveMultimap headers;
            headers.emplace("Content-Length", std::to_string(file.size()));
            headers.emplace("Content-Type", it->second.second);

            ifs.seekg(0, std::ios::beg);
            ifs.read(file.data(), file.size());
            if (ifs.gcount() != static_cast<ssize_t>(file.size()))
            {
                resp->write(SimpleWeb::StatusCode::server_error_internal_server_error);
                return;
            }
            
            resp->write(SimpleWeb::StatusCode::success_ok, std::string_view(file.data(), file.size()), headers);
        }
        else
            resp->write(SimpleWeb::StatusCode::client_error_not_found);
    };
    
    httpServer.start([](uint16_t listenPort)
    {
        printf("Startup done.\nOpen http://localhost:%u/ in your browser to begin sending packets.\n", listenPort);
    });
    
    return 0;
}
