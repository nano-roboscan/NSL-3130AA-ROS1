#ifndef __ROBOSCAN_FRAME_H__
#define __ROBOSCAN_FRAME_H__

#include <cstdint>
#include <vector>

namespace nanosys {

    typedef std::vector<uint8_t> Packet;

    struct Frame
    {
        enum DataType { GRAYSCALE, DISTANCE, DISTANCE_AMPLITUDE, DCS, DISTANCE_GRAYSCALE, DISTANCE_AMPLITUDE_GRAYSCALE };

        static const int UDP_HEADER_OFFSET = 20;

        uint8_t stride;        
        uint16_t dataType;
        uint16_t width;
        uint16_t height;
        uint16_t payloadHeaderOffset;
        uint32_t px_size;        
        uint64_t frame_id;
        std::vector<uint8_t> grayData;        
        std::vector<uint8_t> distData;
        std::vector<uint8_t> amplData;
        std::vector<uint8_t> dcsData;
        std::vector<uint8_t> DistGrayData;

        std::vector<uint16_t> dist2BData;
        std::vector<uint16_t> ampl2BData;
        std::vector<uint16_t> gray2BData;
        std::vector<uint16_t> dcs2BData;

        Frame(uint16_t, uint64_t, uint16_t, uint16_t, uint16_t);
        void sortData(const Packet&);
    };

} //end namespace nanosys

#endif // __ROBOSCAN_FRAME_H__
