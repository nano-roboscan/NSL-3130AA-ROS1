#include "frame.hpp"
#include <stdint.h>
#include <iostream>
#include <ros/ros.h>

namespace nanosys {

Frame::Frame(uint16_t dataType_, uint64_t frame_id_, uint16_t width_, uint16_t height_, uint16_t payloadOffset) :
frame_id(frame_id_),
dataType(dataType_),
width(width_),
height(height_),
px_size(sizeof(uint16_t)),
grayData(std::vector<uint8_t>(width * height * px_size)), //16 bit
distData(std::vector<uint8_t>(width * height * px_size)), //16 bit
amplData(std::vector<uint8_t>(width * height * px_size)), //16 bit
dcsData(std::vector<uint8_t> (width * height * px_size * 4)), //16 bit 4 dcs
dist2BData(std::vector<uint16_t>(width * height)), //16 bit
ampl2BData(std::vector<uint16_t>(width * height)), //16 bit
gray2BData(std::vector<uint16_t>(width * height)), //16 bit
dcs2BData(std::vector<uint16_t>(width * height * 4)), //16 bit
payloadHeaderOffset(payloadOffset)
{    
}

void Frame::sortData(const Packet &data)
{    
    int i,j;

    if(dataType == Frame::DISTANCE_AMPLITUDE){ //distance - amplitude

        int sz = payloadHeaderOffset + width * height * px_size * 2;
        for(j=0, i = payloadHeaderOffset; i < sz; i+=4, j+=2){
            //if(data[i+1] > 61) { continue; }
            dist2BData[j>>1] = (data[i+1] << 8) + data[i];
			ampl2BData[j>>1] = (data[i+3] << 8) + data[i+2];

            distData[j]   = data[i];
            distData[j+1] = data[i+1];
            amplData[j]   = data[i+2];
            amplData[j+1] = data[i+3] & 0x0f;
        }

    }else if(dataType == Frame::DISTANCE){ //distance

        int sz = payloadHeaderOffset + width * height * px_size;
        for(j=0, i = payloadHeaderOffset; i < sz; i+=2, j+=2){
            //if(data[i+1] > 61) { continue; }
            dist2BData[j>>1] = (data[i+1] << 8) + data[i];
            distData[j]    = data[i];
            distData[j+1]  = data[i+1];

        }

    }else if(dataType == Frame::GRAYSCALE){ //grayscale

        int sz = payloadHeaderOffset + width * height * px_size;
        for(j=0, i = payloadHeaderOffset; i < sz; i+=2, j+=2){
//            if(amplData[i+1] > 61) { continue; }
			gray2BData[j>>1] = (data[i+1] << 8) + data[i];
            amplData[j]    = data[i];
            amplData[j+1]  = data[i+1] & 0x0f;
        }
    }else if(dataType == Frame::DISTANCE_GRAYSCALE){ //distance - grayscale

        int sz = payloadHeaderOffset + width * height * px_size * 2;
        for(j=0, i = payloadHeaderOffset; i < sz; i+=4, j+=2){
            //if(data[i+1] > 61) { continue; }
            dist2BData[j>>1] = (data[i+1] << 8) + data[i];
			gray2BData[j>>1] = (data[i+3] << 8) + data[i+2];

            distData[j]   = data[i];
            distData[j+1] = data[i+1];
            amplData[j]   = data[i+2];
            amplData[j+1] = data[i+3] & 0x0f;
        }
    }else if(dataType == Frame::DISTANCE_AMPLITUDE_GRAYSCALE){//distance-amplitude-grayscale
        int sz = payloadHeaderOffset + width * height * px_size * 3;
        for(j=0, i = payloadHeaderOffset; i < sz; i+=6, j+=2){

            dist2BData[j>>1] = (data[i+1] << 8) + data[i];
			ampl2BData[j>>1] = (data[i+3] << 8) + data[i+2];
			gray2BData[j>>1] = (data[i+5] << 8) + data[i+4];

            distData[j]   = data[i];
            distData[j+1] = data[i+1];
            amplData[j]   = data[i+2];
            amplData[j+1] = data[i+3] & 0x0f;
            grayData[j]   = data[i+4];
            grayData[j+1] = data[i+5] & 0x0f;
        }
    }else{ //DCS

        int sz = payloadHeaderOffset + width * height * px_size * 4;
        for(j=0, i = payloadHeaderOffset; i < sz; i+=2, j+=2){
//            if(dcsData[i+1] > 61) { continue; }
            dcs2BData[j>>1] = (data[i+1] << 8) + data[i];
            dcsData[j]    = data[i];
            dcsData[j+1]  = data[i+1] & 0x0f;
        }
    }

}


} //end namespace nanosys

