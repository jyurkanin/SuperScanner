#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <iostream>
#include <math.h>


typedef struct{
    char          RIFF[4];        // RIFF Header      Magic header
    uint32_t      chunkSize;      // RIFF Chunk Size  
    char          WAVE[4];        // WAVE Header      
    char          fmt[4];         // FMT header       
    uint32_t      subchunk1Size;  // Size of the fmt chunk                                
    uint16_t      audioFormat;    // Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM 
    uint16_t      numOfChan;      // Number of channels 1=Mono 2=Sterio                   
    uint32_t      samplesPerSec;  // Sampling Frequency in Hz                             
    uint32_t      bytesPerSec;    // bytes per second 
    uint16_t      blockAlign;     // 2=16-bit mono, 4=16-bit stereo 
    uint16_t      bitsPerSample;  // Number of bits per sample      
    char          subchunk2ID[4]; // "data"  string   
    uint32_t      subchunk2Size;  // Sampled data length    
} WavHeader;


class WavFile{
public:
    WavFile(char * filename);
    ~WavFile();
    void print();
    float *data;
    int data_len;
    WavHeader header;
};
