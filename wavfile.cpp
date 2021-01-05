#include "wavfile.h"

#define PARSE_VAR(X) if(read(fd, &X, sizeof(X)) != sizeof(X)) printf("error1\n")
WavFile::WavFile(char * filename){
    int fd = open(filename, O_RDONLY);
    if(read(fd, &header, sizeof(header)) != sizeof(header)) printf("error2\n");
    //print();

    data_len = header.subchunk2Size / (header.bitsPerSample >> 3); //had a divide by zero right here.
    int16_t *pcm = new int16_t[data_len];
    data = new float[data_len];
    
    if(read(fd, pcm, header.subchunk2Size) != header.subchunk2Size) printf("error3\n");
    for(unsigned i = 0; i < data_len; i++){
        data[i] = (float) pcm[i] / (1 << (header.bitsPerSample - 1));
    }
    delete[] pcm;
}

WavFile::~WavFile(){
    delete[] data;
}

void WavFile::print(){
    printf("RIFF %s\n", header.RIFF);
    printf("chunkSize %d\n", header.chunkSize);
    printf("WAVE %s\n", header.WAVE);
    printf("fmt %s\n", header.fmt);
    printf("subchunk1Size %d\n", header.subchunk1Size);
    printf("audioFormat %d\n", header.audioFormat);
    printf("num_channels %d\n", header.numOfChan);
    printf("samplesPerSec %d\n", header.samplesPerSec);
    printf("bytes per sec %d\n", header.bytesPerSec);
    printf("blockAlign %d\n", header.blockAlign);
    printf("bits per sample %d\n", header.bitsPerSample);
    printf("num samples %d\n", header.subchunk2Size / (header.bitsPerSample >> 3));
}

/*
int main(){
    int num_puts = 152;
    char fn[100];

    float *fft_list[100];
    for(int i = 0; i < 100; i++){
        memset(fn, 0, sizeof(fn));
        sprintf(fn, "AKWF/AKWF_%04d.wav", i+1);
        
        WavFile *temp = new WavFile(fn);
        delete temp;
        
    }
    //
}
*/
