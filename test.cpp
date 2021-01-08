#include "super_scanner.h"
#include "scanner_window.h"
#include "audio_engine.h"

int main(int argc, char *argv[]){
    SuperScanner *scanner = new SuperScanner(64);
    float sample_l, sample_r;
    float in = 3;
    
    scanner->reverb.tick(in, in, sample_l, sample_r);
    printf("L:%f R:%f\n", sample_l, sample_r);

    delete scanner;
    return 0;
}
