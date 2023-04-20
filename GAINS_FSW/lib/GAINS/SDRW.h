/*
 ============================================================================
 Name        : SDRW.h
 Author      : Jason Popich & Bennett Grow
 Version     : 0.1
 Copyright   : 
 Description : 
 ============================================================================
 */

#ifndef _SDRW_H_
#define _SDRW_H_

#include <SdFat.h>

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
    #define SD_CONFIG SdioConfig(FIFO_SDIO)
#else  // HAS_SDIO_CLASS
    #define SD_CONFIG SdSpiConfig(SS,DEDICATED_SPI, SD_SCK_MHZ(50))
#endif  // HAS_SDIO_CLASS

class SDRW {
    private:
        bool running = false;
        SdFs sd;
        FsFile file;
        char foldername[10];
        
        bool openACCEL();
        void printACCEL(float accel, float temp);
        bool openSTATE();
        void printSTATE(float t1, float *state, float accel, float g);

    public:
        SDRW();
        bool initFolder();

        bool sampleACCEL(float accel, float temp);
        bool sampleSTATE(float t1, float *state, float accel, float g);
}; 

SDRW::SDRW() {
    if (sd.begin(SD_CONFIG)) {
        running = true;
    }
    else
    {
        Serial.println("Failed to begin SD!");
    }
}

bool SDRW::initFolder() {
    uint16_t counter = 1;

    while(true) {
        sprintf(foldername,"GAINS%04d",counter);

        if(sd.exists(foldername)) {
            counter+=1;
            if(counter>999){return false;}
        }
        else {break;}
    }
    if(!sd.mkdir(foldername)) {
        Serial.printf("1 SD - Error creating folder %s\n",foldername);
        return false;
        }
    if(!sd.chdir(foldername)) {
        Serial.printf("2 SD - Error entering folder %s\n",foldername);
        return false;
        }
    return true;
}

bool SDRW::openACCEL(){
    char filename[9];
    sprintf(filename, "accel.csv");
    file = sd.open(filename,FILE_WRITE);

    return true;
}

void SDRW::printACCEL(float accel, float temp) {
    file.printf("%u,%.8f,%.8f\n",micros(), accel, temp);
    Serial.printf("%u,%.8f,%.8f\n",micros(), accel, temp);
}

bool SDRW::sampleACCEL(float accel, float temp) {
    openACCEL();
    printACCEL(accel,temp);
    file.close();
    return true;
}

bool SDRW::openSTATE(){
    char filename[9];
    sprintf(filename, "state.csv");
    file = sd.open(filename,FILE_WRITE);

    return true;
}

void SDRW::printSTATE(float t1, float *state, float accel, float g) {
    file.printf("%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", t1, accel, g, state[0], state[1], state[2], state[3], state[4], state[5]);
}

bool SDRW::sampleSTATE(float t1, float *state, float accel, float g) {
    openSTATE();
    printSTATE(t1, state, accel, g);
    file.close();
    return true;
}




#endif