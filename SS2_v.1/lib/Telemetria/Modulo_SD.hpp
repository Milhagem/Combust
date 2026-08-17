#pragma once
#include <SD.h>
#include <SPI.h>
#include <FS.h>

#define SD_MISO 12
#define SD_MOSI 13
#define SD_SCK  14
#define SD_CS   4

class Gerencia_SD {
    private:
        char bufferSD[2048] = ""; 
        int contagemBuffer = 0;

    public:
        bool sdOnline = false;
        int fileNumber = 0; 
        char currentFileName[32] = "";
 
        void AtivarSD(const char* cabecalho); 
        void salvarTelemetriaNoSD(float* dados, size_t quantidade_dados);
        // Adeia dessa função acima é ser genérica, ela salva um float em uma linha junto com o respectivo time, a forma de plotar o grafico é generica e Já possui um código. GIT
        bool getStatus_sd();
};