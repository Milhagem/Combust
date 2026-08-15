#include "Modulo_SD.hpp"


void Gerencia_SD::AtivarSD(const char* cabecalho) {
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    
    if (!SD.begin(SD_CS, SPI, 4000000)) { 
        sdOnline = false; 
        return; 
    }

    
    while (true) {
        snprintf(currentFileName, sizeof(currentFileName), "/log%d.csv", fileNumber);
        if (!SD.exists(currentFileName)) break;
        fileNumber++; 
        yield(); 
    }
    
    
    File file = SD.open(currentFileName, FILE_WRITE);
    if (file) { 

        file.println(cabecalho); //Para plotar o grafico de forma altomático deve preencher esse cabecalho com os nomes das variaveis que serão gravadas no SD, separados por vírgula.
        file.close(); 
        sdOnline = true; 
    } else { 
        sdOnline = false; 
    }
}

void Gerencia_SD::salvarTelemetriaNoSD(float* dados, size_t quantidade_dados) {
    if (!sdOnline) return;
    
    char linha[128] = "";
    char temp[16];

    snprintf(linha, sizeof(linha), "%lu", millis());

    for (size_t i = 0; i < quantidade_dados; ++i) {
        snprintf(temp, sizeof(temp), ",%.2f", dados[i]);
        strlcat(linha, temp, sizeof(linha));
    }

    strlcat(linha, "\n", sizeof(linha));

    if (strlen(bufferSD) + strlen(linha) < sizeof(bufferSD)) {
        strlcat(bufferSD, linha, sizeof(bufferSD));
        contagemBuffer++;
    } else {
        contagemBuffer = 20; 
    }

    if (contagemBuffer >= 20) {
        if (sdOnline) {
            File dataFile = SD.open(currentFileName, FILE_APPEND);
            if (dataFile) { 
                dataFile.print(bufferSD); 
                dataFile.close();
                bufferSD[0] = '\0'; 
                contagemBuffer = 0;
            } else {
                sdOnline = false;
                Serial.println("⚠️ ERRO: SD Card falhou! (Vibracao?)");
            }
        }
        bufferSD[0] = '\0'; 
        contagemBuffer = 0; 
    }
}

bool Gerencia_SD::getStatus_sd() {
    return SD.begin(SD_CS);
}



