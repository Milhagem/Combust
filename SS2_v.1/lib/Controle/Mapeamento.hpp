#pragma once

#include <Arduino.h>

class Mapeamento {
public:
    struct Dados {
        bool valido;            // true se a associação com a pista foi calculada
        bool gpsValido;         // true se latitude/longitude vieram de GPS válido

        double latitude;        // [graus]
        double longitude;       // [graus]

        float x_m;              // posição local Este  [m], referência = ponto 0 da pista
        float y_m;              // posição local Norte [m], referência = ponto 0 da pista
        float dist_acum_m;      // distância projetada ao longo da pista [m]
        float erro_lateral_m;   // distância lateral até a linha central da pista [m]

        int segmento_atual_id;
        const char* segmento_atual_nome;
        const char* tipo_segmento_atual;

        int proximo_segmento_id;
        const char* proximo_segmento_nome;
        const char* tipo_proximo_segmento;

        float distancia_proximo_segmento_m;
        int indice_trecho_mais_proximo;
    };

    // ── Inicialização ──
    static void begin();

    // ── Atualização de Posição ──
    // Atualiza a posição usando GPS bruto.
    static bool atualizarGPS(double latitude, double longitude, bool gpsValido = true);

    // Atualiza usando coordenadas locais em metros (X = Este, Y = Norte)
    static bool atualizarXY(float x_m, float y_m);

    // ── Leitura de Dados ──
    static const Dados& getDados();

    // Debug via Serial
    static void printSerial();

private:
    static constexpr uint8_t N_PONTOS = 25;
    static constexpr uint8_t N_SEGMENTOS = 4;
    static constexpr float COMPRIMENTO_PISTA_M = 129.82f;

    // ── Variáveis Estáticas Globais ──
    inline static float pontoX[N_PONTOS] = {0};
    inline static float pontoY[N_PONTOS] = {0};
    inline static float trechoComprimentoS[N_PONTOS] = {0};

    inline static Dados ultimo = {};

    // ── Métodos Internos de Cálculo ──
    static void calcularGeometria();
    static void latLonParaXY(double latitude, double longitude, float& x_m, float& y_m);
    static bool atualizarInterno(float x_m, float y_m, double latitude, double longitude, bool gpsValido);
    static int acharSegmento(float dist_acum_m);
    static float normalizarDistancia(float dist_m);
    static void limparDadosInvalidos(double latitude = 0.0, double longitude = 0.0);
};