#ifndef MAPEAMENTO_HPP
#define MAPEAMENTO_HPP

#include <Arduino.h>
#include "Filtro_Kalman_Extendido.hpp"

class Mapeamento {
public:
    struct Dados {
        bool valido;
        bool gpsValido;

        double latitude;
        double longitude;

        float x_m;
        float y_m;
        float dist_acum_m;
        float erro_lateral_m;

        int segmento_atual_id;
        const char* segmento_atual_nome;
        const char* tipo_segmento_atual;

        int proximo_segmento_id;
        const char* proximo_segmento_nome;
        const char* tipo_proximo_segmento;

        float distancia_proximo_segmento_m;
        int indice_trecho_mais_proximo;
    };

    Mapeamento();

    bool atualizarGPS(double latitude, double longitude, bool gpsValido = true);
    bool atualizarXY(float x_m, float y_m);
    bool atualizarComEKF(const FiltroKalmanExtendido& ekf);

    const Dados& getDados() const;

    void printSerial() const;

private:
    static constexpr uint8_t N_PONTOS = 25;
    static constexpr uint8_t N_SEGMENTOS = 4;
    static constexpr float COMPRIMENTO_PISTA_M = 129.82f;

    float pontoX[N_PONTOS];
    float pontoY[N_PONTOS];
    float trechoComprimentoS[N_PONTOS];

    Dados ultimo;

    void calcularGeometria();
    void latLonParaXY(double latitude, double longitude, float& x_m, float& y_m) const;

    bool atualizarInterno(float x_m, float y_m,
                          double latitude, double longitude,
                          bool gpsValido);

    int acharSegmento(float dist_acum_m) const;
    float normalizarDistancia(float dist_m) const;
    void limparDadosInvalidos(double latitude = 0.0, double longitude = 0.0);
};

using Posicao = Mapeamento;

#endif
