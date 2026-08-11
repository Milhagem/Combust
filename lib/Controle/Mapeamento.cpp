#include "Mapeamento.hpp"

#include <math.h>
#include <float.h>

namespace {
    constexpr double DEG2RAD = 0.017453292519943295769;
    constexpr double RAIO_TERRA_M = 6371000.0;

    struct PontoPista {
        double lat;
        double lon;
        float distAcum;
    };

    struct SegmentoPista {
        int id;
        const char* nome;
        const char* tipo;
        float distInicio;
    };

    //Essa é a pista da FAE
    const PontoPista PONTOS[] = {
        {-19.8690724, -43.9595478,   0.00f},
        {-19.8690030, -43.9595398,   7.77f},
        {-19.8689463, -43.9595425,  14.09f},
        {-19.8689243, -43.9595456,  16.56f},
        {-19.8688575, -43.9595349,  24.08f},
        {-19.8688033, -43.9595155,  30.45f},
        {-19.8687616, -43.9594893,  35.84f},
        {-19.8687255, -43.9594586,  40.98f},
        {-19.8686915, -43.9594271,  46.00f},
        {-19.8686814, -43.9593835,  50.70f},
        {-19.8687047, -43.9593526,  54.85f},
        {-19.8687343, -43.9593386,  58.46f},
        {-19.8687596, -43.9593339,  61.32f},
        {-19.8687930, -43.9593339,  65.03f},
        {-19.8688239, -43.9593406,  68.55f},
        {-19.8688627, -43.9593404,  72.86f},
        {-19.8688999, -43.9593437,  77.02f},
        {-19.8689302, -43.9593457,  80.40f},
        {-19.8689535, -43.9593484,  83.01f},
        {-19.8689958, -43.9593511,  87.73f},
        {-19.8690611, -43.9593695,  95.25f},
        {-19.8691229, -43.9594124, 103.46f},
        {-19.8691695, -43.9594566, 110.41f},
        {-19.8691948, -43.9594902, 114.92f},
        {-19.8691292, -43.9595210, 122.90f}
    };

    const SegmentoPista SEGMENTOS[] = {
        {0, "Reta 1",  "RETA",   0.00f},
        {1, "Curva 1", "CURVA", 40.98f},
        {2, "Reta 2",  "RETA",  61.32f},
        {3, "Curva 2", "CURVA", 95.25f}
    };

    float clamp01(float v) {
        if (v < 0.0f) return 0.0f;
        if (v > 1.0f) return 1.0f;
        return v;
    }
}

Mapeamento::Mapeamento() {
    limparDadosInvalidos();
    calcularGeometria();
}

bool Mapeamento::atualizarGPS(double latitude, double longitude, bool gpsValido) {
    if (!gpsValido || (latitude == 0.0 && longitude == 0.0) ||
        !isfinite(latitude) || !isfinite(longitude)) {
        limparDadosInvalidos(latitude, longitude);
        return false;
    }

    float x_m = 0.0f;
    float y_m = 0.0f;
    latLonParaXY(latitude, longitude, x_m, y_m);

    return atualizarInterno(x_m, y_m, latitude, longitude, true);
}

bool Mapeamento::atualizarXY(float x_m, float y_m) {
    if (!isfinite(x_m) || !isfinite(y_m)) {
        limparDadosInvalidos();
        return false;
    }

    return atualizarInterno(x_m, y_m, 0.0, 0.0, false);
}

bool Mapeamento::atualizarComEKF(const FiltroKalmanExtendido& ekf) {
    const FiltroKalmanExtendido::DadosGPS gps = ekf.getUltimaPosicaoGPS();
    if (!gps.valido) {
        limparDadosInvalidos();
        return false;
    }

    const FiltroKalmanExtendido::Estado estado = ekf.getEstado();
    return atualizarInterno(estado.X, estado.Y, gps.latitude, gps.longitude, true);
}

const Mapeamento::Dados& Mapeamento::getDados() const {
    return ultimo;
}

void Mapeamento::printSerial() const {
    Serial.print("Segmento atual: ");
    Serial.print(ultimo.segmento_atual_nome);
    Serial.print(" [");
    Serial.print(ultimo.tipo_segmento_atual);
    Serial.print("] | Proximo: ");
    Serial.print(ultimo.proximo_segmento_nome);
    Serial.print(" | Distancia: ");
    Serial.print(ultimo.distancia_proximo_segmento_m, 2);
    Serial.print(" m | Erro lateral: ");
    Serial.print(ultimo.erro_lateral_m, 2);
    Serial.println(" m");
}

void Mapeamento::calcularGeometria() {
    for (uint8_t i = 0; i < N_PONTOS; i++) {
        latLonParaXY(PONTOS[i].lat, PONTOS[i].lon, pontoX[i], pontoY[i]);
    }

    for (uint8_t i = 0; i < N_PONTOS; i++) {
        uint8_t j = (i + 1) % N_PONTOS;

        float ds = 0.0f;
        if (i < N_PONTOS - 1) {
            ds = PONTOS[j].distAcum - PONTOS[i].distAcum;
        } else {
            ds = COMPRIMENTO_PISTA_M - PONTOS[i].distAcum;
        }

        if (ds <= 0.01f) {
            ds = 0.01f;
        }

        trechoComprimentoS[i] = ds;
    }
}

void Mapeamento::latLonParaXY(double latitude, double longitude, float& x_m, float& y_m) const {
    const double lat0 = PONTOS[0].lat * DEG2RAD;
    const double lon0 = PONTOS[0].lon * DEG2RAD;

    const double lat = latitude * DEG2RAD;
    const double lon = longitude * DEG2RAD;

    x_m = static_cast<float>((lon - lon0) * cos(lat0) * RAIO_TERRA_M);
    y_m = static_cast<float>((lat - lat0) * RAIO_TERRA_M);
}

bool Mapeamento::atualizarInterno(float x_m, float y_m,
                                  double latitude, double longitude,
                                  bool gpsValido) {
    float melhorD2 = FLT_MAX;
    float melhorS = 0.0f;
    int melhorTrecho = -1;

    for (uint8_t i = 0; i < N_PONTOS; i++) {
        uint8_t j = (i + 1) % N_PONTOS;

        float ax = pontoX[i];
        float ay = pontoY[i];
        float bx = pontoX[j];
        float by = pontoY[j];

        float vx = bx - ax;
        float vy = by - ay;
        float wx = x_m - ax;
        float wy = y_m - ay;

        float len2 = vx * vx + vy * vy;
        if (len2 <= 0.0001f) continue;

        float t = clamp01((wx * vx + wy * vy) / len2);

        float projX = ax + t * vx;
        float projY = ay + t * vy;

        float dx = x_m - projX;
        float dy = y_m - projY;
        float d2 = dx * dx + dy * dy;

        if (d2 < melhorD2) {
            melhorD2 = d2;
            melhorS = PONTOS[i].distAcum + t * trechoComprimentoS[i];
            melhorTrecho = i;
        }
    }

    if (melhorTrecho < 0) {
        limparDadosInvalidos(latitude, longitude);
        return false;
    }

    melhorS = normalizarDistancia(melhorS);

    int idxAtual = acharSegmento(melhorS);
    int idxProximo = (idxAtual + 1) % N_SEGMENTOS;

    float distProx = SEGMENTOS[idxProximo].distInicio - melhorS;
    if (distProx < 0.0f) distProx += COMPRIMENTO_PISTA_M;

    ultimo.valido = true;
    ultimo.gpsValido = gpsValido;
    ultimo.latitude = latitude;
    ultimo.longitude = longitude;
    ultimo.x_m = x_m;
    ultimo.y_m = y_m;
    ultimo.dist_acum_m = melhorS;
    ultimo.erro_lateral_m = sqrtf(melhorD2);

    ultimo.segmento_atual_id = SEGMENTOS[idxAtual].id;
    ultimo.segmento_atual_nome = SEGMENTOS[idxAtual].nome;
    ultimo.tipo_segmento_atual = SEGMENTOS[idxAtual].tipo;

    ultimo.proximo_segmento_id = SEGMENTOS[idxProximo].id;
    ultimo.proximo_segmento_nome = SEGMENTOS[idxProximo].nome;
    ultimo.tipo_proximo_segmento = SEGMENTOS[idxProximo].tipo;

    ultimo.distancia_proximo_segmento_m = distProx;
    ultimo.indice_trecho_mais_proximo = melhorTrecho;

    return true;
}

int Mapeamento::acharSegmento(float dist_acum_m) const {
    float s = normalizarDistancia(dist_acum_m);

    int idx = 0;
    for (uint8_t i = 0; i < N_SEGMENTOS; i++) {
        if (s + 0.001f >= SEGMENTOS[i].distInicio) {
            idx = i;
        }
    }

    return idx;
}

float Mapeamento::normalizarDistancia(float dist_m) const {
    while (dist_m < 0.0f) dist_m += COMPRIMENTO_PISTA_M;
    while (dist_m >= COMPRIMENTO_PISTA_M) dist_m -= COMPRIMENTO_PISTA_M;
    return dist_m;
}

void Mapeamento::limparDadosInvalidos(double latitude, double longitude) {
    ultimo.valido = false;
    ultimo.gpsValido = false;

    ultimo.latitude = latitude;
    ultimo.longitude = longitude;

    ultimo.x_m = 0.0f;
    ultimo.y_m = 0.0f;
    ultimo.dist_acum_m = -1.0f;
    ultimo.erro_lateral_m = -1.0f;

    ultimo.segmento_atual_id = -1;
    ultimo.segmento_atual_nome = "INVALIDO";
    ultimo.tipo_segmento_atual = "INVALIDO";

    ultimo.proximo_segmento_id = -1;
    ultimo.proximo_segmento_nome = "INVALIDO";
    ultimo.tipo_proximo_segmento = "INVALIDO";

    ultimo.distancia_proximo_segmento_m = -1.0f;
    ultimo.indice_trecho_mais_proximo = -1;
}

