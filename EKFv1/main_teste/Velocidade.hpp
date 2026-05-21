#ifndef VELOCIDADE_HPP
#define VELOCIDADE_HPP

/**
 * Velocidade — Sensor Hall + integração ICM
 *
 * Para rodar sem o sensor Hall conectado, passe
 * habilitarHall=false no construtor ou chame
 * setHallHabilitado(false) antes do loop.
 * Nesse modo getVelocidadeHALL() retorna sempre 0 e
 * calc() vira no-op, sem corromper o EKF.
 */

#include <Arduino.h>

#define N_IMAS       2.0       // número de imãs no rotor
#define CIRCUNF_RODA 1.81      // [m] circunferência da roda
#define G_ACEL       9.80665   // [m/s²] gravidade (padrão ISO)
#define MS2_TO_KMHS  3.6

class Velocidade {
public:
    /**
     * @param pino_sensor   Pino digital ligado ao sensor Hall
     * @param habilitarHall false → desativa o Hall completamente
     *                      (use quando o sensor não estiver conectado)
     */
    explicit Velocidade(uint8_t pino_sensor, bool habilitarHall = true);

    /**
     * Habilita ou desabilita o sensor Hall em tempo de execução.
     * Ao desabilitar, zera o estado interno e torna calc() um no-op.
     */
    void setHallHabilitado(bool habilitado);

    /** Retorna true se o sensor Hall está habilitado. */
    bool isHallHabilitado() const;

    /**
     * Deve ser chamado pela ISR do sensor Hall.
     * No-op se o Hall estiver desabilitado.
     */
    void calc();

    /**
     * Velocidade escalar obtida do sensor Hall.
     * @return [m/s] — 0 se desabilitado, parado ou sem pulsos há >5 s
     */
    double getVelocidadeHALL();

    /** Conveniência: velocidade Hall em km/h. */
    double getVelocidadeHALL_kmh();

    /** Converte aceleração ICM de [g] para [m/s²]. */
    static float gParaMs2(float accel_g);

private:
    uint8_t pino;
    bool    hallHabilitado;

    volatile unsigned long periodo;
    volatile unsigned long ultimoTempo;
    volatile unsigned long ultimoDebounce;
    volatile double        rpmAtual;
    volatile double        rpmAntigo;
};

#endif // VELOCIDADE_HPP