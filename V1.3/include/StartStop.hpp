#ifndef STARTSTOP_H
#define STARTSTOP_H



class StartStop {
public:
    enum StatesStartStop {
        stateSwitchOFF,
        stateSwitchON,
        stateMonitoraVel,
        stateIncrementaVel,
        stateDesligaMotor,
        stateLigaMotor,
        stateFreando,
        stateNãoLigou,
        stateEstabilizaVel
    };

    StartStop  ();

    StatesStartStop getFSMstate();

    void setFSMstate(StatesStartStop state);

    StatesStartStop switchOFF ();

    StatesStartStop switchON (); 

    StatesStartStop monitoraVel ();

    StatesStartStop incrementaVel ();

    StatesStartStop desligaMotor ();

    StatesStartStop ligaMotor ();

    StatesStartStop freando ();

    StatesStartStop nãoLigou ();

    StatesStartStop estabilizaVel ();

private:
    StatesStartStop FSMstate;
};

#endif