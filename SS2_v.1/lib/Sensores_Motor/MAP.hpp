#ifndef MAP_HPP
#define MAP_HPP

#include <Arduino.h>

#define pinMAP          3

class Map {
 public:
    static float analisaMap();
    static float getMap() { return map; }
    static bool return_status_map() { return status_map; }

 private:
    inline static float map = 0;
    inline static bool status_map = false;
};

#endif
