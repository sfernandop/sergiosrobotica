#ifndef HILO_H
#define HILO_H

#include <thread>         // std::thread
#include <functional>
#include <unistd.h>

class hilo{
private:
  bool parado;
public:
 hilo();
 ~hilo();
 void run(std::function <void()> incrementar);
 void startStop();
 bool estaParado();
};


#endif // HILO_H