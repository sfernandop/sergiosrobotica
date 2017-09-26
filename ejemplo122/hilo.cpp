#include "hilo.h"
#include <unistd.h>

hilo::hilo(
){
  parado = true;
}
hilo::~hilo(){}
void hilo::run(std::function <void()> incrementar){
  while(1){
    incrementar();
    sleep(1);
  }

}
void hilo::startStop(){
  parado = !parado;
}

bool hilo::estaParado(){
  return (parado);
}
