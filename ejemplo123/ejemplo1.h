#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include <thread>
#include <condition_variable>
#include <functional>
#include <unistd.h>
#include "ui_counterDlg.h"
#include "hilo.h"
//Hay que declarar un timer

class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
public:
   
    ejemplo1();
    ~ejemplo1();
private:
    hilo h;
    bool parado;
    std::thread miHilo;
    std::condition_variable condicion;
    std::mutex mutex;
    
public slots:
	void doButton();
	void incrementar();
	
};

#endif // ejemplo1_H
