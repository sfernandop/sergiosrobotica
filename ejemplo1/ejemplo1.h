#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
//Hay que declarar un timer

class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
public:
    QTimer timer;
    ejemplo1();
    virtual ~ejemplo1();
    
public slots:
	void doButton();
	void incrementar();
};

#endif // ejemplo1_H
