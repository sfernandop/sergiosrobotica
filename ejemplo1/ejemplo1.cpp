#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	//timer = QTimer(this);
	setupUi(this);//Se abre la ventana
	show();
	connect(&timer, SIGNAL(timeout()), this, SLOT(incrementar()));
	timer.start(1000);
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );//Conecta el boton con la aplicacion
	//Una funcion emite una señal y el slot la recibe
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
	qDebug() << "Han pulsado el boton";
	timer.stop();
}
void ejemplo1::incrementar()
{
	
	lcdNumber->display(lcdNumber->intValue()+1);
}



