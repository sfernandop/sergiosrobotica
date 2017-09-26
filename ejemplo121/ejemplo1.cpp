#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{	
	parado =false;
	setupUi(this);//Se abre la ventana
	show();
	h.start();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );//Conecta el boton con la aplicacion
 	//connect(&h, SIGNAL(started()), this, SLOT(incrementar()));
	connect(&h, SIGNAL(valor()), this, SLOT(incrementar()));

	
	//Una funcion emite una señal y el slot la recibe
}

ejemplo1::~ejemplo1()
{
  h.terminate();
}

void ejemplo1::doButton()
{
	qDebug() << "Han pulsado el boton";
	if(!parado){//Si está corriendo,se para
	parado = true;
	button->setText("Continuar");
	h.stop();
	}
	else{
	  parado = false;
	  h.estaParado(false);
	  button->setText("Parar");
	  h.start();
	}
	
}
void ejemplo1::incrementar()
{
  
	lcdNumber->display(lcdNumber->intValue()+1);
	
}



