#include "ejemplo1.h"


ejemplo1::ejemplo1(): Ui_Counter()
{
	parado = false;
	setupUi(this);//Se abre la ventana
	show();
	miHilo=  std::move(std::thread(&hilo::run, h, std::bind(&ejemplo1::incrementar, this)));
	
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );//Conecta el boton con la aplicacion
	//Una funcion emite una señal y el slot la recibe
}

ejemplo1::~ejemplo1()
{
  miHilo.detach();
}

void ejemplo1::doButton()
{
	qDebug() << "Han pulsado el boton";
	if(!parado){
	button->setText("Continuar");
	parado = true;
	}
	else {
	button->setText("Parar");
	parado = false;
	condicion.notify_all();
	}
	h.startStop();

}
void ejemplo1::incrementar()
{
	std::unique_lock<std::mutex> lock(mutex); 
	
	while(h.estaParado())
	    condicion.wait(lock);    
	
	lcdNumber->display(lcdNumber->intValue()+1);
	
}



