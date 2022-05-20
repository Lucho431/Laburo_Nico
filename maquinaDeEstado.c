typedef enum{
	P_OK,
	P_DESHAB,
	P_FUENTE,
	P_RESET_FUENTE,
}T_PROTEC;
	
uint16_t transitorioFunete = 300;  //en 10 * ms.


T_PROTEC status_proteccion;



switch (status_proteccion){
	case P_OK:
		if (/*pulsador presionado*/){
			status_proteccion = P_DESHAB;
			//avisa por pantalla
			break;
		}
	 
		 if (/*proteccion temp*/){
			 status_proteccion = P_DESHAB;
			 //aviso por pantalla
			 break;
		 }
		 
		 if (/*proteccion OL*/){
			 status_proteccion = P_DESHAB;
			 //aviso por pantalla
			 break;
		 }
	 
		 if (/*pulsador presionado*/){
			 status_proteccion = P_DESHAB;
			 break;
		 }	
	
	break;
	case P_DESHAB:
		
		//apaga HAB
		
		if (/*pulsador NO presionado*/) break;
		if (/*P_temp activo*/) break;
		if (/*pote distinto de 0*/) break;
		if (/*P_OL activo*/){
			/*apaga fuente*/
			 break;
		 }		
		
		transitorioFunete = 300; //en 10 * ms.
		status_proteccion = P_FUENTE;
			
	break;
	case P_FUENTE:
		
		//prende fuente
		
		if (/*proteccion temp*/){
			 status_proteccion = P_DESHAB;
			 //aviso por pantalla
			 break;
		 }
		 
		 if (/*proteccion OL*/){
			 status_proteccion = P_DESHAB;
			 //aviso por pantalla
			 break;
		 }
		
		if (transitorioFunete != 0){
			transitorioFunete--;
			 break;
		 }
		 
		 if (/*pote igual a 0*/){
			 //habilita HAB
			 status_proteccion = P_OK;
			 break;
		 }
		 			 				 	
		status_proteccion = P_RESET_FUENTE;
		
	break;
	case P_RESET_FUENTE:
		
		if (/*proteccion temp*/){
			 status_proteccion = P_DESHAB;
			 //aviso por pantalla
			 break;
		 }
		 
		 if (/*proteccion OL*/){
			 status_proteccion = P_DESHAB;
			 //aviso por pantalla
			 break;
		 }
		
		if (/*pulsador NO pulsado */) break;
		
		transitorioFunete = 300;
		status_proteccion = P_FUENTE;
	break;
	
} //end switch status_proteccion
