/*Piccola libreria per i controlli a tempo*/


/*
TimeTrigger:
Temporizzatore semplificato che restituisce ciclicamente 1 (per una chiamata di funzione) ogni volta che l'intervallo viene superato
*/
class TimeTrigger	
{
public:
	void setInterval(unsigned long);	//setta l'intervallo del temporizzatore
	bool checkTime();					//Ritorna 1 (per una sola chiamata) ogni volta che l'intervallo � trascorso. Attenzione, dopo aver restituito 1, la successiva chiamata restituisce 0 sino al nuovo superamento dell'intervallo. La prima chiamata fa partire il timer e rimane attivo finche non viene resettato con  reset()
	void reset();						//Reimposta il timer
private:
	unsigned long lastMillis;
	unsigned long actualMillis;
	bool memMillis = false;
	unsigned long Intervallo;
};

//Setta l'intervallo del temporizzatore
void TimeTrigger::setInterval(unsigned long Interval)
{
	Intervallo = Interval;
}

/*
Ritorna 1 (per una sola chiamata) ogni volta che l'intervallo � trascorso. 
Attenzione, dopo aver restituito 1, la successiva chiamata restituisce 0 sino al nuovo superamento dell'intervallo. 
La prima chiamata fa partire il timer e rimane attivo finche non viene resettato con  reset() */
bool TimeTrigger::checkTime()
{
	actualMillis = millis();

	if (memMillis == false)
	{
		lastMillis = actualMillis;
		memMillis = true;
	}

	if (lastMillis < actualMillis)				//Se millis non � andata in overflow...
	{
		if (actualMillis >= lastMillis + Intervallo) {
			memMillis = false;
			return 1;
		}
		else { return 0; }
	}
	else if (lastMillis > actualMillis)									//Se andato in overflow, aggiorno la variabile lastMillis al valore compatibile con millis
	{
		lastMillis = actualMillis + 4294967296 - lastMillis;
		return 0;
	}
	return 0;
}

//Reimposta il timer
void TimeTrigger::reset()
{
	memMillis = false;
}




/*
Timer:
Temporizzatore pi� esteso che consente un controllo maggiore ed una maggiore flessibilit� a scapito di una maggiore necessit� di funzioni
*/
class Timer
{
public:
	void setTimer(unsigned long);	//Setta l'intervallo
	bool startTimer();				//Fa partire il timer solo se non � stato gi� fatto partire oppure se � stato resettato
	bool checkTimer();				//Controlla se il timer � scaduto. Quando scaduto restituisce 1 finch� il timer non viene resettato
	void resetTimer();				//Resetta il timer
	bool started = 0;				//Restituisce lo stato del timer ovvero se � stato avviato o meno. Il valore rimane a 1 anche una volta scaduto l'intervallo.
	unsigned long msPassed = 0;		//Restituisce il tempo trascorso in millisecondi 
private:
	unsigned long lastMillis = 0;
	unsigned long actualMillis = 0;
	bool memMillis = false;
	unsigned long Intervallo;
	bool status = 0;
	
};

//Setta l'intervallo
void Timer::setTimer(unsigned long Interval)	
{	
	Intervallo = Interval;
	lastMillis = 0;
	status = 0;
	started = false;
}

//Fa partire il timer solo se non � stato gi� fatto partire oppure se � stato resettato
bool Timer::startTimer() {
	if (started == false)
	{
		lastMillis = millis();
	}
	started = true;
	return status;
}

//Controlla se il timer � scaduto. La funzione restituisce 1 finch� il timer non viene resettato
bool Timer::checkTimer()
{
	if (started == true)
	{
		actualMillis = millis();
		msPassed = actualMillis - lastMillis;
		if (lastMillis < actualMillis)				//Se millis non � andata in overflow...
		{
			if (actualMillis >= lastMillis + Intervallo) {
				status = 1;
				return status;
			}
			else { return 0; }
		}
		else if (lastMillis > actualMillis)									//Se andato in overflow, aggiorno la variabile lastMillis al valore compatibile con millis
		{
			lastMillis = actualMillis + 4294967296 - lastMillis;
			status = 0;
			return status;
		}
		status = 0;
		return status;
	}
	else
	{
		status = 0;
		return status;
	}
}

//Resetta il timer
void Timer::resetTimer() 
{
	lastMillis = 0;
	status = 0;
	started = false;
}