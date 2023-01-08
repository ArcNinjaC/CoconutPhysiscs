#include <iostream>
#include "Simulation.h"


Simulation SimInstance = Simulation();
Simulation* pSimInstance = &SimInstance;

int main()
{
	pSimInstance->render(900, 260, 1);
}



