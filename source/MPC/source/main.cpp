#include "ALMPC.h"
#include "LMModelLinear.h"


int main(int argc, char const *argv[])
{
	/* code */

	LMModelLinear * test;
	test = new LMModelLinear();
	test->createModel();
	test->printAllParameters();

	return 0;
}

