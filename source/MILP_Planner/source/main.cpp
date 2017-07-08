#include "UMPathPlanner.h"


int main(int argc, char const *argv[])
{
	UMPathPlanner path;

	std::vector<std::vector<double> > wp = std::vector<std::vector<double> >(4, std::vector<double>(3, 100));
	wp[0][0] = 0;wp[0][1] = 0;wp[0][2] = 100;
	wp[1][0] = 100;wp[1][1] = 100;wp[1][2] = 200;
	wp[2][0] = 400;wp[2][1] = 200;wp[2][2] = 400;
	wp[3][0] = 300;wp[3][1] = 200;wp[3][2] = 100;

	path.runAsync(wp);

	bool fin = false;
	std::vector<std::vector<std::vector<double> > >pathWp;

	std::cout << "t";
	while (!fin){
		if (path.isDone()){
			std::cout << "t";
			//pathWp = path.getPath();
			fin = true;
			std::cout << "DONE";
		}

	}
	return 0;
}