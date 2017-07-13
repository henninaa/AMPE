#pragma once
#include <string>
#include <vector>
#include <cmath>


#define STEPLENGTH 0.25

	class WP{
	public:

		WP(double n, double e, double d, double t = 0.0) : n(n), e(e), d(d), t(t) {}
		~WP(){}

		double n,e,d;
		double t;
	};

	class Entity{
	public:
		Entity(double n, double e, double d, int id, double angle = 0.0, double speed = 0.0) : n(n), e(e), d(d), id(id), isActive(true), vel(speed), angle(angle), currentTick(0) {}
		~Entity() {}
		
		double n,e,d;
		int id;
		bool isActive;

		double angle, vel;

		int currentTick;

		void setPosition(double n, double e, double d){ this->n = n; this->e = e; this->d = d;}
		void updatePos(int currentTick){

				int td = currentTick - this->currentTick;
				this->currentTick = currentTick;

				if(td <= 0)
					return;

				n += (vel * td * STEPLENGTH) * std::cos(angle);
				e += (vel * td * STEPLENGTH) * std::sin(angle);



		}
	};

	typedef Entity Node;
	
	class UAV
		: public Entity
	{
	public:
		UAV(double n, double e, double d, int id, double memorySize = 1000.0, double dataInMemory = 0.0) : Entity(n, e, d, id), memorySize(memorySize), dataInMemory(dataInMemory) {}
		~UAV() {}

		double dataInMemory;
		double memorySize;
	};