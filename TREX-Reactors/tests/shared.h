#pragma once
#include <string>
#include <vector>


	class WP{
	public:

		WP(double n, double e, double d) : n(n), e(e), d(d) {}
		~WP(){}

		double n,e,d;
	};

	class Entity{
	public:
		Entity(double n, double e, double d, int id) : n(n), e(e), d(d), id(id), isActive(true) {}
		~Entity() {}
		
		double n,e,d;
		int id;
		bool isActive;

		void setPosition(double n, double e, double d){ this->n = n; this->e = e; this->d = d;}
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