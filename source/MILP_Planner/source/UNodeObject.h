#pragma once

#include "../../Utillities/Vector3.h"

class NodeObject{

public:
	NodeObject();
	NodeObject(unsigned int ID);
	NodeObject(Vector3 position, unsigned int ID = 0);
	~NodeObject();

	unsigned int getID() { return ID; }
	
	void setPos(Vector3 pos) { position = pos; }

	double & operator[](int i){ return position[i]; }

private:

	static unsigned int currentUnusedID;
	unsigned int ID;						//0 is undefined
	void setID(unsigned int id);
	void setNewID();
	Vector3 position;

};