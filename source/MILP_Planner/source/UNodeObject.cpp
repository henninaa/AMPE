#include "UNodeObject.h"



unsigned int NodeObject::currentUnusedID = int(1);



NodeObject::NodeObject() : position(0.0) {
	setNewID();
}

NodeObject::NodeObject(unsigned int ID) : position(0.0){

	if (ID == 0){
		setNewID();
	}
	else{
		setID(ID);
	}
}

NodeObject::NodeObject(Vector3 position, unsigned int ID){
	this->position = position;
	setID(ID);
}


NodeObject::~NodeObject(){

}


void NodeObject::setID(unsigned int id){

	if (id == 0){
		setNewID();
	}
	else {
		this->ID = id;

		if (id >= currentUnusedID)
			currentUnusedID = id+1;
	}
}

void NodeObject::setNewID(){
	ID = currentUnusedID;
	currentUnusedID++;
}