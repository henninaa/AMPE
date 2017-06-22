#include "Moduleshopping.hh"
#include "shoppingCustomCode.hh"

// Pieces necessary for various customizations:
#include "PSPlanDatabase.hh"
#include "ConstraintType.hh"
#include "FlawHandler.hh"


namespace EUROPA {

// static C init method to get handle when loading module as shared library
extern "C"
{
	Module* initializeModule()
	{
          return (new Moduleshopping());
	}
}

  static bool & shoppingInitialized() {
    static bool sl_alreadyDone(false);
    return sl_alreadyDone;
  }

  Moduleshopping::Moduleshopping()
      : Module("shopping")
  {
  }

  Moduleshopping::~Moduleshopping()
  {
  }

  void Moduleshopping::initialize()
  {
      if(shoppingInitialized())
    	  return;
	  shoppingInitialized() = true;
  }

  void Moduleshopping::uninitialize()
  {
	  shoppingInitialized() = false;
  }

  void Moduleshopping::initialize(EngineId engine)
  {
  }

  void Moduleshopping::uninitialize(EngineId engine)
  {
  }
}
