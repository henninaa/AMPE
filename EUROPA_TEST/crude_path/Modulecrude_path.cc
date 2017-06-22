#include "Modulecrude_path.hh"
#include "crude_pathCustomCode.hh"

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
          return (new Modulecrude_path());
	}
}

  static bool & crude_pathInitialized() {
    static bool sl_alreadyDone(false);
    return sl_alreadyDone;
  }

  Modulecrude_path::Modulecrude_path()
      : Module("crude_path")
  {
  }

  Modulecrude_path::~Modulecrude_path()
  {
  }

  void Modulecrude_path::initialize()
  {
      if(crude_pathInitialized())
    	  return;
	  crude_pathInitialized() = true;
  }

  void Modulecrude_path::uninitialize()
  {
	  crude_pathInitialized() = false;
  }

  void Modulecrude_path::initialize(EngineId engine)
  {
  }

  void Modulecrude_path::uninitialize(EngineId engine)
  {
  }
}
