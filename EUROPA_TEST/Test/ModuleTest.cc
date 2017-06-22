#include "ModuleTest.hh"
#include "TestCustomCode.hh"

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
          return (new ModuleTest());
	}
}

  static bool & TestInitialized() {
    static bool sl_alreadyDone(false);
    return sl_alreadyDone;
  }

  ModuleTest::ModuleTest()
      : Module("Test")
  {
  }

  ModuleTest::~ModuleTest()
  {
  }

  void ModuleTest::initialize()
  {
      if(TestInitialized())
    	  return;
	  TestInitialized() = true;
  }

  void ModuleTest::uninitialize()
  {
	  TestInitialized() = false;
  }

  void ModuleTest::initialize(EngineId engine)
  {
  }

  void ModuleTest::uninitialize(EngineId engine)
  {
  }
}
