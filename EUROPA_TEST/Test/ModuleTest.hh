#ifndef _H_ModuleTest
#define _H_ModuleTest

#include "Module.hh"

namespace EUROPA {
  class ModuleTest : public Module
  {
    public:
      ModuleTest();
      virtual ~ModuleTest();

      /**
       * @brief Initialize all default elements of the module 
       */
	  virtual void initialize();
	  /**
	   * @brief Uninitialize all default elements of the module 
	   */
	  virtual void uninitialize();   

	  virtual void initialize(EngineId engine);   // initialization of a particular engine instance

	  virtual void uninitialize(EngineId engine); // cleanup of a particular engine instance	  
  };

  typedef Id<ModuleTest> ModuleTestId;  
}  


#endif /* #ifndef _H_ModuleTest */
