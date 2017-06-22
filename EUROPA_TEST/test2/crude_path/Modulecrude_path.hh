#ifndef _H_Modulecrude_path
#define _H_Modulecrude_path

#include "Module.hh"

namespace EUROPA {
  class Modulecrude_path : public Module
  {
    public:
      Modulecrude_path();
      virtual ~Modulecrude_path();

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

  typedef Id<Modulecrude_path> Modulecrude_pathId;  
}  


#endif /* #ifndef _H_Modulecrude_path */
