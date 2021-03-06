/**
 * @file Main.cc
 *
 * @brief Provides a main() for your project which uses
 * - a standard chronological backtracking planner
 * - a PSEngine to encapsulate EUROPA
 */

#include "PSEngine.hh"
#ifndef USE_EUROPA_DLL
		#include "ModuleLight.hh"
#endif
#include "LightCustomCode.hh"

using namespace EUROPA;

bool solve(const char* plannerConfig, const char* txSource, int startHorizon, int endHorizon, int maxSteps);
void runSolver(PSSolver* solver, int startHorizon, int endHorizon, int maxSteps);
void checkSolver(PSSolver* solver, int i);
void printFlaws(int it, PSList<std::string>& flaws);
#define logMsg(category,msg) { std::cout << category << "::" << msg << std::endl; }

int main(int argc, const char ** argv)
{
  if (argc < 3) {
    std::cerr << "Must provide nddl file." << std::endl;
    return -1;
  }

  const char* nddlFile = argv[1];
  const char* plannerConfig = argv[2];

  solve(
      plannerConfig,
      nddlFile,
      0,   // startHorizon
      100, // endHorizon
      1000 // maxSteps
  );

  return 0;
}

bool solve(const char* plannerConfig,
           const char* nddlFile,
           int startHorizon,
           int endHorizon,
           int maxSteps)
{
    try {

        PSEngine* engine = PSEngine::makeInstance();
        engine->start();
#ifndef USE_EUROPA_DLL
        engine->addModule((new ModuleLight()));
#endif                
        std::string errors = engine->executeScript("nddl",nddlFile,true/*isFile*/);
        if(!errors.empty()) {
          std::cout << errors << std::endl;
          return false;
        }

        PSSolver* solver = engine->createSolver(plannerConfig);
        runSolver(solver,startHorizon,endHorizon,maxSteps);
        delete solver;
	std::cout << engine->planDatabaseToString() << std::endl;
        delete engine;

        return true;
    }
    catch (Error& e) {
        std::cerr << "PSEngine failed:" << e.getMsg() << std::endl;
        return false;
    }
}

void printFlaws(int it, PSList<std::string>& flaws)
{
    logMsg("Main","Iteration:" << it << " " << flaws.size() << " flaws");

    for (int i=0; i<flaws.size(); i++) {
        logMsg("Main", "    " << (i+1) << " - " << flaws.get(i));
    }
}

void runSolver(PSSolver* solver, int startHorizon, int endHorizon, int maxSteps)
{
    solver->configure(startHorizon,endHorizon);
    int i;
    for (i = 0;
         !solver->isExhausted() &&
         !solver->isTimedOut() &&
         i<maxSteps;
         i = solver->getStepCount()) {

      solver->step();
      PSList<std::string> flaws;
      if (solver->isConstraintConsistent()) {
          flaws = solver->getFlaws();
          printFlaws(i,flaws);
          if (flaws.size() == 0)
              break;
      }
      else
          logMsg("Main","Iteration " << i << " Solver is not constraint consistent");
    }

    checkSolver(solver,i);
}

void checkSolver(PSSolver* solver, int i)
{
    if (solver->isExhausted()) {
      logMsg("Main","Solver was exhausted after " << i << " steps");
    }
    else if (solver->isTimedOut()) {
      logMsg("Main","Solver timed out after " << i << " steps");
    }
    else {
      logMsg("Main","Solver finished after " << i << " steps");
    }
}
