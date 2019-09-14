#include <ea.h>

double function(Chromosome<double> *a){
  std::vector<double> values = a->getChromosome();

  double sum = values[0]*2 + values[1]*4;

  double area = values[0] * values[1];

  if(sum>500){
    area = area/(1+(sum-500));
  }

  boost::this_thread::sleep( boost::posix_time::milliseconds(10) );

  return area;
}

int main(int argc, char** argv){
  EA<double> c = EA<double>(100, 2, VALUE, 0.0, 500, UNIFORM_CROSSOVER, 1.0, GAUSSIAN, 1.0, TOURNAMENT, 70, ELITISM);

  c.setFitnessFunction(function);

  c.randomInitialization();

  c.printPopulation();

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  Chromosome<double> *res = c.evolveThreaded(10000, 8);
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

  c.printPopulation();

  std::cout << std::endl << res->getFitness() << std::endl;

  std::cout << "Duration: " << duration << std::endl;

  return 0;
}
