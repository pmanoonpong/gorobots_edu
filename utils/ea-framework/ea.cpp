template <class T>
Chromosome<T>::Chromosome(encoding_type Chromosome_encoding_type, T min, T max)
{
    this->chromosome_encoding = Chromosome_encoding_type;
    fitness_value = 0;
    evaluated = false;
    size = 0;
    lower_boundary = min;
    upper_boundary = max;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
    chromosome_values.clear();
}

template <class T>
Chromosome<T>::Chromosome()
{
    lower_boundary = 0;
    upper_boundary = 1;
    size = 1;
    fitness_value = 0;
    evaluated = false;
    chromosome_encoding = encoding_type::BINARY;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
    chromosome_values.clear();
}

template <class T>
Chromosome<T>::Chromosome(const Chromosome& chromosome_copy)
{
    lower_boundary = chromosome_copy.lower_boundary;
    upper_boundary = chromosome_copy.upper_boundary;
    size = chromosome_copy.size;
    fitness_value = chromosome_copy.fitness_value;
    evaluated = true;
    chromosome_encoding = chromosome_copy.chromosome_encoding;
    generator = chromosome_copy.generator;
    for (unsigned int value = 0; value < size; ++value) {
        chromosome_values[value] = chromosome_copy.chromosome_values[value];
    }
}

template <class T>
Chromosome<T>& Chromosome<T>::operator=(const Chromosome& chromosome_copy)
{
    lower_boundary = chromosome_copy.lower_boundary;
    upper_boundary = chromosome_copy.upper_boundary;
    size = chromosome_copy.size;
    fitness_value = chromosome_copy.fitness_value;
    evaluated = true;
    chromosome_encoding = chromosome_copy.chromosome_encoding;
    generator = chromosome_copy.generator;
    for (unsigned int value = 0; value < size; ++value) {
        chromosome_values.push_back(chromosome_copy.chromosome_values[value]);
    }
    return *this;
}

template <class T>
Chromosome<T>::~Chromosome()
{
    chromosome_values.clear();
}

template <class T>
void Chromosome<T>::setChromosome(std::vector<T> values)
{
    size = values.size();
    //chromosome_values = new T[size];

    for (unsigned int i = 0; i < size; i++) {
        chromosome_values.push_back(values[i]);
    }
}

template <class T>
void Chromosome<T>::setChromosome(T values[], int size)
{
    this->size = size;
    chromosome_values = new T[size];

    for (unsigned int i = 0; i < size; i++) {
        chromosome_values[i] = values[i];
    }
}

template <class T>
std::vector<T> Chromosome<T>::getChromosome()
{
    return chromosome_values;
}

template <class T>
void Chromosome<T>::copyChromosome(std::vector<T>& chromosome_values)
{
    chromosome_values.clear();
    for (unsigned int gen = 0; gen < size; ++gen)
        chromosome_values.push_back(getChromosome()[gen]);
}

template <class T>
void Chromosome<T>::setFitness(double value)
{
    fitness_value = value;
    evaluated = true;
}

template <class T>
bool Chromosome<T>::isEvaluated(){
    return evaluated;
}

template <class T>
double Chromosome<T>::getFitness()
{
    return fitness_value;
}

template <class T>
void Chromosome<T>::bitStringMutation()
{
    double probability;
    for (unsigned int i = 0; i < size; i++) {
        probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
        if (probability > 1.0 / size) {
            chromosome_values[i] = !chromosome_values[i];
        }
    }
}

template <class T>
void Chromosome<T>::flipBitMutation()
{
    for (unsigned int i = 0; i < size; i++) {
        chromosome_values[i] = !chromosome_values[i];
    }
}

template <class T>
void Chromosome<T>::boundaryMutation(double mutation_rate)
{
    double ud, probability;
    for (unsigned int i = 0; i < size; i++) {
        probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
        if (probability < mutation_rate) {
            ud = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (ud > 0.5) {
                chromosome_values[i] = upper_boundary;
            }
            else {
                chromosome_values[i] = lower_boundary;
            }
        }
    }
}

template <class T>
void Chromosome<T>::uniformMutation(double mutation_rate)
{
    double probability;
    for (unsigned int i = 0; i < size; i++) {
        probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
        if (probability < mutation_rate) {
            double random_value = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            chromosome_values[i] = random_value * (upper_boundary - lower_boundary) + lower_boundary;
        }
    }
}

template <class T>
void Chromosome<T>::gaussianMutation(double mutation_rate)
{
    double probability;
    std::normal_distribution<double> distribution = std::normal_distribution<double>((lower_boundary + upper_boundary) / 2, lower_boundary / 2 + upper_boundary);
    for (unsigned int i = 0; i < size; i++) {
        probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
        if (probability < mutation_rate) {
            chromosome_values[i] += distribution(generator);
            if (chromosome_values[i] > upper_boundary)
                chromosome_values[i] = upper_boundary;
            else if (chromosome_values[i] < lower_boundary)
                chromosome_values[i] = lower_boundary;
        }
    }
}

template <class T>
void Chromosome<T>::printChromosome()
{
    for (unsigned int i = 0; i < size; i++) {
        std::cout << chromosome_values[i] << " ";
    }
    std::cout << std::endl;
}

template <class T>
EA<T>::EA(int population, int size, encoding_type chromosome_encoding, T min, T max, crossover_type crossover, double crossover_rate, mutation_type mutation, double mutation_rate, selection_type selection, int selection_size, replacement_type replacement, bool pre_breeding)
{
	//pre_breeding = false;
    population_size = population;
    Chromosome_size = size;
    encoding = chromosome_encoding;
    lower_boundary = min;
    upper_boundary = max;
    this->crossover = crossover;
    this->mutation = mutation;
    this->mutation_rate = mutation_rate;
    this->selection = selection;
    this->crossover_rate = crossover_rate;
    if (selection_size % 2 != 0) {
        std::cerr << "The selection size has to be an even number." << std::endl;
    }
    this->selection_size = selection_size;
    this->replacement = replacement;
    this->pre_breeding = pre_breeding;
    generation = 0;

    plot_live = true;

    fitnessFile.open("fitness_function.cvs", std::ios::trunc);
    fitnessFile << "Generation,Fitness value,Fitness mean,";
    for (unsigned int gen_number = 0; gen_number < Chromosome_size; ++gen_number)
        fitnessFile << "Gen " << gen_number << ",";
    fitnessFile << std::endl;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
}

template <class T>
EA<T>::EA(std::string file){
    std::ifstream checkpoint(file);

    std::string line;
    std::string value;
    int character = 0;
    getline(checkpoint, line);
    std::stringstream ss(line);
    while(getline(ss, value, ',')){
        switch(character){
        case 0:
            population_size = stoi(value);
            break;

        case 1:
            Chromosome_size = stoi(value);
            break;

        case 2:
            encoding = static_cast<encoding_type>(stoi(value));
            break;

        case 3:
            lower_boundary = stod(value);
            break;

        case 4:
            upper_boundary = stod(value);
            break;

        case 5:
            crossover = static_cast<crossover_type>(stoi(value));
            break;

        case 6:
            crossover_rate = stod(value);
            break;

        case 7:
            mutation = static_cast<mutation_type>(stoi(value));
            break;

        case 8:
            mutation_rate = stod(value);
            break;

        case 9:
            selection = static_cast<selection_type>(stoi(value));
            break;

        case 10:
            selection_size = stoi(value);
            break;

        case 11:
            replacement = static_cast<replacement_type>(stoi(value));
            break;

        case 12:
            generation = stoi(value);
            break;

        default:
            break;
        }
        character ++;
    }

    Chromosome<T>* new_chromosome;

    while(getline(checkpoint, line)){
        std::stringstream ss(line);
        std::string value;
        new_chromosome = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
        std::vector<T> values;
        double fitness_value;
        bool fitness_string = true;
        while(getline(ss, value, ',')){
            if(fitness_string){
                fitness_value = stod(value);
                fitness_string = false;
            }
            else{
                values.push_back(stod(value));
            }
        }
        new_chromosome->setChromosome(values);
        new_chromosome->setFitness(fitness_value);
        population.push_back(new_chromosome);
    }

    plot_live = true;

    fitnessFile.open("fitness_function.cvs", std::ios::app);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
}

template <class T>
EA<T>::~EA()
{
}

template <class T>
void EA<T>::setFitnessFunction(double (*function)(Chromosome<T>*))
{
    fitnessFunction = function;
}

template <class T>
void EA<T>::randomInitialization()
{
    Chromosome<T>* c;
    for (unsigned int i = 0; i < population_size; i++) {
        c = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
        std::vector<T> values;
        for (unsigned int j = 0; j < Chromosome_size; j++) {
            T value = (T)(std::generate_canonical<double, std::numeric_limits<double>::digits>(generator) * (upper_boundary - lower_boundary) + lower_boundary);
            values.push_back(value);
        }
        c->setChromosome(values);
        population.push_back(c);
    }
}

template <class T>
void EA<T>::initializePopulation(std::vector<Chromosome<T>*> initial_population)
{
    if (initial_population.size() != population_size) {
        std::cerr << "Wrong vector size. It doesn't match the population size!" << std::endl;
    }
    else {
        population = initial_population;
    }
}

template <class T>
void EA<T>::printPopulation()
{
    for (unsigned int i = 0; i < population.size(); i++) {
        std::cout << i + 1 << "(" << population[i]->getFitness() << ")"
                  << ": ";
        population[i]->printChromosome();
    }
}

template <class T>
void EA<T>::evaluate()
{
    for (unsigned int i = 0; i < population.size(); i++) {
        if (!population[i]->isEvaluated()){
            population[i]->setFitness(fitnessFunction(population[i]));
        }
    }
}

template <class T>
void EA<T>::evaluateOffspring()
{
    for (unsigned int i = 0; i < offspring.size(); i++) {
        if (offspring[i]){
            offspring[i]->setFitness(fitnessFunction(offspring[i]));
        }
    }
}

template <class T>
void EA<T>::evaluateThreaded(unsigned int threads)
{
    //boost::threadpool::fifo_pool tp(threads);
    std::vector<boost::threadpool::future<double>> results;
    std::vector<int> indexes;
    for (unsigned int i = 0; i < population.size(); i++) {
        if(!population[i]->isEvaluated()){
            std::function<double()> func = std::bind<double>(&EA<T>::evaluateOne, std::ref(*this), population[i]);
            results.push_back(boost::threadpool::schedule(tp, func));
            indexes.push_back(i);
        }
    }
    tp.wait();

    for(unsigned int i=0; i<indexes.size(); i++){
        population[indexes[i]]->setFitness(results[i].get());
    }
}

template <class T>
void EA<T>::evaluateOffspringThreaded(unsigned int threads)
{
    //boost::threadpool::fifo_pool tp(threads);
    std::vector<boost::threadpool::future<double>> results;
    std::vector<int> indexes;
    for (unsigned int i = 0; i < offspring.size(); i++) {
        if(offspring[i]){
            std::function<double()> func = std::bind<double>(&EA<T>::evaluateOne, std::ref(*this), offspring[i]);
            results.push_back(boost::threadpool::schedule(tp, func));
            indexes.push_back(i);
        }
    }
    tp.wait();

    for(unsigned int i=0; i<indexes.size(); i++){
        offspring[indexes[i]]->setFitness(results[i].get());
    }
}

template <class T>
double EA<T>::evaluateOne(Chromosome<T>* i){
    return fitnessFunction(i);
}

template <class T>
void EA<T>::select()
{
    std::sort(population.begin(), population.end(), ChromosomeComp<T>);

    selected.clear();

    switch (selection) {
    case ROULETTE: {
        T roulette_sum = 0;
        for (unsigned int i = 0; i < population.size(); i++) {
            roulette_sum += population[i]->getFitness();
        }

        for (unsigned int i = 0; i < selection_size; i++) {
            std::uniform_real_distribution<double> distribution(0.0, roulette_sum);
            T roulette_rand = (T)distribution(generator);
            T pick_sum = 0;
            for (unsigned int j = 0; j < population.size(); j++) {
                pick_sum += population[j]->getFitness();
                if (pick_sum >= roulette_rand) {
                    selected.push_back(population[j]);
                    roulette_sum -= population[j]->getFitness();
                    population.erase(population.begin() + j);
                    break;
                }
            }    std::ofstream fitnessFile;

        }
        std::sort(selected.begin(), selected.end());
        break;
    }

    case RANK: {
        std::vector<double> ranks;
        double rank_sum = 0;
        for (unsigned int i = 0; i < population.size(); i++) {
            rank_sum += population[i]->getFitness();
        }
        for (unsigned int i = 0; i < population.size(); i++) {
            ranks.push_back(population[i]->getFitness() * 100.0 / rank_sum);
        }
        for (unsigned int i = 0; i < selection_size; i++) {
            std::uniform_int_distribution<int> distribution(0, 100);
            int rank_rand = distribution(generator);
            double pick_sum = 0;
            for (unsigned int j = 0; j < population.size(); j++) {
                pick_sum += ranks[j];
                if (pick_sum >= rank_rand) {
                    selected.push_back(population[j]);
                    population.erase(population.begin() + j);
                    ranks.clear();
                    rank_sum = 0;
                    for (unsigned int k = 0; k < population.size(); k++) {
                        rank_sum += population[k]->getFitness();
                    }
                    for (unsigned int k = 0; k < population.size(); k++) {
                        ranks.push_back(population[k]->getFitness() * 100 / rank_sum);
                    }
                    break;
                }
            }
        }
        std::sort(selected.begin(), selected.end());
        break;
    }

    case TOURNAMENT: {
        std::vector<int> candidates;
        for (unsigned int i = 0; i < selection_size; i++) {
            std::uniform_int_distribution<int> distribution(0, population.size() - 1);
            for (unsigned int j = 0; j < (population_size - selection_size) / 2; j++) {
                candidates.push_back(distribution(generator));
            }
            int best = 0;
            double max = 0;
            for (unsigned int j = 0; j < candidates.size(); j++) {
                if (population[candidates[j]]->getFitness() > max) {
                    max = population[candidates[j]]->getFitness();
                    best = candidates[j];
                }
            }
            selected.push_back(population[best]);
            population.erase(population.begin() + best);
            candidates.clear();
        }
        std::sort(selected.begin(), selected.end());
        break;
    }
    }
}

template <class T>
void EA<T>::preBreeding(){
    std::sort(population.begin(), population.end(), ChromosomeCompInv<T>);

    selected.clear();

    std::vector<Chromosome<T>*> elite;
    std::vector<Chromosome<T>*> selected_for_elite;

    for(unsigned int i=0; i<population_size-selection_size; i++){
        elite.push_back(population.front());
        population.erase(population.begin());
    }
    std::cout << std::endl;

    switch (selection) {
    case ROULETTE: {
        T roulette_sum = 0;
        for (unsigned int i = 0; i < population.size(); i++) {
            roulette_sum += population[i]->getFitness();
        }

        for (unsigned int i = 0; i < population_size-selection_size; i++) {
            std::uniform_real_distribution<double> distribution(0.0, roulette_sum);
            T roulette_rand = (T)distribution(generator);
            T pick_sum = 0;
            for (unsigned int j = 0; j < population.size(); j++) {
                pick_sum += population[j]->getFitness();
                if (pick_sum >= roulette_rand) {
                    selected_for_elite.push_back(population[j]);
                    roulette_sum -= population[j]->getFitness();
                    population.erase(population.begin() + j);
                    break;
                }
            }    std::ofstream fitnessFile;

        }
        break;
    }

    case RANK: {
        std::vector<double> ranks;
        double rank_sum = 0;
        for (unsigned int i = 0; i < population.size(); i++) {
            rank_sum += population[i]->getFitness();
        }
        for (unsigned int i = 0; i < population.size(); i++) {
            ranks.push_back(population[i]->getFitness() * 100.0 / rank_sum);
        }
        for (unsigned int i = 0; i < population_size-selection_size; i++) {
            std::uniform_int_distribution<int> distribution(0, 100);
            int rank_rand = distribution(generator);
            double pick_sum = 0;
            for (unsigned int j = 0; j < population.size(); j++) {
                pick_sum += ranks[j];
                if (pick_sum >= rank_rand) {
                    selected_for_elite.push_back(population[j]);
                    population.erase(population.begin() + j);
                    ranks.clear();
                    rank_sum = 0;
                    for (unsigned int k = 0; k < population.size(); k++) {
                        rank_sum += population[k]->getFitness();
                    }
                    for (unsigned int k = 0; k < population.size(); k++) {
                        ranks.push_back(population[k]->getFitness() * 100 / rank_sum);
                    }
                    break;
                }
            }
        }
        break;
    }

    case TOURNAMENT: {
        std::vector<int> candidates;
        for (unsigned int i = 0; i < population_size-selection_size; i++) {
            std::uniform_int_distribution<int> distribution(0, population.size() - 1);
            for (unsigned int j = 0; j < (population_size - selection_size) / 2; j++) {
                candidates.push_back(distribution(generator));
            }
            int best = 0;
            double max = 0;
            for (unsigned int j = 0; j < candidates.size(); j++) {
                if (population[candidates[j]]->getFitness() > max) {
                    max = population[candidates[j]]->getFitness();
                    best = candidates[j];
                }
            }
            selected_for_elite.push_back(population[best]);
            population.erase(population.begin() + best);
            candidates.clear();
        }
        break;
    }
    }

    while(population.size()>0){
        selected.push_back(population.front());
        population.erase(population.begin());
    }

    offspring.clear();

    switch (crossover) {
    case SINGLE_POINT:
        for (unsigned int i = 0; i < elite.size(); i++) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            elite[i]->copyChromosome(parent1);
            selected_for_elite[i]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
                if (j < Chromosome_size / 2) {
                    values1.push_back(parent1[j]);
                    values2.push_back(parent2[j]);
                }
                else {
                    values1.push_back(parent2[j]);
                    values2.push_back(parent1[j]);
                }
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
        }
        break;

    case TWO_POINT:
        for (unsigned int i = 0; i < elite.size(); i++) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            elite[i]->copyChromosome(parent1);
            selected_for_elite[i]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
                if (j < Chromosome_size / 3 || j > Chromosome_size * 2 / 3) {
                    values1.push_back(parent1[j]);
                    values2.push_back(parent2[j]);
                }
                else {
                    values1.push_back(parent2[j]);
                    values2.push_back(parent1[j]);
                }
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
        }
        break;

    case UNIFORM_CROSSOVER: {
        std::vector<bool> mask;
        for (unsigned int i = 0; i < Chromosome_size; i++) {
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (probability > 0.5) {
                mask.push_back(true);
            }
            else {
                mask.push_back(false);
            }
        }

        for (unsigned int i = 0; i < elite.size(); i++) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            elite[i]->copyChromosome(parent1);
            selected_for_elite[i]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
                if (mask[j]) {
                    values1.push_back(parent1[j]);
                    values2.push_back(parent2[j]);
                }
                else {
                    values1.push_back(parent2[j]);
                    values2.push_back(parent1[j]);
                }
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
        }
        break;
    }

    case FLAT: {
        std::vector<double> r;
        for(unsigned int i = 0; i < Chromosome_size; i++){
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            r.push_back(probability);
        }
        for (unsigned int i = 0; i < elite.size(); i++) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            elite[i]->copyChromosome(parent1);
            selected_for_elite[i]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
                values1.push_back(parent1[j]*r[j]+parent2[j]*(1-r[j]));
                values2.push_back(parent1[j]*(1-r[j])+parent2[j]*r[j]);
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
        }
    }
    }

    for (unsigned int i = 0; i < offspring.size(); i++) {
        if (offspring[i]) {
            switch (mutation) {
            case BIT_STRING:
                offspring[i]->bitStringMutation();
                break;

            case FLIP_BIT:
                offspring[i]->flipBitMutation();
                break;

            case BOUNDARY:
                offspring[i]->boundaryMutation(mutation_rate);
                break;

            case NON_UNIFORM:
                offspring[i]->uniformMutation(1.0 / generation);
                break;

            case UNIFORM:
                offspring[i]->uniformMutation(mutation_rate);
                break;

            case GAUSSIAN:
                offspring[i]->gaussianMutation(mutation_rate);
                break;
            }
        }
    }

    while(elite.size()>0){
        population.push_back(elite.front());
        elite.erase(elite.begin());
    }
    evaluateOffspring();
    std::sort(offspring.begin(), offspring.end(), ChromosomeComp<T>);
    for(int i=0; i<10; i++){
        population.push_back(offspring.front());
        offspring.erase(offspring.begin());
    }
    offspring.clear();
}

template <class T>
void EA<T>::reproduce()
{
    offspring.clear();

    switch (crossover) {
    case SINGLE_POINT:
        for (unsigned int i = 0; i < selected.size(); i += 2) {
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (probability < crossover_rate) {
                Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                std::vector<T> values1, values2, parent1, parent2;
                selected[i]->copyChromosome(parent1);
                if ((i + 1) >= selected.size())
                    selected[0]->copyChromosome(parent2);
                else
                    selected[i + 1]->copyChromosome(parent2);
                for (unsigned int j = 0; j < Chromosome_size; j++) {
                    if (j < Chromosome_size / 2) {
                        values1.push_back(parent1[j]);
                        values2.push_back(parent2[j]);
                    }
                    else {
                        values1.push_back(parent2[j]);
                        values2.push_back(parent1[j]);
                    }
                }
                child1->setChromosome(values1);
                child2->setChromosome(values2);
                offspring.push_back(child1);
                offspring.push_back(child2);
            }
            else {
                offspring.push_back(NULL);
                offspring.push_back(NULL);
            }
        }
        break;

    case TWO_POINT:
        for (unsigned int i = 0; i < selected.size(); i += 2) {
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (probability < crossover_rate) {
                Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                std::vector<T> values1, values2, parent1, parent2;
                selected[i]->copyChromosome(parent1);
                if ((i + 1) >= selected.size())
                    selected[0]->copyChromosome(parent2);
                else
                    selected[i + 1]->copyChromosome(parent2);
                for (unsigned int j = 0; j < Chromosome_size; j++) {
                    if (j < Chromosome_size / 3 || j > Chromosome_size * 2 / 3) {
                        values1.push_back(parent1[j]);
                        values2.push_back(parent2[j]);
                    }
                    else {
                        values1.push_back(parent2[j]);
                        values2.push_back(parent1[j]);
                    }
                }
                child1->setChromosome(values1);
                child2->setChromosome(values2);
                offspring.push_back(child1);
                offspring.push_back(child2);
            }
            else {
                offspring.push_back(NULL);
                offspring.push_back(NULL);
            }
        }
        break;

    case UNIFORM_CROSSOVER: {
        std::vector<bool> mask;
        for (unsigned int i = 0; i < Chromosome_size; i++) {
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (probability > 0.5) {
                mask.push_back(true);
            }
            else {
                mask.push_back(false);
            }
        }

        for (unsigned int i = 0; i < selected.size(); i += 2) {
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (probability < crossover_rate) {
                Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                std::vector<T> values1, values2, parent1, parent2;
                unsigned int random_parent1 = (std::rand() % (int)(selected.size()));
                unsigned int random_parent2 = (std::rand() % (int)(selected.size()));
                selected[random_parent1]->copyChromosome(parent1);
                selected[random_parent2]->copyChromosome(parent2);
                for (unsigned int j = 0; j < Chromosome_size; j++) {
                    if (mask[j]) {
                        values1.push_back(parent1[j]);
                        values2.push_back(parent2[j]);
                    }
                    else {
                        values1.push_back(parent2[j]);
                        values2.push_back(parent1[j]);
                    }
                }
                child1->setChromosome(values1);
                child2->setChromosome(values2);
                offspring.push_back(child1);
                offspring.push_back(child2);
            }
            else {
                offspring.push_back(NULL);
                offspring.push_back(NULL);
            }
        }
        break;
    }

    case FLAT: {
        std::vector<double> r;
        for(unsigned int i = 0; i < Chromosome_size; i++){
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            r.push_back(probability);
        }
        std::random_shuffle(selected.begin(), selected.end());
        for (unsigned int i = 0; i < selected.size(); i += 2) {
            double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
            if (probability < crossover_rate) {
                Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
                std::vector<T> values1, values2, parent1, parent2;
                selected[i]->copyChromosome(parent1);
                selected[i+1]->copyChromosome(parent2);
                for (unsigned int j = 0; j < Chromosome_size; j++) {
                    values1.push_back(parent1[j]*r[j]+parent2[j]*(1-r[j]));
                    values2.push_back(parent1[j]*(1-r[j])+parent2[j]*r[j]);
                }
                child1->setChromosome(values1);
                child2->setChromosome(values2);
                offspring.push_back(child1);
                offspring.push_back(child2);
            }
            else {
                offspring.push_back(NULL);
                offspring.push_back(NULL);
            }
        }
    }
    }

    for (unsigned int i = 0; i < offspring.size(); i++) {
        if (offspring[i]) {
            switch (mutation) {
            case BIT_STRING:
                offspring[i]->bitStringMutation();
                break;

            case FLIP_BIT:
                offspring[i]->flipBitMutation();
                break;

            case BOUNDARY:
                offspring[i]->boundaryMutation(mutation_rate);
                break;

            case NON_UNIFORM:
                offspring[i]->uniformMutation(1.0 / generation);
                break;

            case UNIFORM:
                offspring[i]->uniformMutation(mutation_rate);
                break;

            case GAUSSIAN:
                offspring[i]->gaussianMutation(mutation_rate);
                break;
            }
        }
    }
}

template <class T>
void EA<T>::replace()
{
    switch (replacement) {
    case GENERATIONAL:
        for (unsigned int i = 0; i < offspring.size(); i++) {
            if (offspring[i]) {
                population.push_back(offspring[i]);
            }
            else {
                population.push_back(selected[i]);
            }
        }
        break;

    case STEADY_STATE:
        for (unsigned int i = 0; i < offspring.size(); i += 2) {
            if (offspring[i] && offspring[i + 1]) {
                std::vector<Chromosome<T>*> replacing_order;
                replacing_order.push_back(selected[i]);
                replacing_order.push_back(selected[i + 1]);
                replacing_order.push_back(offspring[i]);
                replacing_order.push_back(offspring[i + 1]);
                std::sort(replacing_order.begin(), replacing_order.end(), ChromosomeComp<T>);

                population.push_back(replacing_order.back());
                replacing_order.pop_back();
                population.push_back(replacing_order.back());
                replacing_order.clear();
            }
            else {
                population.push_back(selected[i]);
                population.push_back(selected[i + 1]);
            }
        }
        break;

    case ELITISM:
        for (unsigned int i = 0; i < selected.size(); i++) {
            population.push_back(selected[i]);
        }
        std::sort(population.begin(), population.end(), ChromosomeCompInv<T>);
        for (unsigned int i = 0; i < offspring.size(); i++) {
            if (offspring[i]) {
                population.pop_back();
            }
        }
        for (unsigned int i = 0; i < offspring.size(); i++) {
            if (offspring[i]) {
                population.push_back(offspring[i]);
            }
        }
        break;
    }
}

template <class T>
void EA<T>::evolve(unsigned int generations, int solution)
{
    bool solution_found = false;
    for (unsigned int i = 0; i < generations; i++) {
        evaluate();
        for (unsigned int i = 0; i < population.size(); i++) {
            if (population[i]->getFitness() == solution) {
                solution_found = true;
                std::cout << generation << " generations. Found solution: ";
                population[i]->printChromosome();
            }
        }
        if (solution_found) {
            break;
        }

        generation++;

        evaluate();
        select();
        reproduce();
        replace();
    }
    std::cout << "Solution not found..." << std::endl;
}

template <class T>
Chromosome<T>* EA<T>::evolve(unsigned int generations)
{
    Gnuplot livePlot;
    livePlot << "set terminal wxt noraise\n";

    fitnessValuesMax.clear();
    fitnessMean.clear();
    evaluate();
    checkpoint("checkpoint.csv");
    for (unsigned int i = generation; i < generations; i++) {

        generation++;

        if(pre_breeding){
            preBreeding();
            reproduce();
            evaluateOffspring();
            replace();
        }
        else{
            select();
            reproduce();
            evaluateOffspring();
            replace();
        }

        double fitness_mean = 0;
        for(uint i=0; i<population_size; i++){
            fitness_mean += population[i]->getFitness();
        }
        fitness_mean /= population_size;

        std::sort(population.begin(), population.end(), ChromosomeCompInv<T>);

        if(plot_live){
            fitnessValuesMax.push_back(std::make_pair(generation, population[0]->getFitness()));
            fitnessMean.push_back(std::make_pair(generation, fitness_mean));
        }
        fitnessFile << generation << "," << population[0]->getFitness();
        fitnessFile << "," << fitness_mean;
        for (auto gen : population[0]->getChromosome())
            fitnessFile << "," << gen;
        fitnessFile << std::endl;

        if(plot_live){
            livePlot << "plot '-' with lines title 'Fitness values', '-' with lines title 'Fitness mean'\n";
            livePlot.send1d(fitnessValuesMax);
            livePlot.send1d(fitnessMean);
            livePlot.flush();
        }

        std::random_shuffle(population.begin(), population.end());
        checkpoint("checkpoint.csv");
    }
    fitnessFile.close();

    int best_index = 0;
    double max = 0;
    for (unsigned int i = 0; i < population.size(); i++) {
        if (population[i]->getFitness() > max) {
            max = population[i]->getFitness();
            best_index = i;
        }
    }

    return population[best_index];
}

template <class T>
Chromosome<T>* EA<T>::evolveThreaded(unsigned int generations, unsigned int threads)
{
    Gnuplot livePlot;
    livePlot << "set terminal wxt noraise\n";

    fitnessValuesMax.clear();

    tp = boost::threadpool::fifo_pool(threads);

    evaluateThreaded(threads);
    for (unsigned int i = 0; i < generations; i++) {

        generation++;

        select();
        reproduce();
        evaluateOffspringThreaded(threads);
        replace();
        std::sort(population.begin(), population.end(), ChromosomeComp<T>);

        if(plot_live){
            fitnessValuesMax.push_back(std::make_pair(generation, population.back()->getFitness()));
        }

        fitnessFile << generation << "," << population.back()->getFitness();
        for (auto gen : population.back()->getChromosome())
            fitnessFile << "," << gen;
        fitnessFile << std::endl;

        if(plot_live){
            livePlot << "plot '-' with lines title 'Fitness values'\n";
            livePlot.send1d(fitnessValuesMax);
            livePlot.flush();
        }
    }

    fitnessFile.close();

    int best_index = 0;
    double max = population[0]->getFitness();
    for (unsigned int i = 0; i < population.size(); i++) {
        if (population[i]->getFitness() > max) {
            max = population[i]->getFitness();
            best_index = i;
        }
    }

    return population[best_index];
}

template <class T>
bool EA<T>::checkpoint(std::string file_name){
    std::ofstream checkpoint;
    checkpoint.open(file_name, std::ios::trunc);

    checkpoint << population_size << "," << Chromosome_size << ",";
    checkpoint << encoding << "," << lower_boundary << "," << upper_boundary << ",";
    checkpoint << crossover << "," << crossover_rate << ",";
    checkpoint << mutation << "," << mutation_rate << ",";
    checkpoint << selection << "," << selection_size << ",";
    checkpoint << replacement << "," << generation << std::endl;

    for (unsigned int i = 0; i < population.size(); i++) {
        checkpoint << population[i]->getFitness();
        for (unsigned int j = 0; j < Chromosome_size; j++) {
            checkpoint << "," << population[i]->chromosome_values[j];
        }
        checkpoint << std::endl;
    }
    checkpoint.close();

    return true;
}

template <class T>
bool ChromosomeComp(Chromosome<T>* A, Chromosome<T>* B)
{
    return A->getFitness() < B->getFitness();
}

template <class T>
bool ChromosomeCompInv(Chromosome<T>* A, Chromosome<T>* B)
{
    return A->getFitness() > B->getFitness();
}
