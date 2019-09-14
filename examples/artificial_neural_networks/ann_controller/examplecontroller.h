/***************************************************************************
 *   Copyright (C) 2012 by Timo Nachstedt                                  *
 *    nachstedt@physik3.gwdg.de                                            *
 *                                                                         *
 * This is an example controller!                                          *
 *                                                                         *
 * Normally, the controller files should be placed in controller/amosII    *
 *                                                                         *
 **************************************************************************/

#ifndef EXAMPLECONTROLLER_H_
#define EXAMPLECONTROLLER_H_

#include <selforg/abstractcontroller.h>

// forward declaration (ExampleANN is defined in the .cpp file)
class ExampleANN;

class ExampleController : public AbstractController {
  public:
    ExampleController();
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~ExampleController();

    // returns the number of sensors the controller was initialised with or 0
    // if not initialised
    virtual int getSensorNumber() const;

    // returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const;

    // performs one step (includes learning).
    // Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, int number_sensors, motor*,
            int number_motors);

    // performs one step without learning. Calulates motor commands from sensor
    // inputs.
    virtual void stepNoLearning(const sensor*, int number_sensors,
        motor*, int number_motors);

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);
  private:
    ExampleANN* myANN;
};

#endif /* EXAMPLECONTROLLER_H_ */
