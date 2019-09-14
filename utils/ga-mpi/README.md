MPI Implementation of the Genetic Algorithm Library GAlib (http://lancet.mit.edu/ga/)// 
=================================================
This code has been checked out 22. Nov 2016 from https://github.com/B0RJA/GAlib-mpi master branch.
Only slight modifications have been applied, including applying fixes suggested by liugoanywhere in the comments for the following two issues:
https://github.com/B0RJA/GAlib-mpi/issues/1
https://github.com/B0RJA/GAlib-mpi/issues/2

Review the original github page (https://github.com/B0RJA/GAlib-mpi) or GALib documentation (http://lancet.mit.edu/ga/) for information on how to use this library.

Installation and usage
----------------------
* Compile with `make` (requires mpic++)
* Run the example: `mpirun -np 1 example seed 1234`

The output must be:

    GA result:
    x = 7.861659, y = 7.861659

Dive in the library usage reading the example and visit http://lancet.mit.edu/ga/ for detailed documentation.

Any problems?
-------------
Feel free to [write an issue](https://github.com/B0RJA/GAlib-mpi/issues) if you have any questions or problems.


Copyright and license
---------------------
This library is based on [GAlib](http://lancet.mit.edu/ga/), available under a BSD-style license found in the COPYRIGHT file.
