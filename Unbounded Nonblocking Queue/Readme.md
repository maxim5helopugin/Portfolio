# Unbounded Nonblocking Queue

- test_class.cpp - test class which contains the spawn of threads and allocation of operations to the specific data structure
- nbdeque.h - nonblocking design of the deque

- nbdeqe_coarse_trans.h
- nbdeqe_fine_trans.h
- nbdeqe_select_trans.h 
- different compositions of transactional sizes implementation of the deque.

To compile and run the source code, navigate to the /src folder and 
run "g++ test_class.cpp -o3 -o test_class". (if you are not using c++11
-pthread might be needed to be added as an argument / alternatively force 
the compilation by c++11)
This will compile the test_class with the specific deque implementation
specified in the header. To compile it with another version, specify the 
file inside the "#include<*>" and recompile again.

To run the compiled program, type "test_class a1 a2 a3", where 
a1 = number of threads
a2 = percentage of pushes
a3 = percentage of pops

For example, to run a 4 threaded test with 50% of pushes and 50% of pops,
run "test_class 4 50 50".

The test class will produce a a1 threaded run of mixture of a2 pushes and a3 pops.
The side of each push/pop is chosen by random.
1.000.000.000 operations are performed, with composition and time taken printed to the screen.

To run transactional deque, add -fgnu-tm to the command during compilation, 
and change the file in the header of the test_class.
"g++ test_class.cpp -o3 -fgnu-tm -o test_class"
