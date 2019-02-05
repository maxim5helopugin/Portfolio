# Concurrent Stack

## C++
- Concurrent stack with descriptor object
- Compile with "g++ test_class.cpp -o test"
- Run with "test #of threads #of operations"
- The stats will be output after the program finishes

## Java
- Contains two stack implementations: Non-blocking and Elimination-backoff
- Non-blocking stack is a simplest concurrent stack implementation - does not use locks, only CAS principles
- Still does not provide a significant performance boost in multithreaded environment
- Elimination-Backoff stack applies the elimination array to counter the sequential bottleneck
- Results in a high-performant data structure
- Compile with "javac Test.java"
- Run with "java Test #of threads #of operations"
- The stats will be output after the program finishes
