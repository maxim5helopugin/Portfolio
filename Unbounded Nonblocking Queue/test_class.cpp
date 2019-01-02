// Maxim Shelopugin
// Test Class
// Takes in a specific type of the queue
// And 3 arguments:
// #of threads %of pushes %of pops
// Runs for a million operations with the specified percentages
// Outputs composition of operations, as well as execution time

#include <iostream>
#include "nbdeque.h"		// Change the type of the queue here
#include <cstdlib>

using namespace std;

// Perform a million operations with specified amount of pushes and pops
// Choose a side with 50-50 destribution
void operations(Deque*deque, int pushes, int pops){
	int op = 0, type = 0;
	int pl=0, pr=0, pol=0, por=0;
	for(int i = 0; i<(pushes+pops)*1250; i++){
		op = rand()%(pushes+pops);
		if(op<pushes){
			type = rand()%2;
			if(type == 0){
				deque->push_left(10);
				pl++;
			}
			else{
				deque->push_right(-10);
				pr++;
			}
		}
		else{
			type = rand()%2;
			if(type == 0){
				deque->pop_left();
				pol++;
			}
			else{
				deque->pop_right();
				por++;
			}
		}
	}
	cout << "Composition : " << pl << ' ' << pr << ' '<< pol << ' '<< por << '\n'; 
}
// main function,
// is used to test and debug
int main(int argc,char* argv[]){
	Deque* deque = new Deque();

	int num_threads = atoi(argv[1]);
	int pushes = atoi(argv[2]);
	int pops = atoi(argv[3]);

	thread threads[num_threads];
	int split = 8/num_threads;

	clock_t start = clock();
	
	for(int i = 0; i<num_threads; i++){
		threads[i] = thread(operations,deque,pushes*split,pops*split);
	}

	for(int i=0; i<num_threads; i++)
		threads[i].join();
	cout << "took " << (clock()-start)/1000.0;

	return 0;
}