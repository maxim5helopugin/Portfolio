// Maxim Shelopugin
// COP 6616
// October 22 2018
// Lock-free Stack with descriptor

#include <atomic>
#include <iostream>
#include <thread>
#include <time.h> 
#include "Node.h"
#include "WriteDescriptor.h"
#include "Descriptor.h"
#include "Stack.h"

using namespace std;
// test function, pushes size amount of nodes on the stack
template<class T>
void doPushes(Stack<T>* stck, Node<T>* nodes[], int size){
	for(int i = 0; i<size; i++){;
		stck->push(nodes[i%100]);
	}
}

// test function, pops size amount of nodes from the stack
template<class T>
void doPops(Stack<T>* stck, int size){
	for(int i = size; i>0; i--){
		cout << stck->pop() << endl;
	}
}

// test function, used to change the % of operations
template<class T>
void operation(Stack<T>* stack, Node<T>* nodes[], int operations){
		doPushes(stack, nodes, operations/2);
		doPops(stack, operations/2);
}

// test main class
int main(int argc, char* argv[]){

	Stack<int> *stck = new Stack<int>();
	int numthreads = atoi(argv[1]);
	int operations = atoi(argv[2]);

// pre - allocate nodes and threads
	thread threads[numthreads];
	Node<int>* nodes[numthreads][100];
	for(int i = 0; i<numthreads; i++){
		for(int j=0; j<100; j++){
			nodes[i][j]= new Node<int>(j);
		}
	}
// start clock when threads are ready to spawn
	clock_t start = clock();
	for(int i = 0; i<numthreads; i++){
		threads[i] = thread(operation<int>, stck, nodes[i], operations/numthreads);
	}
// wait untill finished
	for(int i = 0; i<numthreads; i++){
		threads[i].join();
	}	
// stop the clock
	clock_t stop = clock();
	cout << "Threads : "<< numthreads << '\n';
	cout << "Execution took " << (stop-start)/1000.0 << '\n';
	cout << "Operations done : "<< stck->getNumOps() << '\n';
}