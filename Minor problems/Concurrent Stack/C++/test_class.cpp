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
void doPushes(Stack* stck, Node* nodes[], int size){
	for(int i = 0; i<size; i++){;
		stck->push(nodes[i%100]);
	}
}

// test function, pops size amount of nodes from the stack
void doPops(Stack* stck, int size){
	for(int i = size; i>0; i--){
		stck->pop();
	}
}

// test function, used to change the % of operations
void operation(Stack* stack, Node* nodes[]){
		doPushes(stack, nodes, 4000000);
		doPops(stack, 500000);
		for(int i = 0; i<500000; i++){
			stack->size();
		}
}

// test main class
int main(){
	Stack *stck = new Stack();
	int numthreads = 2;
// pre - allocate nodes and threads
	thread threads[numthreads];
	Node* nodes[numthreads][100];
	for(int i = 0; i<numthreads; i++){
		for(int j=0; j<100; j++){
			nodes[i][j]= new Node(j);
		}
	}
// start clock when threads are ready to spawn
	clock_t start = clock();
	for(int i = 0; i<numthreads; i++){
		threads[i] = thread(operation, stck, nodes[i]);
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