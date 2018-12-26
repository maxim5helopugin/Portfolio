// Maxim Shelopugin
// COP 6616
// October 22 2018
// Lock-free Stack with descriptor

#include <atomic>
#include <iostream>
#include <thread>
#include <time.h> 

using namespace std;
class Node;

// Write descriptor/ stroes pointers 
class WriteDescriptor{
public:
	//expected and desired nodes
	Node* expected;
	Node* desired;
	//are any operations pending?
	std::atomic<bool> pending;

	WriteDescriptor(Node* nd1, Node* nd2, bool progress){
		expected = nd1;
		desired = nd2;
		pending.store(progress);
	}
};

// Descriptor, stores the pointer to writedescr and size
class Descriptor{
public:
	// size stored in the descriptor
	// Store head in the descriptor as well
	std::atomic<int> size;
	std::atomic<Node*> head;
	WriteDescriptor*wd;

	Descriptor(int sz, WriteDescriptor* writeop, Node* temp){
		size.store(sz);
		wd = writeop;
		head.store(temp);
	}
};

// Stack node
class Node{
public:
	int val;
	Node* next;

	Node(int _val){
		val = _val;
		next = NULL;
	}
};

class Stack{
// Get class specific descriptor and writedescr
	std::atomic<int> numOps;
	std::atomic<Descriptor*> d;
	WriteDescriptor*wd;

public:
	Stack(){
		numOps = 0;
		// initialize size = 0, pending operations = false, head = NULL
		wd = new WriteDescriptor(NULL, NULL, false);
		d = new Descriptor(0, wd, NULL);
	}

	bool push(Node* new_head){

		while(true){
			// Store current descriptor
			Descriptor *temp_desc = d.load();
			// Finish old operations first
			complete(temp_desc);
			new_head->next = temp_desc->head.load();

			// Create a pending write operation, increase the size
			WriteDescriptor *writeop = new WriteDescriptor(temp_desc->head.load(), new_head, true);
			Descriptor *nextDesc = new Descriptor(temp_desc->size+1, writeop, new_head);

			// keep retrying untill descriptors match
			if(atomic_compare_exchange_weak(&d, &temp_desc, nextDesc)){
				numOps++;
				return true;
			}
			// complete old operations
			complete(nextDesc);
		}
	}

	int pop(){
		Node* current_head = NULL;
		Node* new_head = NULL;
		while(true){
			// Store current descriptor
			Descriptor*temp_desc = d.load();
			// Finish old operations first
			complete(temp_desc);

			current_head = temp_desc->head.load();
			if(current_head == NULL)
				return -1;
			new_head = current_head->next;

			// Create a pending write operation, decrease the size
			WriteDescriptor*writeop = new WriteDescriptor(current_head, new_head, true);
			Descriptor*nextDesc = new Descriptor(temp_desc->size-1, writeop, new_head);
			if(atomic_compare_exchange_weak(&d, &temp_desc, nextDesc)){
				numOps++;
				return current_head->val;
			}
		}
	}

	// Complete old operations, write false to pending
	void complete(Descriptor *desc){
		if(desc->wd->pending.load()){
			atomic_compare_exchange_weak(&desc->head, &desc->wd->expected, desc->wd->desired);
			desc->wd->pending.store(false);
		}
	}

	// return the size of the stack
	int size(){
		numOps++;
		return d.load()->size;
	}

	int getNumOps(){
		return numOps.load();
	}
};

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
		doPushes(stack, nodes, 125000);
		doPops(stack, 125000);
		for(int i = 0; i<62500; i++){
			stack->size();
		}
}

// test main class
int main(){
	Stack stck;
	int numthreads = 32;
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
		threads[i] = thread(operation, &stck, nodes[i]);
	}
// wait untill finished
	for(int i = 0; i<numthreads; i++){
		threads[i].join();
	}	
// stop the clock
	clock_t stop = clock();
	cout << "Threads : "<< numthreads << '\n';
	cout << "Execution took " << (stop-start)/1000.0 << '\n';
	cout << "Operations done : "<< stck.getNumOps() << '\n';
}
