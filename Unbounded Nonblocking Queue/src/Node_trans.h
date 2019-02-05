#include <iostream>

// define important flags
#define LN INT32_MAX	
#define RN INT32_MIN
#define LS INT32_MAX-1
#define RS INT32_MIN+1
#define SZ 100000

typedef uint64_t slot;
// Node class, stores an array of slots
class Node{
public:
	int left_slot_hint;
	int right_slot_hint;
	slot buffer[SZ];
	Node(int split);
// return the reference to a specific slot
	slot* get_slot(int i){return &buffer[i];}	
	void printnode();
	slot translate(int data, short index);
	void print(slot num){ std::cout << (short)num << " " << (int)(num>>16) << "\n";}	
};
// Create an array of slots. Populate 0 -> split with LN and split -> SZ with RN
	Node::Node(int split){
		for(int i = 0; i< split; i++){
			buffer[i] = translate(LN, 0);
		}
		for( int i =split; i<SZ; i++){
			buffer[i] = translate(RN, 0);
		}
		left_slot_hint = split-1;
		right_slot_hint = split;
	}

// Print nodes from leftmost to rightmost
	void Node::printnode(){
		if((int)(buffer[0]>>16) == LN || (int)(buffer[0]>>16) == RN)
			print(buffer[0]);
		else
			std::cout << " vv \n";
		for(int i = 1; i<SZ-1; i++){
			print(buffer[i]);
		}
		if((int)(buffer[SZ-1]>>16)==LN || (int)(buffer[SZ-1]>>16) == RN)
			print(buffer[SZ-1]);
		else{
			std::cout << " vv \n";
			((Node*)(buffer[SZ-1]>>16))->printnode();
		}
	}



// translate index-data pair into a single slot 
	slot Node::translate(int data, short index){
		slot temp = 0;
		temp=(((temp)|data)<<16)|index;
		return temp;
	}

// print index and data fields separately
