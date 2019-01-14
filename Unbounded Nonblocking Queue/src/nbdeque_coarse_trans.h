// Maxim Shelopugin
// GIS deque reimplementation using coarse transactions
// Based on the code from "An Unbounded Nonblocking Double-ended Queue"
// by Matthew Graichen, Joseph Izraelevitz, Michael L. Scott
// 2016 45th International Conference on Parallel Processing
// COP 6616 Fall 2018

#include <iostream>
#include <atomic>
#include <thread>
#include <time.h>

using namespace std;

// define important flags
#define LN INT32_MAX	
#define RN INT32_MIN
#define LS INT32_MAX-1
#define RS INT32_MIN+1
#define SZ 100000

// a slot tuple - 64 bit for idex-value pair
typedef uint64_t slot;

// Node class, stores an array of slots
class Node{
public:
	int left_slot_hint;
	int right_slot_hint;
	slot buffer[SZ];

// Create an array of slots. Populate 0 -> split with LN and split -> SZ with RN
	Node(int split){
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
	void printnode(){
		if((int)(buffer[0]>>16) == LN || (int)(buffer[0]>>16) == RN)
			print(buffer[0]);
		else
			cout << " vv \n";
		for(int i = 1; i<SZ-1; i++){
			print(buffer[i]);
		}
		if((int)(buffer[SZ-1]>>16)==LN || (int)(buffer[SZ-1]>>16) == RN)
			print(buffer[SZ-1]);
		else{
			cout << " vv \n";
			((Node*)(buffer[SZ-1]>>16))->printnode();
		}
	}

// return the reference to a specific slot
	slot* get_slot(int i){
		return &buffer[i];
	}

// translate index-data pair into a single slot 
	slot translate(int data, short index){
		slot temp = 0;
		temp=(((temp)|data)<<16)|index;
		return temp;
	}

// print index and data fields separately
	void print(slot num){
		cout << (short)num << " " << (int)(num>>16) << "\n";
	}
};

// stores a pointer to a node and a multipurpose counter
struct node_hint{
	Node *buffer;
	int ct;			
};


// Double ended queue class with left and right node hints defined explicitly
class Deque{
	node_hint left_node_hint, right_node_hint;

public:
// Constructor - create a single node, and separate it 50/50 with LN and RN
	Deque(){
		Node* node = new Node(SZ/2);
		left_node_hint.buffer = node;
		left_node_hint.ct = 0;
		right_node_hint = left_node_hint;
	}


// return the left edge of the Node pointed by the hint
// traverse from the hint to the actual edge and return the edge index
// if the hint points to the beginning of the node, return the 1st possible value slot index
	node_hint l_oracle(node_hint hint){
		node_hint new_hint;
		new_hint.buffer = hint.buffer;
		int border = new_hint.buffer->left_slot_hint;
		if(border <= 1){
			new_hint.ct = 1;
			return new_hint;
		}
		for(int i = border; i>=0; i--){
			if((int)((*new_hint.buffer->get_slot(i))>>16) == LN){
				new_hint.ct = i+1;
				break;
			}
		}
		return new_hint;
	}

// return the right edge of the Node pointed by the hint
// traverse from the hint to the actual edge and return the edge index
// if the hint points to the beginning of the node, return the 1st possible value slot index
	node_hint r_oracle(node_hint hint){
		node_hint new_hint;
		new_hint.buffer = hint.buffer;
		int border = new_hint.buffer->right_slot_hint;
		if(border >= SZ-2){
			new_hint.ct = SZ-2;
			return new_hint;
		}
		for(int i = border; i<SZ; i++){
			if((int)((*new_hint.buffer->get_slot(i))>>16) == RN){
				new_hint.ct = i-1;
				break;
			}
		}
		return new_hint;
	}

// update left hint node with new node and new index. return the old hint
	node_hint hint_l (node_hint old, Node* nw_nd, int nw_idx){
		if (nw_idx == SZ-1){
			left_node_hint.buffer = ((Node*)(*(nw_nd->get_slot(SZ-1))>>16));
		}
		else{
			left_node_hint.buffer = nw_nd;
			nw_nd->left_slot_hint = nw_idx;
		}
		return old;
	}

// update right hint node with new node and new index. return the old hint
	node_hint hint_r (node_hint old, Node* nw_nd, int nw_idx){
		if (nw_idx == 0){
			right_node_hint.buffer = ((Node*)(*(nw_nd->get_slot(0))>>16));
		}
		else{
			right_node_hint.buffer = nw_nd;
			nw_nd->right_slot_hint = nw_idx;
		}
		return old;
	}

// free the memory of a node
	void retire(Node* nd){
		free(nd);
	}

// Push left.
// Get the edge index, determine the type of the edge, make the transition
	int push_left(int o){
// Start the atomic push_left
		__transaction_atomic{

			node_hint hint_cpy = left_node_hint;
			node_hint temp_edge = l_oracle(hint_cpy);

			Node *edge_nd = temp_edge.buffer;
			int edge_idx = temp_edge.ct;

			slot* in = edge_nd->get_slot(edge_idx);
			slot* out = edge_nd->get_slot(edge_idx-1);

			slot in_cpy = *in;
			slot out_cpy = *out;

// Interior push
			if(edge_idx!=1){
				slot new_in_cpy = in_cpy;
				slot new_out_cpy = out_cpy;
				slot temp = 0;

				new_in_cpy +=1;
				new_out_cpy+=1;
				temp = (temp|o)<<16;
				new_out_cpy = (new_out_cpy&0x000000FF)|temp;

				if(*in == in_cpy && *out==out_cpy){
					*in = new_in_cpy;
					*out = new_out_cpy;
					hint_l(hint_cpy, edge_nd, edge_idx-1);
					return 1; 
				}
			}
// Stragling or boundary left push
			else{
				if((int)(out_cpy>>16) == LN){
					Node* nw_nd = new Node(SZ);

					slot far = 0;
					slot back = 0;

					far = (far|o)<<16;
					back = (back|(slot)edge_nd)<<16;

					*(nw_nd->get_slot(SZ-2))= far;
					*(nw_nd->get_slot(SZ-1)) = back;

					slot new_in_cpy = in_cpy;
					slot new_out_cpy = out_cpy;
					slot temp = 0;

					new_in_cpy +=1;
					new_out_cpy+=1;
					temp = (temp|(slot)nw_nd)<<16;
					new_out_cpy = (new_out_cpy&0x000000FF)|temp;

					if(*in == in_cpy && *out == out_cpy){
						*in = new_in_cpy;
						*out = new_out_cpy;
						hint_l(hint_cpy, nw_nd, SZ-2);
						return 1;
					}
				}
// edge is straddling, so either straddle push or help remove sealed node on left
				else{
					Node* out_nd = (Node*)(out_cpy>>16);
					slot*far;

					far = out_nd->get_slot(SZ-2);
					slot far_cpy = *far; 

					// ensure left neighbor points back
					slot back = *(((Node*)(out_cpy>>16))->get_slot(SZ-1));
					slot back_cpy = back;

// check state for straddling push
					if((int)(far_cpy>>16) == LN){
						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy +=1;
						new_far_cpy +=1;
						temp = (temp|o)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;
	
						if(*in == in_cpy && *far == far_cpy){
							*in = new_in_cpy;
							*far = new_far_cpy;
							hint_l(hint_cpy, out_nd, SZ-2);
							return 1; 
						}
					}
// remove sealed node on left
					else if((int)(far_cpy>>16) == LS){

						slot new_in_cpy = in_cpy;
						slot new_out_cpy = out_cpy;
						slot temp = 0;

						new_in_cpy +=1;				
						new_out_cpy +=1;
						temp = (temp|LN)<<16;
						new_in_cpy = (new_out_cpy&0x000000FF)|temp;

						if(*in == in_cpy && *out == out_cpy){
							*in = new_in_cpy;
							*out = new_out_cpy;
							hint_l(hint_cpy, edge_nd, 1);
							retire(out_nd);	
						}
					}
				}
			}
		}
		return 1;
	}

// Push right.
// Same logic as push left, but the indecies are mirrored
	int push_right(int o){
// Start the atomic push_right
		__transaction_atomic{
			node_hint hint_cpy = right_node_hint;
			node_hint temp_edge = r_oracle(hint_cpy);

			Node *edge_nd = temp_edge.buffer;
			int edge_idx = temp_edge.ct;

			slot* in  = edge_nd->get_slot(edge_idx);
			slot* out = edge_nd->get_slot(edge_idx+1);

			slot in_cpy;
			slot out_cpy;
			in_cpy = *in;
			out_cpy = *out;
			
// interior push
			if(edge_idx!=(SZ-2)){

				slot new_in_cpy = in_cpy;
				slot new_out_cpy = 0;
				new_in_cpy +=1;

				short index = (0|out_cpy)+1;
				new_out_cpy = (new_out_cpy|o)<<16;
				new_out_cpy = new_out_cpy|index;

				if(*in == in_cpy && *out == out_cpy){
					*in = new_in_cpy;
					*out = new_out_cpy;
					hint_r(hint_cpy, edge_nd, edge_idx+1);
					return 1; 
				}
			}
			else{
// check state for boundary edge (append)
				if((int)(out_cpy>>16) == RN){
					Node* nw_nd = new Node(0);

					slot far = 0;
					slot back = 0;

					far = (far|o)<<16;
					back = (back|(slot)edge_nd)<<16;

					*(nw_nd->get_slot(1)) = far;
					*(nw_nd->get_slot(0)) = back;

					slot new_in_cpy = in_cpy;
					slot new_out_cpy = out_cpy;
					slot temp = 0;

					new_in_cpy +=1;
					new_out_cpy+=1;
					temp = (temp|(slot)nw_nd)<<16;
					new_out_cpy = (new_out_cpy&0x000000FF)|temp;

					if(*in == in_cpy && *out == out_cpy){
						*in = new_in_cpy;
						*out = new_out_cpy;
						hint_r(hint_cpy, nw_nd, 1);
						return 1;
					}
				}
// edge is straddling, so either straddle push or
// help remove sealed node on right
				else{
					Node* out_nd = (Node*)(out_cpy>>16);
					slot*far;
					far = out_nd->get_slot(1);

					slot far_cpy = *far; 

					slot back = *(((Node*)(out_cpy>>16))->get_slot(0));
					slot back_cpy = back;

// check state for straddling push
					if((int)(far_cpy>>16) == RN){
						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy +=1;
						new_far_cpy +=1;
						temp = (temp|o)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;
	
						if(*in == in_cpy && *far == far_cpy){
							*in = new_in_cpy;
							*far = new_far_cpy;
							hint_r(hint_cpy, out_nd, 1);
							return 1; 
						}
					}
// remove sealed node on left
					else if((int)(far_cpy>>16) == RS){

						slot new_in_cpy = in_cpy;
						slot new_out_cpy = out_cpy;
						slot temp = 0;

						new_in_cpy +=1;				
						new_out_cpy +=1;
						temp = (temp|RN)<<16;
						new_in_cpy = (new_out_cpy&0x000000FF)|temp;

						if(*in == in_cpy && *out == out_cpy){
							*in = new_in_cpy;
							*out = new_out_cpy;
							hint_r(hint_cpy, edge_nd, SZ-2);
							retire(out_nd);	
						}
					}
				}
			}
		}
		return 1;
	}

// Pop left
// Get the edge index,determine the type of the edge, perform operations
// Returns 0 if the queue is empty
	int pop_left(){
// Start the atomic pop_left
		__transaction_atomic{
			node_hint hint_cpy = left_node_hint;
			node_hint temp_edge = l_oracle(hint_cpy);

			int edge_idx = temp_edge.ct;
			Node* edge_nd = temp_edge.buffer;

			slot* in = edge_nd->get_slot(edge_idx);
			slot* out = edge_nd->get_slot(edge_idx-1);

			slot in_cpy = *in;
			slot out_cpy = *out;
					
// interior edge
// so interior pop or empty check
			if(edge_idx!=1){
				slot new_out_cpy = out_cpy;
				slot new_in_cpy = in_cpy;

				new_out_cpy+=1;
				new_in_cpy+=1;

				slot temp = 0;
				temp = (temp|LN)<<16;
				new_out_cpy = (new_out_cpy&0x000000FF)|temp;
				new_in_cpy = (new_in_cpy&0x000000FF)|temp;

				if((int)(in_cpy>>16) == RN && *in == in_cpy){
					return 0;
				}
				if(*out == out_cpy && *in==in_cpy){
					*out = new_out_cpy;
					*in = new_in_cpy;
					hint_l(hint_cpy, edge_nd, edge_idx+1);
					return (int)(in_cpy>>16);
				}
			}

// edge is on border of array, so follow straddling
// pop progression as necessary: seal left node,
// remove left node, then boundary pop
			else{
				if((int)(out_cpy>>16) != LN){
					Node* out_nd = (Node*)(out_cpy>>16);

					slot*far;
					far = out_nd->get_slot(SZ-2);

					slot far_cpy = *far;
	
// ensure left neighbor points back
					slot back = *((Node*)(out_cpy>>16))->get_slot(SZ-1);
					slot back_cpy = back;
				
// check for straddled edge and seal
					if((int)(far_cpy>>16) == LN){
						if(((int)(in_cpy>>16) == RN || (int)(in_cpy>>16) == RS) && *in == in_cpy){
							return 0; // empty
						}

						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy+=1;
						new_far_cpy+=1;
						temp = (temp|LS)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;

						if(*far == far_cpy && *in==in_cpy){
							*far = new_far_cpy;
							*in = new_in_cpy;
							far_cpy = new_far_cpy;
							in_cpy = new_in_cpy;
						}
					}
// check for sealed left node and remove it 
					if((int)(far_cpy>>16) == LS){
						if((int)(in_cpy>>16) == RN && *in == in_cpy){
							return 0;
						}

						slot new_in_cpy = in_cpy;
						slot new_out_cpy = out_cpy;
						slot temp = 0;

						new_in_cpy+=1;
						new_out_cpy+=1;
						temp = (temp|LN)<<16;
						new_out_cpy = (new_out_cpy&0x000000FF)|temp;

						if(*out == out_cpy && *in==in_cpy){
							*out = new_out_cpy;
							*in = new_in_cpy;
							hint_cpy = hint_l(hint_cpy, edge_nd, 1);
							retire(out_nd);
							in_cpy = new_in_cpy;
							out_cpy = new_out_cpy;
						}
					}
				}
// check for boundary edge, then boundary pop
				if((int)(out_cpy>>16) == LN){
					if((int)(in_cpy>>16) == RN && *in==in_cpy){
						return 0;
					}

					slot new_out_cpy = out_cpy;
					slot new_in_cpy = in_cpy;
					slot temp = 0;

					new_out_cpy+=1;
					new_in_cpy+=1;

					temp = (temp|LN)<<16;
					new_out_cpy = (new_out_cpy&0x000000FF)|temp;
					new_in_cpy = (new_in_cpy&0x000000FF)|temp;

					if(*out == out_cpy && *in==in_cpy){
						*out = new_out_cpy;
						*in = new_in_cpy;
						hint_l(hint_cpy, edge_nd, 2);
						return (int)(in_cpy>>16);
					}
				}
			}
		} 
	}

// Pop right
// Get the edge index,determine the type of the edge, perform operations
// Returns 0 if the queue is empty
	int pop_right(){
// Start the atomic pop_right
		__transaction_atomic{
			node_hint hint_cpy = right_node_hint;
			node_hint temp_edge = r_oracle(hint_cpy);

			int edge_idx = temp_edge.ct;
			Node* edge_nd = temp_edge.buffer;

			slot* in = edge_nd->get_slot(edge_idx);
			slot* out = edge_nd->get_slot(edge_idx+1);

			slot in_cpy = *in;
			slot out_cpy = *out;
					
// interior edge
// so interior pop or empty check
			if(edge_idx!=SZ-2){
				slot new_out_cpy = out_cpy;
				slot new_in_cpy = in_cpy;

				new_out_cpy+=1;
				new_in_cpy+=1;

				slot temp = 0;
				temp = (temp|RN)<<16;
				new_out_cpy = (new_out_cpy&0x000000FF)|temp;
				new_in_cpy = (new_in_cpy&0x000000FF)|temp;

				if((int)(in_cpy>>16) == LN && *in == in_cpy){
					return 0;
				}
				if(*out == out_cpy && *in == in_cpy){
					*out = new_out_cpy;
					*in = new_in_cpy;
					hint_r(hint_cpy, edge_nd, edge_idx-1);
					return (int)(in_cpy>>16);
				}
			}

// edge is on border of array, so follow straddling
// pop progression as necessary: seal left node,
// remove left node, then boundary pop
			else{
				if((int)(out_cpy>>16) != RN){
					Node* out_nd = (Node*)(out_cpy>>16);

					slot *far;
					far = out_nd->get_slot(1);

					slot far_cpy = *far;
	
// ensure left neighbor points back
					slot back = *((Node*)(out_cpy>>16))->get_slot(0);
					slot back_cpy = back;
				
// check for straddled edge and seal
					if((int)(far_cpy>>16) == RN){
						if(((int)(in_cpy>>16) == LN || (int)(in_cpy>>16) == LS) && *in == in_cpy){
							return 0; // empty
						}

						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy+=1;
						new_far_cpy+=1;
						temp = (temp|RS)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;


						if(*in == in_cpy && *far == far_cpy){
							*in = new_in_cpy;
							*far = new_far_cpy;
							far_cpy = new_far_cpy;
							in_cpy = new_in_cpy;
						}
					}
// check for sealed left node and remove it 
					if((int)(far_cpy>>16) == RS){
						if((int)(in_cpy>>16) == LN && *in == in_cpy){
							return 0;
						}

						slot new_in_cpy = in_cpy;
						slot new_out_cpy = out_cpy;
						slot temp = 0;

						new_in_cpy+=1;
						new_out_cpy+=1;
						temp = (temp|RN)<<16;
						new_out_cpy = (new_out_cpy&0x000000FF)|temp;

						if(*in == in_cpy && *out == out_cpy){
							*in = new_in_cpy;
							*out = new_out_cpy;
							hint_cpy = hint_r(hint_cpy, edge_nd, SZ-2);
							retire(out_nd);
							in_cpy = new_in_cpy;
							out_cpy = new_out_cpy;
						}
					}
				}
// check for boundary edge, then boundary pop
				if((int)(out_cpy>>16) == RN){
					if((int)(in_cpy>>16) == LN && *in==in_cpy){
						return 0;
					}

					slot new_out_cpy = out_cpy;
					slot new_in_cpy = in_cpy;
					slot temp = 0;

					new_out_cpy+=1;
					new_in_cpy+=1;

					temp = (temp|RN)<<16;
					new_out_cpy = (new_out_cpy&0x000000FF)|temp;
					new_in_cpy = (new_in_cpy&0x000000FF)|temp;


					if(*out == out_cpy && *in == in_cpy){
						*out = new_out_cpy;
						*in = new_in_cpy;
						hint_r(hint_cpy, edge_nd, SZ-3);
						return (int)in_cpy>>16;
					}
				}
			}
		} 
	}

// Print the queue from left to right
	void print_queue(){
		cout << "\nThe contents of the queue are:\n";
		left_node_hint.buffer->printnode();
	}
};