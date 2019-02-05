#include <iostream>
#include <atomic>
#include <thread>
#include <time.h>
#include <unistd.h>
#include "Node.h"

using namespace std;

// a slot tuple - 64 bit for idex-value pair
typedef uint64_t slot;

// stores a pointer to a node and a multipurpose counter
struct node_hint{
	Node *buffer;
	int ct;			
};


// Double ended queue class with left and right node hints defined explicitly
class Deque{
public:
	Deque();
	int push_left(int o);
	int push_right(int o);
	int pop_left();
	int pop_right();
	void print_queue();
private:
	node_hint left_node_hint, right_node_hint;
	node_hint l_oracle(node_hint hint);
	node_hint r_oracle(node_hint hint);
	node_hint hint_l (node_hint old, Node* nw_nd, int nw_idx);
	node_hint hint_r (node_hint old, Node* nw_nd, int nw_idx);
	void retire(Node* nd);
};
// Constructor - create a single node, and separate it 50/50 with LN and RN
	Deque::Deque(){
		Node* node = new Node(SZ/2);
		left_node_hint.buffer = node;
		left_node_hint.ct = 0;
		right_node_hint = left_node_hint;
	}

// return the left edge of the Node pointed by the hint
// traverse from the hint to the actual edge and return the edge index
// if the hint points to the beginning of the node, return the 1st possible value slot index
	node_hint Deque::l_oracle(node_hint hint){
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
	node_hint Deque::r_oracle(node_hint hint){
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
	node_hint Deque::hint_l (node_hint old, Node* nw_nd, int nw_idx){
		if (nw_idx == SZ-1){
			left_node_hint.buffer = ((Node*)(nw_nd->get_slot(SZ-1)->load()>>16));
		}
		else{
			left_node_hint.buffer = nw_nd;
			nw_nd->left_slot_hint = nw_idx;
		}
		return old;
	}

// update right hint node with new node and new index. return the old hint
	node_hint Deque::hint_r (node_hint old, Node* nw_nd, int nw_idx){
		if (nw_idx == 0){
			right_node_hint.buffer = ((Node*)(nw_nd->get_slot(0)->load()>>16));
		}
		else{
			right_node_hint.buffer = nw_nd;
			nw_nd->right_slot_hint = nw_idx;
		}
		return old;
	}

// free the memory of a node
	void Deque::retire(Node* nd){
		free(nd);
	}

// Push left.
// Get the edge index, determine the type of the edge, make the transition
	int Deque::push_left(int o){
		int busy = 0;

		while(true){
			busy++;			// Exponential baackoff to guarantee progress
			usleep(2*busy);

			node_hint hint_cpy = left_node_hint;
			node_hint temp_edge = l_oracle(hint_cpy);

			Node *edge_nd = temp_edge.buffer;
			int edge_idx = temp_edge.ct;

			std::atomic<slot>* in;
			std::atomic<slot>* out;
			
			in  = edge_nd->get_slot(edge_idx);
			out = edge_nd->get_slot(edge_idx-1);

			slot in_cpy = in->load();
			slot out_cpy = out->load();

// check if oracle's edge did not fail
			if(((int)(in_cpy>>16) == LN || (int)(in_cpy>>16) == RS)
				|| (edge_idx != 1 && (int)(out_cpy>>16) != LN)
				|| (edge_idx == SZ - 1 && (int)(in_cpy>>16) != RN))
				continue;

// Interior push
			if(edge_idx!=1){
				slot new_in_cpy = in_cpy;
				slot new_out_cpy = out_cpy;
				slot temp = 0;

				new_in_cpy +=1;
				new_out_cpy+=1;
				temp = (temp|o)<<16;
				new_out_cpy = (new_out_cpy&0x000000FF)|temp;

				if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
					hint_l(hint_cpy, edge_nd, edge_idx-1);
					//cout << "\nThe contents of the queue are :\n";
					//left_node_hint.buffer->printnode();
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

					nw_nd->get_slot(SZ-2)->store(far);
					nw_nd->get_slot(SZ-1)->store(back);

					slot new_in_cpy = in_cpy;
					slot new_out_cpy = out_cpy;
					slot temp = 0;

					new_in_cpy +=1;
					new_out_cpy+=1;
					temp = (temp|(slot)nw_nd)<<16;
					new_out_cpy = (new_out_cpy&0x000000FF)|temp;

					if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
						hint_l(hint_cpy, nw_nd, SZ-2);
						//cout << "\nThe contents of the queue are :\n";
						//left_node_hint.buffer->printnode();
						return 1;
					}
				}
// edge is straddling, so either straddle push or help remove sealed node on left
				else{
					Node* out_nd = (Node*)(out_cpy>>16);
					atomic<slot> *far;

					far = out_nd->get_slot(SZ-2);
					slot far_cpy = far->load(); 

					// ensure left neighbor points back
					slot back = ((Node*)(out_cpy>>16))->get_slot(SZ-1)->load();
					slot back_cpy = back;

					if((Node*)(back>>16) != edge_nd){
						continue;
					}
// check state for straddling push
					if((int)(far_cpy>>16) == LN){
						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy +=1;
						new_far_cpy +=1;
						temp = (temp|o)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;
	
						if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(far, &far_cpy, new_far_cpy)){
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

						if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
							// for memory reclamation update both hints
							hint_l(hint_cpy, edge_nd, 1);
							//hint_r(oracle_r(right_slot_hint));
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
	int Deque::push_right(int o){
		int busy = 0;
		while(true){
			busy++;
			usleep(2*busy);			// Exponential baackoff to guarantee progress

			node_hint hint_cpy = right_node_hint;
			node_hint temp_edge = r_oracle(hint_cpy);

			Node *edge_nd = temp_edge.buffer;
			int edge_idx = temp_edge.ct;

			std::atomic<slot>* in;
			std::atomic<slot>* out;
			
			in  = edge_nd->get_slot(edge_idx);
			out = edge_nd->get_slot(edge_idx+1);

			slot in_cpy;
			slot out_cpy;
			in_cpy = in->load();
			out_cpy = out->load();
			
// check if oracle's edge did not fail
			if(((int)(in_cpy>>16) == RN || (int)(in_cpy>>16) == LS)
				|| (edge_idx != SZ-2 && (int)(out_cpy>>16) != RN)
				|| (edge_idx == 0 && (int)(in_cpy>>16) != LN))
				continue;
			
// interior push
			if(edge_idx!=(SZ-2)){

				slot new_in_cpy = in_cpy;
				slot new_out_cpy = 0;
				new_in_cpy +=1;

				short index = (0|out_cpy)+1;
				new_out_cpy = (new_out_cpy|o)<<16;
				new_out_cpy = new_out_cpy|index;

				if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
					//cout << "\nThe contents of the queue are :\n";
					//right_node_hint.buffer->printnode();
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

					nw_nd->get_slot(1)->store(far);
					nw_nd->get_slot(0)->store(back);

					slot new_in_cpy = in_cpy;
					slot new_out_cpy = out_cpy;
					slot temp = 0;

					new_in_cpy +=1;
					new_out_cpy+=1;
					temp = (temp|(slot)nw_nd)<<16;
					new_out_cpy = (new_out_cpy&0x000000FF)|temp;

					if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
						hint_r(hint_cpy, nw_nd, 1);
						//cout << "\nThe contents of the queue are :\n";
						//left_node_hint.buffer->printnode();
						return 1;
					}
				}
// edge is straddling, so either straddle push or
// help remove sealed node on right
				else{
					Node* out_nd = (Node*)(out_cpy>>16);
					atomic<slot> *far;
					far = out_nd->get_slot(1);

					slot far_cpy = far->load(); 

					slot back = ((Node*)(out_cpy>>16))->get_slot(0)->load();
					slot back_cpy = back;
					if((Node*)(back>>16) != edge_nd){
						continue;
					}
// check state for straddling push
					if((int)(far_cpy>>16) == RN){
						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy +=1;
						new_far_cpy +=1;
						temp = (temp|o)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;
	
						if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(far, &far_cpy, new_far_cpy)){
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

						if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
							// for memory reclamation update both hints
							hint_r(hint_cpy, edge_nd, SZ-2);
							//hint_r(oracle_r(right_slot_hint));
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
	int Deque::pop_left(){
		int busy = 0;
		while(true){
			busy++;
			usleep(2*busy);			// Exponential baackoff to guarantee progress

				node_hint hint_cpy = left_node_hint;
				node_hint temp_edge = l_oracle(hint_cpy);

				int edge_idx = temp_edge.ct;
				Node* edge_nd = temp_edge.buffer;

				std::atomic<slot>* in;
				std::atomic<slot>* out;
			
				in  = edge_nd->get_slot(edge_idx);
				out = edge_nd->get_slot(edge_idx-1);
				slot in_cpy = in->load();
				slot out_cpy = out->load();
					
// check if oracle failed
				if(((int)(in_cpy>>16) == LN || (int)(in_cpy>>16) == RS)
					||(edge_idx!=1 && (int)(out_cpy>>16) != LN)
					||(edge_idx==SZ-1 && (int)(in_cpy>>16) != RN))
					continue;

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

					if((int)(in_cpy>>16) == RN && in->load() == in_cpy){
						return 0;
					}
					if(atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy) && atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy)){
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

						std::atomic<slot> *far;
						far = out_nd->get_slot(SZ-2);

						slot far_cpy = far->load();
	
// ensure left neighbor points back
						slot back = *((Node*)(out_cpy>>16))->get_slot(SZ-1);
						slot back_cpy = back;
						if((Node*)(back>>16) != edge_nd){
							continue;
						}
				
// check for straddled edge and seal
						if((int)(far_cpy>>16) == LN){
							if(((int)(in_cpy>>16) == RN || (int)(in_cpy>>16) == RS) && in->load() == in_cpy){
								return 0; // empty
							}

							slot new_in_cpy = in_cpy;
							slot new_far_cpy = far_cpy;
							slot temp = 0;

							new_in_cpy+=1;
							new_far_cpy+=1;
							temp = (temp|LS)<<16;
							new_far_cpy = (new_far_cpy&0x000000FF)|temp;


							if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy)	&& atomic_compare_exchange_weak(far, &far_cpy, new_far_cpy)){
								far_cpy = new_far_cpy;
								in_cpy = new_in_cpy;
							}
						}
// check for sealed left node and remove it 
						if((int)(far_cpy>>16) == LS){
							if((int)(in_cpy>>16) == RN && in->load() == in_cpy){
								return 0;
							}

							slot new_in_cpy = in_cpy;
							slot new_out_cpy = out_cpy;
							slot temp = 0;

							new_in_cpy+=1;
							new_out_cpy+=1;
							temp = (temp|LN)<<16;
							new_out_cpy = (new_out_cpy&0x000000FF)|temp;

							if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
								// for memory reclamation update both hints
								hint_cpy = hint_l(hint_cpy, edge_nd, 1);
								//hint_r(oracle_r(right_slot_hint));
								retire(out_nd);
								in_cpy = new_in_cpy;
								out_cpy = new_out_cpy;
							}
						}
					}
// check for boundary edge, then boundary pop
					if((int)(out_cpy>>16) == LN){
						if((int)(in_cpy>>16) == RN && in->load()==in_cpy){
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

						if(atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy) && atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy)){
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
	int Deque::pop_right(){
		int busy = 0;
		while(true){
			busy++;
			usleep(2*busy);			// Exponential baackoff to guarantee progress
			
			node_hint hint_cpy = right_node_hint;
			node_hint temp_edge = r_oracle(hint_cpy);

			int edge_idx = temp_edge.ct;
			Node* edge_nd = temp_edge.buffer;

			std::atomic<slot>* in;
			std::atomic<slot>* out;
			
			in  = edge_nd->get_slot(edge_idx);
			out = edge_nd->get_slot(edge_idx+1);
			slot in_cpy = in->load();
			slot out_cpy = out->load();
					
// check if oracle failed
			if(((int)(in_cpy>>16) == RN || (int)(in_cpy>>16) == LS)
				||(edge_idx!=SZ-2 && (int)(out_cpy>>16) != RN)
				||(edge_idx==0 && (int)(in_cpy>>16) != LN))
				continue;
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

				if((int)(in_cpy>>16) == LN && in->load() == in_cpy){
					return 0;
				}
				if(atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy) && atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy)){
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

					std::atomic<slot> *far;
					far = out_nd->get_slot(1);

					slot far_cpy = far->load();
	
// ensure left neighbor points back
					slot back = *((Node*)(out_cpy>>16))->get_slot(0);
					slot back_cpy = back;
					if((Node*)(back>>16) != edge_nd){
						continue;
					}
				
// check for straddled edge and seal
					if((int)(far_cpy>>16) == RN){
						if(((int)(in_cpy>>16) == LN || (int)(in_cpy>>16) == LS) && in->load() == in_cpy){
							return 0; // empty
						}

						slot new_in_cpy = in_cpy;
						slot new_far_cpy = far_cpy;
						slot temp = 0;

						new_in_cpy+=1;
						new_far_cpy+=1;
						temp = (temp|RS)<<16;
						new_far_cpy = (new_far_cpy&0x000000FF)|temp;


						if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy) && atomic_compare_exchange_weak(far, &far_cpy, new_far_cpy)){
							far_cpy = new_far_cpy;
							in_cpy = new_in_cpy;
						}
					}
// check for sealed left node and remove it 
					if((int)(far_cpy>>16) == RS){
						if((int)(in_cpy>>16) == LN && in->load() == in_cpy){
							return 0;
						}

						slot new_in_cpy = in_cpy;
						slot new_out_cpy = out_cpy;
						slot temp = 0;

						new_in_cpy+=1;
						new_out_cpy+=1;
						temp = (temp|RN)<<16;
						new_out_cpy = (new_out_cpy&0x000000FF)|temp;

						if(atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy)	&& atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy)){
							// for memory reclamation update both hints
							hint_cpy = hint_r(hint_cpy, edge_nd, SZ-2);
							//hint_r(oracle_r(right_slot_hint));
							retire(out_nd);
							in_cpy = new_in_cpy;
							out_cpy = new_out_cpy;
						}
					}
				}
// check for boundary edge, then boundary pop
				if((int)(out_cpy>>16) == RN){
					if((int)(in_cpy>>16) == LN && in->load()==in_cpy){
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


					if(atomic_compare_exchange_weak(out, &out_cpy, new_out_cpy) && atomic_compare_exchange_weak(in, &in_cpy, new_in_cpy)){
						hint_r(hint_cpy, edge_nd, SZ-3);
						return (int)in_cpy>>16;
					}
				}
			}
		} 
	}

// Print the queue from left to right
	void Deque::print_queue(){
		cout << "\nThe contents of the queue are:\n";
		left_node_hint.buffer->printnode();
	}
