// Maxim Shelopugin
// COP 6616
// September 17 2018
// Lock-free Stack

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;


public class NB_stack<T>{

// Now references to the head and the counter are atomic
	AtomicReference<Node> head;
	AtomicInteger numOps = new AtomicInteger(0);

	public NB_stack(){
// When initializing, point head to nowhere
		head = new AtomicReference<Node>(null);
	}

// When pushing, atomically increment the number of ops
// only when the push is complete
	public boolean push(T x){
		Node<T> new_head = new Node(x);
		Node<T> current_head = null;
		while(true){
// keep trying assigning the new head's next to the current head
			current_head = head.get();
			new_head.set_next(current_head);
// if it works out, return and increment the ops
			if(head.compareAndSet(current_head,new_head)){
				numOps.incrementAndGet();
				return true;
			}
		}
// Try updating the head until it matches with expectation
	}

// Pop using atomic compareAndSet
	public T pop(){
		Node<T> current_head = null;
		Node<T> new_head = null;
// Keep trying to pop, unless it is an empty stack
		while(true){
			current_head = head.get();
// if stack is empty, then return null
			if(current_head == null)
				return null;
			new_head = current_head.get_next();
// if head did not change during execution, reassign the pointers, return the value
			if(head.compareAndSet(current_head, new_head)){
				numOps.incrementAndGet();
				return current_head.get_val();
			}
		}
	}

// Return the number of ops
	public int getNumOps(){
		return numOps.get();
	}
}