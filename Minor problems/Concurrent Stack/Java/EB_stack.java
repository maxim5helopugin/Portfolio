// Maxim Shelopugin
// COP 6616
// November 9 2018
// Lock-free Stack with elimination backoff array.
// *Implemented solely based on the stack from chapter 11 from the textbook*

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicStampedReference; 
import java.util.concurrent.TimeUnit;

// Elimination Backoff stack
public class EB_stack<T> { 
	
// Node to store info in the stack
	public class Node {
		public T val; 
		public Node next; 
// Constructor
		public Node(T _val) { 
			val = _val;  
		} 
	}

// Exponential backoff class
	public class Backoff{
		final int min, max; 
		final Random random; 
		int limit; 
// Constructor
		public Backoff(int _min, int _max) { 
			min = _min; 
			max = _max; 
			limit = min; 
			random = new Random(); 
		} 
// Sleep for a random amount of time, but no longer than limit
		public void backoff() throws Exception{ 
			int delay = random.nextInt(limit); 
			limit = Math.min(max, 2 * limit); 
			Thread.sleep(delay); 
		} 
	}

// Exchanger object - allow only a single pair of operations to cancell out
	public class LockFreeExchanger<T> {
// Define states of the objects 
		static final int EMPTY = 0, WAITING = 1, BUSY = 2; 
		AtomicStampedReference<T> slot = new AtomicStampedReference<T>(null, 0); 

		public T exchange(T myItem, long timeout, TimeUnit unit) throws Exception{
			long nanos = unit.toNanos(timeout); 
			long timeBound = System.nanoTime() + nanos; 
			int[] stampHolder = {EMPTY}; 
			while (true){ 
				if (System.nanoTime() > timeBound) 
					throw new Exception(); 
				T yrItem = slot.get(stampHolder); 
				int stamp = stampHolder[0]; 
				switch(stamp) { 
					case EMPTY: 
						if (slot.compareAndSet(yrItem, myItem, EMPTY, WAITING)){ 
							while (System.nanoTime() < timeBound){ 
								yrItem = slot.get(stampHolder); 
								if (stampHolder[0] == BUSY) { 
									slot.set(null, EMPTY); 
									return yrItem; 
								}
							}
							if (slot.compareAndSet(myItem, null, WAITING, EMPTY)) { 
								throw new Exception(); 
							} else { 
								yrItem = slot.get(stampHolder); 
								slot.set(null, EMPTY); 
								return yrItem; 
							}
						} 
						break; 
					case WAITING: 
						if (slot.compareAndSet(yrItem, myItem, WAITING, BUSY)) 
							return yrItem;
						break; 
					case BUSY: 
						break; 
					default: 
				} 
			} 
		} 
	}

// Simple Policy object
// Stores current and maximum ranges of elimination
// If elimination is successful - shrink, broaden otherwise
	private static class RangePolicy{
		private int bounds[] = {0,1};

// Define the max index
		RangePolicy(int border){
			bounds[1] = border;
		}

// If successful, shrink
		public void recordEliminationSuccess(){
			if(bounds[0] > 1)
				bounds[0] -= 1;
		}

// If failed, broaden
		public void recordEliminationTimeout() {
			if(bounds[0] < bounds[1])
				bounds[0] += 1;
		}

		public int getRange() {
			return bounds[0];
		}
	}

// Elimination backoff array
	public class EliminationArray<T> { 
		private static final int duration = 10; 
		LockFreeExchanger<T>[] exchanger; 
		Random random; 
// Constructor.
// Create an exchanger array of size "capacity"
		public EliminationArray(int capacity) { 
			exchanger = (LockFreeExchanger<T>[]) new LockFreeExchanger[capacity]; 
			for (int i = 0; i < capacity; i++) { 
				exchanger[i] = new LockFreeExchanger<T>(); 
			} 
			random = new Random(); 
		} 
// Visit a random location within the range given by the policy
		public T visit(T value, int range) throws Exception { 
			int slot = random.nextInt(range); 
			return (exchanger[slot].exchange(value, duration, TimeUnit.MILLISECONDS)); 
		} 
	}
	
// Problem specific counters
	static final int capacity = 32;
	static final int MIN_DELAY = 1; 
	static final int MAX_DELAY = 100; 

	EliminationArray<T> eliminationArray = new EliminationArray<T>(capacity);
	AtomicReference<Node> head = new AtomicReference<Node>(null);
	AtomicInteger numOps = new AtomicInteger(0);
	Backoff backoff = new Backoff(MIN_DELAY, MAX_DELAY);

	static ThreadLocal<RangePolicy> policy = new ThreadLocal<RangePolicy>(){
		protected synchronized RangePolicy initialValue(){
			return new RangePolicy(capacity);
		}
	};

// Push with a single CAS, it successfull, return true
	private boolean tryPush(Node new_head) throws Exception{ 
		Node old_head = head.get(); 
		new_head.next = old_head; 
		return(head.compareAndSet(old_head, new_head)); 
	} 

// Keep pushing untill eliminated or pushed
	public void push(T x) throws Exception{
		RangePolicy rangePolicy = policy.get(); 
		Node new_head = new Node(x); 
		while (true){ 
			if (tryPush(new_head)){
				numOps.incrementAndGet();
				return; 
			}else try{ 
				T otherValue = eliminationArray.visit(x, rangePolicy.getRange());
				if(otherValue == null){
					numOps.incrementAndGet();
					rangePolicy.recordEliminationSuccess();
					return;
				}
			}catch(Exception e){
				rangePolicy.recordEliminationTimeout();
			} 
		}
	}

// Pop a single time. If successfull return the head
	private Node tryPop() throws Exception{ 
		Node old_head = head.get(); 
		if(old_head == null){ 
			throw new Exception(); 
		} 
		Node new_head = old_head.next; 
		if(head.compareAndSet(old_head, new_head)){ 
			return old_head; 
		}else{ 
			return null; 
		} 
	} 
	
// Keep popping untill elliminated or poped
	public T pop() throws Exception{ 
		RangePolicy rangePolicy = policy.get();
		while (true){ 
			Node old_head = tryPop(); 
			if (old_head != null){
				numOps.incrementAndGet(); 
				return old_head.val; 
			} else try{ 
				T otherValue = eliminationArray.visit(null, rangePolicy.getRange());
				if(otherValue != null){
					rangePolicy.recordEliminationSuccess();
					numOps.incrementAndGet();
					return otherValue;
				} 
			}catch(Exception e){
				rangePolicy.recordEliminationTimeout();
			}
		} 
	}

	public int getNumOps(){
		return numOps.get();
	}

}

