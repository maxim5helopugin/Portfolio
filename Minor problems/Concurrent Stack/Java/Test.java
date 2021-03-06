// Maxim Shelopugin
// Test class for stacks

import java.util.*;

// A test function, creates 3 threads, which push randomly
// And then pop in order 
class Test{
	public static void main(String[] args){
		EB_stack<Integer> test_stack = new EB_stack<Integer>();

		int numThreads = Integer.parseInt(args[0]);
		int operations = Integer.parseInt(args[1]);

		Thread[] threads = new Thread[numThreads];
		for(int i = 0; i<numThreads; i++){
			threads[i] = new Thread(new Stack_manipulation(test_stack, true, numThreads,operations/numThreads));
		}

		long startTime = System.nanoTime();
		for(int i = 0; i<numThreads; i++){
			threads[i].start();
		}

		try{
			for(int i = 0; i<numThreads; i++){
				threads[i].join();
			}
			long finish = System.nanoTime()-startTime;
			System.out.println("Took " + finish/1000000 + " ms");
		}
		catch(Exception e){
			System.out.println("Interrupted");
		}
		System.out.println("Operations : " + test_stack.getNumOps());
	} 
}
class Stack_manipulation implements Runnable{
	double t;
	EB_stack st;
	boolean popper;
	int numThreads;
	int num_ops;

	public Stack_manipulation(EB_stack st1, boolean pop, int numThreads, int num_ops){
		this.t = 0;
		this.st = st1;
		this.popper = pop;
		this.numThreads = numThreads;
		this.num_ops = num_ops;
	}

	public void run(){
		try{
			for(int i = 0; i<this.num_ops/2; i++)
				st.push(i);
			}
		catch(Exception x){
			System.out.println("WHOOPS");
		}
		
		if(popper){
			try{
				for(int i=0; i<this.num_ops/2; i++)
					st.pop();
			}
			catch(Exception e){
				System.out.println("Stack is empty");
			}
		}	
	}
}