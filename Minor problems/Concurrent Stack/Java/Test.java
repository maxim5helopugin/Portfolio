// Maxim Shelopugin
// Test class for stacks

import java.util.*;

// A test function, creates 3 threads, which push randomly
// And then pop in order 
class Test{
	public static void main(String[] args){
		EB_stack<Integer> test_stack = new EB_stack<Integer>();

		int numThreads = 32;

		Thread[] threads = new Thread[numThreads];
		for(int i = 0; i<numThreads; i++){
			threads[i] = new Thread(new Stack_manipulation(test_stack, true, numThreads));
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

	public Stack_manipulation(EB_stack st1, boolean pop, int numThreads){
		this.t = 0;
		this.st = st1;
		this.popper = pop;
		this.numThreads = numThreads;
	}

	public void run(){
		try{
		while(t!=4500000.0/(double)numThreads){
			//System.out.println("Pushing + " + t);
			st.push(t);
			t++;
		}
		}
		catch(Exception x){
			System.out.println("WHOOPS");
		}
		
		if(popper){
			//t = this.st.getNumOps();
			t = 4500000.0/(double)numThreads;
			while(t!=0){
				try{
					st.pop();
					//System.out.println("poped + "+ st.pop());
					t--;
				}
				catch(Exception e){
					System.out.println("Stack is empty");
					t--;
				}
			}
		}
		t = 1000000.0/(double)numThreads; 
		while(t!=0){
			st.getNumOps();
			t--;
		}		
	}
}