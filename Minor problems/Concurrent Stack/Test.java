// Maxim Shelopugin
// Test class for stacks

import java.util.*;

// A test function, creates 3 threads, which push randomly
// And then pop in order 
class Test{
	public static void main(String[] args){
		NB_stack<Integer> test_stack = new NB_stack<Integer>();

		Thread t1 = new Thread(new Stack_manipulation(test_stack, true));
		Thread t2 = new Thread(new Stack_manipulation(test_stack, false));
		Thread t3 = new Thread(new Stack_manipulation(test_stack, false));

		t1.start();
		t2.start();
		t3.start();

		try{
			t1.join();
			t2.join();
			t3.join();
		}
		catch(Exception e){
			System.out.println("Interrupted");
		}
		System.out.println(test_stack.getNumOps());
	}
}

class Stack_manipulation implements Runnable{
	int t;
	NB_stack st;
	boolean popper;

	public Stack_manipulation(NB_stack st1, boolean pop){
		this.t = 0;
		this.st = st1;
		this.popper = pop;
	}
	public void run(){
		try{
		while( t!=5){
			System.out.println("Pushing + " + t);
			st.push(t);
			t++;
		}
		Thread.sleep(1000);
		}
		catch(Exception x){
			System.out.println("WHOOPS");
		}
		
		if(popper){
			t = this.st.getNumOps();
			while(t!=0){
				System.out.println("poped + "+st.pop());
				t--;
			}
		}
		
	}
}