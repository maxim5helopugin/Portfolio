// Node stays the same, since we only care about the atomicity of the stack
	class Node<T>{
		private T val;
		private Node next;

		public Node (T _val){
			this.val = _val;
		}

		public T get_val(){
			return this.val;
		}

		public void set_next(Node next){
			this.next = next;
		}

		public Node get_next(){
			return this.next;
		}
	}