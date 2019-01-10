// Node class for the stack
class Node{
	
private:
	int val;
	Node* next;

public:
	Node(int _val)
	:val(_val),
	next(NULL)
	{}

// return value
	int getVal(){
		return val;
	}

// set next pointer
	Node* setNext(Node* new_next){
		next = new_next;
	}

// get next pointer
	Node* getNext(){
		return next;
	}
};