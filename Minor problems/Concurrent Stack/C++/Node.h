// Node class for the stack
class Node{
public:
	Node(int _val);
	int getVal(){return val;}
	void setNext(Node* new_next){next = new_next;}
	Node* getNext(){return next;}
private:
	int val;
	Node* next;
};

//  constructor
Node::Node(int _val)
	:val(_val),
	next(NULL)
	{}