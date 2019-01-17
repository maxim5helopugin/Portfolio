// Node class for the stack
template<class T>
class Node{
public:
	Node(T _val);
	T getVal(){return val;}
	void setNext(Node* new_next){next = new_next;}
	Node* getNext(){return next;}
private:
	T val;
	Node* next;
};

//  constructor
template<class T>
Node<T>::Node(T _val)
	:val(_val),
	next(NULL)
	{}