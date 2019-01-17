// Write descriptor for the stack
template<class T>
class WriteDescriptor{
//expected and desired nodes
public:
	//are any operations pending?
	std::atomic<bool> pending;

	WriteDescriptor(Node<T>* nd1, Node<T>* nd2, bool progress);
	Node<T>** get_expected(){return &expected;}
	Node<T>* get_desired(){return desired;}
	void completed(){pending.store(false);}
private:
	Node<T>* expected;
	Node<T>* desired;
};

template<class T>
WriteDescriptor<T>::WriteDescriptor(Node<T>* nd1, Node<T>* nd2, bool progress):
	expected(nd1),
	desired(nd2)
	{
		pending.store(progress);
	}