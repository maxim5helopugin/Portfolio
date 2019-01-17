// Write descriptor for the stack

class WriteDescriptor{
//expected and desired nodes
public:
	//are any operations pending?
	std::atomic<bool> pending;

	WriteDescriptor(Node* nd1, Node* nd2, bool progress);
	Node** get_expected(){return &expected;}
	Node* get_desired(){return desired;}
	void completed(){pending.store(false);}
private:
	Node* expected;
	Node* desired;
};

WriteDescriptor::WriteDescriptor(Node* nd1, Node* nd2, bool progress):
	expected(nd1),
	desired(nd2)
	{
		pending.store(progress);
	}