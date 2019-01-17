// Descriptor, stores the pointer to writedescr and size
#include <memory>

class Descriptor{

public:
	// size stored in the descriptor
	// Store head in the descriptor as well
	std::atomic<int> size;
	std::atomic<Node*> head;
	
	Descriptor(int sz, std::shared_ptr<WriteDescriptor> writeop, Node* temp);
	std::shared_ptr<WriteDescriptor> get_wd(){return wd;}
	void complete();

private:
	std::shared_ptr<WriteDescriptor> wd;
};
	
Descriptor::Descriptor(int sz, std::shared_ptr<WriteDescriptor> writeop, Node* temp):
	wd(writeop)
	{
		size.store(sz);
		head.store(temp);
	}

	// Complete old operations, write false to pending
void Descriptor::complete(){
		if(wd->pending.load()){
			atomic_compare_exchange_weak(&head, wd->get_expected(), wd->get_desired());
			wd->completed();
		}
	}