// Descriptor, stores the pointer to writedescr and size
#include <memory>

template<class T>
class Descriptor{

public:
	// size stored in the descriptor
	// Store head in the descriptor as well
	std::atomic<int> size;
	std::atomic<Node<T>*> head;
	
	Descriptor(int sz, std::shared_ptr<WriteDescriptor<T>> writeop, Node<T>* temp);
	std::shared_ptr<WriteDescriptor<T>> get_wd(){return wd;}
	void complete();

private:
	std::shared_ptr<WriteDescriptor<T>> wd;
};

template<class T>
Descriptor<T>::Descriptor(int sz, std::shared_ptr<WriteDescriptor<T>> writeop, Node<T>* temp):
	wd(writeop)
	{
		size.store(sz);
		head.store(temp);
	}

	// Complete old operations, write false to pending
template<class T>
void Descriptor<T>::complete(){
		if(wd->pending.load()){
			atomic_compare_exchange_weak(&head, wd->get_expected(), wd->get_desired());
			wd->completed();
		}
	}