// Maxim Shelopugin
// Descriptor based stack
#include <memory>
template<class T>
class Stack{
public:
	Stack();
	~Stack(){
		delete d.load()->head.load();
		delete d.load();
	}
	bool push(Node<T>* new_head);
	T pop();
	int size();
	int getNumOps(){return numOps.load();};

private:
	// Get class specific descriptor and writedescr
	std::atomic<int> numOps;
	std::atomic<Descriptor<T>*> d;
	std::shared_ptr<WriteDescriptor<T>>wd;
};

template<class T>
Stack<T>::Stack():
	numOps(0)
	{
		// initialize size = 0, pending operations = false, head = NULL
		wd = std::shared_ptr<WriteDescriptor<T>>(new WriteDescriptor<T>(NULL, NULL, false));
		d = new Descriptor<T>(0, wd, NULL);
	}

template<class T>
bool Stack<T>::push(Node<T>* new_head){
		while(true){
			// Store current descriptor
			Descriptor<T> *temp_desc = d.load();
			// Finish old operations first
			temp_desc->complete();
			new_head->setNext(temp_desc->head.load());

			// Create a pending write operation, increase the size
			std::shared_ptr<WriteDescriptor<T>> writeop(new WriteDescriptor<T>(temp_desc->head.load(), new_head, true));
			Descriptor<T> *nextDesc = new Descriptor<T>(temp_desc->size+1, writeop, new_head);

			// keep retrying untill descriptors match
			if(atomic_compare_exchange_weak(&d, &temp_desc, nextDesc)){
				numOps++;
				return true;
			}
			// complete old operations
			nextDesc->complete();
		}
	}

template<class T>
T Stack<T>::pop(){
		Node<T>* current_head(NULL);
		Node<T>* new_head(NULL);

		while(true){
			// Store current descriptor
			Descriptor<T>*temp_desc = d.load();
			// Finish old operations first
			temp_desc->complete();
			current_head = temp_desc->head.load();
			if(current_head == NULL)
				return -1;
			new_head = current_head->getNext();

			// Create a pending write operation, decrease the size
			std::shared_ptr<WriteDescriptor<T>> writeop(new WriteDescriptor<T>(current_head, new_head, true));
			Descriptor<T>*nextDesc = new Descriptor<T>(temp_desc->size-1, writeop, new_head);
			if(atomic_compare_exchange_weak(&d, &temp_desc, nextDesc)){
				numOps++;
				return current_head->getVal();
			}
		}
	}

	// return the size of the stack
template<class T>
int Stack<T>::size(){
		numOps++;
		return d.load()->size;
	}