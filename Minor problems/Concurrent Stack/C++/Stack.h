// Maxim Shelopugin
// Descriptor based stack
#include <memory>
class Stack{
public:
	Stack();
	~Stack(){
		delete d.load()->head.load();
		delete d.load();
	}
	bool push(Node* new_head);
	int pop();
	int size();
	int getNumOps(){return numOps.load();};

private:
	// Get class specific descriptor and writedescr
	std::atomic<int> numOps;
	std::atomic<Descriptor*> d;
	std::shared_ptr<WriteDescriptor>wd;
};

Stack::Stack():
	numOps(0)
	{
		// initialize size = 0, pending operations = false, head = NULL
		wd = std::shared_ptr<WriteDescriptor>(new WriteDescriptor(NULL, NULL, false));
		d = new Descriptor(0, wd, NULL);
	}

bool Stack::push(Node* new_head){
		while(true){
			// Store current descriptor
			Descriptor *temp_desc = d.load();
			// Finish old operations first
			temp_desc->complete();
			new_head->setNext(temp_desc->head.load());

			// Create a pending write operation, increase the size
			std::shared_ptr<WriteDescriptor> writeop(new WriteDescriptor(temp_desc->head.load(), new_head, true));
			Descriptor *nextDesc = new Descriptor(temp_desc->size+1, writeop, new_head);

			// keep retrying untill descriptors match
			if(atomic_compare_exchange_weak(&d, &temp_desc, nextDesc)){
				numOps++;
				return true;
			}
			// complete old operations
			nextDesc->complete();
		}
	}

int Stack::pop(){
		Node* current_head(NULL);
		Node* new_head(NULL);

		while(true){
			// Store current descriptor
			Descriptor*temp_desc = d.load();
			// Finish old operations first
			temp_desc->complete();
			current_head = temp_desc->head.load();
			if(current_head == NULL)
				return -1;
			new_head = current_head->getNext();

			// Create a pending write operation, decrease the size
			std::shared_ptr<WriteDescriptor> writeop(new WriteDescriptor(current_head, new_head, true));
			Descriptor*nextDesc = new Descriptor(temp_desc->size-1, writeop, new_head);
			if(atomic_compare_exchange_weak(&d, &temp_desc, nextDesc)){
				numOps++;
				return current_head->getVal();
			}
		}
	}

	// return the size of the stack
int Stack::size(){
		numOps++;
		return d.load()->size;
	}