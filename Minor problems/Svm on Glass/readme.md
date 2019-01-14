# Support Vector Machine on glass dataset
- To run the program just type 'python svm_hw.py' in the terminal.
- It imports the libsvmutil file from libsvm library, and is based largely on that library.
- For a successful run, the python script needs to be in the same directory 
- as the data folder. It opens the file "glass.data" and formats it into
- the LIBSVM format by creating a temporary "formated_data.txt" file. 
- Then it reads in the data from "formated_data.txt" file, and outputs the 
- accuracies of each test set of each kernel of each SVM (with the accirding hyperparameters which lead to that accuracy)
- ALl the output is displayed on the screen.
- The example of the output is included in "out.txt"
