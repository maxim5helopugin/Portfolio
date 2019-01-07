## Maxim Shelopugin
## SVM for Glass clasification
## CAP 5610

import random
import time
from svmutil import *

## Get data from the file
def getdata():
	file = open("data/glass.data","r")
	data = []
	for line in file:
		data.append(line.split(','))
	return data

## Format the data in accordance to the libsvm format
def format(data):
	formated_data = []
	file = open("data/formated_data.txt", "w")
	for line in data:
		classNumber = str(line[-1].replace('\n',' '))
		lane = ''
		lane = lane + classNumber
		for i in range(0,(len(line)-2)):
			lane = lane + str(i+1)
			lane = lane + (':')
			lane = lane + str(line[i+1])
			lane = lane + ' '
		lane = lane + '\n'
		file.write(lane)
	file.close()

## Shuffle the dataset for the separation into the testing, validation and training
def shuffle(labels, vectors):
	for i in range(0, len(labels)):
		spot = int(random.uniform(0, len(labels)))
		temp_label = labels[i]
		temp_vector = vectors[i]

		labels[i] = labels[spot]
		vectors[i] = vectors[spot]

		labels[spot] = temp_label
		vectors[spot] = temp_vector

	return labels, vectors

## Separate the dataset into the 5 equal parts,
## return i's part as a test set, and everything else as a train set
def separate(labels, vectors, index):
	step = len(labels)/5
	test_labels = labels[step*(index-1):step*index]
	test_vectors = vectors[step*(index-1):step*index]
	train_labels = labels[0:step*(index-1)] + labels[step*index:len(labels)]
	train_vectors = vectors[0:step*(index-1)] + vectors[step*index:len(labels)]
	return test_labels, test_vectors, train_labels, train_vectors

## Calculate the weights of the unbalanced dataset
def calculate_weights(labels):
	sizes = {}
	weights = []
## get sizes of each class's dataset
	for i in labels:
		if i not in sizes:
			sizes[i] = 1
		else:
			sizes[i]+=1
## rebalance in accordance to the size
	for i in sizes:
		weights.append(100.0/sizes[i])
	return weights


## Depending on the kernel, return the list of possible hyperparameters to test
def get_params(kernel):
	params = []
	ceiling = 5
## linear - no parameters
	if kernel == 0:
		params.append('-t 0 -q')
## polynomial, change gamma, coef0 and degree
	elif kernel == 1:
		gammas = [temp*0.2 for temp in range(0,ceiling)]
		coefs = [temp*2 for temp in range(0,ceiling)]
		degrees = [temp for temp in range(0,ceiling-1)]
		for gamma in gammas:
			for coef in coefs:
				for degree in degrees:
					params.append('-t 1'+ ' -g '+ str(gamma) + ' -r ' + str(coef) + ' -d '+str(degree) +' -q')
## RBF, change gamma
	elif kernel == 2:
		gammas = [temp*0.1 for temp in range(0,ceiling*2)]
		for gamma in gammas:
			params.append('-t 2'+ ' -g '+ str(gamma)+' -q')
## sigmoid, change coef0 and gamma
	elif kernel == 3:
		gammas = [temp*0.1 for temp in range(0,ceiling*2)]
		coefs = [temp for temp in range(-1*ceiling,ceiling)]
		for gamma in gammas:
			for coef in coefs:
				params.append('-t 3'+ ' -g '+ str(gamma) + ' -r ' + str(coef) +' -q')
	return params;

## Regular SVM
def regular_svm(y, x):
	kernels = {}
	print "Regular SVM execution :"
	print "-------------------------------------------------"
	for kernel in range(0,4):	# for each of the kernels:
		print "!\tBest parameters for kernel = ", kernel
		kernels[kernel] = 0		# initialize accuracy to 0
		test_hyperspace = get_params(kernel)	# get possible parameters
		for i in range(1,6):	# for each of the test sets
			current_max = 0.0	# accuracy so far = 0
			current_prm = None   # best parameter so far = none
			current_prm_str = None
			test_l, test_v, train_l, train_v = separate(y,x,i)		# separate the dataset into testing and training set
			val_l, val_v, train_sub_l, train_sub_v = separate(train_l, train_v, 1)	# separate training set into validation and training
			problem = svm_problem(train_sub_l, train_sub_v)
			for params in test_hyperspace:			# pick all possible parameters
				parameters = svm_parameter(params)	
				m = svm_train(problem, parameters)	# train with the picked parameters
				lbl, acc, val =  svm_predict(val_l, val_v, m, '-q')	# try to test on validation
				if acc[0] >= current_max:			# if best so far, record it
					current_max = acc[0]
					current_prm = parameters
					current_prm_str = params
			### HERE THE PARAMETERS ARE SET
			print "!\t\ttest set",i,"\t",current_prm_str
			problem = svm_problem(train_l, train_v)
			m = svm_train(problem, current_prm)		# Train on the whole set now with best parameters
			lbl, acc, val = svm_predict(test_l, test_v, m, '-q')	# Test on the test set now
			print "!\t\taccuracy = ",acc[0]
			print "-------------------------------------------------"
			kernels[kernel] += acc[0]				# Record accuracy
		kernels[kernel]/=5
	print "\t Average kernel accuracy:"
	print kernels
	print "-------------------------------------------------"

## One vs all SVM
def one_vs_all_svm(y, x):
	kernels = {}
	print "One vs All SVM execution :"
	print "-------------------------------------------------"
	for kernel in range(0,4):	# for each of the kernels:
		print "!\tBest parameters for kernel = ", kernel
		kernels[kernel] = 0		# initialize accuracy to 0
		test_hyperspace = get_params(kernel)	# get possible parameters
		for i in range(1,6):	# for each of the test sets
			current_max = 0.0	# accuracy so far = 0
			current_prm = None   # best parameter so far = none
			current_prm_str = None
			test_l, test_v, train_l, train_v = separate(y,x,i)		# separate the dataset into testing and training set
			val_l, val_v, train_sub_l, train_sub_v = separate(train_l, train_v, 1)	# separate training set into validation and training
			classes = get_classes(train_sub_l)	# Get the class names
			for params in test_hyperspace:			# pick all possible parameters
				parameters = svm_parameter(params)
				ms = train_one_vs_all(train_sub_l, train_sub_v, parameters, classes)	# train different models for each class
				acc = predict_one_vs_all(val_l, val_v, ms, classes)						# Record accuracy of the all models average
				if acc >= current_max:			# if best so far, record it
					current_max = acc
					current_prm = parameters
					current_prm_str = params
			### HERE THE PARAMETERS ARE SET
			print "!\t\ttest set",i,"\t",current_prm_str			
			ms = train_one_vs_all(train_l, train_v, current_prm, classes) # now train on the whole training set
			acc = predict_one_vs_all(test_l, test_v, ms, classes)		  # now test on the test set
			print "!\t\taccuracy = ",acc
			print "-------------------------------------------------"
			kernels[kernel] += acc
		kernels[kernel]/=5
	print "\t Average kernel accuracy:"
	print kernels
	print "-------------------------------------------------"

## Prediction method for the one vs all SVM
def predict_one_vs_all(y, x, models, classes):
	total_acc = 0
	fake_list = []
# test models on each class
	for i in range(0, len(y)):
		fake_list.append(1)
	
	for c in classes:
		lbl, acc, val = svm_predict(fake_list, x, models[c], '-q')
		total_acc += compare(lbl,y, c)
	return total_acc/len(classes)

def compare(prediction, actual, current_class):
	current = int(current_class)
	positives = 0.1
	instances = 0.1
	for i in range(0, len(prediction)):
		if int(prediction[i]) == 1 and int(actual[i]) == current:
			positives+=1.0
		if int(actual[i]) == current:
			instances+=1.0
	return positives/instances

# Training method for the one vs all SVM
def train_one_vs_all(y, x, parameters, classes):
	models = {}
# For all classes 
	for i in classes:
		temp_y = []
# model each class in isolation, where 1 is the class, and 2 is everything else
		for j in range(0, len(y)):
			if int(y[j]) == int(i):
				temp_y.append(1)
			else:
				temp_y.append(2)
# now train this binary one vs all problem
		problem = svm_problem(temp_y,x)
		model = svm_train(problem, parameters)
# store the model which classifies a specific class
		models[i] = model
	return models

# Return the class names
def get_classes(y):
	classes = []
	for i in y:
		if i not in classes:
			classes.append(i)
	return classes

# Weighted svm
def weighted_svm(y,x):
	kernels = {}
	print "Re-weighted SVM execution :"
	print "-------------------------------------------------"
	for kernel in range(0,4):	# for each of the kernels:
		print "!\tBest parameters for kernel = ", kernel
		kernels[kernel] = 0		# initialize accuracy to 0
		test_hyperspace = get_params(kernel)	# get possible parameters
		for i in range(1,6):	# for each of the test sets
			current_max = 0.0	# accuracy so far = 0
			current_prm_str = None
			test_l, test_v, train_l, train_v = separate(y,x,i)		# separate the dataset into testing and training set
			val_l, val_v, train_sub_l, train_sub_v = separate(train_l, train_v, 1)	# separate training set into validation and training
			problem = svm_problem(train_sub_l, train_sub_v)
			weights = calculate_weights(train_sub_l)			# Count the new weights needed
			arg_weight = ' -w1 '+ str(weights[0]) + ' -w2 '+ str(weights[1]) + ' -w3 '+ str(weights[2]) + ' -w5 '+ str(weights[3]) + ' -w6 '+ str(weights[4]) + ' -w7 '+ str(weights[5]) # Reweight 
			for params in test_hyperspace:			# pick all possible parameters
				paramsarg = params + arg_weight				# append the new weights
				parameters = svm_parameter(paramsarg)	
				m = svm_train(problem, parameters)	# train with the picked parameters
				lbl, acc, val =  svm_predict(val_l, val_v, m, '-q')	# try to test on validation
				if acc[0] >= current_max:			# if best so far, record it
					current_max = acc[0]
					current_prm_str = params
			### HERE THE PARAMETERS ARE SET
			problem = svm_problem(train_l, train_v)
			weights = calculate_weights(train_l)			# Count the new weights needed
			arg_weight = ' -w1 '+ str(weights[0]) + ' -w2 '+ str(weights[1]) + ' -w3 '+ str(weights[2]) + ' -w5 '+ str(weights[3]) + ' -w6 '+ str(weights[4]) + ' -w7 '+ str(weights[5]) # Reweight 
			current_prm_str += arg_weight
			parameters = svm_parameter(current_prm_str)
			m = svm_train(problem, parameters)		# Now train on the whole training dataset
			lbl, acc, val = svm_predict(test_l, test_v, m, '-q')	# And test on the test set
			print "!\t\ttest set",i,"\t",current_prm_str	
			print "!\t\taccuracy = ",acc[0]
			print "-------------------------------------------------"
			kernels[kernel] += acc[0]
		kernels[kernel]/=5
	print "\t Average kernel accuracy:"
	print kernels
	print "-------------------------------------------------"

# Main, read data, and run 3 svms
def main():

	data = getdata()
	format(data)
	y,x = svm_read_problem('data/formated_data.txt')
	y,x = shuffle(y, x)

	start_time = time.time()
	regular_svm(y,x)
	print "Took ",(time.time()-start_time)
	print " "

	start_time = time.time()
	one_vs_all_svm(y,x)
	print "Took ",(time.time()-start_time)
	print " "

	start_time = time.time()
	weighted_svm(y,x)
	print "Took ",(time.time()-start_time)

if __name__ == "__main__":
	main()
