## Maxim Shelopugin
## COP 5610 
## KNN Mnist classifier
## 9/11/2018

import time
import math
import numpy
import struct
import sys

## Function reads the single MNIST data file and returns the numpy vector
## This code in the box below was taken from the GitHub repository of the user
## tylerneylon, link to which is https://gist.github.com/tylerneylon/ce60e8a06e7506ac45788443f7269e40#file-mnist-py
## All credit for the function below is due to its author
#-------------------------------------------------------------------------------#
def read_idx(filename):															#
    with open(filename, 'rb') as f:												#
        zero, data_type, dims = struct.unpack('>HBB', f.read(4))				#
        shape = tuple(struct.unpack('>I', f.read(4))[0] for d in range(dims))	#
        return numpy.fromstring(f.read(), dtype=numpy.uint8).reshape(shape)		#
#-------------------------------------------------------------------------------#

## Calculate the Euclidian distance between 2 images
def distance(image1, image2):
	difference = numpy.subtract(image1,image2)
	difference = difference*difference
	likeness = numpy.sum(difference)
	return math.sqrt(likeness)

## Calculate the accuracy between the expected and actual outputs
def accuracy(expected, actual):
	positive = 0.0
	for i in range(0, len(expected)):
		if expected[i] == actual[i]:
			positive+=1
	positive = positive/len(expected)
	return positive

## Calculate and print the confidence interval
def confidence_interval(success_rate, n):
	error_rate = 1-success_rate
	sigma = math.sqrt((error_rate*success_rate)/n)
	min_conf = success_rate - 1.96* sigma
	max_conf = success_rate + 1.96* sigma
	print "The confidence interval is ", min_conf, " ", max_conf

## print the confusion matrix of the expected and actual outputs
def confusion_matrix(expected, actual):
	matrix = numpy.zeros((10,10))
	for i in range(0, len(expected)):
		matrix[expected[i],actual[i]]+=1
	print "\nThe confusion matrix is :"
	print matrix

## Take in the image and slide it into 9 different windows,
## return the array with these 9 images
def slide(img):
	new_images = numpy.zeros((9,28,28), dtype = numpy.int32)
	img_slid = numpy.zeros((30,30),dtype = numpy.int32)
	img_snap = numpy.zeros((28,28),dtype = numpy.int32)
	i=0
	for x in range(0,28):
		for y in range(0,28):
			img_slid[x+1, y+1] = img[x,y]

	for horizontal in range(0,3):
		for vertical in range(0,3):
			for x in range(0,28):
				for y in range(0,28):
					img_snap[x,y] = img_slid[x+horizontal,y+vertical]
			new_images[i] = img_snap
			i+=1
	return new_images

## Train and classify test_images with the help of train_images, using knn neighbours at most.
## 2 flags - is_training - to find the best K, and is_sliding - to test on sliding images.
## For each feature vector in test_images, compare to each sample in train_images,
## record the k closest samples, and then determine the most frequent one.
## Has a visual animation of training time
def train(train_images, test_images, train_labels, test_labels, lower_bound, upper_bound, is_training, knn, is_sliding):
	
	major_vote = 0
	most_frequent = 0
	actual_class = []
	predicted_class = [[] for classes in range(knn)]
	frequencies = numpy.zeros(10)
	error_histogram = numpy.zeros(10)
	distances = numpy.zeros(len(train_images))
	current_feature_vector = lower_bound
	errors = numpy.zeros(knn)
	batch = numpy.zeros(9)
## animation step
	size = upper_bound - lower_bound
	step = size/50

	if is_training:
		string = "\nValidating on"
	else:
		string = "\nTesting on   "
	print("%s set %7d : %7d ______________" % (string,lower_bound, upper_bound))
# Only test the data between lower_bound and upper_bound
	while current_feature_vector < upper_bound:
		if current_feature_vector%step == 0:
			sys.stdout.write('|')
# if needed to be slid, generate 9 images by sliding it
		if is_sliding:
			new_images = slide(test_images[current_feature_vector])
		training_example = 0
# compare the image to each image in training set
		while training_example < len(train_images):
# if training, skip the examples of testing set, by assignning a big distance
			if is_training:
				if training_example >= lower_bound and training_example < upper_bound:
					distances[training_example] = 10000
				else:
					distances[training_example] = distance(test_images[current_feature_vector], train_images[training_example])
			else:
				if not is_sliding:
					distances[training_example] = distance(test_images[current_feature_vector], train_images[training_example])
				else:
# choose the closest image from the sliding set
					for image in range(0, len(new_images)):
						batch[image] = distance(new_images[image], train_images[training_example])
					distances[training_example] = batch[batch.argsort()[0]]
			training_example+=1

# choose the k closest neighbours from the set of distances
		closest_examples = distances.argsort()[:knn]
# for each K choose the best one
		for k in range(1,(knn+1)):
			for sample_size in range(0, k):
				frequencies[train_labels[closest_examples[sample_size]]]+=1	
			for sample_size in range(0,10):
# record the frequencies of each digit in the neighbours
				if frequencies[sample_size] > most_frequent:
					most_frequent = frequencies[sample_size]
					major_vote = sample_size
# if there are more than one candidate, break the tie
				elif frequencies[sample_size] == most_frequent:
					major_vote = train_labels[closest_examples[0]]
			most_frequent = 0
			frequencies = numpy.zeros(10)
			predicted_class[k-1].append(major_vote)
		actual_class.append(test_labels[current_feature_vector])
		current_feature_vector+=1
# collect the accuracies of all K neighbors
	for error_k in range(0, knn):
		errors[error_k] = accuracy(predicted_class[error_k], actual_class)
	error_histogram = 1-errors
# if not training, print confusion matrix
	if not is_training:
		confusion_matrix(predicted_class[knn-1], actual_class)
# return the error histogram
	return error_histogram

# K fold cross validation, returns the optimal K
def getK(train_images, train_labels):
	folding = 10
	errors = numpy.zeros(10)
	boundaries = numpy.zeros((folding, 2), dtype = numpy.int32)
	step = len(train_images)/folding

## determine the bound of each validation set, from the training set
	for i in range(0,folding):
		boundaries[i,0] = step*i
		boundaries[i,1] = step*(i+1)

## for each of 10 test sets, compare to the other part of the training set, sum the errors
	for i in range(0, folding):		
		errors += train(train_images, train_images, train_labels, train_labels, boundaries[i,0], boundaries[i,1],True, 10, False)
## normalize the errors
	errors = errors / folding
	print "\n\nThe normalized error of each choice of k:"
	print  errors

## return the k that has the lowest normalized error
	k = errors.argsort()[0]
	return k

# the main body of the program, get the data, try classifying with k = 1
# train with respect to k, using 10 fold cross validation, 
# classify with the new k, and lastly, classify with new k and sliding window

def main():
	current_time1 = time.time()
	test_images = numpy.array(read_idx('data/t10k-images-idx3-ubyte'), dtype=numpy.int32)
	test_labels = numpy.array(read_idx('data/t10k-labels-idx1-ubyte'), dtype=numpy.int32)
	train_images = numpy.array(read_idx('data/train-images-idx3-ubyte'), dtype=numpy.int32)
	train_labels = numpy.array(read_idx('data/train-labels-idx1-ubyte'), dtype=numpy.int32)
# Small subset for testing. Uncomment if necessary
	#test_labels = test_labels[0:100]
	#test_images = test_images[0:100]
	#train_images = train_images[0:1000]
	#train_labels = train_labels[0:1000]
	print "\n\nClassify with k = 1:"

# First test with k = 1 
	k = 0
	positive = 1-train(train_images, test_images, train_labels, test_labels, 0, len(test_images), False, k+1,False)[k]
	print "\n\nThe accuracy is ", positive*100.0
	confidence_interval(positive, len(test_images))
	print "\n Starting the training\n"
	
# Get an optimal k
	k = getK(train_images, train_labels)
	print "The k chosen = ",(k+1)

# Now test with that optimal K
	positive = 1-train(train_images, test_images, train_labels, test_labels, 0, len(test_images), False, k+1,False)[k]
	print "\n\nThe accuracy is ", positive*100.0
	confidence_interval(positive, len(test_images))

# Now test with the sliding window
	positive = 1-train(train_images, test_images, train_labels, test_labels, 0, len(test_images), False, k+1,True)[k]
	print "\n\nThe accuracy is ", positive*100.0
	confidence_interval(positive, len(test_images))


	print "The execution took:"
	print(time.time() - current_time1)

if __name__ == '__main__':
	main()