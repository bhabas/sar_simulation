import multiprocessing 

def square_list(mylist, result, square_sum): 
	""" 
	function to square a given list 
	"""

	for ii, num in enumerate(mylist): 
		result[ii] = num * num 

	square_sum.value = sum(result) 

	print(f"Result(in process p1): {result[:]}") 
	print(f"Sum of squares(in process p1): {square_sum.value}") 

if __name__ == "__main__": 

	mylist = [1,2,3,4] 

	result = multiprocessing.Array('i', 4) 
	square_sum = multiprocessing.Value('i') 


	p1 = multiprocessing.Process(target=square_list, args=(mylist, result, square_sum)) 

	p1.start() 
	p1.join() 


	print(f"Result(in main program): {result[:]}")
	print(f"Sum of squares(in main program): {square_sum.value}")
