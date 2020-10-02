all: controller
controller:controller.cpp math_linear_algebra.cpp
	g++ controller.cpp math_linear_algebra.cpp -I ../include -o ../controller -lpthread -Wall 
	# -I: look for #includes in ../include
	# -o: output compiled file called "controller"
	# -Wall: "Warn all", turns on all warning g++ can give you

	#g++ controller.cpp -o ../controller -lboost_system -lboost_thread


# Help: http://mrbook.org/blog/tutorials/make/
#		https://courses.cs.washington.edu/courses/cse373/99au/unix/g++.html