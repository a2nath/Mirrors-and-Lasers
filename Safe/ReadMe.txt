========================================================================
		Console Application : Ascent Robotics Project Overview
========================================================================


Problem statement
-------

It is assumed that you have access to the specifications for the Mirror question. In case not, just to recall, we are asked about the positions where we have to add a corrective mirror in a m x n grid (safe security system) which will allow the laser beam to go from the upper left cell traversing East, bounce around certain existing mirrors and somehow reach the bottom right cell also moving East. The laser can reach its destination with or without this new mirror. The direction of the mirror can be either forward or backward facing. It is also possible that no matter what you do, given any mirror positions in addition to the existing mirrors, the laser may never reach its destination.


Code Overview:
-------

Every effort has been made to add and test the code with unique test cases for the program. This will ensure that the program succeeds in simple as well as more complex tests. The program will check for the dimensions of the grid and number of mirrors involved depending on certain parameters as seen in the beginning of the "goodsafe" function. For example, if the grid looks like a horizontal tunnel with 1 row only, then there should be no mirrors in between to block the path of the laser. If there are none, then answer is 0 or else Impossible. If the grid looks like a vertical tube with 1 column only, or if there are more number of columns, then there must be at least one mirror involved in the test case in order for the laser to reach the correct cell after insertion of the new mirror.

Safe::goodsafe static function returns:

1: Possible to reach the destination with the addition of 1 mirror in one or more positions in the grid
0: Possible to reach the destination without inserting the mirror (bad thing)
-1: Impossible to reach the destination given certain dimensions or mirror positions

The function fills out the “output” vector with three values to display on the console when the return value is 1. When this method is called at first, the first thing it does is to allocate a matrix (grid) of the required dimensions with "Cell" objects that will store location, mirror and laser beam's direction. The location helps understand whether we have arrived at the destination cell in the grid, and also efficiently insert and remove the new mirror when doing trial and error at potential locations O(1). The mirror information can convey whether existing mirror in the cell is forwards, backwards or non-existent. If non-existent, then the beam continues to traverse its last known direction until it reaches the end of the grid, or else it bounces off the mirror and its direction is decided based on a certain criteria. Storage space requirements of this data structure is O(n^2) but as per my understanding this is the best you are going to get.

The beam's direction is crucial for the traversal algorithm (recursion) which allows the beam to move in directions decided by the position and type of mirrors or lack thereof. In order for the algorithm to work, the grid (vector of vector) consists of pointer of "Cell" which are connected in their respective UP, DOWN, LEFT and RIGHT pointers to other cells in the grid. This allows each cell to know where exactly they are with respect to other cells. This is very similar to Doubly Linked-List but more complex. Selected cells are configured with mirror information from the test case, and an initial traversal is performed to check that the beam does not reach the destination yet. If it does, then the method returns 0 and memory is deallocated before starting the next test case.

In the next stage a list of possible new mirror locations are maintained based on the coordinates of all possible x-values and y-values of existing mirrors. This is a hash table, because of constant insertion/lookup times θ(1), and its nature of storing unique values since we do not want to check the same locations again. The core of the program logic is that there is a special location that will allow the beam to change directions and perhaps hit another mirror. Hence, it is the intersection of the x-y pairs of existing mirrors where a potential new mirror can change the course of the beam. To address edge cases, it is also the x-y pair intersection with the first and last row of the grid that will help guide the beam in case there are no existing mirrors there. This is due to the special nature of the grid[0][0] and grid[m-1][n-1] cells, i.e. source and destination respectively. To generalize this further, we want to guide this laser outside of and within this network of mirrors where it can traverse differently and reach its destination. No brute-force algorithm is required since we just need a bunch of intersection points so the runtime should be constant.

Finally in order to try these possible locations for the new mirror, simulations are performed with a forward mirror and a backwards mirror for each position. In the beginning of each simulation, the beam direction is reset to East. During the simulation if the traversal algorithm returns with a cell property that is grid[m-1][n-1] and beam direction East, it means that the beam has reached its destination and the corresponding mirror position can be added in a new list of successful positions.

As specified in the instructions, in order to return the number of successful mirror positions and the pair of lexicographically smaller x-y coordinates, a set data structure is maintained. The set is configured with a functor (overloading) which will look at each insertion of a new entry and the one with a smaller x-value will be moved up. If both x-values are the same, then the one with a smaller y-value will be moved up. By nature of a set you cannot have duplicates which also meets the specification that a forward and a backward mirror counts as one. The method returns the positions-count, as well as the x-y pairs from the beginning of the set. The insertion runtime is θ(log_n) as it is a binary search tree, and the lookup is constant.


To compile the source code:
-------

a. On the console switch to the directory where the source code is.
b. Type g++ -std=c++14 mainfile.cpp Safe.cpp -o mainfile

c. Modify the input.txt file as per the test cases you want to test with. Ensure that they are valid test cases. Make sure to remove all empty lines in between mirror coordinates from the input file. Empty lines are fine if they are before or after a test case and all of its mirror coordinates. Don't modify the input file yet (see next section).

d. Now on the console type ./mainfile
e. You should see the output as per the specifications given. For very large grids size, allow more memory allocation/traversal time.
f. When the console stops displaying new information, then all test cases are processed and you can simply press Enter to get back prompt.


Correct developer environment
-------

With the supplied input file, if your compiler and developer environment is the same, then you should get *only* the following output. Originally I made this with Visual Studio 2015 and cross-checked my answers and output with g++ version 7.3. After this validation you are free to change the input file.

Case 1: 3 9 3
Case 2: 1 500 600
Case 3: 0
Case 4: Impossible
Case 5: 2 4 3
Case 6: Impossible
Case 7: 0
Case 8: 1 1 1
Case 9: 6 16 15


Additional command-line arguments:
-------

If you want to see debug outputs, add 1 as an additional parameter like so ./mainfile 1
When using very large grid sizes, if it takes too long to show the output, then you can check which stage is taking too long.


Personal comments
-------

The common file contains all global definitions and group of helper function tools, while avoiding namespace pollution. In addition, instead of creating a new instance of the “Safe” object for each test case, I decided to make the functions and data structure static since we only need one instance and because it made syntactic sense at the time. Lastly, the specifications does not focus on error-handling or checking for invalid inputs. So I assumed that aspect is not being tested. It is a classic try-catch block with a throw syntax for different type of errors. The program handles invalid tests that have r = 0 or c = 0 in the grid dimensions, in which case it ignores all consecutive lines for mirror positions until the next test case is read.

With that said, I would love to hear back from you on whether my submission does well against your test cases, and hopefully pass this stage of the interview round.


-Abhi
