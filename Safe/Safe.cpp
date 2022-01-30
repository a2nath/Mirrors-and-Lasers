#include <fstream>
#include "Safe.h"

using namespace std;
using namespace A_ROBOTICS;
vector<vector<Safe::Cell*>> Safe::grid;

int debug = OFF;
using namespace std;



struct Safe::Cell
{
	Location loc;
	Mirror mirror;
	BeamDir direction;
	Cell *UP, *LEFT, *RIGHT, *DOWN;
	~Cell() {}
};

struct Safe::pair_hash
{
	bool operator () (const Location& LOC1, const Location& LOC2) const
	{
		return LOC1.x != LOC2.x ? (LOC1.x < LOC2.x) : (LOC1.y < LOC2.y);
	}
	size_t operator () (const Location& LOC) const
	{
		return hash<unsigned>{}((LOC.x << 1) + LOC.y);
	}
};


inline bool valid_test(std::string s, Dimension& dims, unsigned& ms, unsigned& ns)
{
	std::string word;
	std::vector<std::string> result;
	std::istringstream iss(s, std::istringstream::in);

	while (iss >> word)
	{
		result.emplace_back(word);
	}

	if (result.size() < 4) return false;

	auto it = result.begin();
	dims.x = atol((*it++).c_str());
	dims.y = atol((*it++).c_str());
	ms = atol((*it++).c_str());
	ns = atol((*it++).c_str());

	return dims.isvalid();
}

void Safe::run(int arg_count, char * inputs[])
{
	int casenum = 1;
	ifstream inputfile;

	if (arg_count > 1 && *inputs[1] == '1')
		debug = ON;

	inputfile.open("input.txt");
	show_debug("\n\n");

	while (true)
	{
		unsigned ms, ns;
		string line;
		Dimension dims;

		/* get inputs from the file and validate them */
		getline(inputfile, line);
		while (!(valid_test(line, dims, ms, ns) || inputfile.eof()))
		{
			getline(inputfile, line);
		}
		if (inputfile.eof()) break;

		vector<Location> mMirrors(ms), nMirrors(ns);
		vector<unsigned> output(3);

		auto status = 1;

		/* handle some trivial cases such as when it is a tunnel,
			or when it is a grid and there are no mirrors */
		if (status > 0)
		{
			if (dims.x == 1 && dims.y > 0) // it's a tube, so shoud be no mirror, or imp.
				status = !(ms + ns) ? 0 : -1;
			else if (ms + ns < 1) // it's a matrix, so should be at least one mirror, or imp.
				status = -1;
		}

		/* populate mirrors and run the test case*/
		if (status > 0)
		{
			for (auto& m : mMirrors)
			{
				inputfile >> m.x >> m.y;
				--m;
			}
			for (auto& n : nMirrors)
			{
				inputfile >> n.x >> n.y;
				--n;
			}
			status = goodsafe(dims, mMirrors, nMirrors, output);
		}

		show_debug("----------");
		if (status > 0)
		{
			cout << "Case " << casenum++ << ": " << output[0] << " " << output[1] << " " << output[2] << endl;
		}
		else if (!status)
		{
			cout << "Case " << casenum++ << ": 0" << endl;
		}
		else
		{
			cout << "Case " << casenum++ << ": Impossible" << endl;
		}
		show_debug("----------");
		cleanup();
	}

	inputfile.close();
	show_debug("\nAll test cases complete. \nPress any key to continue...");
}

int Safe::goodsafe(Dimension dims, vector<Location> m_mir, vector<Location> n_mir, vector<unsigned> &output)
{
	unordered_set<Location, pair_hash> mlocations;
	unordered_set<unsigned> xcords, ycords;
	set<Location, pair_hash> positions;
	auto columns = vector<Cell*>(dims.y, NULL);

	show_debug("Please wait while program allocates memory");

	/* populate the grid and add mirrors */
	for (unsigned i = 0; i < dims.x; ++i)
	{
		grid.emplace_back(columns);
		for (unsigned j = 0; j < dims.y; ++j)
			grid[i][j] = new Cell{ { i, j }, Mirror::NONE, BeamDir::NO_CHANGE };
	}

	show_debug("Memory allocation complete. Running simulations\n");

	for (auto& m : m_mir) grid[m.x][m.y]->mirror = FORWARD;
	for (auto& n : n_mir) grid[n.x][n.y]->mirror = BACKWARD;

	/* form the links between Cells so we can traverse through the grid using recursion */
	vector<vector<bool>> visited(dims.x, vector<bool>(dims.y, false));
	connect_cells(visited, 0, 0);

	/* need mirror correction if not the last cell or beam not going in the correct direction */
	Cell* cell = start_beam();
	if (success(cell, dims)) return 0;

	/* get all possible coordinates of other mirrors on the grid */
	for (auto m : m_mir) xcords.insert(m.x);
	for (auto n : n_mir) xcords.insert(n.x);
	for (auto m : m_mir) ycords.insert(m.y);
	for (auto n : n_mir) ycords.insert(n.y);
	xcords.insert(0);
	xcords.insert(dims.x - 1);
	for (auto x : xcords) for (auto y : ycords) mlocations.insert(Location{ x, y });

	/* remove possible positions of forward mirrors that are already occupied */
	for (auto m : m_mir)
	{
		auto it = find_if(mlocations.begin(), mlocations.end(), [&](const Location& loc) { return loc == m; });
		if (it != mlocations.end()) mlocations.erase(it);
	}

	/* remove possible positions of backward mirrors that are already occupied */
	for (auto n : n_mir)
	{
		auto it = find_if(mlocations.begin(), mlocations.end(), [&](const Location& loc) { return loc == n; });
		if (it != mlocations.end()) mlocations.erase(it);
	}

	/* try adding forward mirror in the XY-intersections of where the other mirrors are */
	for (auto location : mlocations)
	{
		grid[location.x][location.y]->mirror = FORWARD;
		cell = start_beam();
		if (success(cell, dims))
			positions.insert(location);
		grid[location.x][location.y]->mirror = NONE;
	}

	/* try adding backward mirror in the XY-intersections of where the other mirrors are */
	for (auto location : mlocations)
	{
		grid[location.x][location.y]->mirror = BACKWARD;
		cell = start_beam();
		if (success(cell, dims))
			positions.insert(location);
		grid[location.x][location.y]->mirror = NONE;
	}

	if (positions.size())
	{
		output = { positions.size(), positions.begin()->x + 1, positions.begin()->y + 1 };
		return 1;
	}
	else return -1;
}

inline void Safe::cleanup()
{
	if (!grid.size()) return;
	show_debug("\nSimulations complete. Cleaning up memory.\n");
	for (auto& list : grid) for (auto& data : list) delete data;
	grid.clear();
}

Safe::Cell* Safe::connect_cells(vector<vector<bool>>& visited, unsigned row, unsigned col) // where all the nodes are connected
{
	if (row > grid.size() - 1 || col > grid[0].size() - 1)
	{
		return NULL;
	}

	if (visited[row][col] != true)
	{
		grid[row][col]->UP = int(row - 1) < 0 ? NULL : grid[row - 1][col];
		grid[row][col]->LEFT = int(col - 1) < 0 ? NULL : grid[row][col - 1];

		grid[row][col]->RIGHT = connect_cells(visited, row, col + 1);
		grid[row][col]->DOWN = connect_cells(visited, row + 1, col);

		visited[row][col] = true;
	}
	else return grid[row][col];

	return grid[row][col];
}

inline Safe::Cell* Safe::set_celldir(Cell* cell, const BeamDir& dir)
{
	switch (dir)
	{
	case RIGHT: cell->direction = RIGHT; return cell;
	case DOWN: cell->direction = DOWN; return cell;
	case LEFT: cell->direction = LEFT; return cell;
	case UP: cell->direction = UP; return cell;
	}
}

Safe::Cell* Safe::shine_beam(Cell* current) // the traveral algorithm which simulates beam propagating through the grid
{
	switch (current->direction)
	{
	case RIGHT:
		if (current->mirror == NONE)
		{
			return current->RIGHT != NULL ? shine_beam(set_celldir(current->RIGHT, RIGHT)) : set_celldir(current, RIGHT);
		}
		else if (current->mirror == BACKWARD) // \ backward
		{
			return current->DOWN != NULL ? shine_beam(set_celldir(current->DOWN, DOWN)) : set_celldir(current, DOWN);
		}
		else // \ forward mirror
		{
			return current->UP != NULL ? shine_beam(set_celldir(current->UP, UP)) : set_celldir(current, UP);
		}
	case DOWN:
		if (current->mirror == NONE)
		{
			return current->DOWN != NULL ? shine_beam(set_celldir(current->DOWN, DOWN)) : set_celldir(current, DOWN);
		}
		else if (current->mirror == BACKWARD) // \ backward
		{
			return current->RIGHT != NULL ? shine_beam(set_celldir(current->RIGHT, RIGHT)) : set_celldir(current, RIGHT);
		}
		else // / forward mirror
		{
			return current->LEFT != NULL ? shine_beam(set_celldir(current->LEFT, LEFT)) : set_celldir(current, LEFT);
		}
	case LEFT:
		if (current->mirror == NONE)
		{
			return current->LEFT != NULL ? shine_beam(set_celldir(current->LEFT, LEFT)) : set_celldir(current, LEFT);
		}
		else if (current->mirror == BACKWARD) // \ backward
		{
			return current->UP != NULL ? shine_beam(set_celldir(current->UP, UP)) : set_celldir(current, UP);
		}
		else // / forward mirror
		{
			return current->DOWN != NULL ? shine_beam(set_celldir(current->DOWN, DOWN)) : set_celldir(current, DOWN);
		}
	case UP:
		if (current->mirror == NONE)
		{
			return current->UP != NULL ? shine_beam(set_celldir(current->UP, UP)) : set_celldir(current, UP);
		}
		else if (current->mirror == BACKWARD) // \ backward
		{
			return current->LEFT != NULL ? shine_beam(set_celldir(current->LEFT, LEFT)) : set_celldir(current, LEFT);
		}
		else // / forward mirror
		{
			return current->RIGHT != NULL ? shine_beam(set_celldir(current->RIGHT, RIGHT)) : set_celldir(current, RIGHT);
		}
	}
}

inline Safe::Cell* Safe::start_beam()
{
	reset_beam();
	return shine_beam(grid[0][0]);
}

inline void Safe::reset_beam()
{
	grid[0][0]->direction = RIGHT;
}

inline bool Safe::success(const Cell* cell, const Dimension& dims) // check if success condition is met
{
	return cell->direction == RIGHT && cell->loc.x == dims.x - 1 && cell->loc.y == dims.y - 1;
}