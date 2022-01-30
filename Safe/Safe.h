#pragma once
#include <set>
#include <algorithm>
#include <unordered_set>
#include "..\common.h"

/* Safe class and functions */
namespace A_ROBOTICS
{
	class Safe
	{
		enum Mirror { NONE, FORWARD, BACKWARD };
		enum BeamDir { NO_CHANGE, UP, RIGHT, DOWN, LEFT };
		struct Cell;
		struct pair_hash;

		static std::vector<std::vector<Cell*>> grid;
	public:
		static void run(int arg_count, char * inputs[]);

	private:
		static inline void cleanup();
		static int goodsafe(Dimension dims, std::vector<Location> m_mir, std::vector<Location> n_mir, std::vector<unsigned> &output);
		static Cell* connect_cells(std::vector<std::vector<bool>>& visited, unsigned row, unsigned col);
		static inline Cell* set_celldir(Cell* cell, const BeamDir& dir);
		static Cell* shine_beam(Cell* current);
		static inline Cell* start_beam();
		static inline void reset_beam();
		static inline bool success(const Cell* cell, const Dimension& dims);

	};
};