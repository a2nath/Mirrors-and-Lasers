#pragma once
#include <iostream>
#include <sstream>
#include <vector>

/* Program wide methods, structs and flags */

#define ON 1
#define OFF 0
extern int debug;

inline void show_debug(std::string s)
{
	if (debug) std::cout << s.c_str() << std::endl;
}

struct Location
{
	unsigned x, y;

	void operator = (const Location& loc)
	{
		x = loc.x;
		y = loc.y;
	}
	bool operator == (const Location& loc) const
	{
		return x == loc.x && y == loc.y;
	}
	bool isvalid()
	{
		return x > 0 && y > 0;
	}

	void operator--()
	{
		--x;
		--y;
	}
};
typedef Location Dimension;