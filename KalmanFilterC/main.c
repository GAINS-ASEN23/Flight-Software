/*
 ============================================================================
 Name        : main.c
 Author      : Bennett Grow, Kaylie Rick, Jason Popich
 Version     : 0.1
 Copyright   : 
 Description : 
 ============================================================================
 */

#include "CControl/Headers/Functions.h"

int main() {
	clock_t start, end;
	float cpu_time_used;
	start = clock();

	/* Your logic here */

	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %f\n", cpu_time_used);


	return EXIT_SUCCESS;
}
