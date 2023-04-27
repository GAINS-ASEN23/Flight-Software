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
#include "funcs.c"

int main() 
{

	// Start clock to time code
	clock_t start, end;
	float cpu_time_used;
	start = clock();

	uint16_t S = 2;
	float test[4] = {2.0,1.0,1.0,2.0};
	float test2[4] = {2,1,1,2};
	float final[4];

	add(test, test2, final, S*S);

	print(test, S, S);
	print(final, S, S);

	// Stop code timing clock and print execution time
	end = clock();
	cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
	printf("\nTotal speed  was %f\n", cpu_time_used);


	return EXIT_SUCCESS;
}
