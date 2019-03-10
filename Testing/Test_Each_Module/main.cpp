#include <iostream>
#include <stdio.h>

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
double max(double num1, double num2);
double Defuzzication_R(double e, double de);

int main(int argc, char** argv) {
	printf("max = %.3f",max(1.3f,1.2f));
	int num1 = scanf("%f",&num1);
	return 0;
}

double max(double num1,double num2){
	return (num1 > num2 ) ? num1 : num2;
}

