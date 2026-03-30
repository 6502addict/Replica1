#include <stdio.h>
#include <stdint.h>
#include <conio.h>
#include <timer.h>


int main() {
  printf("Hello, World\n");
  printf("cpu speed = %d Mhz\n", timer_cpu_speed());
  return 0;
}

