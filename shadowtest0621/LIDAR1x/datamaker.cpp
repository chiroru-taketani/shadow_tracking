#include <iostream>
#include <unistd.h>
#include <time.h>
#include <math.h>

//メイン関数
int main(int argc, char* argv[])
{
    double theta = 0.0;
    char dataStr[100];
    
    while (1) {
        //printf("A\n");
        usleep(100000);
        FILE *fp = fopen("footpoint0.txt", "w");
        fprintf(fp, "3\n");
        fprintf(fp, "%f,%f\n", 30.0*cos(theta)+50, 30.0*sin(theta)+70.0);
        fprintf(fp, "%f,%f\n", 30.0*sin(theta)-150, 30.0*cos(theta)+70.0);
        fprintf(fp, "%f,%f\n", 30.0*sin(theta), 30.0*cos(theta)+100.0);
        fclose(fp);
        theta += 0.03;
    }
    return 0;
}
