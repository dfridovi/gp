//Copyright (c) 2011 Yuichi Katori All Rights Reserved
//Author: Yuichi Katori (yuichi.katori@gmail.com)
using namespace std;
#include <matplotpp/matplotpp.h>
class MP :public MatPlot{ 
void DISPLAY(){
    vector<double> x(100),y(100);    
    for(int i=0;i<100;++i){
	x[i]=0.1*i;
	y[i]=sin(x[i]);
    }
    plot(x,y);
}
}mp;
void display(){ mp.display(); }
void reshape(int w,int h){ mp.reshape(w,h); }
int main(int argc,char* argv[]){
    glutInit(&argc, argv);
    glutCreateWindow(100,100,400,300);
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
    glutMainLoop();    
    return 0;
}
