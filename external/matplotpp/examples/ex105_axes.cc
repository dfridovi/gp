//Copyright (c) 2011 Yuichi Katori (yuichi.katori@gmail.com) All Rights Reserved
using namespace std;
#include <matplotpp/matplotpp.h>
class MP :public MatPlot{ 
void DISPLAY(){

    // create test data 
    int n=100;
    float d=0.4;
    vector<double> x(n),y1(n),y2(n),y3(n),y4(n);    
    for(int i=0;i<n;++i){
	x[i]=0.1*i;
	y1[i]=sin(x[i])+d*0;
	y2[i]=sin(x[i])+d*1;
	y3[i]=sin(x[i])+d*2;
	y4[i]=sin(x[i])+d*3;
    }
    
    subplot(2,3,1);    
    plot(x,y1);

    // To add a title, labels 
    title("Auto");
    xlabel("x label ");
    ylabel("y label ");

    // To change plot range
    subplot(2,3,2);
    axis(-5,15,-2,0.5);
    plot(x,y1);

    // To add grids
    subplot(2,3,3);
    plot(x,y1);
    grid(1);
    //or grid("off");

    // To delete axes
    subplot(2,3,4);
    plot(x,y1);
    axis(0);

    // To delete ticklabel
    subplot(2,3,5);
    plot(x,y1);
    ticklabel(0);

    // To put tick outside
    subplot(2,3,6);set("TickDir","out");
    plot(x,y1);
    
}
}mp;
void display(){mp.display(); }
void reshape(int w,int h){ mp.reshape(w,h); }
void idle( void ){glutPostRedisplay(); usleep(10000);}
void mouse(int button, int state, int x, int y ){ mp.mouse(button,state,x,y); }
void motion(int x, int y ){mp.motion(x,y); }
void passive(int x, int y ){mp.passivemotion(x,y); }
void keyboard(unsigned char key, int x, int y){mp.keyboard(key, x, y); }
int main(int argc, char* argv[]){
    glutInit(&argc, argv);
    glutCreateWindow(100,100,760,500);
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
    glutIdleFunc( idle );
    glutMotionFunc( motion );
    glutMouseFunc( mouse );
    glutPassiveMotionFunc(passive);    
    glutKeyboardFunc( keyboard );        
    glutMainLoop();    
    return 0;
}
