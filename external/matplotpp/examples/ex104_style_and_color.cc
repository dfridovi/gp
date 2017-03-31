//Copyright (c) 2011 Yuichi Katori (yuichi.katori@gmail.com) All Rights Reserved
using namespace std;
#include <matplotpp/matplotpp.h>
class MP :public MatPlot{ 
void DISPLAY(){

    // Create test data
    int n=100,m=10;
    vector<double> x(n),y(n);
    x=linspace(0,10,n);
    vector<vector<double> > Y(m,vector<double>(n));
    for(int j=0;j<m;++j){
	for(int i=0;i<n;++i){
	    x[i]=0.1*i;
	    Y[j][i]=sin(x[i])+0.4*j;
	}
    }
    // To set line width
    subplot(2,2,1);
    y=Y[0];plot(x,y);set(2);
    y=Y[1];plot(x,y);set(3);
    y=Y[2];plot(x,y);set(4);

    // To color lines
    subplot(2,2,2);
    y=Y[0];plot(x,y);set("r");//red
    y=Y[1];plot(x,y);set("g");//green
    y=Y[2];plot(x,y);set("b");//blue
    y=Y[3];plot(x,y);set("y");//yellow
    y=Y[4];plot(x,y);set("c");//cyan
    y=Y[5];plot(x,y);set("p");//purple
    
    // To set line style
    subplot(2,2,3);
    y=Y[0];plot(x,y);set("-");//solid line
    y=Y[1];plot(x,y);set(":");//dotted line
    y=Y[2];plot(x,y);set("- -");//dashed lne
    y=Y[3];plot(x,y);set("-.");//dash and dots
    y=Y[4];plot(x,y);set(":");  set(2);
    y=Y[5];plot(x,y);set("- -");set(2);
    y=Y[6];plot(x,y);set("-."); set(2);
    
    // To plot with markers
    subplot(2,2,4);
    y=Y[0];plot(x,y);set(".");
    y=Y[1];plot(x,y);set("+");
    y=Y[2];plot(x,y);set("x");
    y=Y[3];plot(x,y);set("d");
    y=Y[4];plot(x,y);set("^"); 
    y=Y[5];plot(x,y);set("v");
    y=Y[6];plot(x,y);set("o"); 
    y=Y[7];plot(x,y);set("*"); 
    y=Y[8];plot(x,y);set("s"); 
    
}
}mp;
void display(){mp.display(); }
void reshape(int w,int h){ mp.reshape(w,h); }
void idle( void ){ glutPostRedisplay(); usleep(10000);}
void mouse(int button, int state, int x, int y ){ mp.mouse(button,state,x,y); }
void motion(int x, int y ){mp.motion(x,y); }
void passive(int x, int y ){mp.passivemotion(x,y); }
void keyboard(unsigned char key, int x, int y){mp.keyboard(key, x, y); }
int main(int argc, char* argv[]){
    glutInit(&argc, argv);
    glutCreateWindow(100,100,500,400);
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
