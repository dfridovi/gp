//Copyright (c) 2011 Yuichi Katori (yuichi.katori@gmail.com) All Rights Reserve
using namespace std;
#include <matplotpp/matplotpp.h>

int is_run=1;
double t=0;// time

class MP :public MatPlot{ 
void DISPLAY(){
    // Create test data 
    int n=40;
    dvec x,y,z;
    x=linspace(-2,2,n);
    y=linspace(-2,2,n);
    dmat Z(n,dvec(n));
    double r2;
    for(int i=0;i<n;++i){//y
	for(int j=0;j<n;++j){//x	    
	    r2=x[j]*x[j]+y[i]*y[i];
	    Z[i][j]=exp(-r2)*sin(2*r2+t)+0.5;
	}
    }
    z.resize(n);
    for(int i=0;i<n;++i){
	z[i]=sin(3*x[i]+t);
    }

    //plot
    subplot(2,1,1);
    title("press r to run and stop");
    plot(x,z);
   
    //mesh
    subplot(2,1,2);
    axis(-2,2,-2,2,0,1);
    mesh(x,y,Z);

}
}mp;
void display(){mp.display(); }
void reshape(int w,int h){ mp.reshape(w,h); }
void idle( void ){ 

    if(is_run){ t+=0.02; } 

    glutPostRedisplay(); 
    usleep(10000);
}
void mouse(int button, int state, int x, int y ){ mp.mouse(button,state,x,y); }
void motion(int x, int y ){mp.motion(x,y); }
void passive(int x, int y ){mp.passivemotion(x,y); }
void keyboard(unsigned char key, int x, int y){
    mp.keyboard(key, x, y); 
    if(key=='r'){ if(is_run==0){is_run=1;}else{is_run=0;}}
}
int main(int argc, char* argv[]){
    glutInit(&argc, argv);
    glutCreateWindow(100,100,400,300);
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
