//Copyright (c) 2011 Yuichi Katori (yuichi.katori@gmail.com) All Rights Reserve
using namespace std;
#include <matplotpp/matplotpp.h>
class MP :public MatPlot{ 
void DISPLAY(){

    // Prepare test data
    int n=50;
    vector<double> x,y,z;
    x=linspace(-2,2,n);
    y=linspace(-2,2,n);
    vector< vector< double > > Z(n,vector<double>(n)),C(n,vector<double>(n));
    for(int i=0;i<n;++i){
	for(int j=0;j<n;++j){
	    Z[i][j]=sin(3*x[j])+sin(3*y[i])+0.5;	    
	}
    }

    // To set current color map, use following commands
    //hsv();
    jet();
    
    // To generate 3D surface plot
    surface(x,y,Z);
    
    // Another test data
    n=300;
    x.resize(n);
    y.resize(n);
    z.resize(n);    
    double t;
    for(int i=0;i<n;++i){
	t=0.1*i;
	x[i]=sin(t);
	y[i]=cos(t);
	z[i]=0.1*t;
    }

    // To plot 3D line
    plot3(x,y,z);
    
    // To color this line
    set("r");set(3);

    // To add colorbar
    colorbar();

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
    glutCreateWindow(100,100,600,600);
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
