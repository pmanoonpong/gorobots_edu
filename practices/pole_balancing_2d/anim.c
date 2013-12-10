/***********************************************/
/*                                             */
/*  anim.c                                     */
/*                                             */
/*  Jun Morimoto 2010.6.5                      */
/*                                             */
/*  Limited use for the collaboration study    */
/*  between Univ. of Gottingen and ATR-BRI     */
/*                                             */
/***********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h> 
#include <GL/glut.h>

#include "cart_pole.h"


GLUquadricObj* gluObj;

int color_flag = 1;

#define SPLIT 36  

struct timespec req;
struct timespec rem;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*================================*/
int WindowDump(int cx,int cy,int width,int height)
{
   int i,j;
   FILE *fptr;
   static char syscom[256];
   static int counter = 0; /* This supports animation sequences */
   char fname[32];
   unsigned char *image;
   int stereo=0;

   /* Allocate our buffer for the image */
   if ((image = malloc(3*width*height*sizeof(char))) == NULL) {
      fprintf(stderr,"Failed to allocate memory for image\n");
      return(FALSE);
   }

   glPixelStorei(GL_PACK_ALIGNMENT,1);

   /* Open the file */
   if (stereo)
      sprintf(fname,"Anim/Img/L_%04d.ppm",counter);
   else
      sprintf(fname,"Anim/Img/C_%04d.ppm",counter);
   if ((fptr = fopen(fname,"w")) == NULL) {
      fprintf(stderr,"Failed to open file for window dump\n");
      return(FALSE);
   }

   /* Copy the image into our buffer */
   glReadBuffer(GL_BACK_LEFT);
   glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,image);

   /* Write the raw file */
   printf("Writing file : %s\n",fname);
   fprintf(fptr,"P6\n%d %d\n255\n",width,height); /*for ppm */
   for (j=height-1;j>=0;j--) {
      for (i=0;i<width;i++) {
         fputc(image[3*j*width+3*i+0],fptr);
         fputc(image[3*j*width+3*i+1],fptr);
         fputc(image[3*j*width+3*i+2],fptr);
      }
   }
   fclose(fptr);
   sprintf(syscom,"gzip %s",fname);
   system(syscom);

   if (stereo) {
      /* Open the file */
      sprintf(fname,"Anim/Img/R_%04d.ppm",counter);
      if ((fptr = fopen(fname,"w")) == NULL) {
         fprintf(stderr,"Failed to open file for window dump\n");
         return(FALSE);
      }

      /* Copy the image into our buffer */
      glReadBuffer(GL_BACK_RIGHT);
      glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,image);

      /* Write the raw file */
      fprintf(fptr,"P6\n%d %d\n255\n",width,height); /*for ppm */
      for (j=height-1;j>=0;j--) {
         for (i=0;i<width;i++) {
            fputc(image[3*j*width+3*i+0],fptr);
            fputc(image[3*j*width+3*i+1],fptr);
            fputc(image[3*j*width+3*i+2],fptr);
         }
      }
      fclose(fptr);
   }

   /* Clean up */
   counter++;
   free(image);
   return(TRUE);
}
/*================================*/

/***************** draw cart ******************/
GLfloat n[6][3] = {  /* Normals for the 6 faces of a cube. */
  {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
  {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0} };
GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
  {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
  {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };
GLfloat v[8][3];  /* Will be filled in with X,Y,Z vertexes. */
void drawBox(double box_l, double box_w, double box_d)
{
  int i;

 /* Setup cube vertex data. */
    //bottom half length of x direction from origin
  v[0][0] = v[1][0] = v[2][0] = v[3][0] =-box_l;
    //top half length of x direction from origin
  v[4][0] = v[5][0] = v[6][0] = v[7][0] = box_l;

  //width
  v[0][1] = v[1][1] = v[4][1] = v[5][1] = -box_w;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] = box_w;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] = box_d;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] = -box_d;

  for (i = 0; i < 6; i++) {
    glBegin(GL_QUADS);
    glNormal3fv(&n[i][0]);
    glVertex3fv(&v[faces[i][0]][0]);
    glVertex3fv(&v[faces[i][1]][0]);
    glVertex3fv(&v[faces[i][2]][0]);
    glVertex3fv(&v[faces[i][3]][0]);
    glEnd();
  }
}


void MySolidCylinder(GLdouble radius, GLdouble height)
{
  GLUquadricObj *qobj = gluNewQuadric();

  glPushMatrix();
  glTranslatef(0.0, 0.0, -height/2.0 );
  gluDisk( qobj, 0.0, radius, SPLIT, 1 );               
  gluCylinder( qobj, radius, radius, height, SPLIT, 1 ); 
  glTranslatef(0.0, 0.0,  height );
  gluDisk( qobj, 0.0, radius, SPLIT, 1 );               
  glPopMatrix();

  gluDeleteQuadric(qobj);
}

GLint* pixarray;	
static int  Width=400, Height=200;
double state_to_gl_scale = 10.0;
double x_limit = 2.6; //[m]
void gl_update_anim(double *xt)
{
  static int flag = 1, m = 0;
  FILE *tmp_fp;
  int h, v, i;
  char filename[20];
  int capture_anim = 0;
  static int c = 0;


  /*clear display*/
  glClearColor (0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /****************************/
  /* Limit              */
  /****************************/
   glPushMatrix();
   glTranslatef(state_to_gl_scale*(-x_limit), 0.0, 0.0); 
   glColor3f(0.0,1.0,0.0);
   glRotatef( 90.0,1.0,0.0,0.0); 
   MySolidCylinder(0.4, 5.0);  
   glPopMatrix();

   glPushMatrix();
   glTranslatef(state_to_gl_scale*(x_limit), 0.0, 0.0); 
   glColor3f(0.0,1.0,0.0);
   glRotatef( 90.0,1.0,0.0,0.0); 
   MySolidCylinder(0.4, 5.0);  
   glPopMatrix();

  /****************************/
  /* Cart               */
  /****************************/
  glPushMatrix();
  glTranslatef(state_to_gl_scale*xt[_X], -1.0, 0.0);  
  glColor3f(0.0,1.0,1.0);
  drawBox(2.0,1.0,1.0); 

  /****************************/
  /* Pendulum           */
  /****************************/
  glColor3f(1.0,0.0,0.0);

//  glPushMatrix();
//  glRotatef( -xt[TH]/M_PI*180.,0.0,0.0,1.0);  
//  glTranslatef(0.0, 10.0, 0.0);
//  glutSolidSphere(1.5,10,10);
//  glPopMatrix();
//  glColor3f(0.0,0.5,1.0);

  glPushMatrix();
  glRotatef( -xt[TH]/M_PI*180.,0.0,0.0,1.0);  
  glTranslatef(0.0,5.0,0.0);
  glColor3f(1.0,0.0,0.0);
  glRotatef( 90.0,1.0,0.0,0.0);  
  MySolidCylinder(0.4, 10.0);
  glPopMatrix();



  glPopMatrix();

  glutSwapBuffers();
}

void glut_loop()
{
  glutMainLoop();  
}

int writeImg = 0;
void cart_pole_loop()
{

  int i;
  static int m = 0;

  static int    cycle = 0;            // counter
  //  static int    writeFrameCnt = 1200; // # of frames to write
  static int    writeFrameCnt = 500; // # of frames to write
  static int    writtenFrameCnt = 0;  // # of frames written so far
  //  static int    writePer = 110;       // period of writing
  static int    writePer = 2;       // period of writing //25Hz
  //  extern int    writeImg;

  glutPostRedisplay();

  simulation_loop();//------------------------------------------------------------- CALL CONTROLLER

  //#if 0
		/*************************************/
		/*  Wait for visualization   */       
		/*************************************/ 
		req.tv_sec = 0;
		req.tv_nsec = 1000;
                memcpy(&rem, &req,sizeof(req));
		memset(&req,0,sizeof(req));
		nanosleep(&req,&rem);
		//#endif


  if (writeImg){
    if (cycle % writePer == 0){
      WindowDump(0,0,Width,Height);
      writtenFrameCnt++;
      if (writtenFrameCnt == writeFrameCnt){
	writeImg = 0;
	printf("Written %d frames.\n", writtenFrameCnt);
      }
    }
    cycle ++;
  }

}

void disp_cart_pole()
{
  gl_update_anim(xt);
}

void glut_redisplay()
{
  glutPostRedisplay();
}

void init_lighting(void)
{
  GLfloat mat_specular[] = { 1.0,1.0,1.0,1.0 };
  GLfloat mat_shininess[] = { 100.0 };
  GLfloat light_position[] = { 50.0,50.0,50.0,1.0};
  GLfloat light_specular[] = {1.0,1.0,1.0,1.0};

  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
}


void gl_init_anim(int argc, char **argv)
{
  int i;

  glutInit(&argc, argv);
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize (Width,Height);
  glutCreateWindow("cart pole");
  glutDisplayFunc(disp_cart_pole);
  glutIdleFunc(cart_pole_loop);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();

  glOrtho( -30.0, 30.0, -15.0, 15.0, -10., 10.); 
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  gluObj = gluNewQuadric();

  //initialize lighting setting
  init_lighting();

  glClearColor (0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  pixarray = (GLint*)malloc((Height+1)*(Width+1)*3*sizeof(GLint));
}
