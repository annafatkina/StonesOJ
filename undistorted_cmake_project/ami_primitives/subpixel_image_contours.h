/*
   Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
   License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
   see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
 */


#ifndef AMI_DLL_H
  #define AMI_DLL_H
#endif

/**
 * \file subpixel_image_contours.h
 * \brief Subpixel image contour class AMI_DLL_H definition
 * \author Luis Alvarez \n \n
*/
#ifndef SUBPIXEL_IMAGE_CONTOURS_H
#define SUBPIXEL_IMAGE_CONTOURS_H

#include <stdlib.h>
#include "../ami_image/image.h"
#include "../ami_primitives/point2d.h"


#ifdef __AMIDEBUG__
  #include "wxAmiDebugLog.h"
#endif
using namespace std;

namespace ami
{

// class AMI_DLL_H OF SUBPIXEL PRECISION CONTOURS
/**
 * \class  subpixel_image_contours
 * \brief Class to store subpixel contours.
 * \author Luis Alvarez
 */
class AMI_DLL_H   subpixel_image_contours{
 bool *c/** Image contour c[i]=1 in contours points */;
 float *x/** Image with horizontal subpixel precision contour value */;
 float *y/** Image with vertical subpixel precision contour value*/;
 float *d/** Distance of the center line point to the white line border
          contour*/;
 float *coseno /** x orientation of the center lines*/;
 float *seno /** y orientation of the center lines*/;
 int N /** Length of x,y,d,coseno,seno*/;
 int width/** Image width*/;
 int height /** Image height */;

public:
 
 /**
  * \fn subpixel_image_contours()
  * \brief Constructor without taking memory
	* \author Luis Alvarez
  */
 subpixel_image_contours(){
  width = 0;
  height = 0;
  c = NULL;
  x = NULL;
  y = NULL;
  d = NULL;
  coseno = NULL;
  seno = NULL;
 }
 
 /** \fn subpixel_image_contours(int width_c,int height_c)
	 * \brief	Constructor taking memory
	 * \author Luis Alvarez
	 */
 subpixel_image_contours(int width_c,int height_c);
 
 /** \fn ~subpixel_image_contours()
   * \brief Destructor to free memory
	 * \author Luis Alvarez
	 */
 ~subpixel_image_contours();
 
 /** \fn subpixel_image_contours(const subpixel_image_contours &)
   * \brief Constructor taking memory
	 * \author Luis Alvarez
	 */
 subpixel_image_contours(const subpixel_image_contours &);

	/**
	 * \fn bool subpixel_empty()
	 * \brief Determine if a subpixel_image_contours is empty
	 * \author Pedro Henriquez, Carlos Falcon
	*/
  bool subpixel_empty();
	
	/**
	 * \fn point2d<double> find_nearest_subpixel(point2d<double>  point )
	 * \brief Find the nearest point to point px,py in subpixel image contours
	 * \author Pedro Henriquez, Carlos Falcon
	*/
  point2d<double> find_nearest_subpixel(point2d<double>  point );

 /**
  * \fn bool *get_c()
  * \brief Return array c to identity contour points
	* \author Luis Alvarez
  */
 bool *get_c(){return c;}

 const bool *get_c() const {return c;}

 /**
  * \fn float *get_x()
  * \brief Return array x of subpixel x coordinate location
	* \author Luis Alvarez
  */
 float *get_x(){return x;}

 const float *get_x() const {return x;}

 /**
  * \fn float *get_y()
  * \brief Return array y of subpixel y coordinate location
	* \author Luis Alvarez
  */
 float *get_y(){return y;}

 const float *get_y() const {return y;}

 /**
  * \fn float *get_d()
  * \brief Return array d of distance to a contour pixel to the boundary of
  *        contour pixel area
	* \author Luis Alvarez
  */
 float *get_d(){return d;}

 const float *get_d() const {return d;}

 /**
  * \fn const float *get_coseno()
  * \brief Return array coseno of x coordinate contour point orientation
	* \author Luis Alvarez
  */
 float *get_coseno(){return coseno;}

 const float *get_coseno() const {return coseno;}

 /**
  * \fn float *get_seno()
  * \brief Return array seno of y coordinate contour point orientation
	*	\author Luis Alvarez
  */
 float *get_seno(){return seno;}

 const float *get_seno() const {return seno;}

 /**
  * \fn const int get_width() const
  * \brief Return image width
	* \author Luis Alvarez
  */
 int get_width() const {return width;}

 /**
  * \fn const int get_height() const
  * \brief Return image height
	* \author Luis Alvarez
  */
 int get_height() const {return height;}


 /**
  * \fn set_c(bool *c2)
  * \brief Set array c to identity contour points
	* \author Luis Alvarez
  */
 void set_c(bool *c2/** Not described */){if(c!=NULL) free(c); c=c2;}

 /**
  * \fn void set_x(float *x2)
  * \brief Set array x of subpixel x coordinate location
	* \author Luis Alvarez
  */
void set_x(float *x2/** Not described */){if(x!=NULL) free(x); x=x2;}

 /**
  * \fn void set_y(float *y2)
  * \brief Set array y of subpixel y coordinate location
	* \author Luis Alvarez
  */
void set_y(float *y2/** Not described */){if(y!=NULL) free(y); y=y2;}

 /**
  * \fn void set_d(float *d2)
  * \brief Set array d of distance to a contour pixel to the boundary of
  *        contour pixel area
	* \author Luis Alvarez
  */
 void set_d(float *d2/** Not described */){if(d!=NULL) free(d); d=d2;}

 /**
  * \fn void set_coseno(float *coseno2)
  * \brief Set array coseno of x coordinate contour point orientation
	* \author Luis Alvarez
  */
 void set_coseno(float *coseno2/** Not described */){
   if(coseno!=NULL) free(coseno); coseno=coseno2;}

 /**
  * \fn void set_seno(float *seno2)
  * \brief Set array seno of y coordinate contour point orientation
	* \author Luis Alvarez
  */
 void set_seno(float *seno2/** Not described */){
   if(seno!=NULL) free(seno); seno=seno2;}

 /**
  * \fn void set_width(int width2)
  * \brief Set image width
	* \author Luis Alvarez
  */
 int set_width(int width2/** Not described */){return width=width2;}

 /**
  * \fn int set_height(int height2)
  * \brief Set image height
	* \author Luis Alvarez
  */
 int set_height(int height2/** Not described */){return height=height2;}

};


}

#endif
