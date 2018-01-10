/*
   Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
   License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
   see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
 */

#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>
#include "../ami_image/image.h"
#include "../ami_image_draw/image_draw.h"
#include "../ami_primitives/image_primitives.h"
#include <fstream>

/* *
 * \def ami_free1d(direccion)
 * \brief Frees the memory of the array created by ami_calloc1d o ami_malloc1d.
 * \author Luis Alvarez
 */
#define ami_free1d(direccion) { if (direccion != NULL){\
  free(direccion);direccion=NULL;}}

/* *
 * \def ami_free2d(direccion)
 * \brief Frees the memory of the matrix created by ami_calloc2d or ami_malloc2d.
 * \author Luis Alvarez
 */
#define ami_free2d(direccion) { if (direccion != NULL){\
  ami_free1d(direccion[0]);  free(direccion); direccion = NULL;}}

/* *
 * \def ami_free3d(direccion)
 * \brief Frees the memory of the matrix created by  ami_calloc3d o ami_malloc3d.
 * \author Luis Alvarez
 */
#define ami_free3d(direccion) {if (direccion != NULL){\
  ami_free2d(direccion[0]);  free(direccion); direccion=NULL;}}

/* *
 * \def ami_calloc2d(direccion,tipo,height,width) \anchor <calloc2d>
 * \brief Get memory for array direccion with type X, width and height certain.
          Initialize to 0 the adress positions
 * \author Luis Alvarez
 */
#define ami_calloc2d(direccion,tipo,height,width) {int ml,mk; \
	if ( width > 0 && height > 0){\
          direccion=(tipo **) malloc(sizeof(tipo *)*(height)); \
          if (direccion == NULL){\
            \
            int val = scanf("%d",&ml); if(val==EOF) ;}\
          direccion[0]=(tipo *)malloc(sizeof(tipo)*(width)*(height)); \
          if (direccion[0] == NULL){free(direccion); direccion = NULL;\
            \
            int val = scanf("%d",&ml); if(val==EOF) ;}\
          for(ml=0;ml<(height);ml++) direccion[ml]=&(direccion[0][ml*(width)]);\
          for(ml=0;ml<height;ml++) for(mk=0;mk<width;mk++) direccion[ml][mk]=0;\
	}\
	else direccion = NULL;}

/* *
 * \def ami_calloc3d(direccion,tipo,depth,height,width)
 * \brief  Get memory in three-dimensional variable direccion with the width and
            height given. Initialize to 0 the adress positions
 * \author Luis Alvarez
 */
#define ami_calloc3d(direccion,tipo,depth,height,width) {int ml,mk,mu; \
	if ( depth > 0 && width > 0 && height > 0){\
          direccion=(tipo ***) malloc(sizeof(tipo **)*(depth)); \
          if (direccion == NULL){\
            ; \
            int val = scanf("%d",&ml); if(val==EOF) ;}\
          direccion[0]=(tipo **)malloc(sizeof(tipo *)*(depth)*(height)); \
          if (direccion[0] == NULL){free(direccion); direccion = NULL;\
            ; \
            int val = scanf("%d",&ml); if(val==EOF) ;}\
          direccion[0][0]=(tipo *)malloc(sizeof(tipo)*(depth)*(height)*(width));\
          if (direccion[0][0] == NULL){free(direccion[0]); free(direccion);\
            direccion = NULL; ;\
            int val = scanf("%d",&ml); if(val==EOF) ;}\
          for(mu=0;mu<depth;mu++){ \
            for(ml=0;ml<(height);ml++) \
              direccion[0][ml+mu*(height)]=&(direccion[0][0][ml*(width)+\
                                             mu*(width)*(height)]); \
            direccion[mu]=&(direccion[0][mu*(height)]);}\
          for(mu=0;mu<depth;mu++)\
            for(ml=0;ml<height;ml++)\
              for(mk=0;mk<width;mk++) direccion[mu][ml][mk]=0; \
 	}\
	else direccion = NULL;}

/**
 * \def ami_malloc1d(direccion,tipo,size)
 * \brief Get memory in the array direccion with size given.
 * \author Luis Alvarez
 */
#ifdef _AMI_TIF_H_
#undef ami_malloc1d
#endif
#define ami_malloc1d(direccion,tipo,size) {\
	if (size > 0){\
	    direccion=(tipo *) malloc(sizeof(tipo)*(size)); \
		if (direccion == NULL)\
		{int ml; ; \
                 int val = scanf("%d",&ml); if(val==EOF) ;}}\
	else direccion = NULL;}

std::vector < std::vector <unsigned int> >
  boundary_neighborhood_9n(const unsigned int width, const unsigned int height);

std::vector < std::vector <unsigned int> >
  boundary_neighborhood_5n(const unsigned int width, const unsigned int height);

/**
 * \fn ami::image<unsigned char> drawHoughLines(image_primitives ip,
																								int width, int height,
																								ami::image<unsigned char> &bn)
 * \brief Draw the points detected by the improved Hough transform
 * \author Luis Alvarez and Daniel Santana-Cedres
*/
void drawHoughLines(image_primitives ip /**Detected primitives*/,
										ami::image<unsigned char> &bn /**Gray level image where we
										draw the lines*/);

/**
 * \fn void invert(ami::image<unsigned char> &input,
									 ami::image<unsigned char> &output)
 * \brief Invert the colors of the input image
 * \author Luis Alvarez and Daniel Santana-Cedres
*/
void invert(ami::image<unsigned char> &input /**Input image*/,
            ami::image<unsigned char> &output/**Output image with the inverted
						colors*/);

/**
 * \fn print_function_syntax()
 * \brief Print the function syntax
 * \author Luis Alvarez and Daniel Santana-Cedres
*/
void print_function_syntax_lens_distortion_correction_division_model_1p();

/**
 * \fn check_params(char *argv[])
 * \brief Check the input parameters
 * \author Luis Alvarez and Daniel Santana-Cedres
*/
int check_params_lens_distortion_correction_division_model_1p(char *argv[]);

/**
 * \fn count_points(image_primitives ip)
 * \brief This function counts the number of points of the primitives
 * \author Luis Alvarez and Daniel Santana-Cedres
*/
int count_points(image_primitives ip);

/**
 * \fn manage_failure(char *argv[])
 * \brief This function manages the output in case of failure
 * \author Luis Alvarez and Daniel Santana-Cedres
*/
void manage_failure(char *argv[]);

#endif
