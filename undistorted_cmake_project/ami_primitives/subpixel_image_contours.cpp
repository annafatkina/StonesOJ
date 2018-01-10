/*
   Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
   License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
   see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
 */


#ifndef AMI_DLL_CPP
  #define AMI_DLL_CPP
#endif

/**
 * \file subpixel_image_contours.cpp
 * \brief Subpixel image contour basic methods
 * \author Luis Alvarez \n \n
 */

#include "subpixel_image_contours.h"
#include "point2d.h"
#include <math.h>

using namespace ami;


/**
 * \fn  subpixel_image_contours::subpixel_image_contours(int width_c,
                                                         int height_c)
 * \brief Constructor taking memory
 * \author Luis Alvarez
 */
AMI_DLL_CPP subpixel_image_contours::
  subpixel_image_contours(int width_c/** New subpixel width */,
                          int height_c/** New subpixel height */)
{
  width=width_c;
  height=height_c;
  c=new bool[width*height];
  x=new float[width*height];
  y=new float[width*height];
  d=new float[width*height];
  coseno=new float[width*height];
  seno=new float[width*height];
 }

/**
 * \fn  subpixel_image_contours::
          subpixel_image_contours(const subpixel_image_contours &subpixel)
 * \brief Copy constructor
 */
AMI_DLL_CPP subpixel_image_contours::
  subpixel_image_contours(const subpixel_image_contours &subpixel
                          /** Subpixel to copy */)
{
  //FREE MEMORY
  delete[] c;
  delete[] x;
  delete[] y;
  delete[] d;
  delete[] coseno;
  delete[] seno;
  //TAKE MEMORY AND COPY CURRENT VALUES
  width   = subpixel.width;
  height  = subpixel.height;
  int lim = width*height;
  c      = new bool [lim];
  x      = new float[lim];
  y      = new float[lim];
  d      = new float[lim];
  coseno = new float[lim];
  seno   = new float[lim];
  //COPY CURRENT VALUES
  for (int i = 0; i < lim; i++)
  {
    c[i]      = subpixel.c[i];
    x[i]      = subpixel.x[i];
    y[i]      = subpixel.y[i];
    d[i]      = subpixel.d[i];
    coseno[i] = subpixel.coseno[i];
    seno[i]   = subpixel.seno[i];
  }
}

/**
 * \fn subpixel_image_contours::~subpixel_image_contours()
 * \brief Destructor to free memory
 */
AMI_DLL_CPP subpixel_image_contours::~subpixel_image_contours()
{
  delete[] c;
  delete[] x;
  delete[] y;
  delete[] d;
  delete[] coseno;
  delete[] seno;
}

/**
 * \fn bool subpixel_image_contours::subpixel_empty()
 * \brief Determine if a subpixel_image_contours is empty
 * \author Pedro Henriquez, Carlos Falcon
 */
AMI_DLL_CPP bool subpixel_image_contours::subpixel_empty()
{
  if( get_width()==0 && get_height()==0 )
    return false;
  else
    return true;
}


/**
 * \fn point2d<double> subpixel_image_contours::
        find_nearest_subpixel(point2d<double>  point )
 * \brief Find the nearest point to point px,py in subpixel image contours.
 * \param [in] point coordinates of selected point
 * \author Pedro Henriquez, Carlos Falcon
 */
AMI_DLL_CPP point2d<double> subpixel_image_contours::
  find_nearest_subpixel(point2d<double>  point )
{
  float distance=99999,dist;
  point2d<double> coordenadas,salida;
  salida.x=-1;
  salida.y=-1;
  for(int i=0;i<width*height;i++)
  {
    if(c[i]==1)
    {
      coordenadas.x=i%width;
      coordenadas.y=i/width;
      dist=(point-coordenadas).norm2();
      if (dist<distance)
      {
         distance= dist;
         salida.x=coordenadas.x;
         salida.y=coordenadas.y;
      }
    }
  }
  return salida;
}
