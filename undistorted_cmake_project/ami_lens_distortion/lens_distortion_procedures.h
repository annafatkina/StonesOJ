/*
   Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
   License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
   see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
 */


#ifndef LENS_DISTORTION_PROCEDURE_H
#define LENS_DISTORTION_PROCEDURE_H

#include "../ami_primitives/point2d.h"
#include "../ami_lens_distortion/lens_distortion_model.h"
#include "../ami_primitives/line_points.h"
#include "../ami_image/image.h"
#include <vector>
using namespace std;

double model_estimation_1p_quotient(point2d<double>  distortion_center,
									vector< line_points > &lines ,
									lens_distortion_model &d);

int build_l1r_quotient_vector(std::vector<double> &l1r,
                              double max_distance_corner, double *a,int Na);

double distortion_points_to_line_equation_quotient(lens_distortion_model &d,
												   line_points &l);

/**
 * \fn  template <class  U>
        ami::image<U> lens_distortion_procedures::undistort_quotient_image_inverse(
        ami::image<U> input_image,
        lens_distortion_model &d,
        const double &image_amplification_factor)
 * \brief ESTIMATE AN UNDISTORTED IMAGE USING a QUOTIENT DISTORTION MODEL (inverse method)
 * \author Luis Alvarez
 */
template <class  U>
ami::image<U> undistort_quotient_image_inverse(
ami::image<U> input_image,
lens_distortion_model &d,
const double &image_amplification_factor
)
{
  int width0=input_image.width();
  int height0=input_image.height();
  int size =width0*height0;
  int width=width0,height=height0;

  // DISPLACEMENT OF IMAGE CENTER
  //point2d<double> displacement=point2d<double>((double) (width-width0)/2.,(double) (height-height0)/2.);

  // WE CREATE OUTPUT IMAGE
  ami::image<U> output_image(width,height,0,0,0);

  //CALCULATE MAXIMUM DISTANCE FROM CENTRE TO A CORNER
  point2d<double> ldm_centre=d.get_distortion_center();
  point2d<double> corner(0,0);
  //  cout << "Width " << width0 << " Height " << height0 << endl;
  //  cout << "LDM centre " << ldm_centre.x << " " << ldm_centre.y << endl;
  double max_distance_corner= (ldm_centre-corner).norm();
  corner.y=height0;
  double distance_corner=(ldm_centre-corner).norm();
  if(distance_corner>max_distance_corner) max_distance_corner=distance_corner;
  corner.x=width0;
  distance_corner=(ldm_centre-corner).norm();
  if(distance_corner>max_distance_corner) max_distance_corner=distance_corner;
  corner.y=0;
  distance_corner=(ldm_centre-corner).norm();
  if(distance_corner>max_distance_corner) max_distance_corner=distance_corner;

  //  cout << "Max distance corner " << max_distance_corner << endl;

  //BUILD INTERMEDIATE VECTOR
  vector<double> l1r;
  double *a;
  int Na;
  vector<double> d1=d.get_d();
  Na=2*(d1.size()-1);
  if(d1.size()<2) {output_image=input_image; return(output_image);}
  a=(double*)malloc(sizeof(double)*(Na+1));
  a[0]=d1[0];
  for(int i=1;i<(int)d1.size();i++){a[2*i-1]=0.; a[2*i]=d1[i]; }
  int cont2=Na;
  while(a[cont2]==0){ cont2--; if (cont2 == 0)break;}
  Na=cont2;

  //WE UPDATE THE max_distance_corner ACCORDING TO LENS DISTORSION MAX DISPLACEMENT
  if(d1.size()==2){
    max_distance_corner=max_distance_corner/(d1[0]+d1[1]*max_distance_corner*max_distance_corner);
  }
  else{
    max_distance_corner=max_distance_corner/(d1[0]+d1[1]*max_distance_corner*max_distance_corner+
                        d1[2]*max_distance_corner*max_distance_corner*max_distance_corner*max_distance_corner);
  }
  // WE BUILD THE LENS DISTORTION INVERSE VECTOR
  if(d1.size()==2){
    if(build_l1r_quotient_vector(l1r,max_distance_corner,a,Na)<0){
      output_image=input_image;
      return(output_image);
    }
  }
  else{
    l1r.resize((int)(max_distance_corner+1.5));
    l1r[0]=1;
    double r0=1;
    for(int m=1;m<(int)l1r.size();m++){
      //WE COMPUTE THE INVERSE USING NEWTON-RAPHSON
      double h=1e-5;
      for(int i=0;i<1000;i++){
        //EVALUATION OF LENS DISTORTION MODEL AND DERIVATIVE
        double r2=r0*r0;
        double sum=d1[0];
        for(int k=1;k<(int)d1.size();k++){
          sum+=d1[k]*r2;
          r2*=r0*r0;
        }
        double f_r0=r0/sum;
        //DERIVATIVE
        r2=(r0+h)*(r0+h);
        sum=d1[0];
        for(int k=1;k<(int)d1.size();k++){
          sum+=d1[k]*r2;
          r2*=(r0+h)*(r0+h);
        }
        double f_r0_h=(r0+h)/sum;
        double derivative=(f_r0_h-f_r0)/h;
        // WE COMPUTE THE NEW ROOT
        double r1=r0-(f_r0-m)/derivative;
        if(fabs(r0-r1)<fabs(r0)*1e-5){
          r0=r1;
          break;
        }
        r0=r1;
        //printf("iter=%d,m=%d, r0=%lf\n",i,m,r0);
      }
      l1r[m]=r0/m;
      //system("pause");
    }
  }

  // WE FIT IMAGE SCALING
  double scale;
  ami::point2d<double> t;
  if(image_amplification_factor==1.){ //ALL IMAGE IS INCLUDED IN CORRECTED ONE
    ami::point2d<double> temp2=d.evaluation_quotient( ami::point2d<double>((double) width,(double) height));
    scale=(temp2-ldm_centre ).norm()/ldm_centre.norm();

    temp2=d.evaluation_quotient( ami::point2d<double>((double) width,(double) 0.));
    double scale2=(temp2-ldm_centre ).norm()/ldm_centre.norm();
    //if(scale2>scale) scale=scale2;
    if(scale2<scale) scale=scale2;

    temp2=d.evaluation_quotient( ami::point2d<double>((double) 0.,(double) height));
    scale2=(temp2-ldm_centre ).norm()/ldm_centre.norm();
    //if(scale2>scale) scale=scale2;
    if(scale2<scale) scale=scale2;

    temp2=d.evaluation_quotient( ami::point2d<double>((double) 0.,(double) 0.));
    scale2=(temp2-ldm_centre ).norm()/ldm_centre.norm();
    //if(scale2>scale) scale=scale2;
    if(scale2<scale) scale=scale2;


   // printf("scale=%lf\n",scale);
//    t.x=(scale*width-width)*0.5;
//    t.y=(scale*height-height)*0.5;
    t.x=(scale*ldm_centre.x-ldm_centre.x);
    t.y=(scale*ldm_centre.y-ldm_centre.y);
  }
  else if(image_amplification_factor==2.){ //IMAGE IS FITTED TO KEEP WIDTH
    ami::point2d<double> temp2=d.evaluation_quotient( ami::point2d<double>((double) width,ldm_centre.y));
    scale=(temp2-ldm_centre ).norm()/(ldm_centre.x);

    temp2=d.evaluation_quotient( ami::point2d<double>((double) 0.,ldm_centre.y));
    double scale2=(temp2-ldm_centre ).norm()/(ldm_centre.x);
    //if(scale2>scale) scale=scale2;
    if(scale2<scale) scale=scale2;

    //printf("scale=%lf\n",scale);
//    t.x=(scale*width-width)*0.5;
//    t.y=(scale*height-height)*0.5;
    t.x=(scale*ldm_centre.x-ldm_centre.x);
    t.y=(scale*ldm_centre.y-ldm_centre.y);
  }
  else{ //IMAGE IS FITTED TO KEEP WIDTH
    ami::point2d<double> temp2=d.evaluation_quotient( ami::point2d<double>(ldm_centre.x,(double) height));
    scale=(temp2-ldm_centre ).norm()/(ldm_centre.y);

    temp2=d.evaluation_quotient( ami::point2d<double>(ldm_centre.x,(double) 0.));
    double scale2=(temp2-ldm_centre ).norm()/(ldm_centre.y);
    //if(scale2>scale) scale=scale2;
    if(scale2<scale) scale=scale2;

    //printf("scale=%lf\n",scale);
//    t.x=(scale*width-width)*0.5;
//    t.y=(scale*height-height)*0.5;
    t.x=(scale*ldm_centre.x-ldm_centre.x);
    t.y=(scale*ldm_centre.y-ldm_centre.y);
  }


  int nc,n2,i,j;
  #ifdef _OPENMP
  #pragma omp parallel for \
   shared(width,height,width0,height0,output_image,input_image,size)\
   private(nc,i,j,n2)
  #endif
  for (nc=0;nc<3;nc++)
  {
     n2=nc*size;
    //#pragma omp for nowait
     for(i=0;i<height;i++){
      for(j=0;j<width;j++){
        ami::point2d<double> temp((double) j*scale-t.x,(double) i*scale-t.y);
        double distance_centre= (ldm_centre-temp).norm();

        //INTERPOLATION
        int ind=(int)distance_centre;
        if(ind>=(int)l1r.size()) continue;
        double dl1r=l1r[ind]+(distance_centre-ind)*(l1r[ind+1]-l1r[ind]);
        ami::point2d<double> p;

        p.x=ldm_centre.x+(temp.x-ldm_centre.x)*dl1r;
        p.y=ldm_centre.y+(temp.y-ldm_centre.y)*dl1r;

        //p=d.inverse_evaluation_quotient(temp);

         int m = (int)p.y;
         int n = (int)p.x;
         if(0<=m && m<height0 && 0<=n && n<width0)
         {
           //COLOUR INTERPOLATION
           double di=p.y-m;
           double dj=p.x-n;
           unsigned int k=i*width+j;
           unsigned int k0=m*width0+n;
           double accum=0;
           double w_accum=0;
           double w=((1.-di)*(1.-dj));

           accum+=(double)w*input_image[k0+n2];
           w_accum+=w;


           if( (di*(1.-dj))>0. && (m+1)<height0)
           {
                k0=(m+1)*width0+n;
                w=(di*(1.-dj));
                accum+=(double)w*input_image[k0+n2];
                w_accum+=w;
           }
           if( ((1-di)*(dj))>0. && (n+1)<width0)
           {
                k0=(m)*width0+n+1;
                w=(1.-di)*(dj);
                accum+=(double)w*input_image[k0+n2];
                w_accum+=w;
           }
           if( ((di)*(dj))>0. && (n+1)<width0 && (m+1)<height0)
           {
                k0=(m+1)*width0+n+1;
                w=(di)*(dj);
                accum+=(double)w*input_image[k0+n2];
                w_accum+=w;
           }
           if(w_accum>0.) output_image[k+n2]=(U) (accum/w_accum);
         }
      }
    }
  }
  free(a);
  return(output_image);
}

#endif
