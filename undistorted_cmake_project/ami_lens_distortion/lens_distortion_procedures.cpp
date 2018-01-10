/*
   Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
   License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
   see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
 */


#include "lens_distortion_procedures.h"
#include <iostream>
using namespace std;

/**
 * \fn distortion_points_to_line_equation_quotient(lens_distortion_model &d,
                                                    line_points &l)
 * \brief LINE ESTIMATION FROM POINT AFTER APPLYING A LENS DISTORTION MODEL TO
          THE POINTS. RETURN -1 IF IT DOES NOT WORK. OTHERWISE RETURN THE
          AVERAGE OF THE SQUARED DISTANCE OF THE POINTS TO THE LINE
          AFTER APPLYING A LENS DISTORTION MODEL
 * \param d INPUT DISTORTION CENTER
 * \param l OUTPUT LINE
 * \author Luis Alvarez
 */
double distortion_points_to_line_equation_quotient(
    lens_distortion_model &d,line_points &l)
{
  int i,j,k;
  long double suu,suv,svv/*,h0,h1*/,um,vm,h,r[4][3],min,paso,norma;
  long double cero=10e-100;
  int N=l.get_points().size();
  double a,b,c;

  if(N<2){
    //printf("Numero de puntos para el Calculo de la recta 2D menor que 2\n");
    return(-1.);
  }

  // WE CREATE AN AUXILIAR VECTOR OF POINTS
  vector< point2d<double> > points(l.get_points().size());
  for(i=0;i<((int)points.size());i++)
    points[i]=d.evaluation_quotient(l.get_points()[i]);

  suu=0; suv=0; svv=0; um=0; vm=0;
  for(i=0;i<N;i++){
    um+=points[i].x;
    vm+=points[i].y;
  }
  um/=N; vm/=N;
  for(i=0;i<N;i++){
    suu+=(points[i].x-um)*(points[i].x-um);
    svv+=(points[i].y-vm)*(points[i].y-vm);
    suv+=(points[i].x-um)*(points[i].y-vm);
  }
  suu/=N; svv/=N; suv/=N;
  /*  printf("um=%f,vm=%f,suu=%f,svv=%f,suv=%f",um,vm,suu,svv,suv); */
  if(fabs(suv)<= cero){
    if(suu<svv && svv>cero){
      //a=1; b=0; c=-um;
      a=1.;
      b=0.;
      c=-um;
      l.set_abc(a,b,c);
      return(0.);
    }
    if(svv<suu && suu>cero){
      //a=0; b=1; c=-vm;
      a=1.;
      b=0.;
      c=-vm;
      l.set_abc(a,b,c);
      return(0.);
    }
    //printf("No se pudo calcular la recta 2D\n");
    return(-1);
  }

  r[2][1]=r[3][1]=r[0][0]=r[1][0]=1.;
  h=0.5*(suu-svv)/suv;
  if(h>0){
    r[0][1]=-h-sqrt(1.+h*h);
    r[0][2]=-(um+r[0][1]*vm);
    r[1][1]=-1./r[0][1];
    r[1][2]=-(um+r[1][1]*vm);
    r[2][0]=h+sqrt(1.+h*h);
    r[2][2]=-(r[2][0]*um+vm);
    r[3][0]=-1./r[2][0];
    r[3][2]=-(r[3][0]*um+vm);
  }
  else{
    r[0][1]=-h+sqrt(1+h*h);
    r[0][2]=-(um+r[0][1]*vm);
    r[1][1]=-1./r[0][1];
    r[1][2]=-(um+r[1][1]*vm);
    r[2][0]=h-sqrt(1+h*h);
    r[2][2]=-(r[2][0]*um+vm);
    r[3][0]=-1./r[2][0];
    r[3][2]=-(r[3][0]*um+vm);
  }

  for(j=0;j<4;j++){
    norma=sqrt(r[j][0]*r[j][0]+r[j][1]*r[j][1]);
    for(i=0;i<3;i++)
     r[j][i]/=norma;
  }

  min=0.; k=0;
  for(i=0;i<N;i++){
    paso=r[0][0]*points[i].x+r[0][1]*points[i].y+r[0][2];
    min+=paso*paso;
  }
  for(j=1;j<4;j++){
    h=0;
    for(i=0;i<N;i++){
      paso=r[j][0]*points[i].x+r[j][1]*points[i].y+r[j][2];
      h+=paso*paso;
    }
    if(h<min){
      k=j;
      min=h;
    }
  }

  l.set_abc((double) r[k][0],(double)r[k][1],(double)r[k][2]);

  return min;
}

/**
 * \fn model_estimation_1p_quotient( point2d<double>  distortion_center,
                                    vector< line_points > &lines,
                                    lens_distortion_model &d)
 * \brief OPTIMISE THE DISTORTION PARAMETER OF THE DIVISION MODEL BY MEANS OF
          MINIMIZING THE DISTANCE BETWEEN THE CORRECTED POINTS AND THE STRAIGHT
          LINES
 * \author Luis Alvarez and Daniel Santana-Cedres
 */
double model_estimation_1p_quotient(
   point2d<double>  distortion_center /** INPUT DISTORTION CENTER */,
   vector< line_points > &lines /** INPUT VECTOR OF LINES TO COMPUTE */,
   lens_distortion_model &d /** OUTPUT DISTORTION MODEL */)
{
  double Ep0=0., Ep0_plus_h, Ep0_minus_h ,Ep1=0.;
  double gamma = 1.0;
  double k0=d.get_d()[1], k1=1.0;
  double dmi = distortion_center.norm();
  dmi*=dmi;
  // WE OBTAIN THE VALUE OF P0 WITH THE INITIAL VALUE OF K, FROM THE INPUT
  // DISTORTION MODEL
  double p0 = (-d.get_d()[1]*dmi) / (1+d.get_d()[1]*dmi);
  double p1=0.0;
  double h = 1e-4;

  // MODELS FOR p0+h, p0-h AND p1
  lens_distortion_model pldm, mldm, p1ldm;
  pldm.set_distortion_center(distortion_center);
  mldm.set_distortion_center(distortion_center);
  p1ldm.set_distortion_center(distortion_center);
  pldm.get_d().resize(2);
  mldm.get_d().resize(2);
  p1ldm.get_d().resize(2);
  pldm.get_d()[0]  = 1.;
  mldm.get_d()[0]  = 1.;
  p1ldm.get_d()[0] = 1.;

	//cout << "initial normalized distoriton parameter = " << p0 << endl;
	
  int convergence_it = 0;
	bool first = true;
  while((fabs(k0-k1))>((fabs(k1)+1e-30)*1e-8))
  {
    // INITIALIZATION OF THE SUMS
    Ep0 = Ep0_plus_h = Ep0_minus_h = Ep1 = 0.;

    //MODELS OF k0, k0+h AND k0-h
    k0 = -p0 / (dmi+dmi*p0);
    d.get_d()[1]    = k0;
    pldm.get_d()[1] = -(p0+h)/(dmi+dmi*(p0+h));
    mldm.get_d()[1] = -(p0-h)/(dmi+dmi*(p0-h));

    for(int i=0; i<(int)lines.size(); i++)
    {
      Ep0        += distortion_points_to_line_equation_quotient(d,lines[i]);
      Ep0_plus_h += distortion_points_to_line_equation_quotient(pldm,lines[i]);
      Ep0_minus_h+= distortion_points_to_line_equation_quotient(mldm,lines[i]);
    }
		
		if(first)
		{
			first = false;
			//cout << "E(p0) = " << Ep0 << endl;
		}

    double Ep0_prime_1st = (Ep0_plus_h-Ep0_minus_h) / (2*h);
    double Ep0_prime_2nd = (Ep0_plus_h - 2*Ep0 + Ep0_minus_h) / (h*h);

    p1 = p0 - (Ep0_prime_1st / (Ep0_prime_2nd + gamma));
    k1 = -p1 / (dmi+dmi*p1);
    p1ldm.get_d()[1] = k1;
    for(int i=0; i<(int)lines.size(); i++)
    {
      Ep1 += distortion_points_to_line_equation_quotient(p1ldm,lines[i]);
    }

    if(Ep1 < Ep0)
    {
      p0 = p1;
      gamma /= 10;
    }
    else
    {
      gamma *= 10;
    }
    convergence_it++;
  }

  d.get_d()[1] = -p0 / (dmi+dmi*p0);

	//cout << "estimated normalized distoriton parameter = " << p0 << endl;

  if(Ep1<Ep0)
	{
		//cout << "E(p0opt) = " << Ep1 << endl;
    return Ep1;
	}
  else
	{
		//cout << "E(p0opt) = " << Ep0 << endl;
    return Ep0;
	}
}

/**
 * \fn int build_l1r_quotient_vector(vector<double> &l1r,point2d<double> &dc,
                                     double max_distance_corner,int Na,
                                     double *a)
 * \brief Build an intermediate vector with values of L-1(r) for
          d = (0 to max_distance_corner)
 * \param [in] [out] l1r vector to store the roots
 * \param [in] dc distortion center
 * \param [in] max_distance_corner Maximum distance from distortion center to a
                corner
 * \param [in] a Lens distortion model polynom
 * \param [in] Na Degree of the lens distortion model polynom
 * \author Luis Alvarez, Pedro Henriquez
 */
int build_l1r_quotient_vector(std::vector<double> &l1r,
                              double max_distance_corner, double *a,int Na)
{
    //BUILD INTERMEDIATE VECTOR WITH L-1(r) FROM CENTER TO FURTHEST CORNER
    if(a[Na]==0. || Na<2) return(-1);
    l1r.resize((int)(max_distance_corner+1.5));

    // WE BUILD THE VECTOR OF INVERSE LENS DISTORTION FUNCTION
    double root,disc,root1,root2;
    int dist=(int)(max_distance_corner+0.5);
    disc= 1.-4*dist*dist*a[2];
    if(disc<0) return(-2);
    disc=sqrt(disc);
    root1=(1-disc)/(2*dist*a[2]);
    root2=(1+disc)/(2*dist*a[2]);
    if(root1>0 ) { l1r[dist]=root1/dist; root=root1;}
    else {l1r[dist]=root2/dist; root=root2; }

    while( (dist--)>0)
    {
        disc= 1.-4*dist*dist*a[2];
        if(disc<0) return(-2);
        disc=sqrt(disc);
        root1=(1-disc)/(2*dist*a[2]);
        root2=(1+disc)/(2*dist*a[2]);
        if(fabs(root-root1)<fabs(root-root2) ) { l1r[dist]=root1/dist; root=root1;}
        else {l1r[dist]=root2/dist; root=root2; }
    }

    l1r[0]=l1r[1];

    return 0;
}
